/*
 * The PCIe root complex of the Xiaomi 14 Pro (shennong, SM8650) that carries
 * its WLAN: a WCN7850 ("kiwi") on pcie0 (qcom,pcie@1c00000, cnss_pci0 under
 * pcie0_rp; qcom,cnss-kiwi@b0000000, compatible "qcom,cnss-kiwi").
 *
 * The common kernel has no PCI: this driver does the host side by hand, the
 * way the vendor's Linux driver (drivers/pci/controller/dwc/pci-msm.c over
 * drivers/pci/controller/dwc/pcie-designware-host.c) and the Goodix touch
 * driver do (drivers/input/hakios.goodix.c): map the registers, follow the
 * device tree's sequence, poll for what should happen. Only with boot-arg
 * pcie=1. No interrupt is used: a thread looks at the link and, later, rings
 * the MHI doorbells (the modem's and the DSP's drivers share that pattern).
 *
 * Steps (boot-arg pcie_steps=N stops after step N; pcie_probe_us=N waits):
 *   1  map PARF/PHY/ELBI/dm_core/pcie_sm and read the link state (read-only)
 *   2  power the host side: port GDSCs on, CESTA sequence loaded, port clock
 *      votes, resets pulsed, PARF for training (RC mode, PHY clocks on,
 *      lane upscaling bits), PHY sequence, SerDes ready poll, LTSSM enabled
 *   3  CNSS power through RPMh, wlan_en high, AOP PDC votes for the RF
 *      island, PERST out, LTSSM enabled again, poll ELBI link-up and DLL
 *      active; read-only probes if still down
 *   4  ECAM: the endpoint's vendor/device IDs (link-gated, read-only)
 * Later: MHI, amss.bin download, ath11k attach.
 *
 * Register layouts: Linux drivers/pci/controller/dwc/pcie-designware.h
 * (ELBI/LTSSM), the phone's own vendor kernel (same compatible
 * "qcom,pci-msm", shennong-u-oss: drivers/pci/controller/pci-msm.c for the
 * PARF programming, reset pulse, PHY init, CESTA load and link_train order;
 * ath11k pci.c for the endpoint-side order; drivers/clk/qcom/gcc-pineapple.c
 * and qcom,gcc-pineapple.h for the port clocks, BCRs and reset IDs),
 * drivers/pmdomain/qcom/gdsc.c (collapse clear and power-on poll), and the
 * phone's own device tree (qcom,pcie@1c00000: parf/phy/dm_core/elbi/iatu/
 * conf/pcie_sm windows, sm-seq/branch-seq/start offsets, qcom,phy-sequence
 * with power-down/power-status offsets, perst-gpio 94, wlan-en-gpio 16 in
 * qcom,cnss-kiwi) plus its bootarg pci-msm-drv.pcie_sm_regs.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern void hakios_tl(const char *format, ...) __printflike(1, 2);

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/* RPMh's sequencer lock (shared with the touch, ADSP, Wi-Fi and modem drivers) */
extern lck_mtx_t hakios_rsc_lock;

/* The always-on processor's mailbox lock (drivers/core/hakios.platform.c,
 * shared with the modem, Wi-Fi and ADSP drivers). */
extern lck_mtx_t hakios_aop_mailbox_lock;

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* ---- TLMM GPIOs (per-pin 4KB maps, like the Synaptics driver) --------------------- */

#define TLMM_BASE               0x0f100000UL
#define PIN_PAGE(n)             (TLMM_BASE + 0x1000UL * (n))

#define PIN_WLAN_EN             16      /* wlan-en-gpio: high = WLAN powered */
#define PIN_PERST               94      /* perst-gpio: low = endpoint in reset */
#define PIN_WAKE                96      /* wake-gpio (input) */

static vm_offset_t pin_map[97];

static inline uint32_t
tlmm_rd(vm_offset_t addr)
{
	return *(volatile uint32_t *)addr;
}

static inline void
tlmm_wr(vm_offset_t addr, uint32_t v)
{
	*(volatile uint32_t *)addr = v;
}

static inline void
wr(vm_offset_t addr, uint32_t v)
{
	*(volatile uint32_t *)addr = v;
}

#define GPIO_CFG(n)             (pin_map[n] + 0)
#define GPIO_IO(n)              (pin_map[n] + 4)

/* TLMM values (pinctrl-msm: [1:0] pull, [5:2] function, [8:6] drive, [9] output).
 * wlan_en active is dt cnss_wlan_en_active (gpio, 16 mA, output high, pull up);
 * PERST is dt pcie0_perst_default (gpio, 2 mA, pull down, driven by hand). */
#define GPIO_WLAN_EN_CFG        (3u | 0 << 2 | 7u << 6 | 1u << 9)
#define GPIO_PERST_CFG          (1u | 0 << 2 | 0u << 6 | 1u << 9)

/* PERST (pci-msm asserts it across the whole bring-up and polls LTSSM after it
 * goes out; PCIe CEM T_PVPERL wants power stable 100 ms before it goes out). */
#define PERST_LOW_MS            100
#define PERST_SETTLE_MS         100

/* ---- PCIe0 windows (dt: qcom,pcie@1c00000) ------------------------------------------ */

#define PCIE_PARF               0x01c00000UL
#define PCIE_PARF_SIZE          0x3000UL
#define PCIE_PHY                0x01c06000UL
#define PCIE_PHY_SIZE           0x2000UL
#define PCIE_DM_CORE            0x60000000UL
#define PCIE_DM_CORE_SIZE       0xf1dUL
#define PCIE_ELBI               0x60000f20UL
#define PCIE_ELBI_SIZE          0xa8UL
#define PCIE_SM                 0x01d07000UL
#define PCIE_SM_SIZE            0x7000UL

static vm_offset_t parf, phy, dmcore, elbi, pcie_sm;

static inline uint32_t
rd(vm_offset_t addr)
{
	return *(volatile uint32_t *)addr;
}

/* PARF (Qualcomm application registers): version and link state first. */
#define PARF_VERSION            0x00
#define PARF_LTSSM              0x1b0           /* dt first parf-debug-reg; link state */
#define PARF_LTSSM_EN           (1u << 8)       /* pci-msm msm_pcie_enable: written to start training */
#define PARF_LTSSM_UP           0x111           /* L0 with the enable bit: what the Wi-Fi driver's
						 * step_cnss_probe gates ECAM on, with PHY bit 6 */
#define PARF_PHY_STATUS         0x214           /* dt qcom,phy-status-offset; upstream
						 * applies it on the phy window (see
						 * PHY_STATUS), parf+0x214 printed
						 * for continuity only */
#define PARF_PHY_STATUS_BIT     (1u << 6)       /* dt qcom,phy-status-bit */

/* PARF training programming (upstream pci-msm msm_pcie_enable_link; every
 * offset is inside the 0x3000 parf window). Upstream writes SYS_CTRL 0x365E
 * unconditionally, so 0x00 is not a version register to preserve. */
#define PARF_SYS_CTRL           0x00            /* = 0x365E (RC system control) */
#define PARF_SYS_CTRL_RC        0x365e
#define PARF_PHY_CTRL           0x40            /* clear bit0: PCIe clocks and resets on */
#define PARF_MHI_CLOCK_RESET_CTRL 0x174         /* clear bit4 */
#define PARF_AXI_MSTR_WR_ADDR_HALT 0x1a8        /* set bit31, keep the size */
#define PARF_INT_ALL_MASK       0x22c           /* all masked: no interrupts used here */
#define PARF_CFG_BITS_3         0x2c4           /* clear bit0 (fal10_veto override),
						 * set bit8 (dynamic width upscaling) */
#define PARF_DBI_BASE_ADDR      0x350           /* = 0 */
#define PARF_SLV_ADDR_SPACE_SIZE 0x358          /* dt qcom,slv-addr-space-size */
#define PARF_SLV_ADDR_SIZE      0x4000000UL
#define PARF_DEVICE_TYPE        0x1000          /* = 4: root complex */
#define PARF_DEVICE_TYPE_RC     0x4

/* Link-up test (upstream msm_pcie_link_train polls ELBI, not PARF). */
#define ELBI_SYS_STTS           0x08
#define ELBI_LINK_UP            (1u << 10)      /* pci-msm XMLH_LINK_UP */

/* DWC core link status (upstream confirm_linkup also wants DLL active). */
#define DM_CAP_LINKCTRLSTATUS   0x80            /* cap 0x70 + 0x10 */
#define DM_DLL_ACTIVE           (1u << 29)
#define DM_LNKCAP               0x7c            /* cap + PCI_EXP_LNKCAP (32-bit) */
/* LNKSTA lives at cap + 0x12, not 32-bit aligned: read the 32-bit LINKCTRLSTA
 * at 0x80 and shift instead (an unaligned MMIO load faults). */
#define DM_LNKSTA_SHIFT         16
#define DM_AUX_CLK_FREQ         0xb40           /* what aux clock the core sees */
#define DM_LANE_SKEW_OFF        0x714           /* clear bit5 (dynamic width upscaling) */

/* PCIe_SM debug (dt qcom,pcie-sm-debug; read-only CESTA state probe). */
#define SM_DBG0                 0x1040
#define SM_DBG1                 0x1048
#define SM_START                0x1090          /* dt qcom,pcie-sm-start-offset */
/* CESTA state-manager load (upstream msm_pcie_cesta_load_sm_seq at probe).
 * PWR_CTRL/MASK offsets come from the phone's own bootarg
 * pci-msm-drv.pcie_sm_regs=0x1D07000,0x1040,0x1048,0x3000,0x2 (rc 0, no
 * stride); the sequences are dt qcom,pcie-sm-seq (13 words at +0x0) and
 * qcom,pcie-sm-branch-seq (7 words at branch offset 0x1000); enabling sets
 * BIT(0) of the start offset. All offsets lie in the 0x7000 pcie_sm window. */
#define SM_PWR_CTRL             0x1040
#define SM_PWR_MASK             0x1048
#define SM_BRANCH_OFF           0x1000          /* dt qcom,pcie-sm-branch-offset */
static const uint32_t sm_seq[] = {
	0x1c018081, 0x70074002, 0x50028000, 0x28007003, 0x80804002, 0x70021c01,
	0x18001802, 0x70005000, 0x10004000, 0x80814002, 0x18001c01, 0x1c018080,
	0x00000100,
};
static const uint32_t sm_branch[] = {
	0x04, 0x1c, 0x24, 0x2c, 0x00, 0x00, 0x00,
};

/* ECAM (dt conf 0x60100000): one function; read-only here, BAR programming is ath11k's later. */
#define PCIE_CONF               0x60100000UL
#define PCIE_CONF_SIZE          0x100000UL
#define ECAM_VENDOR_DEVICE      0x00            /* [15:0] vendor, [31:16] device */
#define ECAM_CLASS              0x08
#define ECAM_BAR0_LO            0x10
#define ECAM_BAR0_HI            0x14
#define PCI_VENDOR_QCOM         0x17cb
#define PCI_DEVICE_WCN7850      0x1107          /* dt supported-ids of qcom,cnss-kiwi */

/* ---- GCC + GDSC for pcie0 (Linux gcc-sm8650.c) ---------------------------------------- */

#define GCC_BASE                0x00100000UL
#define GCC_PCIE_VOTE           0x52008         /* the port's vote register */
/* Port clock votes (gcc-pineapple.c; every pcie_0 branch votes 0x52008):
 * BIT(0) slv_axi, BIT(1) mstr_axi, BIT(2) cfg_ahb, BIT(3) aux, BIT(4) pipe,
 * BIT(5) slv_q2a, BIT(6) cnoc_pcie_sf_axi (the port's only DT clock). The DT
 * lists just that one but the bootloader's state is unknown and votes only
 * enable, so vote them all, the way Linux would for listed clocks. Halt
 * status (read-only): 0x6b01c/0x6b020/0x6b02c/0x6b038/0x6b03c/0x6b048.
 * Not written: phy_rchng (0x52000 BIT(22), a shared vote register; only the
 * rate-change clock, unneeded for initial training). */
#define GCC_PCIE_VOTE_ALL       ((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | \
				 (1u << 4) | (1u << 5) | (1u << 6))
#define GCC_PCIE_0_BCR          0x6b000         /* dt reset pcie_0_core_reset (gcc 3) */
#define GCC_PCIE_0_PHY_BCR      0x6c01c         /* dt reset pcie_0_phy_reset (gcc 6) */
/* Port branch CBCRs (gcc-pineapple.c halt_reg; all branch2 ops: enabling is
 * the 0x52008 vote plus CBCR_ENABLE bit0 here, the GPU CBCR's way. OFF bit31
 * reads 0 even for an unenabled branch, so the check is bit0, not OFF; hwcg
 * bit1 on slv/mstr/cfg/cnoc is power gating only, untouched). */
#define GCC_CBCR_ENABLE         (1u << 0)
#define GCC_PCIE_0_SLV_Q2A_CBCR 0x6b01c
#define GCC_PCIE_0_SLV_AXI_CBCR 0x6b020
#define GCC_PCIE_0_MSTR_AXI_CBCR 0x6b02c
#define GCC_PCIE_0_CFG_AHB_CBCR 0x6b038
#define GCC_PCIE_0_AUX_CBCR     0x6b03c
#define GCC_PCIE_0_PIPE_CBCR    0x6b048
#define GCC_CNOC_PCIE_SF_AXI_CBCR 0x10058
#define GDSC_PCIE_0             0x6b004         /* bit31 on; clear bit0 (collapse) to power */
#define GDSC_PCIE_0_PHY         0x6c000
#define GDSC_ON                 (1u << 31)
#define GDSC_HW_CONTROL         (1u << 1)
#define GDSC_SW_COLLAPSE        (1u << 0)

static vm_offset_t gcc;

static inline uint32_t
gcc_rd(uint32_t off)
{
	return rd(gcc + off);
}

static inline void
gcc_wr(uint32_t off, uint32_t v)
{
	wr(gcc + off, v);
}

/* Power a port domain: clear the software collapse bit and wait for bit 31,
 * the GPU CX GDSC's way (gdsc.c gdsc_enable). False: not ours, or stays off. */
static bool
gdsc_on(uint32_t off, const char *name)
{
	uint32_t gdscr = gcc_rd(off);

	printf("pcie: gdsc %s at %#x: %#x\n", name, GCC_BASE + off, gdscr);
	if (gdscr & GDSC_ON) {
		printf("pcie: gdsc %s is on already\n", name);
		return true;
	}
	if (gdscr & GDSC_HW_CONTROL) {
		printf("pcie: gdsc %s is under hardware control: not ours to operate\n", name);
		return false;
	}
	gcc_wr(off, gdscr & ~GDSC_SW_COLLAPSE);
	for (int i = 0; i < 500 && !(gcc_rd(off) & GDSC_ON); i++)
		sleep_us(100);
	gdscr = gcc_rd(off);
	printf("pcie: gdsc %s now %#x%s\n", name, gdscr, gdscr & GDSC_ON ? ": on" : ": STAYS OFF");
	hakios_tl("tl: pcie gdsc %s=%#x\n", name, gdscr);
	return (gdscr & GDSC_ON) != 0;
}

/* Load the CESTA state manager (upstream msm_pcie_cesta_load_sm_seq: clear
 * the PWR overrides, write both sequences, set START bit0). Goes after power:
 * an unpowered window must not be written. Readback printed, never gated. */
static void
cesta_load(void)
{
	unsigned i;

	wr(pcie_sm + SM_PWR_CTRL, 0);
	wr(pcie_sm + SM_PWR_MASK, 0);
	for (i = 0; i < sizeof(sm_seq) / sizeof(sm_seq[0]); i++)
		wr(pcie_sm + 4 * i, sm_seq[i]);
	for (i = 0; i < sizeof(sm_branch) / sizeof(sm_branch[0]); i++)
		wr(pcie_sm + SM_BRANCH_OFF + 4 * i, sm_branch[i]);
	wr(pcie_sm + SM_START, rd(pcie_sm + SM_START) | 1u);
	printf("pcie: CESTA loaded: start %#x seq0 %#x seq12 %#x branch %#x/%#x\n",
	    rd(pcie_sm + SM_START), rd(pcie_sm + 0), rd(pcie_sm + 4 * 12),
	    rd(pcie_sm + SM_BRANCH_OFF), rd(pcie_sm + SM_BRANCH_OFF + 4 * 3));
	hakios_tl("tl: pcie cesta start=%#x\n", rd(pcie_sm + SM_START));
}

/* (port branch clocks are vote-only; halt regs are read-only status, reported
 * inline in step 2 instead of a helper.) */

/* The PHY init sequence (dt qcom,phy-sequence of qcom,pcie@1c00000): triples of
 * register offset, value, delay in us. pci-msm's pcie_phy_init writes each
 * value to phy + offset and waits the delay; all delays here are 0. */
/* SerDes ready (upstream pci-msm pcie_phy_is_ready, two call sites plus the
 * "PHY status bit is set ... PHY is off" comment): dt qcom,phy-status-offset
 * applied on the *phy* window (dev->phy + 0x214), bit set = PHY off, bit
 * clear = ready. Polled before pipe reset and PERST, so it is the SerDes
 * itself (power/sequence/start), not the pipe to the controller and not the
 * endpoint. The 0x200/0x240/0x244 readbacks below only prove the sequence's
 * own values stuck; no IP meaning is claimed for them. */
#define PHY_POWER_DOWN          0x240           /* dt qcom,phy-power-down-offset;
						 * sequence leaves 3 (powered up:
						 * upstream only clears it to power
						 * down, "active low") */
#define PHY_SW_RESET            0x200           /* sequence leaves 0: stuck check */
#define PHY_START_CTRL          0x244           /* sequence leaves 3: stuck check */
#define PHY_STATUS              0x214           /* dt qcom,phy-status-offset, phy window;
						 * poll bit6 CLEAR = SerDes ready */
#define PHY_STATUS_BIT          (1u << 6)       /* set = PHY off (upstream) */
#define PHY_READY_WAIT_MS       200
#define PHY_STAB_MS             1               /* upstream PHY_STABILIZATION_DELAY ~1 ms */
static const uint32_t phy_seq[] = {
	0x240, 0x03, 0x00, 0xc0, 0x01, 0x00, 0xcc, 0x62, 0x00, 0xd0, 0x02, 0x00,
	0x60, 0xf8, 0x00, 0x64, 0x01, 0x00, 0x00, 0x93, 0x00, 0x04, 0x01, 0x00,
	0xe0, 0x90, 0x00, 0xe4, 0x82, 0x00, 0xf4, 0x07, 0x00, 0x70, 0x02, 0x00,
	0x10, 0x02, 0x00, 0x74, 0x16, 0x00, 0x14, 0x16, 0x00, 0x78, 0x36, 0x00,
	0x18, 0x36, 0x00, 0x110, 0x08, 0x00, 0xbc, 0x0a, 0x00, 0x120, 0x42, 0x00,
	0x80, 0x04, 0x00, 0x84, 0x0d, 0x00, 0x20, 0x0a, 0x00, 0x24, 0x1a, 0x00,
	0x88, 0x41, 0x00, 0x28, 0x34, 0x00, 0x90, 0xab, 0x00, 0x94, 0xaa, 0x00,
	0x98, 0x01, 0x00, 0x30, 0x55, 0x00, 0x34, 0x55, 0x00, 0x38, 0x01, 0x00,
	0x140, 0x14, 0x00, 0x164, 0x34, 0x00, 0x3c, 0x01, 0x00, 0x1c, 0x04, 0x00,
	0x174, 0x16, 0x00, 0x1bc, 0x0f, 0x00, 0x170, 0xa0, 0x00, 0x11a4, 0x38, 0x00,
	0x10dc, 0x11, 0x00, 0x1160, 0xbf, 0x00, 0x1164, 0xbf, 0x00, 0x1168, 0xb7, 0x00,
	0x116c, 0xea, 0x00, 0x115c, 0x3f, 0x00, 0x1174, 0x5c, 0x00, 0x1178, 0x9c, 0x00,
	0x117c, 0x1a, 0x00, 0x1180, 0x89, 0x00, 0x1170, 0xdc, 0x00, 0x1188, 0x94, 0x00,
	0x118c, 0x5b, 0x00, 0x1190, 0x1a, 0x00, 0x1194, 0x89, 0x00, 0x10cc, 0x00, 0x00,
	0x1008, 0x09, 0x00, 0x1014, 0x05, 0x00, 0x104c, 0x08, 0x00, 0x1050, 0x08, 0x00,
	0x10d8, 0x0f, 0x00, 0x1118, 0x1c, 0x00, 0x10f8, 0x07, 0x00, 0x11f8, 0x08, 0x00,
	0x1600, 0x00, 0x00, 0xe84, 0x15, 0x00, 0xe90, 0x3f, 0x00, 0xee4, 0x02, 0x00,
	0xe40, 0x09, 0x00, 0xe3c, 0x15, 0x00, 0x19a4, 0x38, 0x00, 0x18dc, 0x11, 0x00,
	0x1960, 0xbf, 0x00, 0x1964, 0xbf, 0x00, 0x1968, 0xb7, 0x00, 0x196c, 0xea, 0x00,
	0x195c, 0x3f, 0x00, 0x1974, 0x5c, 0x00, 0x1978, 0x9c, 0x00, 0x197c, 0x1a, 0x00,
	0x1980, 0x89, 0x00, 0x1970, 0xdc, 0x00, 0x1988, 0x94, 0x00, 0x198c, 0x5b, 0x00,
	0x1990, 0x1a, 0x00, 0x1994, 0x89, 0x00, 0x18cc, 0x00, 0x00, 0x1808, 0x09, 0x00,
	0x1814, 0x05, 0x00, 0x184c, 0x08, 0x00, 0x1850, 0x08, 0x00, 0x18d8, 0x0f, 0x00,
	0x1918, 0x1c, 0x00, 0x18f8, 0x07, 0x00, 0x19f8, 0x08, 0x00, 0x1684, 0x15, 0x00,
	0x1690, 0x3f, 0x00, 0x16e4, 0x02, 0x00, 0x1640, 0x09, 0x00, 0x163c, 0x15, 0x00,
	0x2dc, 0x05, 0x00, 0x388, 0x77, 0x00, 0x398, 0x0b, 0x00, 0x6a4, 0x1e, 0x00,
	0x6f4, 0x27, 0x00, 0x3e0, 0x0f, 0x00, 0x60c, 0x1d, 0x00, 0x614, 0x07, 0x00,
	0x620, 0xc1, 0x00, 0x694, 0x00, 0x00, 0x3d0, 0x8c, 0x00, 0x368, 0x17, 0x00,
	0x370, 0x2e, 0x00, 0x200, 0x00, 0x00, 0x244, 0x03, 0x00,
};

static void
phy_init(void)
{
	for (unsigned i = 0; i < sizeof(phy_seq) / sizeof(phy_seq[0]); i += 3) {
		wr(phy + phy_seq[i], phy_seq[i + 1]);
		if (phy_seq[i + 2] != 0)
			sleep_us(phy_seq[i + 2]);
	}
	printf("pcie: PHY sequence done (%u writes), status %#x\n",
	    (unsigned)(sizeof(phy_seq) / sizeof(phy_seq[0]) / 3), rd(parf + PARF_PHY_STATUS));
	hakios_tl("tl: pcie phy-seq done phy=%#x\n", rd(parf + PARF_PHY_STATUS));
}

/* SerDes ready: the sequence's key writes stuck, and the PHY status bit went
 * clear (upstream fails the enable (-ENODEV) when it never does). False: the
 * SerDes itself never came out of power-down (refclk/analog power live in
 * the bootloader's half); nothing after this point can train. */
static bool
phy_serdes_ready(void)
{
	printf("pcie: SerDes state: 0x200=%#x (want 0), 0x240=%#x (want 3), 0x244=%#x (want 3)\n",
	    rd(phy + PHY_SW_RESET), rd(phy + PHY_POWER_DOWN), rd(phy + PHY_START_CTRL));
	hakios_tl("tl: pcie serdes sw=%#x pd=%#x start=%#x\n",
	    rd(phy + PHY_SW_RESET), rd(phy + PHY_POWER_DOWN), rd(phy + PHY_START_CTRL));
	for (int i = 0; i < PHY_READY_WAIT_MS / 10; i++) {
		if (!(rd(phy + PHY_STATUS) & PHY_STATUS_BIT))
			break;
		sleep_ms(10);
	}
	printf("pcie: SerDes PHY status %#x%s\n", rd(phy + PHY_STATUS),
	    rd(phy + PHY_STATUS) & PHY_STATUS_BIT ? ": NOT READY (PHY off)" : ": ready");
	hakios_tl("tl: pcie serdes status=%#x\n", rd(phy + PHY_STATUS));
	return !(rd(phy + PHY_STATUS) & PHY_STATUS_BIT);
}

/* ---- RPMh votes for the CNSS rails (v3 TCS layout, like the touch driver) --------------- */

#define SYN_CMD_DB_PHYS         0x81c60000UL
#define SYN_CMD_DB_SIZE         0x20000UL
#define SYN_RSC_PHYS            0x17a20000UL
#define SYN_RSC_SIZE            0x10000UL

static vm_offset_t pcie_rsc, pcie_db;

struct pcie_cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct pcie_cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct pcie_cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct pcie_cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static const struct {
	uint32_t wait, control, status, enable, msgid, addr, data, cmd_status, stride;
} pcie_rh_v3 = { 0x20, 0x24, 0x28, 0x2c, 0x34, 0x38, 0x3c, 0x40, 24 };

static uint32_t
pcie_rpmh_address(const char *name)
{
	const struct pcie_cmd_db_header *db = (const struct pcie_cmd_db_header *)pcie_db;
	char id[8] = { 0 };

	if (!db || db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c)
		return 0;
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct pcie_cmd_db_slave *slave = &db->slave[s];
		const struct pcie_cmd_db_entry *entry =
		    (const struct pcie_cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->header_offset >= SYN_CMD_DB_SIZE - 0x1000 || slave->count > 512)
			continue;
		for (unsigned i = 0; i < slave->count; i++)
			if (memcmp(entry[i].id, id, sizeof(id)) == 0)
				return entry[i].address;
	}
	return 0;
}

/* vote one VRM rail (voltage uV, HPM); true: RPMh says done */
static bool
pcie_vote_rail(const char *name, uint32_t uv)
{
	const uint32_t TCS = 0xd00;
	uint32_t base = pcie_rpmh_address(name);
	uint32_t addr[3] = { base + 0, base + 4, base + 8 };
	uint32_t data[3] = { uv / 1000, 1, 7 };
	const typeof(pcie_rh_v3) *L = &pcie_rh_v3;
	int chosen = -1;

	if (base == 0 || (base >> 16 & 0xf) != 4) {
		hakios_tl("tl: pcie rail %s: no VRM (%#x)\n", name, base);
		return false;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	for (unsigned t = 0; t < 3; t++) {
		uint32_t b = TCS + t * 0x2a0;

		if (chosen < 0 && rd(pcie_rsc + b + L->status) != 0 &&
		    rd(pcie_rsc + b + L->control) == 0 && rd(pcie_rsc + b + L->enable) == 0)
			chosen = (int)t;
	}
	if (chosen < 0) {
		lck_mtx_unlock(&hakios_rsc_lock);
		hakios_tl("tl: pcie rail %s: no free TCS\n", name);
		return false;
	}
	{
	uint32_t b = TCS + (uint32_t)chosen * 0x2a0, bit = 1u << (unsigned)chosen, done = 0;

	wr(pcie_rsc + TCS + 0, rd(pcie_rsc + TCS + 0) & ~bit);
	for (unsigned i = 0; i < 3; i++) {
		uint32_t cmd = b + i * L->stride;

		wr(pcie_rsc + cmd + L->msgid, 0x10008 | 0x100);
		wr(pcie_rsc + cmd + L->addr, addr[i]);
		wr(pcie_rsc + cmd + L->data, data[i]);
	}
	wr(pcie_rsc + b + L->wait, 7);
	wr(pcie_rsc + b + L->enable, 7);
	wr(pcie_rsc + b + L->control, 0);
	wr(pcie_rsc + b + L->control, 1u << 16);
	wr(pcie_rsc + b + L->control, (1u << 16) | (1u << 24));
	for (int i = 0; i < 1000; i++) {
		bool all = true;

		for (unsigned c = 0; c < 3; c++) {
			uint32_t cs = rd(pcie_rsc + b + L->cmd_status + c * L->stride);

			all &= cs != 0xffffffffu && (cs & 0x10100u) == 0x10100u;
		}
		if (all && rd(pcie_rsc + b + L->status) != 0) {
			done = 1;
			break;
		}
		sleep_us(100);
	}
	wr(pcie_rsc + b + L->control, 1u << 16);
	wr(pcie_rsc + b + L->control, 0);
	wr(pcie_rsc + b + L->enable, 0);
	wr(pcie_rsc + b + L->wait, 0);
	wr(pcie_rsc + TCS + 8, bit);
	wr(pcie_rsc + TCS + 0, rd(pcie_rsc + TCS + 0) | 0);
	lck_mtx_unlock(&hakios_rsc_lock);
	hakios_tl("tl: pcie rail %s=%u %s\n", name, uv, done ? "on" : "TIMEOUT");
	return done != 0;
	}
}

/* ---- AOP votes for the WLAN RF island (QMP mailbox, like the modem's) --------------- */
/*
 * The 8 RSC rails are the PDC rails by another name (dt qcom,cnss-kiwi vdd
 * configs and pdc_init_table upvals agree to the volt: s4i=smpi4@940000,
 * s2c=smpc2@976000, s3c=smpc3@916000, s6c=smpc6@1864000, s1c=smpc1@1316000,
 * l15B=ldob15@1800000, l3c=ldoc3@1200000; ldok6 is extra). Voltages alone do
 * not run the island: the PDC table goes to the AOP as QMP text (dmesg shows
 * the same strings from bt_aop_send_msg), which sequences the RF power
 * domain. Linux qmp_send over the AOSS mailbox; the modem driver speaks the
 * same mailbox, under the same lock. Only the five upvals are sent; the
 * dwnvals belong to a teardown this flow never does. l15B/l3c have no table
 * entries, so there is nothing to send for them.
 */
#define QMP_MAGIC               0x00
#define QMP_MAGIC_VALUE         0x4d41494c      /* "MAIL" */
#define QMP_VERSION             0x04
#define QMP_THEIR_LINK          0x0c
#define QMP_THEIR_LINK_ACK      0x10
#define QMP_THEIR_CHANNEL       0x14
#define QMP_THEIR_CHANNEL_ACK   0x18
#define QMP_OUR_LINK            0x24
#define QMP_OUR_LINK_ACK        0x28
#define QMP_OUR_CHANNEL         0x2c
#define QMP_OUR_CHANNEL_ACK     0x30
#define QMP_OUR_BOX_SIZE        0x34
#define QMP_OUR_BOX             0x38
#define QMP_UP                  0x0000ffffu
#define QMP_DOWN                0xffff0000u
#define QMP_MESSAGE             64

#define QMP_AOSS_BASE           0x0c300000UL
#define QMP_AOSS_SIZE           0x400UL
#define QMP_IPCC_BASE           0x00406000UL    /* the APPS side (modem/Wi-Fi/ADSP all use 0x406000
						 * through soc_ipcc; 0x408000 is the modem's own and hangs) */
#define QMP_IPCC_SIZE           0x1000UL
#define QMP_IPCC_SEND           0x0c
#define QMP_IPCC_AOP            0u              /* client 0, signal 0 */

static vm_offset_t qmp_aoss, qmp_ipcc, qmp_box; /* qmp_box 0: mailbox not open */

/* Wait for a mailbox word, at most 2 s (200 x 10 ms); announce what is waited
 * for and what came back, so any stuck point shows within one round. */
static bool
qmp_wait(uint32_t off, uint32_t v, const char *what)
{
	printf("pcie: AOP waiting for %s == %#x\n", what, v);
	for (int i = 0; i < 200; i++) {
		if (rd(qmp_aoss + off) == v) {
			printf("pcie: AOP %s == %#x\n", what, v);
			return true;
		}
		sleep_ms(10);
	}
	printf("pcie: AOP %s timed out, reads %#x\n", what, rd(qmp_aoss + off));
	return rd(qmp_aoss + off) == v;
}

static void
qmp_ring(void)
{
	wr(qmp_ipcc + QMP_IPCC_SEND, QMP_IPCC_AOP << 16 | 0);
}

/* Linux qmp_open: what is done already is found done. Lock held by caller. */
static bool
qmp_open(void)
{
	uint32_t box, size;

	if (qmp_box != 0)
		return true;
	if (qmp_aoss == 0)
		qmp_aoss = ml_io_map(QMP_AOSS_BASE, QMP_AOSS_SIZE);
	if (qmp_ipcc == 0)
		qmp_ipcc = ml_io_map(QMP_IPCC_BASE, QMP_IPCC_SIZE);
	printf("pcie: AOP mailbox %p/%p\n", qmp_aoss, qmp_ipcc);
	if (!qmp_aoss || !qmp_ipcc) {
		printf("pcie: AOP mailbox failed to map\n");
		return false;
	}
	printf("pcie: AOP magic %#x version %#x (want 0x4d41494c/1)\n",
	    rd(qmp_aoss + QMP_MAGIC), rd(qmp_aoss + QMP_VERSION));
	if (rd(qmp_aoss + QMP_MAGIC) != QMP_MAGIC_VALUE || rd(qmp_aoss + QMP_VERSION) != 1) {
		printf("pcie: not a mailbox this knows\n");
		return false;
	}
	box = rd(qmp_aoss + QMP_OUR_BOX);
	size = rd(qmp_aoss + QMP_OUR_BOX_SIZE);
	printf("pcie: AOP our box %#x size %#x\n", box, size);
	if (size < QMP_MESSAGE + 4 || box < 0x3c || box + size > QMP_AOSS_SIZE || (box & 3)) {
		printf("pcie: our mailbox is not where a message fits (%#x+%#x)\n", box, size);
		return false;
	}
	printf("pcie: AOP link/ch as found: our_link %#x ack %#x, our_ch %#x ack %#x, their_ch %#x\n",
	    rd(qmp_aoss + QMP_OUR_LINK), rd(qmp_aoss + QMP_OUR_LINK_ACK),
	    rd(qmp_aoss + QMP_OUR_CHANNEL), rd(qmp_aoss + QMP_OUR_CHANNEL_ACK),
	    rd(qmp_aoss + QMP_THEIR_CHANNEL));
	if (rd(qmp_aoss + QMP_OUR_LINK) == QMP_UP && rd(qmp_aoss + QMP_OUR_LINK_ACK) == QMP_UP &&
	    rd(qmp_aoss + QMP_OUR_CHANNEL) == QMP_UP && rd(qmp_aoss + QMP_THEIR_CHANNEL) == QMP_UP &&
	    rd(qmp_aoss + QMP_OUR_CHANNEL_ACK) == QMP_UP) {
		printf("pcie: AOP mailbox is open already\n");
		qmp_box = box;
		return true;
	}
	wr(qmp_aoss + QMP_THEIR_LINK_ACK, rd(qmp_aoss + QMP_THEIR_LINK));
	wr(qmp_aoss + QMP_OUR_LINK, QMP_UP);
	qmp_ring();
	if (!qmp_wait(QMP_OUR_LINK_ACK, QMP_UP, "our link ack")) {
		printf("pcie: AOP does not ack our link; taking ours down\n");
		wr(qmp_aoss + QMP_OUR_LINK, QMP_DOWN);
		qmp_ring();
		return false;
	}
	wr(qmp_aoss + QMP_OUR_CHANNEL, QMP_UP);
	qmp_ring();
	if (!qmp_wait(QMP_THEIR_CHANNEL, QMP_UP, "their channel")) {
		printf("pcie: AOP does not open its channel; taking ours down\n");
		wr(qmp_aoss + QMP_OUR_CHANNEL, QMP_DOWN);
		wr(qmp_aoss + QMP_OUR_LINK, QMP_DOWN);
		qmp_ring();
		return false;
	}
	wr(qmp_aoss + QMP_THEIR_CHANNEL_ACK, QMP_UP);
	qmp_ring();
	if (!qmp_wait(QMP_OUR_CHANNEL_ACK, QMP_UP, "our channel ack")) {
		printf("pcie: AOP does not ack our channel; taking ours down\n");
		wr(qmp_aoss + QMP_OUR_CHANNEL, QMP_DOWN);
		wr(qmp_aoss + QMP_OUR_LINK, QMP_DOWN);
		qmp_ring();
		return false;
	}
	printf("pcie: AOP mailbox is open\n");
	qmp_box = box;
	return true;
}

/* Linux qmp_send: text in, length word, doorbell, wait for taken. Lock held. */
static bool
qmp_send(const char *text)
{
	uint32_t words[QMP_MESSAGE / 4] = { 0 };

	strlcpy((char *)words, text, sizeof(words));
	printf("pcie: telling AOP \"%s\"\n", text);
	if (rd(qmp_aoss + qmp_box) != 0) {
		printf("pcie: AOP mailbox still holds %u bytes: not sent\n", rd(qmp_aoss + qmp_box));
		return false;
	}
	for (unsigned i = 0; i < QMP_MESSAGE / 4; i++)
		wr(qmp_aoss + qmp_box + 4 + 4 * i, words[i]);
	wr(qmp_aoss + qmp_box, QMP_MESSAGE);
	(void)rd(qmp_aoss + qmp_box);
	qmp_ring();
	if (!qmp_wait(qmp_box, 0, "message taken")) {
		printf("pcie: AOP does not take the message; taken back\n");
		wr(qmp_aoss + qmp_box, 0);
		return false;
	}
	printf("pcie: AOP has taken it\n");
	return true;
}

/* The WLAN PDC upvals (dt qcom,pdc_init_table of qcom,cnss-kiwi). False: the
 * RF island is not sequenced; PERST must not go out into a dark endpoint. */
static bool
pcie_rf_power(void)
{
	static const char *const votes[] = {
		"{class: wlan_pdc, ss: rf, res: s3c.v, upval: 916}",
		"{class: wlan_pdc, ss: rf, res: s1c.v, upval: 1316}",
		"{class: wlan_pdc, ss: rf, res: s6c.v, upval: 1864}",
		"{class: wlan_pdc, ss: bb, res: s2c.v, upval: 976}",
		"{class: wlan_pdc, ss: bb, res: s4i.v, upval: 940}",
	};
	bool ok = false;

	lck_mtx_lock(&hakios_aop_mailbox_lock);
	if (!qmp_open())
		goto out;
	ok = true;
	for (unsigned i = 0; i < sizeof(votes) / sizeof(votes[0]); i++) {
		if (!qmp_send(votes[i])) {
			ok = false;
			break;
		}
	}
	hakios_tl("tl: pcie pdc %s\n", ok ? "voted" : "FAILED");
out:
	lck_mtx_unlock(&hakios_aop_mailbox_lock);
	return ok;
}

/* ---- the probe thread ---------------------------------------------------------------- */

static void
pcie_thread(__unused void *parameter, __unused wait_result_t result)
{
	static const unsigned pins[] = { PIN_WLAN_EN, PIN_PERST, PIN_WAKE };
	uint32_t steps = 99;

	PE_parse_boot_argn("pcie_steps", &steps, sizeof(steps));
	hakios_tl("tl: pcie probe thread running\n");
	printf("pcie: probe thread running (pcie_steps=%u)\n", steps);

	/* step 1: map PARF/PHY/dm_core/ELBI/pcie_sm, read link state (read-only) */
	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
		if (pins[i] < sizeof(pin_map) / sizeof(pin_map[0]))
			pin_map[pins[i]] = ml_io_map(PIN_PAGE(pins[i]), 0x1000);
	parf = ml_io_map(PCIE_PARF, PCIE_PARF_SIZE);
	phy = ml_io_map(PCIE_PHY, PCIE_PHY_SIZE);
	dmcore = ml_io_map(PCIE_DM_CORE, PCIE_DM_CORE_SIZE);
	elbi = ml_io_map(PCIE_ELBI, PCIE_ELBI_SIZE);
	pcie_sm = ml_io_map(PCIE_SM, PCIE_SM_SIZE);
	printf("pcie: step 1: pins 16/94/96 %s, PARF %p PHY %p DM %p ELBI %p SM %p\n",
	    pin_map[16] && pin_map[94] && pin_map[96] ? "mapped" : "MAP FAILED",
	    parf, phy, dmcore, elbi, pcie_sm);
	hakios_tl("tl: pcie pins=%d parf=%p phy=%p dm=%p elbi=%p sm=%p\n",
	    pin_map[16] && pin_map[94] && pin_map[96], parf, phy, dmcore, elbi, pcie_sm);
	if (!parf || !phy || !dmcore || !elbi || !pcie_sm) {
		printf("pcie: windows failed to map: no PCIe\n");
		goto out;
	}
	printf("pcie: PARF sys_ctrl %#x, LTSSM %#x, PHY status %#x (bit6 %s)\n",
	    rd(parf + PARF_VERSION), rd(parf + PARF_LTSSM), rd(parf + PARF_PHY_STATUS),
	    (rd(parf + PARF_PHY_STATUS) & PARF_PHY_STATUS_BIT) ? "on" : "OFF");
	printf("pcie: PARF as found: device_type %#x phy_ctrl %#x dbi %#x slv %#x wr_halt %#x int_mask %#x\n",
	    rd(parf + PARF_DEVICE_TYPE), rd(parf + PARF_PHY_CTRL), rd(parf + PARF_DBI_BASE_ADDR),
	    rd(parf + PARF_SLV_ADDR_SPACE_SIZE), rd(parf + PARF_AXI_MSTR_WR_ADDR_HALT),
	    rd(parf + PARF_INT_ALL_MASK));
	hakios_tl("tl: pcie parf-ver=%#x ltssm=%#x phy=%#x\n",
	    rd(parf + PARF_VERSION), rd(parf + PARF_LTSSM), rd(parf + PARF_PHY_STATUS));
	printf("pcie: pins as found: wlan_en cfg %#x/io %#x, perst cfg %#x/io %#x, wake cfg %#x/io %#x\n",
	    tlmm_rd(GPIO_CFG(16)), tlmm_rd(GPIO_IO(16)),
	    tlmm_rd(GPIO_CFG(94)), tlmm_rd(GPIO_IO(94)),
	    tlmm_rd(GPIO_CFG(96)), tlmm_rd(GPIO_IO(96)));
	hakios_tl("tl: pcie pins 16=%#x 94=%#x 96=%#x\n",
	    tlmm_rd(GPIO_CFG(16)), tlmm_rd(GPIO_CFG(94)), tlmm_rd(GPIO_CFG(96)));
	if (steps < 2)
		goto out;

	/* step 2: power the host side (port GDSCs, votes, resets); endpoint rails come later */
	gcc = ml_io_map(GCC_BASE, 0x80000);
	printf("pcie: step 2: gcc %p, gdsc pcie_0 %#x phy %#x\n", gcc,
	    gcc ? gcc_rd(GDSC_PCIE_0) : 0, gcc ? gcc_rd(GDSC_PCIE_0_PHY) : 0);
	hakios_tl("tl: pcie gcc=%p gdsc=%#x/%#x\n", gcc,
	    gcc ? gcc_rd(GDSC_PCIE_0) : 0, gcc ? gcc_rd(GDSC_PCIE_0_PHY) : 0);
	if (!gcc)
		goto out;
	/* PERST low first: the endpoint stays in reset while the host comes up (pci-msm
	 * asserts PERST across the bring-up and lets it out only for training). */
	tlmm_wr(GPIO_IO(PIN_PERST), 0);
	tlmm_wr(GPIO_CFG(PIN_PERST), GPIO_PERST_CFG);
	printf("pcie: step 2: PERST asserted (io %#x cfg %#x)\n",
	    tlmm_rd(GPIO_IO(PIN_PERST)), tlmm_rd(GPIO_CFG(PIN_PERST)));
	/* the port GDSC (dt qcom,gdsc gcc_pcie_0_gdsc) and the PHY GDSC (dt gdsc-phy-vdd-supply):
	 * collapse clear and power-on poll, the GPU CX GDSC's way */
	if (!gdsc_on(GDSC_PCIE_0, "pcie_0") || !gdsc_on(GDSC_PCIE_0_PHY, "pcie_0_phy"))
		goto out;
	/* CESTA state manager first (upstream loads its sequence at probe, before
	 * anything else; writing only after power is the safe equivalent here) */
	cesta_load();
	/* votes: every pcie_0 branch (gcc-pineapple.c BITs 0-6) and resets pulsed
	 * (upstream msm_pcie_core_phy_reset asserts, waits 1 ms, deasserts; dt
	 * reset-names are gcc 3 and 6 = BCR 0x6b000 and 0x6c01c, verified). */
	gcc_wr(GCC_PCIE_VOTE, gcc_rd(GCC_PCIE_VOTE) | GCC_PCIE_VOTE_ALL);
	gcc_wr(GCC_PCIE_0_BCR, gcc_rd(GCC_PCIE_0_BCR) | 1u);
	gcc_wr(GCC_PCIE_0_PHY_BCR, gcc_rd(GCC_PCIE_0_PHY_BCR) | 1u);
	sleep_ms(1);
	gcc_wr(GCC_PCIE_0_BCR, gcc_rd(GCC_PCIE_0_BCR) & ~1u);
	gcc_wr(GCC_PCIE_0_PHY_BCR, gcc_rd(GCC_PCIE_0_PHY_BCR) & ~1u);
	sleep_ms(5);
	printf("pcie: after power: votes %#x bcr %#x/%#x sys_ctrl %#x\n",
	    gcc_rd(GCC_PCIE_VOTE),
	    gcc_rd(GCC_PCIE_0_BCR), gcc_rd(GCC_PCIE_0_PHY_BCR),
	    rd(parf + PARF_VERSION));
	hakios_tl("tl: pcie powered votes=%#x bcr=%#x/%#x\n",
	    gcc_rd(GCC_PCIE_VOTE), gcc_rd(GCC_PCIE_0_BCR), gcc_rd(GCC_PCIE_0_PHY_BCR));
	if ((gcc_rd(GCC_PCIE_0_BCR) | gcc_rd(GCC_PCIE_0_PHY_BCR)) & 1u) {
		printf("pcie: resets stuck asserted: not training into that\n");
		goto out;
	}
	/* branch clocks are vote-only (upstream enable_reg = 0x52008 for all of them;
	 * the halt regs are read-only status: never write them). The votes above are the
	 * enable; report the halt bits for diagnosis and go on regardless. */
	printf("pcie: halt slv_q2a %#x slv_axi %#x mstr_axi %#x cfg_ahb %#x aux %#x pipe %#x cnoc %#x\n",
	    gcc_rd(GCC_PCIE_0_SLV_Q2A_CBCR), gcc_rd(GCC_PCIE_0_SLV_AXI_CBCR),
	    gcc_rd(GCC_PCIE_0_MSTR_AXI_CBCR), gcc_rd(GCC_PCIE_0_CFG_AHB_CBCR),
	    gcc_rd(GCC_PCIE_0_AUX_CBCR), gcc_rd(GCC_PCIE_0_PIPE_CBCR),
	    gcc_rd(GCC_CNOC_PCIE_SF_AXI_CBCR));
	/* The DT-listed port clock (cnoc sf axi) gets a direct enable too: harmless if the
	 * vote already did it, informative if hyp gates it (write takes but OFF stays). */
	gcc_wr(GCC_CNOC_PCIE_SF_AXI_CBCR, gcc_rd(GCC_CNOC_PCIE_SF_AXI_CBCR) | 1u);
	printf("pcie: cnoc CBCR after direct enable %#x\n", gcc_rd(GCC_CNOC_PCIE_SF_AXI_CBCR));
	/* PARF for training (upstream msm_pcie_enable_link; l1 PM left alone:
	 * the DT has no l1-supported for this port) */
	wr(parf + PARF_DEVICE_TYPE, PARF_DEVICE_TYPE_RC);
	wr(parf + PARF_PHY_CTRL, rd(parf + PARF_PHY_CTRL) & ~1u);
	wr(parf + PARF_DBI_BASE_ADDR, 0);
	wr(parf + PARF_SYS_CTRL, PARF_SYS_CTRL_RC);
	wr(parf + PARF_MHI_CLOCK_RESET_CTRL, rd(parf + PARF_MHI_CLOCK_RESET_CTRL) & ~(1u << 4));
	wr(parf + PARF_INT_ALL_MASK, 0);
	wr(parf + PARF_SLV_ADDR_SPACE_SIZE, PARF_SLV_ADDR_SIZE);
	wr(parf + PARF_AXI_MSTR_WR_ADDR_HALT, rd(parf + PARF_AXI_MSTR_WR_ADDR_HALT) | (1u << 31));
	wr(parf + PARF_CFG_BITS_3, rd(parf + PARF_CFG_BITS_3) & ~1u);
	wr(parf + PARF_CFG_BITS_3, rd(parf + PARF_CFG_BITS_3) | (1u << 8));
	wr(dmcore + DM_LANE_SKEW_OFF, rd(dmcore + DM_LANE_SKEW_OFF) & ~(1u << 5));
	printf("pcie: PARF programmed: device_type %#x phy_ctrl %#x dbi %#x sys_ctrl %#x slv %#x cfg3 %#x lane_skew %#x\n",
	    rd(parf + PARF_DEVICE_TYPE), rd(parf + PARF_PHY_CTRL), rd(parf + PARF_DBI_BASE_ADDR),
	    rd(parf + PARF_SYS_CTRL), rd(parf + PARF_SLV_ADDR_SPACE_SIZE),
	    rd(parf + PARF_CFG_BITS_3), rd(dmcore + DM_LANE_SKEW_OFF));
	/* the PHY (dt qcom,phy-sequence), SerDes ready poll, then start training
	 * (pci-msm sets LTSSM_EN; read-modify-write like its write_mask) */
	phy_init();
	sleep_ms(PHY_STAB_MS);
	if (!phy_serdes_ready())
		goto out;
	wr(parf + PARF_LTSSM, rd(parf + PARF_LTSSM) | PARF_LTSSM_EN);
	printf("pcie: LTSSM enabled, %#x\n", rd(parf + PARF_LTSSM));
	if (steps < 3)
		goto out;

	/* step 3: CNSS rails through RPMh, wlan_en high, PERST out, poll the link to L0 */
	pcie_rsc = ml_io_map(SYN_RSC_PHYS, SYN_RSC_SIZE);
	pcie_db = ml_io_map_wcomb(SYN_CMD_DB_PHYS, SYN_CMD_DB_SIZE);
	hakios_tl("tl: pcie rsc=%p db=%p\n", pcie_rsc, pcie_db);
	if (pcie_rsc && pcie_db) {
		static const struct { const char *name; uint32_t uv; } rails[] = {
			{ "smpi4", 940000 }, { "smpc2", 976000 }, { "smpc3", 916000 },
			{ "smpc6", 1864000 }, { "smpc1", 1316000 },
			{ "ldob15", 1800000 }, { "ldoc3", 1200000 }, { "ldok6", 1800000 },
		};
		unsigned ok = 0;

		for (unsigned i = 0; i < sizeof(rails) / sizeof(rails[0]); i++)
			ok += pcie_vote_rail(rails[i].name, rails[i].uv);
		hakios_tl("tl: pcie rails %u/8 on\n", ok);
	}
	/* wlan_en high (dt cnss_wlan_en_active), AOP PDC votes for the RF island,
	 * PERST out, poll the link to L0 */
	tlmm_wr(GPIO_IO(PIN_WLAN_EN), 2);
	tlmm_wr(GPIO_CFG(PIN_WLAN_EN), GPIO_WLAN_EN_CFG);
	printf("pcie: wlan_en high (io %#x cfg %#x)\n",
	    tlmm_rd(GPIO_IO(PIN_WLAN_EN)), tlmm_rd(GPIO_CFG(PIN_WLAN_EN)));
	/* the RF island is sequenced by the AOP, not by voltages alone; without
	 * its votes PERST would go out into a dark endpoint */
	if (!pcie_rf_power())
		goto out;
	/* power stable before PERST goes out (CEM T_PVPERL 100 ms; the old 10 ms is short) */
	sleep_ms(PERST_LOW_MS);
	tlmm_wr(GPIO_IO(PIN_PERST), 2);
	printf("pcie: PERST released (io %#x cfg %#x)\n",
	    tlmm_rd(GPIO_IO(PIN_PERST)), tlmm_rd(GPIO_CFG(PIN_PERST)));
	sleep_ms(PERST_SETTLE_MS);
	/* enable training again now the endpoint is out of reset, then poll what
	 * upstream polls: ELBI link-up (bit10) and DWC DLL active (dm_core 0x80
	 * bit29). Upstream polls 20x5 ms; 100x10 ms here is only more patient. */
	wr(parf + PARF_LTSSM, rd(parf + PARF_LTSSM) | PARF_LTSSM_EN);
	for (unsigned t = 0; t < 100; t++) {
		sleep_ms(10);
		if ((rd(elbi + ELBI_SYS_STTS) & ELBI_LINK_UP) &&
		    (rd(dmcore + DM_CAP_LINKCTRLSTATUS) & DM_DLL_ACTIVE))
			break;
	}
	hakios_tl("tl: pcie after PERST ltssm=%#x phy=%#x elbi=%#x dll=%#x\n",
	    rd(parf + PARF_LTSSM), rd(parf + PARF_PHY_STATUS),
	    rd(elbi + ELBI_SYS_STTS), rd(dmcore + DM_CAP_LINKCTRLSTATUS));
	printf("pcie: step 3 done: LTSSM %#x PHY %#x ELBI %#x DLL %#x\n",
	    rd(parf + PARF_LTSSM), rd(parf + PARF_PHY_STATUS),
	    rd(elbi + ELBI_SYS_STTS), rd(dmcore + DM_CAP_LINKCTRLSTATUS));
	if (!(rd(elbi + ELBI_SYS_STTS) & ELBI_LINK_UP) ||
	    !(rd(dmcore + DM_CAP_LINKCTRLSTATUS) & DM_DLL_ACTIVE)) {
		/* read-only probes for the next suspect, one line each: a hung
		 * read wedges the bus, so each value is printed before the next
		 * read runs. Proven-readable first (0x80), pcie_sm last (its
		 * zeros mean the SM FSM was never kicked: no CESTA D0 runs
		 * here; upstream runs it before the resets, so a PHY that is
		 * ready but unreachable is the CESTA case). */
		printf("pcie: link is down: dll %#x (want bit29)\n",
		    rd(dmcore + DM_CAP_LINKCTRLSTATUS));
		printf("pcie: link is down: lnkcap %#x\n", rd(dmcore + DM_LNKCAP));
		printf("pcie: link is down: lnksta %#x\n", (rd(dmcore + DM_CAP_LINKCTRLSTATUS) >> DM_LNKSTA_SHIFT) & 0xffff);
		printf("pcie: link is down: aux_freq %#x (want 0x14)\n",
		    rd(dmcore + DM_AUX_CLK_FREQ));
		printf("pcie: link is down: sm_dbg0 %#x\n", rd(pcie_sm + SM_DBG0));
		printf("pcie: link is down: sm_dbg1/start %#x/%#x\n",
		    rd(pcie_sm + SM_DBG1), rd(pcie_sm + SM_START));
		printf("pcie: no ECAM (the Wi-Fi gate wants LTSSM %#x + PHY bit6)\n", PARF_LTSSM_UP);
		goto out;
	}
	if (steps < 4)
		goto out;

	/* step 4: ECAM vendor/device (link-gated: a configuration read with the endpoint
	 * absent aborts this SoC; the Wi-Fi driver's step_cnss_probe keeps the same gate) */
	{
	vm_offset_t conf = ml_io_map(PCIE_CONF, PCIE_CONF_SIZE);
	uint32_t ids;

	printf("pcie: step 4: conf %p\n", conf);
	if (!conf)
		goto out;
	ids = rd(conf + ECAM_VENDOR_DEVICE);
	printf("pcie: ECAM vendor/device %#x (vendor %#x device %#x), class %#x BAR0 %#x/%#x\n",
	    ids, ids & 0xffff, ids >> 16, rd(conf + ECAM_CLASS) >> 8,
	    rd(conf + ECAM_BAR0_LO), rd(conf + ECAM_BAR0_HI));
	hakios_tl("tl: pcie ecam=%#x bar0=%#x/%#x\n",
	    ids, rd(conf + ECAM_BAR0_LO), rd(conf + ECAM_BAR0_HI));
	if ((ids & 0xffff) != PCI_VENDOR_QCOM || ids >> 16 != PCI_DEVICE_WCN7850)
		printf("pcie: that is not a WCN7850 (want 0x17cb/0x1107)\n");
	else
		printf("pcie: THE ENDPOINT IS A WCN7850 (0x17cb/0x1107)\n");
	}
out:
	thread_terminate(current_thread());
}

void
hakios_pcie_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("pcie", &enabled, sizeof(enabled)) || !enabled)
		return;
	if (kernel_thread_start(pcie_thread, NULL, &thread) == KERN_SUCCESS)
		thread_deallocate(thread);
}
