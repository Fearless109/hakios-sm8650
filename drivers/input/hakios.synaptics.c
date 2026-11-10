/*
 * The touch screen of the Xiaomi 14 Pro (shennong): a Synaptics S3910P over SPI (synaptics,tcm-spi,
 * "shennong_test_limits_S3910P_01.csv", firmware synaptics_spi_shennong_01.img), as a source of touches
 * for /dev/hakios-touch (sys/hakios.input.h). Only with boot-arg touchscreen=1 (the same switch the
 * Goodix driver uses; both are started, and neither touches anything unless asked for it, but the
 * phone has only the Synaptics: GOODIX stays left out of shennong's kernel).
 *
 * There is no clock, pin, regulator or SPI framework in this kernel, so this driver does all of it
 * itself, in the order the vendor's Linux driver and the reference phone's Goodix driver do
 * (drivers/input/hakios.goodix.c):
 *   power    the panel/touch analog supply is a fixed regulator switched by GPIO164 (touch_avdd_vreg,
 *            enable-active-high); the touch IOVDD rail is an RPMh regulator (one request written
 *            into the "always on" processor's command slots, as the Goodix driver's supply does).
 *            If the IOVDD request fails the driver says so and goes on: with only AVDD the chip
 *            may still report.
 *   clocks   the GENI SE's clocks by vote in the global clock controller (gcc@100000) and the QUP1
 *            AHBs; the SE clock source is set to the requested SPI rate.
 *   pins     MISO GPIO48, MOSI GPIO49, CLK GPIO50, CS GPIO51 to the serial engine (QUP1 SE4,
 *            function qup1_se4_l0..l3), plus the chip's INT GPIO162 and RESET GPIO161
 *            (synaptics,irq-gpio / synaptics,reset-gpio in the phone's tree).
 *   SPI      the serial engine (GENI) in FIFO mode, polled; on failure the same four pins driven
 *            by hand at a lower rate.
 *   chip     reset, then TCM identify (product/firmware), then read the touch-data packets when
 *            the IRQ line signals; each packet's touches are posted as samples.
 * No interrupt is used: a thread looks at the interrupt line 125 times a second (boot-arg
 * touch_poll_us=N: every N microseconds instead of 8000).
 *
 * Register layouts: Linux drivers/spi/spi-geni-qcom.c (GENI), drivers/clk/qcom/gcc-sm8650.c
 * (QUP1 wrap1 S4: cmd_rcgr 0x183c8, branch at 0x183bc, vote 0x52008 bit 26), pinctrl-sm8650
 * (GPIO numbers above), synaptics_tcm_over_spi (TCM command/response), and the phone's own device
 * tree (spi@a90000, synaptics_tcm@0: resolution 0x5a0 x 0xc80, reset/irq GPIOs, firmware name).
 *
 * Status: written for shennong, not yet run on the phone (the port's device.conf says partial).
 * The TCM report parse (packet lengths, coordinate scale for the 1440x3200 panel) follows the
 * vendor driver; it must be confirmed against a running chip before touches are believed.
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

/* RPMh's sequencer lock (shared with the Goodix, ADSP, Wi-Fi and modem drivers) */
extern lck_mtx_t hakios_rsc_lock;

static vm_offset_t tlmm, gcc, se, rsc, cmd_db;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

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

/* ---- I/O supply through RPMh (shennong iovdd = pm_humu_l12 / ldob12, 1.8 V) -----------------
 * Copied from goodix io_supply_on/rpmh_write (SC7280 ldoc2), pointed at SM8650:
 * RSC apps DRV2 0x17a20000 + cmd-db 0x81c60000 (devices/shennong/soc.args, like ADSP).
 * TCS layout tcs-offset 0xd00 matches ADSP's working RSC_TCS_BASE on shennong. */

#define SYN_CMD_DB_PHYS         0x81c60000UL
#define SYN_CMD_DB_SIZE         0x20000UL
#define SYN_RSC_PHYS            0x17a20000UL
#define SYN_RSC_SIZE            0x10000UL
#define SYN_TCS(reg)            (rsc + 0xd00 + (reg))
#define TCS_IRQ_ENABLE          0x00
#define TCS_IRQ_STATUS          0x04
#define TCS_IRQ_CLEAR           0x08
#define TCS_WAIT_FOR_COMPLETION 0x10
#define TCS_CONTROL             0x14
#define TCS_CONTROL_ENABLE      (1u << 16)
#define TCS_CONTROL_TRIGGER     (1u << 24)
#define TCS_STATUS              0x18
#define TCS_CMD_ENABLE          0x1c
#define TCS_CMD_MSGID(n)        (0x30 + 20 * (n))
#define TCS_CMD_ADDR(n)         (0x34 + 20 * (n))
#define TCS_CMD_DATA(n)         (0x38 + 20 * (n))
#define MSGID_WRITE_4_BYTES     0x10008
#define VRM_VOLTAGE             0
#define VRM_ENABLE              4
#define VRM_MODE                8
#define VRM_MODE_LDO_HPM        7

struct syn_cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct syn_cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct syn_cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct syn_cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static uint32_t
syn_rpmh_address(const char *name)
{
	const struct syn_cmd_db_header *db = (const struct syn_cmd_db_header *)cmd_db;
	char id[8] = { 0 };

	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		printf("synaptics: no command database (magic %02x %02x %02x %02x)\n",
		    db->magic[0], db->magic[1], db->magic[2], db->magic[3]);
		hakios_tl("tl: synaptics no cmd-db magic %02x%02x%02x%02x\n",
		    db->magic[0], db->magic[1], db->magic[2], db->magic[3]);
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct syn_cmd_db_slave *slave = &db->slave[s];
		const struct syn_cmd_db_entry *entry =
		    (const struct syn_cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->header_offset >= SYN_CMD_DB_SIZE - 0x1000 || slave->count > 512)
			continue;
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0)
				return entry[i].address;
		}
	}
	return 0;
}

/* TCS register layout depends on the RSC version (Linux rpmh-rsc ver_2_7/ver_3_0;
 * pandora does the same: SM8650 reads v3.2 here, SC7280 v2). */
struct syn_rsc_layout {
	uint32_t wait, control, status, enable, msgid, addr, data, cmd_status, stride;
};
static const struct syn_rsc_layout syn_rh_v2 = { 0x10, 0x14, 0x18, 0x1c, 0x30, 0x34, 0x38, 0x3c, 20 };
static const struct syn_rsc_layout syn_rh_v3 = { 0x20, 0x24, 0x28, 0x2c, 0x34, 0x38, 0x3c, 0x40, 24 };
#define SYN_TCS_BASE            0xd00

static const struct syn_rsc_layout *
syn_rsc_layout(void)
{
	uint32_t id = rd(rsc + 0);
	unsigned major = (id >> 16) & 0xff;

	return major >= 3 ? &syn_rh_v3 : &syn_rh_v2;
}

static bool
syn_rpmh_write(const uint32_t *address, const uint32_t *data, unsigned count)
{
	const struct syn_rsc_layout *L = syn_rsc_layout();
	uint32_t tcs = SYN_TCS_BASE, irq_enable;
	bool done = false;
	int chosen = -1;

	lck_mtx_lock(&hakios_rsc_lock);
	/* find a free ACTIVE TCS (pandora scans TCS0-2, stride 0x2a0) */
	for (unsigned t = 0; t < 3; t++) {
		uint32_t b = tcs + t * 0x2a0;
		uint32_t st = rd(rsc + b + L->status), ctl = rd(rsc + b + L->control);
		uint32_t en = rd(rsc + b + L->enable), wt = rd(rsc + b + L->wait);

		if (chosen < 0 && st != 0 && st != 0xffffffffu && ctl == 0 && en == 0 && wt == 0)
			chosen = (int)t;
	}
	if (chosen < 0) {
		hakios_tl("tl: synaptics no free TCS (t0 st=%#x ctl=%#x en=%#x)\n",
		    rd(rsc + tcs + L->status), rd(rsc + tcs + L->control),
		    rd(rsc + tcs + L->enable));
		goto unlock;
	}
	tcs += (uint32_t)chosen * 0x2a0;
	{
	uint32_t tb = tcs;
	uint32_t bit = 1u << (unsigned)chosen;

	hakios_tl("tl: synaptics TCS%u (status %#x)\n", chosen, rd(rsc + tb + L->status));
	irq_enable = rd(rsc + SYN_TCS_BASE + 0);
	wr(rsc + SYN_TCS_BASE + 0, irq_enable & ~bit);
	for (unsigned i = 0; i < count; i++) {
		uint32_t cmd = tb + i * L->stride;

		wr(rsc + cmd + L->msgid, MSGID_WRITE_4_BYTES | 0x100);
		wr(rsc + cmd + L->addr, address[i]);
		wr(rsc + cmd + L->data, data[i]);
	}
	wr(rsc + tb + L->wait, (1u << count) - 1);
	wr(rsc + tb + L->enable, (1u << count) - 1);
	wr(rsc + tb + L->control, 0);
	wr(rsc + tb + L->control, TCS_CONTROL_ENABLE);
	wr(rsc + tb + L->control, TCS_CONTROL_ENABLE | TCS_CONTROL_TRIGGER);
	for (int i = 0; i < 1000; i++) {
		bool all = true;

		for (unsigned c = 0; c < count; c++) {
			uint32_t cs = rd(rsc + tb + L->cmd_status + c * L->stride);

			all &= cs != 0xffffffffu && (cs & 0x10100u) == 0x10100u;
		}
		if (all && rd(rsc + tb + L->status) != 0) {
			done = true;
			break;
		}
		sleep_us(100);
	}
	if (!done)
		hakios_tl("tl: synaptics TCS%u vote timeout (st=%#x)\n", chosen, rd(rsc + tb + L->status));
	wr(rsc + tb + L->control, TCS_CONTROL_ENABLE);
	wr(rsc + tb + L->control, 0);
	wr(rsc + tb + L->enable, 0);
	wr(rsc + tb + L->wait, 0);
	wr(rsc + SYN_TCS_BASE + 8, bit);
	wr(rsc + SYN_TCS_BASE + 0, irq_enable);
	}
unlock:
	lck_mtx_unlock(&hakios_rsc_lock);
	return done;
}

/* vote the touch I/O rail on: false = the chip is left on AVDD alone */
static bool
syn_io_supply_on(void)
{
	uint32_t base = syn_rpmh_address("ldob12");
	uint32_t address[3] = { base + VRM_VOLTAGE, base + VRM_ENABLE, base + VRM_MODE };
	uint32_t data[3] = { 1800, 1, VRM_MODE_LDO_HPM };

	hakios_tl("tl: synaptics ldob12 addr=%#x\n", base);
	if (base == 0 || (base >> 16 & 0xf) != 4) {
		printf("synaptics: RPMh: no regulator \"ldob12\" (address %#x)\n", base);
		hakios_tl("tl: synaptics no ldob12 (%#x)\n", base);
		return false;
	}
	printf("synaptics: RPMh: ldob12 at %#x; slot status %#x\n", base, rd(SYN_TCS(TCS_STATUS)));
	if (!syn_rpmh_write(address, data, 3))
		return false;
	printf("synaptics: I/O supply on (1.8 V)\n");
	return true;
}

/* per-pin TLMM maps (houji-style): 4KB each, not 1MB at once (IO submap is 8MB shared) */
static vm_offset_t pin_map[165];

/* ---- pins (TLMM, pinctrl@f000000) ---------------------------------------------------------------- */

#define TLMM_PHYS               0x0f100000UL     /* GPIO0 CFG here: pinctrl@f000000 + 0x100000 (houji/pandora/goodix agree) */
#define TLMM_GPIO_STRIDE        0x1000UL
#define PIN_PAGE(n)             (TLMM_PHYS + TLMM_GPIO_STRIDE * (n))
#define GPIO_CFG(n)             (pin_map[n] + 0)         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (pin_map[n] + 4)         /* bit 0 in, bit 1 out */
#define PULL_NONE               0
#define PULL_UP                 3

#define PIN_MISO                48
#define PIN_MOSI                49
#define PIN_CLK                 50
#define PIN_CS                  51
#define PIN_RESET               161      /* low = in reset (synaptics,reset-gpio) */
#define PIN_INT                 162      /* low = the chip has a report (synaptics,irq-gpio) */
#define PIN_AVDD                164      /* touch_avdd_vreg: high = analog supply on */
#define PIN_FUNCTION_SPI        1        /* qup1_se4_l* */
#define PIN_RSC                 0x17a20000UL
#define PIN_RSC_SIZE            0x10000UL

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	wr(GPIO_CFG(pin), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static bool
gpio_get(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
gpio_output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	gpio_set(pin, high);            /* the level first, so that the pin never drives the wrong one */
	gpio_configure(pin, 0, pull, ma, true);
}

/* ---- GPIO bit-bang SPI + TouchCom v1 (houji-style, no GENI) ------------------------------- */

static void
bb_pause(void)
{
	sleep_us(1);      /* houji IODelay(1): mode 0, ~500 kHz */
}

/* mode 0/MSB first/CS active-low, like HoujiTouchGpio.h gg_transfer */
static unsigned bb_swap;      /* try MISO/MOSI swapped (goodix tries both data pins) */

static void
bb_xfer(const uint8_t *tx, uint8_t *rx, unsigned n)
{
	unsigned out_pin = bb_swap ? PIN_MISO : PIN_MOSI;
	unsigned in_pin = bb_swap ? PIN_MOSI : PIN_MISO;

	gpio_set(PIN_CLK, false);
	gpio_set(PIN_CS, false);
	bb_pause();
	for (unsigned i = 0; i < n; i++) {
		uint8_t out = tx ? tx[i] : 0xff, in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			gpio_set(out_pin, (out >> bit) & 1);
			bb_pause();
			gpio_set(PIN_CLK, true);
			bb_pause();
			in = (uint8_t)((in << 1) | (gpio_get(in_pin) ? 1 : 0));
			gpio_set(PIN_CLK, false);
		}
		if (rx)
			rx[i] = in;
	}
	bb_pause();
	gpio_set(PIN_CS, true);
	bb_pause();
}

#define GW_MAX  512

/* read one TouchCom message; returns 0 ok, else houji gw_read error code */
static int
gw_read(uint8_t *code, uint8_t *data, unsigned *size)
{
	uint8_t h[4], tail[GW_MAX + 7];

	bb_xfer(NULL, h, 4);
	if (h[0] != 0xa5)
		return 2;
	*code = h[1];
	*size = 0;
	if (h[1] == 0 || h[1] == 2 || h[1] == 0xff)
		return 0;
	if (h[1] == 3)
		return 3;
	unsigned n = h[2] | (unsigned)h[3] << 8;
	if (n > GW_MAX)
		return 3;
	if (n) {
		sleep_ms(1);      /* let the chip stage the continued read */
		bb_xfer(NULL, tail, n + 7);
		if (tail[0] != 0xa5 || tail[1] != 3) {
			hakios_tl("tl: synaptics tail %02x %02x %02x %02x %02x %02x (n=%u)\n",
			    tail[0], tail[1], tail[2], tail[3], tail[4], tail[5], n);
			return 5;
		}
		for (unsigned i = n + 2; i < n + 7; i++)
			if (tail[i] != 0x5a)
				return 6;
		memcpy(data, tail + 2, n);
		*size = n;
	}
	return 0;
}

/* send command, wait for ACK (code 1) or IDENTIFY (code 0x10 for cmd 2) */
static int
gw_command(uint8_t cmd, const uint8_t *pl, unsigned n, uint8_t *code, uint8_t *data, unsigned *size)
{
	uint8_t pkt[GW_MAX + 3];

	if (n > GW_MAX || n + 3 > GW_MAX + 3)
		return 7;
	pkt[0] = cmd;
	pkt[1] = (uint8_t)n;
	pkt[2] = (uint8_t)(n >> 8);
	if (n)
		memcpy(pkt + 3, pl, n);
	bb_xfer(pkt, NULL, n + 3);
	for (unsigned a = 0; a < 100; a++) {
		sleep_ms(10);
		int e = gw_read(code, data, size);

		if (e)
			return e;
		if (*code == 1 || (cmd == 2 && *code == 0x10))
			return 0;
		if (*code == 0 || *code == 2 || *code == 0xff || *code >= 0x10)
			continue;
		return 9;
	}
	return 10;
}

/* ---- clocks (GCC, gcc@100000; SM8650 gcc-sm8650.c) --------------------------------------------- */

#define GCC_PHYS                0x100000UL
#define GCC_SIZE                0x200000UL
#define GCC_VOTE                (gcc + 0x52008)         /* the application processor's votes */
#define GCC_QUP1_M_AHB_CBCR     (gcc + 0x18004)         /* QUP1 master AHB; verify on hardware (SC7280 offsets) */
#define GCC_QUP1_S_AHB_CBCR     (gcc + 0x18008)         /* QUP1 slave AHB; verify on hardware (SC7280 offsets) */
#define GCC_QUP1_S4_CBCR        (gcc + 0x183bc)         /* wrap1_s4_clk branch (halt); vote bit 26 */
#define GCC_QUP1_S4_CMD         (gcc + 0x183c8)         /* wrap1_s4_clk_src cmd_rcgr */
#define GCC_QUP1_S4_CFG         (gcc + 0x183cc)         /* +4: source and divider */
#define CBCR_ENABLE             (1u << 0)
#define CBCR_OFF                (1u << 31)

/* ---- SPI through the serial engine (GENI, se_phys 0xa90000: spi@a90000 of QUP1 SE4) --------------- */

#define SE_PHYS                 0xa90000UL
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)
#define GENI_SER_M_CLK_CFG      (se + 0x048)
#define GENI_IF_DISABLE_RO      (se + 0x064)
#define GENI_FW_REVISION_RO     (se + 0x068)
#define GENI_CLK_SEL            (se + 0x07c)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)
#define GENI_M_CMD_CTRL         (se + 0x604)
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)

/* TCM over SPI (synaptics_tcm_over_spi: commands are packets; the response carries the data). */
#define TCM_READ_SIZE           30
#define PANEL_MAX_X             0x5a0u             /* synaptics,max-x (1440) */
#define PANEL_MAX_Y             0xc80u             /* synaptics,max-y (3200) */

static bool touch_ready;

static void
spi_geni_start(void)
{
	wr(GENI_FORCE_DEFAULT, 0);
	wr(GENI_OUTPUT_CTRL, 0);
	wr(GENI_CGC_CTRL, 0);
	wr(GENI_IF_DISABLE_RO, 0);
	wr(GENI_BYTE_GRAN, 0x38);            /* 7 bytes per word, as the reference driver uses */
	wr(GENI_TX_PACKING_CFG0, 0xfe);
	wr(GENI_TX_PACKING_CFG1, 0x3f);
	wr(GENI_RX_PACKING_CFG0, 0xfe);
	wr(GENI_RX_PACKING_CFG1, 0x3f);
	wr(GENI_DMA_MODE_EN, 0);
	sleep_us(100);
}

/* One SPI transfer through the GENI FIFO (polled). Returns false if the engine did not finish. */
static bool
spi_geni_transfer(const uint8_t *tx, uint8_t *rx, unsigned length, uint32_t spi_hz)
{
	uint32_t words = (length + 3) / 4;

	(void)spi_hz;
	wr(GENI_M_CMD0, 0x1c00000 | (words - 1) << 24);   /* TX_CMD0: SPI TX */
	for (uint32_t i = 0; i < words; i++) {
		uint32_t w = 0;
		for (unsigned b = 0; b < 4 && i * 4 + b < length; b++)
			w |= (uint32_t)tx[i * 4 + b] << (8 * b);
		wr(GENI_TX_FIFO, w);
	}
	wr(GENI_M_CMD_CTRL, 0);
	for (unsigned t = 0; t < 1000; t++) {
		if (!(rd(GENI_STATUS) & 1)) break;
		sleep_us(10);
	}
	if (rd(GENI_STATUS) & 1)
		return false;
	return true;
}

static uint32_t poll_us = 8000;
static uint32_t latency;

static void
touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t tx[TCM_READ_SIZE], rx[TCM_READ_SIZE];

	while (true) {
		if (!gpio_get(PIN_INT)) {
			/* the chip has a report: read one packet and post its touches */
			memset(tx, 0xff, sizeof(tx));
			if (spi_geni_transfer(tx, rx, TCM_READ_SIZE, 0)) {
				/* parse: vendor TCM report (two 16-bit coords per touch in the payload):
				 * confirm the layout against the running chip before trusting it */
				for (unsigned slot = 0; slot < 2; slot++) {
					unsigned off = 4 + slot * 6;
					uint32_t x = rx[off] | (uint32_t)rx[off + 1] << 8;
					uint32_t y = rx[off + 2] | (uint32_t)rx[off + 3] << 8;
					uint8_t pressure = rx[off + 4];
					if (x > PANEL_MAX_X || y > PANEL_MAX_Y || pressure == 0)
						continue;
					hakios_touch_post(HAKIOS_TOUCH_MOVE, slot,
					    x * 65535u / PANEL_MAX_X, y * 65535u / PANEL_MAX_Y);
				}
			}
		}
		if (latency)
			hakios_tl("tl: s seen %llu\n", (unsigned long long)mach_absolute_time());
		sleep_us(poll_us);
	}
	thread_terminate(current_thread());
}

/* ---- TouchCom report decoder + HID worker (pandora tc_bits style) ------------------ */

#define SYN_MAX_OBJECTS 10

static uint8_t syn_config[256];
static unsigned syn_config_len;
static unsigned syn_max_x = 1439, syn_max_y = 3199, syn_max_objects = 10;

static unsigned
syn_bits(const uint8_t *r, unsigned n, unsigned off, unsigned bits)
{
	unsigned out = 0, done = 0;

	if (bits == 0 || bits > 32 || off + bits > n * 8)
		return 0;
	while (done < bits) {
		unsigned bo = off % 8, take = 8 - bo;

		if (take > bits - done)
			take = bits - done;
		out |= ((unsigned)(r[off / 8] >> bo) & (0xffu >> (8 - take))) << done;
		off += take;
		done += take;
	}
	return out;
}

static struct {
	bool down;
	uint32_t x, y;
} syn_contact[SYN_MAX_OBJECTS];

static void
syn_report(const uint8_t *r, unsigned n)
{
	unsigned status[SYN_MAX_OBJECTS] = { 0 }, xs[SYN_MAX_OBJECTS] = { 0 }, ys[SYN_MAX_OBJECTS] = { 0 };
	unsigned idx = 0, off = 0, obj = 0, next = 0, objects = 0, active_objects = 0;
	bool active_only = false, have_count = false;
	unsigned end_of_foreach = 0;

	while (idx < syn_config_len) {
		uint8_t c = syn_config[idx++];
		unsigned bits, v;

		if (c == 0x00)
			break;
		if (c == 0x01) {
			obj = 0;
			next = idx;
			active_only = true;
			continue;
		}
		if (c == 0x02) {
			obj = 0;
			next = idx;
			active_only = false;
			continue;
		}
		if (c == 0x03) {
			end_of_foreach = idx;
			if (active_only) {
				if (have_count) {
					objects++;
					obj++;
					if (objects < active_objects)
						idx = next;
				} else if (off < n * 8) {
					obj++;
					idx = next;
				}
			} else {
				obj++;
				if (obj < syn_max_objects)
					idx = next;
			}
			continue;
		}
		if (c == 0x04) {
			off = (off + 7) / 8 * 8;
			continue;
		}
		if (idx >= syn_config_len)
			break;
		bits = syn_config[idx++];
		v = syn_bits(r, n, off, bits);
		off += bits;
		if (c == 0x06) {
			obj = v;
		} else if (obj < SYN_MAX_OBJECTS && c == 0x07) {
			status[obj] = v;
		} else if (obj < SYN_MAX_OBJECTS && c == 0x08) {
			xs[obj] = v;
		} else if (obj < SYN_MAX_OBJECTS && c == 0x09) {
			ys[obj] = v;
		} else if (c == 0x18) {
			active_objects = v;
			have_count = true;
			if (v == 0 && end_of_foreach != 0)
				idx = end_of_foreach;
		}
		if (obj > 64)
			break;
	}
	for (unsigned id = 0; id < SYN_MAX_OBJECTS; id++) {
		uint32_t x, y;

		if (status[id] < 1 || status[id] > 5)
			continue;
		x = xs[id] >= syn_max_x ? 65535 : xs[id] * 65535u / syn_max_x;
		y = ys[id] >= syn_max_y ? 65535 : ys[id] * 65535u / syn_max_y;
		if (!syn_contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
			hakios_tl("tl: synaptics finger %u down %u,%u\n", id, x, y);
		} else if (syn_contact[id].x != x || syn_contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		syn_contact[id].down = true;
		syn_contact[id].x = x;
		syn_contact[id].y = y;
	}
	for (unsigned id = 0; id < SYN_MAX_OBJECTS; id++) {
		bool seen = status[id] >= 1 && status[id] <= 5;

		if (syn_contact[id].down && !seen) {
			syn_contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, syn_contact[id].x, syn_contact[id].y);
			hakios_tl("tl: synaptics finger %u up\n", id);
		}
	}
}

/* poll INT forever, decode 0x11 reports into /dev/hakios-touch */
static void
syn_worker(unsigned mx, unsigned my, unsigned mo)
{
	static uint8_t msg[GW_MAX];
	uint8_t code = 0;
	unsigned size = 0, idle = 0;

	syn_max_x = mx ? mx : 1439;
	syn_max_y = my ? my : 3199;
	syn_max_objects = (mo && mo <= SYN_MAX_OBJECTS) ? mo : SYN_MAX_OBJECTS;
	printf("synaptics: worker on: %u x %u, %u objects\n", syn_max_x, syn_max_y, syn_max_objects);
	hakios_tl("tl: synaptics worker on %u x %u obj %u\n", syn_max_x, syn_max_y, syn_max_objects);
	for (;;) {
		sleep_ms(10);
		if (gpio_get(PIN_INT)) {
			if (++idle % 3000 == 0)
				hakios_tl("tl: synaptics idle (INT high)\n");
			continue;
		}
		idle = 0;
		{
			int e = gw_read(&code, msg, &size);
			static unsigned anomalies;

			if ((e || code != 0x11) && anomalies < 8) {
				anomalies++;
				hakios_tl("tl: synaptics poll e=%d code=%#x size=%u\n", e, code, size);
			}
			if (e)
				continue;
			if (code == 0x11)
				syn_report(msg, size);
		}
	}
}

static void
probe_thread(__unused void *parameter, __unused wait_result_t result)
{
	static const unsigned pins[] = { 48, 49, 50, 51, 161, 162, 164 };

	printf("synaptics: probe thread running (touchscreen=1 seen)\n");
	hakios_tl("tl: synaptics probe thread running\n");
	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		unsigned n = pins[i];

		if (n >= sizeof(pin_map) / sizeof(pin_map[0]))
			continue;
		pin_map[n] = ml_io_map(PIN_PAGE(n), 0x1000);
		if (!pin_map[n]) {
			printf("synaptics: pin %u map failed\n", n);
			hakios_tl("tl: synaptics pin %u map failed\n", n);
			goto out;
		}
	}
	printf("synaptics: S3910P probe: 7 pins mapped (4KB each)\n");
	hakios_tl("tl: synaptics 7 pins mapped\n");
	printf("synaptics: pins as found: 48=%#x/%u 49=%#x/%u 50=%#x/%u 51=%#x/%u 161=%#x/%u 162=%#x/%u 164=%#x/%u\n",
	    rd(GPIO_CFG(48)), rd(GPIO_IO(48)) & 3,
	    rd(GPIO_CFG(49)), rd(GPIO_IO(49)) & 3,
	    rd(GPIO_CFG(50)), rd(GPIO_IO(50)) & 3,
	    rd(GPIO_CFG(51)), rd(GPIO_IO(51)) & 3,
	    rd(GPIO_CFG(161)), rd(GPIO_IO(161)) & 3,
	    rd(GPIO_CFG(162)), rd(GPIO_IO(162)) & 3,
	    rd(GPIO_CFG(164)), rd(GPIO_IO(164)) & 3);
	hakios_tl("tl: synaptics pins 48=%#x 49=%#x 50=%#x 51=%#x 161=%#x 162=%#x 164=%#x\n",
	    rd(GPIO_CFG(48)), rd(GPIO_CFG(49)), rd(GPIO_CFG(50)), rd(GPIO_CFG(51)),
	    rd(GPIO_CFG(161)), rd(GPIO_CFG(162)), rd(GPIO_CFG(164)));
	/* IOVDD first (RPMh ldob12 1.8V); without it MISO stays 0 while INT asserts */
	rsc = ml_io_map(SYN_RSC_PHYS, SYN_RSC_SIZE);
	cmd_db = ml_io_map_wcomb(SYN_CMD_DB_PHYS, SYN_CMD_DB_SIZE);
	printf("synaptics: RSC/CmdDB: rsc=%p cmd_db=%p\n", rsc, cmd_db);
	hakios_tl("tl: synaptics rsc=%p cmddb=%p\n", rsc, cmd_db);
	if (rsc && cmd_db) {
		uint32_t rid = rd(rsc + 0), rcfg = rd(rsc + 0x0c);

		hakios_tl("tl: synaptics RSC id=%#x cfg=%#x v=%u.%u\n", rid, rcfg,
		    (rid >> 16) & 0xff, (rid >> 8) & 0xff);
		bool io = syn_io_supply_on();

		printf("synaptics: IOVDD vote %s\n", io ? "done" : "FAILED (AVDD alone)");
		hakios_tl("tl: synaptics IOVDD %s\n", io ? "on" : "FAILED");
		sleep_ms(15);
	} else {
		printf("synaptics: RSC/CmdDB map failed: IOVDD skipped\n");
		hakios_tl("tl: synaptics RSC map failed\n");
	}
	/* houji htPrepare tail: bit-bang pins to GPIO, AVDD on, reset 10/200ms */
	gpio_set(PIN_MOSI, true);
	gpio_set(PIN_CLK, false);
	gpio_set(PIN_CS, true);
	gpio_set(PIN_RESET, true);
	gpio_configure(PIN_MISO, 0, PULL_NONE, 2, false);
	gpio_configure(PIN_MOSI, 0, PULL_NONE, 2, true);
	gpio_configure(PIN_CLK, 0, PULL_NONE, 2, true);
	gpio_configure(PIN_CS, 0, PULL_NONE, 2, true);
	gpio_configure(PIN_RESET, 0, PULL_NONE, 2, true);
	gpio_configure(PIN_INT, 0, PULL_UP, 2, false);
	gpio_output(PIN_AVDD, true, PULL_NONE, 2);
	sleep_ms(50);
	gpio_set(PIN_RESET, false);
	sleep_ms(10);
	gpio_set(PIN_RESET, true);
	sleep_ms(200);
	printf("synaptics: reset done, reading TouchCom header\n");
	hakios_tl("tl: synaptics reset done\n");
	{
		static uint8_t msg[GW_MAX], code = 0, h[4];
		static unsigned size = 0;
		int e;

		printf("synaptics: post-reset levels: INT=%u MISO=%u AVDD io=%#x\n",
		    gpio_get(PIN_INT) ? 1 : 0, gpio_get(PIN_MISO) ? 1 : 0, rd(GPIO_IO(PIN_AVDD)));
		hakios_tl("tl: synaptics post-reset INT=%u MISO=%u\n",
		    gpio_get(PIN_INT) ? 1 : 0, gpio_get(PIN_MISO) ? 1 : 0);
		e = gw_read(&code, msg, &size);

		for (unsigned r = 0; e == 2 && r < 100; r++) {
			sleep_ms(10);
			e = gw_read(&code, msg, &size);
		}
		printf("synaptics: first read e=%d code=%#x size=%u\n", e, code, size);
		hakios_tl("tl: synaptics first read e=%d code=%#x size=%u\n", e, code, size);
		if (!e && code != 0x10) {
			e = gw_command(2, NULL, 0, &code, msg, &size);
			printf("synaptics: identify e=%d code=%#x size=%u\n", e, code, size);
			hakios_tl("tl: synaptics identify e=%d code=%#x size=%u\n", e, code, size);
		}
		if (!e && code == 0x10 && size >= 24 && msg[1] == 1 && memcmp(msg + 2, "S3910", 5) == 0) {
			uint32_t build = msg[18] | (uint32_t)msg[19] << 8 | (uint32_t)msg[20] << 16 | (uint32_t)msg[21] << 24;
			unsigned adv = msg[22] | (unsigned)msg[23] << 8;

			printf("synaptics: S3910P TouchCom v1 app firmware build %u, max write %u\n", build, adv);
			hakios_tl("tl: synaptics S3910P build %u maxwr %u\n", build, adv);
			e = gw_command(0x20, NULL, 0, &code, msg, &size);
			printf("synaptics: app-info e=%d code=%#x size=%u\n", e, code, size);
			hakios_tl("tl: synaptics app-info e=%d code=%#x size=%u\n", e, code, size);
			if (!e && size >= 38) {
				unsigned mx = msg[32] | (unsigned)msg[33] << 8;
				unsigned my = msg[34] | (unsigned)msg[35] << 8;
				unsigned mo = msg[36] | (unsigned)msg[37] << 8;

				printf("synaptics: panel max %u x %u, %u objects\n", mx, my, mo);
				hakios_tl("tl: synaptics panel %u x %u obj %u\n", mx, my, mo);
				{
					uint8_t raw;
					int e2;

					raw = 0xc0;
					e2 = gw_command(6, &raw, 1, &code, msg, &size);
					hakios_tl("tl: synaptics THP off e=%d code=%#x\n", e2, code);
					raw = 0x23;
					e2 = gw_command(6, &raw, 1, &code, msg, &size);
					hakios_tl("tl: synaptics raw off e=%d code=%#x\n", e2, code);
					e2 = gw_command(0x25, NULL, 0, &code, msg, &size);
					hakios_tl("tl: synaptics descriptor e=%d code=%#x size=%u\n", e2, code, size);
					for (unsigned o = 0; e2 == 0 && o < size && o < 128; o += 16)
						hakios_tl("tl: synaptics desc+%u: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
						    o, msg[o], msg[o+1], msg[o+2], msg[o+3], msg[o+4], msg[o+5], msg[o+6], msg[o+7],
						    msg[o+8], msg[o+9], msg[o+10], msg[o+11], msg[o+12], msg[o+13], msg[o+14], msg[o+15]);
					syn_config_len = (e2 == 0 && size < sizeof(syn_config)) ? size : 0;
					if (syn_config_len)
						memcpy(syn_config, msg, syn_config_len);
					raw = 0x11;
					e2 = gw_command(5, &raw, 1, &code, msg, &size);
					hakios_tl("tl: synaptics report-on e=%d code=%#x\n", e2, code);
					if (!e2 && syn_config_len)
						syn_worker(mx, my, mo);
					hakios_tl("tl: synaptics worker exited\n");
				}
			} else if (!e) {
			printf("synaptics: unexpected identify payload (size %u, mode %#x)\n", size, size ? msg[1] : 0);
		}
	}
	}
	printf("synaptics: S3910P bit-bang bring-up pending (need GPIO-driven TouchCom, no GENI)\n");
out:
	thread_terminate(current_thread());
}

static void
touch_setup(void)
{
	touch_ready = false;
}

void
hakios_synaptics_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled)
		return;
	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("touch_lat", &latency, sizeof(latency));
	touch_setup();
	/* probe thread always runs when asked: its prints survive (unlike init's) */
	if (kernel_thread_start(probe_thread, NULL, &thread) == KERN_SUCCESS)
		thread_deallocate(thread);
	if (touch_ready && kernel_thread_start(touch_thread, NULL, &thread) == KERN_SUCCESS)
		thread_deallocate(thread);
}
