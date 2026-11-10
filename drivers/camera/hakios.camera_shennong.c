/*
 * The cameras of the Xiaomi 14 Pro (shennong): the CCI buses (qcom,cci0@ac15000, cci1@ac16000,
 * cci2@ac17000) and their sensors (qcom,cam-sensor0..3, aperture/actuator/eeprom/OIS per sensor),
 * with sensor tables and tuning from the phone's own firmware (fw-camera/, devices/shennong/
 * pull-firmware.sh). This stands in for the reference phone's sensor set (HAKIOS_CAMERA). Only
 * with boot-arg cam=N: 1 = a look (read-only), 2 = probe (power up, chip ID, power down).
 *
 * The four sensors, as Android's device tree has them (devices/shennong/prebuilt/device-tree.dts,
 * decompiled with dtc) and as their module files describe them (fw-camera/
 * com.qti.sensormodule.shennong_*.bin, decoded with tools/camsensor.py into camera-*.tab):
 *
 *   wide  (cam-sensor0): OmniVision OVX9000 (Light Hunter 900, 50 MP), CCI 0 master 1
 *           (GPIO 115 data, 116 clock), slave 0x20, chip ID 0x5650 at 0x300a; reset GPIO 108;
 *           MCLK0 (camcc RCG 0x15000, branch 0x15018) at 19.2 MHz on GPIO 100 (function cam_mclk);
 *           VANA a fixed 2.9 V regulator switched by GPIO 127 (camera_wide_avdd_ldo);
 *           VIO ldob10 at 1.8 V, VDIG ldom2 at 1.2 V, VAF ldom6 at 3.29 V, VCUSTOM1 ldom7 at
 *           3.29 V, BOB2 bobb2 at 3.296 V (all RPMh VRM votes); 3 C-PHY trios into CSIPHY 3.
 *           Second source semco_ovx9000 (same slave and ID): its table is tried when the sunny
 *           one does not match.
 *   ultra (cam-sensor2): Samsung S5KJN1 (50 MP), CCI 0 master 0 (GPIO 113/114), slave 0x7a,
 *           chip ID 0x38e1 at 0x0000; reset GPIO 109; MCLK1 (RCG 0x1501c, branch 0x15034) on
 *           GPIO 101; VANA fixed 2.8 V by GPIO 126; VIO ldob10 1.8 V, VDIG ldoe1 1.059 V,
 *           VAF ldom4 2.8 V, BOB2 3.296 V; 4 D-PHY lanes into CSIPHY 1.
 *   tele  (cam-sensor3): Samsung S5KJN1 (50 MP, with OIS), CCI 1 master 0 (GPIO 117/118),
 *           slave 0x5a, chip ID 0x38e1 at 0x0000; reset GPIO 110; MCLK3 (RCG 0x15054,
 *           branch 0x1506c) on GPIO 103; VANA fixed 2.8 V by GPIO 125; VIO ldob10 1.8 V,
 *           VDIG ldoc2 1.059 V, VAF ldob8 3.104 V, BOB2 3.296 V; 4 D-PHY lanes into CSIPHY 0.
 *   front (cam-sensor1): OmniVision OV32B40 (32 MP), on GENI I2C qupv3_se9 (GPIO 4/5, NOT a CCI;
 *           reg 0x10), slave 0x20, chip ID 0x3242 at 0x300b; reset GPIO 7; MCLK4
 *           (camcc RCG 0x15070, branch 0x15088, always-on) on GPIO 104 (function cam_aon_mclk4);
 *           VANA ldom3 2.8 V, VIO ldom5 1.8 V, VDIG ldom1 1.1 V, BOB2 3.296 V.
 *
 * Rails are RPMh VRM votes (like the touch screen's I/O supply) or GPIO-switched fixed
 * regulators; MCLK comes from camcc's RCGs straight off the crystal (19.2 MHz, no PLL), behind
 * titan_top_gdsc (camcc 0x132bc); the CCI pins are driven by hand for the ID read (like the
 * reference driver) and never clock the CCI hardware. Nothing here is done unless a boot-arg
 * asks for it: cam=1 reads only (pins, tables), cam=2 powers the sensor as the module file's
 * power sequence says, reads its chip ID and powers it down again. Boot-arg cam_sensor=
 * wide|tele|ultra|front picks the sensor (default: the wide one).
 *
 * Limits, whatever asks: only the voltages named above are voted (the device tree's and the
 * module files'); only the registers named above are written (camcc's MCLK RCGs/branches and
 * titan_top, each read back); an unacknowledged byte, a vote RPMh does not take, and everything
 * is switched off again in the power-down's order. At the run's end whatever was switched on is
 * switched off. The sensors' bayer orders are the vendors' convention (OmniVision BGGR, Samsung
 * GRBG): the first picture on hardware decides (red and blue come out swapped if wrong).
 */
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <kern/thread.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern void hakios_tl(const char *format, ...) __printflike(1, 2);
extern kern_return_t thread_terminate(thread_t thread);
extern lck_mtx_t hakios_rsc_lock;
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk);

static uint32_t step;                   /* the step being performed */
static uint32_t level;                  /* boot-arg cam= */
static bool broken;                     /* a fault: nothing more is written in this run */
static bool quiet;

static void
announce(const char *what)
{
	printf("camera-s: step %u: %s\n", step, what);
	hakios_tl("tl: camera-s step %u: %s\n", step, what);
}

static void
give_up(const char *why)
{
	if (!broken) {
		broken = true;
		printf("camera-s: FAILED: %s: nothing more is written in this run\n", why);
		hakios_tl("tl: camera-s FAILED: %s\n", why);
	}
}

/* ---- blocks, get/put -------------------------------------------------------------------------- */

struct block {
	const char *name;
	uint32_t phys, size;
	vm_offset_t base;
};

static void
need(struct block *b)
{
	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
}

static uint32_t
get(struct block *b, const char *name, uint32_t offset)
{
	uint32_t value;

	need(b);
	if (!quiet) {
		printf("camera-s: %u: read %s %s at %#x\n", step, b->name, name, b->phys + offset);
	}
	value = *(volatile uint32_t *)(b->base + offset);
	if (!quiet) {
		printf("camera-s: %u: %s %s (%#x) = %#x\n", step, b->name, name, b->phys + offset, value);
	}
	return value;
}

/* A register written and read back where it keeps what it is given. */
static bool
put(struct block *b, const char *name, uint32_t offset, uint32_t value, uint32_t check_mask)
{
	need(b);
	if (!quiet) {
		printf("camera-s: %u: %s %s at %#x <- %#x\n", step, b->name, name, b->phys + offset, value);
	}
	*(volatile uint32_t *)(b->base + offset) = value;
	if (((*(volatile uint32_t *)(b->base + offset)) & check_mask) != (value & check_mask)) {
		printf("camera-s: %u: %s %#x <- %#x, but it reads %#x\n", step, name, offset, value,
		    *(volatile uint32_t *)(b->base + offset));
		give_up("a register that did not read back");
		return false;
	}
	return true;
}

static bool
wait_bits(struct block *b, uint32_t offset, uint32_t mask, uint32_t want, uint32_t us)
{
	uint32_t waited;

	need(b);
	for (waited = 0; waited <= us; waited += 10) {
		if ((*(volatile uint32_t *)(b->base + offset) & mask) == want) {
			return true;
		}
		delay_for_interval(10, NSEC_PER_USEC);
	}
	return false;
}

/* ---- pins (TLMM, pinctrl@f000000) ------------------------------------------------------------- */
/* Per-pin 4KB maps (like the touch screen driver): GPIO n's CFG at PIN_PAGE(n). */

#define TLMM_PHYS               0x0f100000UL
#define TLMM_GPIO_STRIDE        0x1000UL
#define PIN_PAGE(n)             (TLMM_PHYS + TLMM_GPIO_STRIDE * (n))

static vm_offset_t pin_map[165];

static vm_offset_t
pin_base(unsigned pin)
{
	if (pin >= sizeof(pin_map) / sizeof(pin_map[0])) {
		return 0;
	}
	if (pin_map[pin] == 0) {
		pin_map[pin] = ml_io_map(PIN_PAGE(pin), 0x1000);
	}
	return pin_map[pin];
}

#define GPIO_CFG(pin)           (pin_base(pin) + 0)     /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(pin)            (pin_base(pin) + 4)     /* bit 0 in, bit 1 out */
#define CFG_PULL_NONE           0
#define CFG_PULL_UP             3
#define CFG_OUTPUT              (1u << 9)
#define CFG_FUNCTION(f)         ((uint32_t)(f) << 2)
#define CFG_DRIVE_MA(ma)        ((uint32_t)((ma) / 2 - 1) << 6)
#define FUNC_GPIO               0
#define FUNC_CAM_MCLK           1       /* cam_mclk and cam_aon_mclk4 (pinctrl-sm8650.c) */

static uint32_t
pin_rd_cfg(unsigned pin)
{
	return *(volatile uint32_t *)GPIO_CFG(pin);
}

static uint32_t
pin_rd_io(unsigned pin)
{
	return *(volatile uint32_t *)GPIO_IO(pin);
}

static void
show_pin(unsigned pin, const char *what)
{
	printf("camera-s: gpio%u (%s): function %u, pull %u, %u mA, %s, reads %u\n", pin, what,
	    (pin_rd_cfg(pin) >> 2) & 15, pin_rd_cfg(pin) & 3, (((pin_rd_cfg(pin) >> 6) & 7) + 1) * 2,
	    (pin_rd_cfg(pin) & CFG_OUTPUT) ? (pin_rd_io(pin) & 2 ? "drives high" : "drives low") : "input",
	    pin_rd_io(pin) & 1);
}

/* ---- the sensors --------------------------------------------------------------------------------
 * Kinds of rail: an RPMh VRM regulator (voted by resource name), or a GPIO-switched fixed one.
 * The voltages are the device tree's (the cam-sensor nodes' rgltr-min/max-voltage); BOB2's 3296 mV
 * is the module files' (the power sequences' kind 14).
 */

enum { SUPPLY_NONE, SUPPLY_VRM, SUPPLY_GPIO };
struct supply {
	uint8_t kind;
	const char *vrm;        /* SUPPLY_VRM: RPMh resource name ("ldob10") */
	uint16_t mv;            /* ... at this set point (mV) */
	uint16_t gpio;          /* SUPPLY_GPIO: the pin, active high */
};

struct sensor_hw {
	const char *name;       /* cam_sensor= names match on a prefix ("wide" matches "wide ...") */
	const char *tab;        /* in fw/ (tools/camsensor.py); the wide one falls back to tab2 */
	const char *tab2;
	uint32_t cci;           /* which CCI (0/1; ~0u: GENI I2C, not a CCI) */
	unsigned sda, scl;      /* the serial pins, taken by hand */
	unsigned reset;         /* reset GPIO, high = running */
	unsigned mclk;          /* which MCLK (0/1/3/4): RCG 0x15000+0x1c*mclk, branch +0x18 ... */
	unsigned mclk_pin;      /* ... on this pin (function 1) */
	bool mclk_aon;          /* already on is fine (the front camera's always-on MCLK4) */
	struct supply vana, vdig, vio, vaf, vcustom;
	unsigned csiphy;        /* for the frames' header (cam=4 comes later) */
	uint8_t regs;           /* REGS_SMIA or REGS_OV16 (exposure registers for cam=4) */
	uint8_t bayer;          /* the picture's order (camraw.py's codes: 1 GRBG, 3 BGGR) */
	uint8_t slave, id_bytes;/* from the module files (checked against the tables) */
	uint16_t id_address, id;
};

enum {
	REGS_SMIA,              /* SMIA: frame length 0x0340, coarse integration 0x0202, analog gain 0x0204, hold 0x0104 */
	REGS_OV16,              /* OmniVision, 16-bit addresses: exposure 0x3500.., gain 0x3508, VTS 0x380e, hold 0x3208 */
};

static const struct sensor_hw sensors[] = {
	{
		.name = "wide (OVX9000)", .tab = "camera-sunny_ovx9000_wide.tab",
		.tab2 = "camera-semco_ovx9000_wide_ii.tab",
		.cci = 0, .sda = 115, .scl = 116, .reset = 108,
		.mclk = 0, .mclk_pin = 100, .mclk_aon = false,
		.vana = { SUPPLY_GPIO, NULL, 2900, 127 }, .vdig = { SUPPLY_VRM, "ldom2", 1200, 0 },
		.vio = { SUPPLY_VRM, "ldob10", 1800, 0 }, .vaf = { SUPPLY_VRM, "ldom6", 3293, 0 },
		.vcustom = { SUPPLY_VRM, "ldom7", 3293, 0 },
		.csiphy = 3, .regs = REGS_OV16, .bayer = 3, .slave = 0x20, .id_bytes = 2,
		.id_address = 0x300a, .id = 0x5650,
	},
	{
		.name = "ultra-wide (S5KJN1)", .tab = "camera-ofilm_s5kjn1_ultra.tab", .tab2 = NULL,
		.cci = 0, .sda = 113, .scl = 114, .reset = 109,
		.mclk = 1, .mclk_pin = 101, .mclk_aon = false,
		.vana = { SUPPLY_GPIO, NULL, 2800, 126 }, .vdig = { SUPPLY_VRM, "ldoe1", 1059, 0 },
		.vio = { SUPPLY_VRM, "ldob10", 1800, 0 }, .vaf = { SUPPLY_VRM, "ldom4", 2800, 0 },
		.vcustom = { SUPPLY_NONE, NULL, 0, 0 },
		.csiphy = 1, .regs = REGS_SMIA, .bayer = 1, .slave = 0x7a, .id_bytes = 2,
		.id_address = 0x0000, .id = 0x38e1,
	},
	{
		.name = "tele (S5KJN1)", .tab = "camera-ofilm_s5kjn1_tele.tab", .tab2 = NULL,
		.cci = 1, .sda = 117, .scl = 118, .reset = 110,
		.mclk = 3, .mclk_pin = 103, .mclk_aon = false,
		.vana = { SUPPLY_GPIO, NULL, 2800, 125 }, .vdig = { SUPPLY_VRM, "ldoc2", 1059, 0 },
		.vio = { SUPPLY_VRM, "ldob10", 1800, 0 }, .vaf = { SUPPLY_VRM, "ldob8", 3104, 0 },
		.vcustom = { SUPPLY_NONE, NULL, 0, 0 },
		.csiphy = 0, .regs = REGS_SMIA, .bayer = 1, .slave = 0x5a, .id_bytes = 2,
		.id_address = 0x0000, .id = 0x38e1,
	},
	{
		.name = "front (OV32B40)", .tab = "camera-sunny_ov32b40_front.tab", .tab2 = NULL,
		.cci = ~0u, .sda = 4, .scl = 5, .reset = 7,
		.mclk = 4, .mclk_pin = 104, .mclk_aon = true,
		.vana = { SUPPLY_VRM, "ldom3", 2800, 0 }, .vdig = { SUPPLY_VRM, "ldom1", 1100, 0 },
		.vio = { SUPPLY_VRM, "ldom5", 1800, 0 }, .vaf = { SUPPLY_NONE, NULL, 0, 0 },
		.vcustom = { SUPPLY_NONE, NULL, 0, 0 },
		.csiphy = 4, .regs = REGS_OV16, .bayer = 3, .slave = 0x20, .id_bytes = 2,
		.id_address = 0x300b, .id = 0x3242,
	},
};

static const struct sensor_hw *hw = &sensors[0];

/* ---- I2C by hand -------------------------------------------------------------------------------- */

struct bus {
	unsigned sda, scl;
	uint32_t cfg[2], io[2];  /* the pins as they were found, while taken */
};
static struct bus cci_bus;              /* the sensor's (from hw) */

static void
half_bit(void)
{
	delay_for_interval(5, NSEC_PER_USEC);    /* 100 kHz at most */
}

static void
line(unsigned pin, bool high)
{
	*(volatile uint32_t *)GPIO_IO(pin) = 0;
	*(volatile uint32_t *)GPIO_CFG(pin) = CFG_PULL_UP | (high ? 0 : CFG_OUTPUT);
}

static bool
level_of(unsigned pin)
{
	return pin_rd_io(pin) & 1;
}

static void
i2c_start(struct bus *b)
{
	line(b->sda, true);
	line(b->scl, true);
	half_bit();
	line(b->sda, false);
	half_bit();
	line(b->scl, false);
	half_bit();
}

static void
i2c_stop(struct bus *b)
{
	line(b->sda, false);
	half_bit();
	line(b->scl, true);
	half_bit();
	line(b->sda, true);
	half_bit();
}

static bool
i2c_bit(struct bus *b, bool out)
{
	bool in;

	line(b->sda, out);
	half_bit();
	line(b->scl, true);
	for (unsigned stretched = 0; !level_of(b->scl) && stretched < 100; stretched++) {
		half_bit();
	}
	half_bit();
	in = level_of(b->sda);
	line(b->scl, false);
	return in;
}

static bool
i2c_write(struct bus *b, uint8_t byte)
{
	for (int bit = 7; bit >= 0; bit--) {
		i2c_bit(b, (byte >> bit) & 1);
	}
	return !i2c_bit(b, true);
}

static uint8_t
i2c_read(struct bus *b, bool more)
{
	uint8_t byte = 0;

	for (int bit = 7; bit >= 0; bit--) {
		byte = (uint8_t)(byte << 1 | i2c_bit(b, true));
	}
	i2c_bit(b, !more);
	return byte;
}

static void
bus_give_back(struct bus *b)
{
	*(volatile uint32_t *)GPIO_IO(b->sda) = b->io[0] & 2;
	*(volatile uint32_t *)GPIO_IO(b->scl) = b->io[1] & 2;
	*(volatile uint32_t *)GPIO_CFG(b->sda) = b->cfg[0];
	*(volatile uint32_t *)GPIO_CFG(b->scl) = b->cfg[1];
}

static bool
bus_take(struct bus *b)
{
	b->cfg[0] = pin_rd_cfg(b->sda);
	b->cfg[1] = pin_rd_cfg(b->scl);
	b->io[0] = pin_rd_io(b->sda);
	b->io[1] = pin_rd_io(b->scl);
	line(b->sda, true);
	line(b->scl, true);
	half_bit();
	if (level_of(b->sda) && level_of(b->scl)) {
		return true;
	}
	bus_give_back(b);
	printf("camera-s: the bus (gpio%u/gpio%u) is not at rest: not touched\n", b->sda, b->scl);
	return false;
}

/*
 * A transaction: `out' (the register's address, and for a write the data) written, then for a read
 * `in_count' bytes read after a repeated start. False if anybody did not acknowledge.
 */
static bool
i2c_transfer(struct bus *b, uint8_t address, const uint8_t *out, unsigned out_count, uint8_t *in, unsigned in_count)
{
	bool answered;

	if (!bus_take(b)) {
		return false;
	}
	i2c_start(b);
	answered = i2c_write(b, (uint8_t)(address << 1));
	for (unsigned i = 0; answered && i < out_count; i++) {
		answered = i2c_write(b, out[i]);
	}
	if (answered && in_count != 0) {
		i2c_start(b);
		answered = i2c_write(b, (uint8_t)(address << 1 | 1));
		for (unsigned i = 0; answered && i < in_count; i++) {
			in[i] = i2c_read(b, i + 1 < in_count);
		}
	}
	i2c_stop(b);
	bus_give_back(b);
	return answered;
}

/* ---- RPMh (rails) ---------------------------------------------------------------------------------
 * Like the touch screen driver: resource addresses from the command DB, votes through a free TCS
 * of DRV 2 (RSC v3 layout). A VRM regulator takes voltage (mV), mode (7: HPM for an LDO) and
 * enable; enable 0 alone switches it off again.
 */

#define CMD_DB_PHYS             0x81c60000UL
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS                0x17a20000UL
#define RSC_SIZE                0x10000UL
#define RSC_TCS_BASE            0xd00
#define RSC_TCS_STRIDE          0x2a0
#define RSC_TCS_COUNT           3
#define TCS_MSGID               0x20    /* v3 layout: msgid, control, status, enable, ... */
#define TCS_CONTROL             0x24
#define TCS_STATUS              0x28
#define TCS_ENABLE              0x2c
#define TCS_WAIT                0x20    /* (the synaptics driver's v3 offsets) */
#define TCS_CONTROL_ENABLE      (1u << 16)
#define TCS_CONTROL_TRIGGER     (1u << 24)
#define TCS_CMD_MSGID(c)        (0x34 + 24 * (c))
#define TCS_CMD_ADDR(c)         (0x38 + 24 * (c))
#define TCS_CMD_DATA(c)         (0x3c + 24 * (c))
#define TCS_CMD_STATUS(c)       (0x40 + 24 * (c))
#define MSGID_WRITE             0x10008
#define VRM_VOLTAGE             0
#define VRM_ENABLE              4
#define VRM_MODE                8
#define VRM_MODE_LDO_HPM        7

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static vm_offset_t cmd_db, rsc;

static uint32_t
rpmh_address(const char *name)
{
	const struct cmd_db_header *db;
	char id[8] = { 0 };

	if (cmd_db == 0) {
		cmd_db = ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);
	}
	db = (const struct cmd_db_header *)cmd_db;
	if (!(db->magic[0] == 0xdb && db->magic[1] == 0x30 && db->magic[2] == 0x03 && db->magic[3] == 0x0c)) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry;

		if (slave->id == 0 || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

/* Up to three writes to RPMh's resources through a free TCS, waited for. */
static bool
rpmh_write(unsigned n, const uint32_t *addresses, const uint32_t *values, bool force)
{
	int chosen = -1;
	bool done = false;

	if (rsc == 0) {
		rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	}
	if (broken && !force) {
		give_up("a vote after a fault");
		return false;
	}
	lck_mtx_lock(&hakios_rsc_lock);
	for (unsigned t = 0; t < RSC_TCS_COUNT; t++) {
		uint32_t b = RSC_TCS_BASE + t * RSC_TCS_STRIDE;
		uint32_t st = *(volatile uint32_t *)(rsc + b + TCS_STATUS);
		uint32_t ctl = *(volatile uint32_t *)(rsc + b + TCS_CONTROL);
		uint32_t en = *(volatile uint32_t *)(rsc + b + TCS_ENABLE);

		if (chosen < 0 && st != 0 && st != 0xffffffffu && ctl == 0 && en == 0) {
			chosen = (int)t;
		}
	}
	if (chosen < 0) {
		lck_mtx_unlock(&hakios_rsc_lock);
		printf("camera-s: no free TCS: not voted\n");
		give_up("RPMh without a free TCS");
		return false;
	}
	{
		uint32_t tb = RSC_TCS_BASE + (uint32_t)chosen * RSC_TCS_STRIDE;

		for (unsigned i = 0; i < n; i++) {
			*(volatile uint32_t *)(rsc + tb + TCS_CMD_MSGID(i)) = MSGID_WRITE | 0x100;
			*(volatile uint32_t *)(rsc + tb + TCS_CMD_ADDR(i)) = addresses[i];
			*(volatile uint32_t *)(rsc + tb + TCS_CMD_DATA(i)) = values[i];
		}
		*(volatile uint32_t *)(rsc + tb + TCS_WAIT) = (1u << n) - 1;
		*(volatile uint32_t *)(rsc + tb + TCS_ENABLE) = (1u << n) - 1;
		*(volatile uint32_t *)(rsc + tb + TCS_CONTROL) = 0;
		*(volatile uint32_t *)(rsc + tb + TCS_CONTROL) = TCS_CONTROL_ENABLE;
		*(volatile uint32_t *)(rsc + tb + TCS_CONTROL) = TCS_CONTROL_ENABLE | TCS_CONTROL_TRIGGER;
		for (int i = 0; i < 1000; i++) {
			bool all = true;

			for (unsigned c = 0; c < n; c++) {
				uint32_t cs = *(volatile uint32_t *)(rsc + tb + TCS_CMD_STATUS(c));

				all &= cs != 0xffffffffu && (cs & 0x10100u) == 0x10100u;
			}
			if (all && *(volatile uint32_t *)(rsc + tb + TCS_STATUS) != 0) {
				done = true;
				break;
			}
			delay_for_interval(100, NSEC_PER_USEC);
		}
		*(volatile uint32_t *)(rsc + tb + TCS_CONTROL) = TCS_CONTROL_ENABLE;
		*(volatile uint32_t *)(rsc + tb + TCS_CONTROL) = 0;
		*(volatile uint32_t *)(rsc + tb + TCS_ENABLE) = 0;
		*(volatile uint32_t *)(rsc + tb + TCS_WAIT) = 0;
	}
	lck_mtx_unlock(&hakios_rsc_lock);
	if (!done) {
		printf("camera-s: RPMh did not take the vote\n");
		give_up("RPMh did not take a vote");
	}
	return done;
}

/* A VRM regulator on at `mv' (mV) in HPM, or off (`on' false: enable 0 alone). */
static uint32_t vrm_on_addr[8];
static unsigned vrm_on_n;

static bool
vrm_vote(const char *resource, uint32_t mv, bool on, bool force)
{
	uint32_t base = rpmh_address(resource);
	uint32_t addresses[3], values[3];

	if (base == 0) {
		printf("camera-s: RPMh has no regulator \"%s\"\n", resource);
		give_up("a rail RPMh does not know");
		return false;
	}
	if (on) {
		addresses[0] = base + VRM_VOLTAGE;
		values[0] = mv;
		addresses[1] = base + VRM_MODE;
		values[1] = VRM_MODE_LDO_HPM;
		addresses[2] = base + VRM_ENABLE;
		values[2] = 1;
		if (!rpmh_write(3, addresses, values, force)) {
			return false;
		}
		if (vrm_on_n < sizeof(vrm_on_addr) / sizeof(vrm_on_addr[0])) {
			vrm_on_addr[vrm_on_n++] = base;
		}
		printf("camera-s: RPMh: %s at %u mV, on\n", resource, mv);
	} else {
		addresses[0] = base + VRM_ENABLE;
		values[0] = 0;
		if (!rpmh_write(1, addresses, values, force)) {
			return false;
		}
		printf("camera-s: RPMh: %s off\n", resource);
	}
	return true;
}

/* ---- camcc: titan_top and the sensor's MCLK ---------------------------------------------------------
 * SM8650 camcc (clock-controller@ade0000, qcom,pineapple-camcc-v2; drivers/clk/qcom/
 * camcc-sm8650.c): cam_cc_mclkN_clk_src at 0x15000 + 0x1c*N (CFG +4, CBCR +0x18),
 * cam_cc_titan_top_gdsc at 0x132bc. MCLK at 19.2 MHz is the crystal itself (source 0), divider 1.
 */

static struct block camcc = { "camcc", 0x0ade0000, 0x20000 };

/* GCC's camera AHB (DT camcc clocks <&gcc 5> = GCC_CAMERA_AHB_CLK, CBCR 0x26004, a critical
 * clock): gcc-sm8650.c has no branch for it, only a critical-cbcrs entry, so there is no
 * parent to vote (GPLL0, already handled below, is the only PLL ancestor worth a vote);
 * an OFF bit that stays set is warn-only, as with the GPU's cfg_ahb on this phone. */
#define GCC_CAM_AHB_CBCR        0x26004
/* The 600 MHz GPLL0 is off after a fastboot boot: its taps cannot run until it does (as the
 * GPU's cfg_ahb). Vote it and wait for the lock before the camera AHB. */
#define GCC_GPLL0_ENABLE        0x52020
#define GCC_PLL_VOTE_REG        0x52000
#define GCC_PLL_VOTE_BITS       ((1u << 15) | (1u << 16))
#define GCC_GPLL0_MODE          0x00000
#define GPLL_LOCK_DET           (1u << 31)
static struct block gcc = { "gcc", 0x00100000, 0x80000 };
#define MCLK_CMD(n)             (0x15000 + 0x1c * (n))
#define MCLK_CFG(n)             (0x15000 + 0x1c * (n) + 4)
#define MCLK_CBCR(n)            (0x15000 + 0x1c * (n) + 0x18)
#define TITAN_TOP_GDSCR         0x132bc
/* Upstream camcc-sm8650 titan_top (POLL_CFG_GDSCR|RETAIN_FF_ENABLE, waits 2,2,0xf,
 * pwrsts OFF_ON): status lives in the CFG register (GDSCR+4), bit 16 up-complete on,
 * bit 15 down-complete off (gdsc.c gdsc_check_status); GDSCR bit 31 and CFG bits 1:0
 * say nothing for this domain. The retain bit (GDSCR bit 11) goes on only after the
 * domain is on (gdsc.c gdsc_retain_ff_on); the wait values are what gdsc_init writes. */
#define TITAN_TOP_CFG           (TITAN_TOP_GDSCR + 4)
#define CFG_UP_COMPLETE         (1u << 16)
#define CFG_DOWN_COMPLETE       (1u << 15)
#define GDSC_RETAIN_FF          (1u << 11)
#define GDSC_HW_CTRL            (1u << 1)
#define GDSC_SW_OVERRIDE        (1u << 2)
#define GDSC_WAIT_MASK          0xfff000u     /* EN_REST_WAIT + EN_FEW_WAIT + CLK_DIS_WAIT */
#define GDSC_WAIT_VAL           (0x2u << 20 | 0x2u << 16 | 0xfu << 12)
#define CMD_UPDATE              1u
#define CBCR_ENABLE             1u
#define CBCR_OFF                (1u << 31)
#define GDSC_COLLAPSE           1u      /* (GDSCR bit 31 is no status for a POLL_CFG_GDSCR domain) */

static bool gdsc_switched, mclk_switched;
static uint32_t found_mclk_cfg, found_mclk_cbcr, found_mclk_pin;

/* The titan_top GDSC's parent supply (DT cam_cc_titan_top_gdsc: pm_v6d_s3_mmcx_voter_level,
 * an ARC regulator, resource "mxc.lvl", DT init level 0x30): voted like the cx/mx rails
 * (a level, not millivolts), without which the domain does not come on. Withdrawn in gdsc_off. */
static bool
mxc_parent(bool on)
{
	uint32_t base = rpmh_address("mxc.lvl");
	uint32_t level = on ? 0x30 : 0;

	if (base == 0) {
		printf("camera-s: RPMh has no regulator \"mxc.lvl\"\n");
		return false;
	}
	if (!rpmh_write(1, &base, &level, false)) {
		return false;
	}
	printf("camera-s: RPMh: mxc.lvl at level %#x, %s\n", level, on ? "on" : "off");
	return true;
}

static bool
gdsc_on(void)
{
	uint32_t found = get(&camcc, "titan top GDSC", TITAN_TOP_GDSCR);
	uint32_t cfg = get(&camcc, "titan top CFG", TITAN_TOP_CFG);

	printf("camera-s: 2: titan top GDSC %#x CFG %#x\n", found, cfg);
	if (cfg & CFG_UP_COMPLETE) {
		printf("camera-s: the titan top power domain was on already\n");
		/* Upstream gdsc_init sets the retain bit whenever the domain is found on. */
		if (!(found & GDSC_RETAIN_FF) &&
		    !put(&camcc, "titan top GDSC", TITAN_TOP_GDSCR, found | GDSC_RETAIN_FF, GDSC_RETAIN_FF)) {
			return false;
		}
		return true;
	}
	/* The wait values upstream programs at init (the boot loader leaves 2,2,0xf, so this
	 * is usually a no-op check); without them the state machine never runs right. */
	{
		uint32_t mask = GDSC_HW_CTRL | GDSC_SW_OVERRIDE | GDSC_WAIT_MASK;
		uint32_t want = (found & ~mask) | GDSC_WAIT_VAL;

		if ((found & mask) != (want & mask)) {
			if (!put(&camcc, "titan top GDSC", TITAN_TOP_GDSCR, want, mask)) {
				return false;
			}
			found = get(&camcc, "titan top GDSC", TITAN_TOP_GDSCR);
		}
	}
	if (!mxc_parent(true)) {
		give_up("the titan top power domain's parent (mxc)");
		return false;
	}
	{
		/* GPLL0 first (off after a fastboot boot): the AHB tap cannot run until it locks. */
		uint32_t gpll0 = get(&gcc, "gpll0", GCC_GPLL0_ENABLE);

		if (!(gpll0 & GPLL_LOCK_DET) &&
		    (!put(&gcc, "gpll0", GCC_GPLL0_ENABLE, gpll0 | 1u, 0) ||
		    !put(&gcc, "pll votes", GCC_PLL_VOTE_REG,
			get(&gcc, "pll votes", GCC_PLL_VOTE_REG) | GCC_PLL_VOTE_BITS, 0) ||
		    !wait_bits(&gcc, GCC_GPLL0_MODE, GPLL_LOCK_DET, GPLL_LOCK_DET, 50))) {
			printf("camera-s: 2: GPLL0 never locks (%#x)\n", get(&gcc, "gpll0 mode", GCC_GPLL0_MODE));
		} else {
			printf("camera-s: 2: GPLL0 locked\n");
		}
		/* The GDSC's own clock next (its DT clocks entry). A hyp-gated clock is a warning,
		 * not a stop: poll the CFG below to see whether the switch needed it. */
		uint32_t cbcr = get(&gcc, "camera ahb", GCC_CAM_AHB_CBCR);

		if ((cbcr & CBCR_OFF) &&
		    (!put(&gcc, "camera ahb", GCC_CAM_AHB_CBCR, cbcr | CBCR_ENABLE, 0) ||
		    !wait_bits(&gcc, GCC_CAM_AHB_CBCR, CBCR_OFF, 0, 100))) {
			printf("camera-s: 2: the camera AHB clock does not come on (stays %#x): trying the GDSC anyway\n",
			    get(&gcc, "camera ahb", GCC_CAM_AHB_CBCR));
		}
	}
	if (!put(&camcc, "titan top GDSC", TITAN_TOP_GDSCR, found & ~GDSC_COLLAPSE, GDSC_COLLAPSE)) {
		(void)mxc_parent(false);
		return false;
	}
	gdsc_switched = true;
	/* Upstream polls the CFG register (bit 16), not GDSCR bit 31, for up to 2 ms
	 * (gdsc.c STATUS_POLL_TIMEOUT_US); poll longer here, a slow start costs nothing. */
	if (!wait_bits(&camcc, TITAN_TOP_CFG, CFG_UP_COMPLETE, CFG_UP_COMPLETE, 10000)) {
		found = get(&camcc, "titan top GDSC", TITAN_TOP_GDSCR);
		cfg = get(&camcc, "titan top CFG", TITAN_TOP_CFG);
		printf("camera-s: the titan top power domain does not come on (GDSC %#x CFG %#x)\n", found, cfg);
		give_up("a power domain that does not come on");
		return false;
	}
	/* The retain bit only after the domain is on (gdsc.c gdsc_enable). A failure here
	 * still unwinds: gdsc_switched is set, so power_down switches the domain off again. */
	found = get(&camcc, "titan top GDSC", TITAN_TOP_GDSCR);
	if (!(found & GDSC_RETAIN_FF) &&
	    !put(&camcc, "titan top GDSC", TITAN_TOP_GDSCR, found | GDSC_RETAIN_FF, GDSC_RETAIN_FF)) {
		return false;
	}
	found = get(&camcc, "titan top GDSC", TITAN_TOP_GDSCR);
	cfg = get(&camcc, "titan top CFG", TITAN_TOP_CFG);
	printf("camera-s: the titan top power domain on (GDSC %#x CFG %#x)\n", found, cfg);
	return true;
}

static void
gdsc_off(void)
{
	if (!gdsc_switched) {
		return;
	}
	*(volatile uint32_t *)(camcc.base + TITAN_TOP_GDSCR) |= GDSC_COLLAPSE;
	(void)mxc_parent(false);
	gdsc_switched = false;
	printf("camera-s: the titan top power domain off\n");
}

static bool
mclk_on(unsigned n, uint32_t khz)
{
	if (khz != 19200) {
		give_up("an MCLK rate not known here");
		return false;
	}
	found_mclk_cfg = get(&camcc, "MCLK RCG cfg", MCLK_CFG(n));
	found_mclk_cbcr = get(&camcc, "MCLK branch", MCLK_CBCR(n));
	if (found_mclk_cbcr & CBCR_ENABLE) {
		/* somebody else's (the front camera's always-on MCLK4): use as is, leave at the end */
		printf("camera-s: MCLK%u on already: left as it is\n", n);
		return true;
	}
	mclk_switched = true;
	if (!put(&camcc, "MCLK RCG cfg", MCLK_CFG(n), (found_mclk_cfg & ~0x371fu) | 0u << 8 | 0u, 0x371f) ||
	    !put(&camcc, "MCLK RCG cmd", MCLK_CMD(n),
	    get(&camcc, "MCLK RCG cmd", MCLK_CMD(n)) | CMD_UPDATE, 0)) {
		return false;
	}
	if (!wait_bits(&camcc, MCLK_CMD(n), CMD_UPDATE, 0, 500)) {
		printf("camera-s: MCLK%u's RCG does not take its configuration\n", n);
		give_up("an RCG that does not update");
		return false;
	}
	if (!put(&camcc, "MCLK branch", MCLK_CBCR(n), found_mclk_cbcr | CBCR_ENABLE, CBCR_ENABLE)) {
		return false;
	}
	if (!wait_bits(&camcc, MCLK_CBCR(n), CBCR_OFF, 0, 500)) {
		printf("camera-s: MCLK%u's branch does not come on\n", n);
		give_up("a clock that does not come on");
		return false;
	}
	found_mclk_pin = pin_rd_cfg(hw->mclk_pin);
	*(volatile uint32_t *)GPIO_CFG(hw->mclk_pin) = CFG_FUNCTION(FUNC_CAM_MCLK) | CFG_DRIVE_MA(2);
	printf("camera-s: MCLK%u on at 19.2 MHz (gpio%u function cam_mclk)\n", n, hw->mclk_pin);
	return true;
}

static void
mclk_off(unsigned n)
{
	if (!mclk_switched) {
		return;
	}
	*(volatile uint32_t *)GPIO_CFG(hw->mclk_pin) = found_mclk_pin;
	*(volatile uint32_t *)(camcc.base + MCLK_CBCR(n)) &= ~CBCR_ENABLE;
	(void)wait_bits(&camcc, MCLK_CBCR(n), CBCR_OFF, CBCR_OFF, 500);
	*(volatile uint32_t *)(camcc.base + MCLK_CFG(n)) = found_mclk_cfg;
	*(volatile uint32_t *)(camcc.base + MCLK_CMD(n)) |= CMD_UPDATE;
	(void)wait_bits(&camcc, MCLK_CMD(n), CMD_UPDATE, 0, 500);
	mclk_switched = false;
	printf("camera-s: MCLK%u off (RCG back as found)\n", n);
}

/* ---- the sensor's tables (RWCTAB02, made by tools/camsensor.py) ---------------------------------- */

struct ctab_head {
	char magic[8];          /* "RWCTAB02" */
	uint32_t slave, address_bytes, data_bytes, id_address, id, id_mask, lane_assign, modes;
};
struct ctab_mode {
	uint32_t width, height, x, y, lanes, cphy, settle, pixel_clock, line_length, frame_length, fps_milli, data_type;
};
struct ctab_entry {
	uint16_t address, data;  /* (a power sequence's: kind, value -- MCLK in kHz, BOB2 in mV) */
	uint8_t address_bytes, data_bytes, operation, zero;
	uint32_t delay_us;
};

static const struct ctab_head *ctab;
static const struct ctab_mode *ctab_modes;
static const uint8_t *ctab_power, *ctab_tables;
static uint64_t ctab_size;

static bool
ctab_load(const char *tab)
{
	const uint8_t *at, *end;

	ctab = (const struct ctab_head *)hakios_image_file("fw", tab, &ctab_size, NULL);
	if (ctab == NULL || ctab_size < sizeof(*ctab) || memcmp(ctab->magic, "RWCTAB02", 8) != 0 || ctab->modes == 0 ||
	    ctab->modes > 64) {
		printf("camera-s: no %s (RWCTAB02) in fw/ of the small image (made from fw-camera/)\n", tab);
		ctab = NULL;
		return false;
	}
	end = (const uint8_t *)ctab + ctab_size;
	ctab_modes = (const struct ctab_mode *)(ctab + 1);
	ctab_power = (const uint8_t *)(ctab_modes + ctab->modes);
	at = ctab_power;
	for (int p = 0; p < 2; p++) {
		if (at + 12 > end) {
			ctab = NULL;
			return false;
		}
		at += 12 + *(const uint32_t *)(at + 8) * sizeof(struct ctab_entry);
	}
	ctab_tables = at;
	if (ctab_tables + 4 > end) {
		ctab = NULL;
		return false;
	}
	printf("camera-s: %s: slave %#x, chip ID %#x at %#x, lanes %#x, %u modes, %u tables\n", tab, ctab->slave,
	    ctab->id, ctab->id_address, ctab->lane_assign, ctab->modes, *(const uint32_t *)ctab_tables);
	if (ctab->slave != hw->slave || (ctab->id & ctab->id_mask) != (hw->id & ctab->id_mask)) {
		printf("camera-s: the table is not the %s sensor's (slave %#x, ID %#x)\n", hw->name, hw->slave, hw->id);
		ctab = NULL;
		return false;
	}
	return true;
}

static const struct ctab_entry *
ctab_block(const uint8_t *at, uint32_t blocks, const char *name, uint32_t *count)
{
	const uint8_t *end = (const uint8_t *)ctab + ctab_size;

	for (uint32_t t = 0; t < blocks && at + 12 <= end; t++) {
		uint32_t n = *(const uint32_t *)(at + 8);

		if (strncmp((const char *)at, name, 8) == 0 && at + 12 + n * sizeof(struct ctab_entry) <= end) {
			*count = n;
			return (const struct ctab_entry *)(at + 12);
		}
		at += 12 + n * sizeof(struct ctab_entry);
	}
	*count = 0;
	return NULL;
}

/* ---- power (the module file's sequences) and the chip ID -------------------------------------------- */

static uint8_t
sensor_address(void)
{
	return (uint8_t)(ctab->slave >> 1);     /* (the module file's is the 8-bit one) */
}

static bool powered, reset_driven;
static uint32_t found_reset_cfg, found_reset_io, found_gpio_cfg[3], found_gpio_io[3];
static bool gpio_switched[3];

static void
sensor_delay(uint32_t us)
{
	if (us >= 1000) {
		delay_for_interval(us / 1000, NSEC_PER_MSEC);
		us %= 1000;
	}
	delay_for_interval(us, NSEC_PER_USEC);
}

/* A GPIO-switched fixed regulator (the sensors' VANA): the pin a high output. */
static bool
gpio_supply_on(const struct supply *s, unsigned which)
{
	if (which >= 3 || s->kind != SUPPLY_GPIO) {
		give_up("a supply this sensor does not have");
		return false;
	}
	found_gpio_cfg[which] = pin_rd_cfg(s->gpio);
	found_gpio_io[which] = pin_rd_io(s->gpio);
	gpio_switched[which] = true;
	*(volatile uint32_t *)GPIO_IO(s->gpio) = 2;
	*(volatile uint32_t *)GPIO_CFG(s->gpio) = CFG_OUTPUT | CFG_DRIVE_MA(2);
	if ((pin_rd_cfg(s->gpio) & 0x3ff) != (CFG_OUTPUT | CFG_DRIVE_MA(2)) || !(pin_rd_io(s->gpio) & 2)) {
		give_up("a supply's pin that does not read back");
		return false;
	}
	sensor_delay(100);      /* the fixed regulators' enable-ramp-delay */
	printf("camera-s: gpio%u high: the fixed %u.%u V regulator on\n", s->gpio, s->mv / 1000, s->mv % 1000 / 100);
	return true;
}

static void
gpio_supply_off(const struct supply *s, unsigned which)
{
	if (which >= 3 || !gpio_switched[which]) {
		return;
	}
	*(volatile uint32_t *)GPIO_IO(s->gpio) = 0;
	delay_for_interval(10, NSEC_PER_USEC);
	*(volatile uint32_t *)GPIO_IO(s->gpio) = found_gpio_io[which] & 2;
	*(volatile uint32_t *)GPIO_CFG(s->gpio) = found_gpio_cfg[which];
	gpio_switched[which] = false;
	printf("camera-s: gpio%u back as it was found: the fixed regulator off\n", s->gpio);
}

/* One step of the module file's power sequence (`up': part of the power-up, where a fault stops it). */
static bool
power_step(const struct ctab_entry *e, bool up)
{
	const struct supply *s = e->address == 1 ? &hw->vana : e->address == 2 ? &hw->vdig :
	    e->address == 3 ? &hw->vio : e->address == 4 ? &hw->vaf : e->address == 6 ? &hw->vcustom : NULL;

	switch (e->address) {
	case 8:
		found_reset_cfg = pin_rd_cfg(hw->reset);
		found_reset_io = pin_rd_io(hw->reset);
		reset_driven = true;
		*(volatile uint32_t *)GPIO_IO(hw->reset) = e->data != 0 ? 2 : 0;
		*(volatile uint32_t *)GPIO_CFG(hw->reset) = CFG_OUTPUT | CFG_DRIVE_MA(2);
		break;
	case 0:
		if (e->data != 0 && e->data != 19200) {
			give_up("an MCLK rate not known here");
			return false;
		}
		if (e->data != 0) {
			if (!up || !gdsc_on() || !mclk_on(hw->mclk, e->data)) {
				return false;
			}
		} else {
			mclk_off(hw->mclk);
		}
		break;
	case 1: case 2: case 3: case 4: case 6: case 14: {
		const char *resource = NULL;
		uint32_t mv = 0;
		bool gpio = false;
		unsigned which = 0;

		if (e->address == 14) {
			resource = "bobb2";     /* the boost rail, in mV from the module file */
			mv = e->data;
		} else {
			if (s == NULL || (s->kind != SUPPLY_VRM && s->kind != SUPPLY_GPIO) ||
			    (s->kind == SUPPLY_VRM && s->vrm == NULL)) {
				give_up("a supply the device tree does not give this sensor");
				return false;
			}
			if (s->kind == SUPPLY_VRM) {
				resource = s->vrm;
				mv = s->mv;
			} else {
				gpio = true;
				which = e->address == 1 ? 0 : e->address == 2 ? 1 : 2;
			}
		}
		if (e->data != 0) {
			if (!up) {
				break;
			}
			if (gpio) {
				if (!gpio_supply_on(s, which)) {
					return false;
				}
			} else if (!vrm_vote(resource, mv, true, false)) {
				return false;
			}
		} else {
			if (gpio) {
				gpio_supply_off(s, which);
			} else {
				(void)vrm_vote(resource, 0, false, true);
			}
		}
		break;
	}
	default:
		printf("camera-s: a power step of kind %u, which this does not know\n", e->address);
		give_up("a power sequence not understood");
		return false;
	}
	sensor_delay(e->delay_us > 100000 ? 100000 : e->delay_us);
	return true;
}

/* Everything that was switched on, off again: the module file's power-down, then the rest as found. */
static void
power_down(void)
{
	bool was = powered || mclk_switched || gdsc_switched || reset_driven || vrm_on_n != 0 ||
	    gpio_switched[0] || gpio_switched[1] || gpio_switched[2];
	uint32_t count = 0;
	const struct ctab_entry *e = ctab != NULL ? ctab_block(ctab_power, 2, "pwrdn", &count) : NULL;

	if (!was) {
		return;
	}
	printf("camera-s: powering down\n");
	for (uint32_t i = 0; e != NULL && i < count; i++) {
		(void)power_step(&e[i], false);
	}
	mclk_off(hw->mclk);
	if (reset_driven) {
		*(volatile uint32_t *)GPIO_IO(hw->reset) = found_reset_io & 2;
		*(volatile uint32_t *)GPIO_CFG(hw->reset) = found_reset_cfg;
		reset_driven = false;
	}
	gpio_supply_off(&hw->vana, 0);
	gpio_supply_off(&hw->vdig, 1);
	gpio_supply_off(&hw->vio, 2);
	for (unsigned i = 0; i < vrm_on_n; i++) {
		uint32_t addresses[1] = { vrm_on_addr[i] + VRM_ENABLE }, values[1] = { 0 };

		(void)rpmh_write(1, addresses, values, true);
	}
	if (vrm_on_n != 0) {
		printf("camera-s: %u RPMh rail vote(s) given back\n", vrm_on_n);
		vrm_on_n = 0;
	}
	gdsc_off();
	powered = false;
	printf("camera-s: powered down\n");
}

static bool
power_up(void)
{
	uint32_t count = 0;
	const struct ctab_entry *e = ctab != NULL ? ctab_block(ctab_power, 2, "pwrup", &count) : NULL;

	if (e == NULL) {
		give_up("no power sequence");
		return false;
	}
	powered = true;
	for (uint32_t i = 0; i < count; i++) {
		if (!power_step(&e[i], true)) {
			return false;
		}
	}
	printf("camera-s: the %s sensor powered up (%u steps)\n", hw->name, count);
	return true;
}

static bool
read_chip_id(void)
{
	uint8_t where[2] = { (uint8_t)(ctab->id_address >> 8), (uint8_t)ctab->id_address }, id[4] = { 0, 0, 0, 0 };
	unsigned out = ctab->address_bytes == 2 ? 2 : 1, in = ctab->data_bytes >= 1 && ctab->data_bytes <= 4 ?
	    ctab->data_bytes : 2;
	uint32_t chip = 0;
	unsigned go;

	for (go = 0; go < 20; go++) {
		if (i2c_transfer(&cci_bus, sensor_address(), out == 2 ? where : where + 1, out, id, in)) {
			break;
		}
		delay_for_interval(2, NSEC_PER_MSEC);
	}
	if (go == 20) {
		printf("camera-s: the %s sensor does not answer at %#04x\n", hw->name, sensor_address());
		return false;
	}
	if (go != 0) {
		printf("camera-s: the %s sensor answered at %#04x after %u more goes\n", hw->name, sensor_address(), go);
	}
	for (unsigned i = 0; i < in; i++) {
		chip = chip << 8 | id[i];
	}
	printf("camera-s: the %s sensor's chip ID: %#06x (%s)\n", hw->name, chip,
	    (chip & ctab->id_mask) == (ctab->id & ctab->id_mask) ? "as the module file expects" :
	    "NOT what the module file expects");
	return (chip & ctab->id_mask) == (ctab->id & ctab->id_mask);
}

/* ---- cam=1: a look, reading only ------------------------------------------------------------------ */

static void
look(void)
{
	show_pin(hw->reset, "the sensor's reset");
	show_pin(hw->mclk_pin, "the sensor's MCLK");
	show_pin(hw->sda, "the sensor's data");
	show_pin(hw->scl, "the sensor's clock");
	if (hw->vana.kind == SUPPLY_GPIO) {
		show_pin(hw->vana.gpio, "the sensor's VANA switch");
	}
}

/* ---- the thread ----------------------------------------------------------------------------------- */

static void
select_sensor(unsigned i)
{
	hw = &sensors[i];
	cci_bus.sda = hw->sda;
	cci_bus.scl = hw->scl;
	ctab = NULL;
}

static void
camera_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t secs = 20;
	char which[16] = "";

	(void)PE_parse_boot_argn("cam_after", &secs, sizeof(secs));
	select_sensor(0);       /* the wide sensor; cam_sensor= picks another */
	if (PE_parse_boot_arg_str("cam_sensor", which, sizeof(which))) {
		for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
			if (strncmp(which, sensors[i].name, strlen(which)) == 0) {
				select_sensor(i);
			}
		}
	}
	delay_for_interval(secs, NSEC_PER_SEC);     /* (the system up first: nothing else is on these buses then) */
	if (level == 1) {
		step = 1;
		announce("a look, reading only: the sensor's pins");
		look();
		step = 2;
		announce("the sensor's tables from fw/ of the small image");
		for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
			select_sensor(i);
			if (ctab_load(hw->tab) || (hw->tab2 != NULL && ctab_load(hw->tab2))) {
				uint32_t n = *(const uint32_t *)ctab_tables;

				printf("camera-s: the %s sensor: %u tables\n", hw->name, n);
			}
		}
		printf("camera-s: done\n");
		thread_terminate(current_thread());
	}
	/* cam=2: powered up, its chip ID read, powered down */
	step = 1;
	announce("load the sensor's tables");
	if (!ctab_load(hw->tab) && (hw->tab2 == NULL || !ctab_load(hw->tab2))) {
		power_down();
		printf("camera-s: done\n");
		thread_terminate(current_thread());
	}
	step = 2;
	announce("power the sensor up as its module file powers it");
	if (!power_up()) {
		power_down();
		printf("camera-s: done\n");
		thread_terminate(current_thread());
	}
	step = 3;
	announce("read the sensor's chip ID");
	(void)read_chip_id();
	step = 4;
	announce("power the sensor down again");
	power_down();
	printf("camera-s: done\n");
	thread_terminate(current_thread());
}

void
hakios_camera_init(void)
{
	thread_t thread;

	if (!PE_parse_boot_argn("cam", &level, sizeof(level)) || level == 0) {
		return;
	}
	level = level > 2 ? 2 : level;
	if (kernel_thread_start(camera_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

void
hakios_camera_quiesce(void)
{
	if (level < 2) {
		return;
	}
	broken = false;
	power_down();
}
