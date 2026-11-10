/*
 * The battery's charge and its charging on the Xiaomi 14 Pro (shennong, SM8650 "pineapple"),
 * without the audio DSP. Only with boot-arg fg=1 (the gauge) or chg=1 (the charger);
 * every address below is this phone's.
 *
 * Why this file. The DSP that runs the battery and charger management (drivers/audio/
 * hakios.adsp.c's battmgr, over PMIC_GLINK) stays silent on this phone (see devices/shennong/
 * device.conf, audio row: no SMP2P, no doorbell after AUTH), so the battery cannot be asked
 * through it. The loader's cache (drivers/power/hakios.battery.c, battery=1) works but only
 * while the UEFI charger drivers answer on core 0. What is always there is the hardware:
 * the PMIC voltage ADC (PMK8550 vadc@9000, slave 0) and the charger core (PM8550B, slave 7).
 *
 * What it does.
 *   fg=1    read the battery through the voltage ADC: its voltage (pm8550b_vbat_sns_qbg),
 *           its temperature (pm8550_msm_therm, the pack NTC), the charge/input currents
 *           (pm8550b_ichg_fb, pm8550b_iin_fb). The charge in percent comes from the voltage
 *           against the float voltage (read from the charger when it answers, else a
 *           default), and is marked estimated. Every round goes to Darwin the same way the
 *           DSP's readings do (hakios_battery_report() -> HakiosBattery -> powerd -> the
 *           status bar, lock screen, Settings > Battery), and to sysctl kern.hakios.chg_*.
 *           With battery=1 it stays out: the loader's cache (a real fuel gauge) wins.
 *   chg=0   (the default) watch the charger only: plug detection and the status the
 *           hardware reports, logged; nothing is written to any PMIC.
 *   chg=1   ... and manage charging: enable the charger, set the fast-charge current and
 *           the float voltage, the input-current limit from the charger kind (APSD), and
 *           lower the current when the battery or the SoC gets hot. Writes go only to four
 *           registers (the allow-list below) and only after the charger block proves what
 *           it is (TYPE/SUBTYPE against boot-arg chg_type, read on a read-only run first);
 *           every write is read back like drivers/camera/hakios.torch_shennong.c does.
 *
 * Register layout.
 *   ADC: Linux drivers/iio/adc/qcom-spmi-adc5-gen3.c and include/linux/iio/adc/
 *   qcom-adc5-gen3-common.h (offsets below), drivers/iio/adc/qcom-vadc-common.c
 *   (voltage = code * 1.875 V * prescale / 0x70e4; temperature from the voltage
 *   over the 100 k NTC table). Channels and scaling are the phone's own device tree
 *   (devices/shennong/prebuilt/device-tree.dtb, vadc@9000 under pmk8550@0):
 *   pm8550b_vbat_sns_qbg reg 0x78f (sid 7, chan 0x8f, 1/3), pm8550_msm_therm 0x144
 *   (sid 1, chan 0x44, ratiometric), pm8550b_iin_fb 0x717 (sid 7, chan 0x17),
 *   pm8550b_ichg_fb 0x7a1 (sid 7, chan 0xa1). The ADC lives at slave 0, 0x9000
 *   (PPID 0x090); single conversions on its channel 0, polled (this kernel takes no
 *   PMIC interrupt, like drivers/input/hakios.buttons.c). The only writes are the
 *   conversion's own control registers (0x4f..0x55, 0x4b, 0xe5); no supply is touched.
 *   Charger: the SMB core at slave 7, 0x1000 (PPID 0x710), as upstream Linux
 *   drivers/power/supply/qcom_smbx.c drives it (offsets from the charger base):
 *   status 0x06/0x07/0x0d, CHARGING_ENABLE_CMD 0x42, FCC 0x61 (25 mA/LSB),
 *   FV 0x70 (3487500 uV + 7500 uV/LSB), APSD 0x307/0x308, plugin/POWER_PATH
 *   0x310/0x60b, ICL 0x370. The Type-C port (0x1500) and the PD PHY (0x1700) are
 *   TCPM's and are never touched (cf. the SMB5 discussion on lkml, Oct 2026).
 *
 * Thermal. The hardware keeps its own JEITA protection (STATUS_2, read only here).
 * On top, when this driver manages charging, it lowers the fast-charge current from
 * the battery NTC above chg_hot (default 45 C) to half, stops above chg_max (50 C),
 * and drops to 500 mA when the SoC itself (kern.hakios.thermal_max_mc, read the way
 * drivers/power/hakios.battguard.c reads sysctls) reaches chg_soc_hot (95 C).
 *
 * No RPMh vote is cast here on purpose: the charger runs by itself in the PMIC once
 * enabled, and the ADC answers over SPMI; a vote would only fight the firmware that
 * owns the rails and the bus clocks. If one is ever needed, the pattern is
 * drivers/modem/hakios.mpss.c's rpmh_tcs_pick() (TCS scan, standing slot: TCS 0 is
 * somebody else's, TCS 3 raises no completion interrupt) with drivers/input/
 * hakios.synaptics.c's v3 layout and hakios_rsc_lock.
 *
 * Status: COMPILED ONLY. The ADC offsets and formulas are Linux's, the channels the
 * phone's own tree's, the charger registers upstream's SMB map; none has yet run on
 * this phone. First run read-only (fg=1, no chg): compare chg_vbat_mv/chg_temp_c with
 * Android (dumpsys battery) and note the charger's TYPE/SUBTYPE for chg_type.
 *
 * Boot-args:
 *   fg=1             read the gauge and report it to Darwin (needs neither adsp nor battery)
 *   fg_poll=<s>      how often (5..600; 10)
 *   chg=1            ... and manage charging (needs fg's probe; chg_type below must match)
 *   chg_type=0xTTSS  the charger block's TYPE/SUBTYPE as the read-only run printed;
 *                    no write unless the probe reads exactly this (default 0: never write)
 *   chg_fcc_ma=<mA>  fast-charge current (default 1000, the SMB2 driver's cautious cap)
 *   chg_fv_mv=<mV>   float voltage (default: the hardware's own setting, left as read)
 *   chg_icl_ma=<mA>  input-current limit (default: from APSD: SDP 500, CDP/DCP 1500)
 *   chg_hot=<C>      halve FCC above this battery temperature (45)
 *   chg_max=<C>      stop above this battery temperature (50)
 *   chg_soc_hot=<C>  FCC to 500 mA above this SoC temperature (95)
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern kern_return_t thread_terminate(thread_t thread);
extern void loader_log_putc(char c);
extern int kernel_sysctlbyname(const char *name, void *oldp, size_t *oldlenp, void *newp, size_t len);

SYSCTL_DECL(_kern_hakios);

static int
say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}

/* ---- our numbers (sysctl kern.hakios.chg_*) ------------------------------------------------------ */

static int chg_vbat_mv = -1;            /* battery voltage from pm8550b_vbat_sns_qbg, mV; -1: not read */
static int chg_temp_c = -273;           /* battery (pack NTC) temperature, C; -273: not read */
static int chg_soc = -1;                /* charge from voltage, 0..100; -1: not known (always estimated) */
static int chg_ichg_raw = -1, chg_iin_raw = -1; /* raw ADC codes of ichg/iin feedback; -1: not read */
static int chg_plugin;                  /* 1: a charger is on the USB port (hardware or heuristic) */
static int chg_status;                  /* Linux POWER_SUPPLY_STATUS_*: 1 charging, 2 discharging, 3 not charging, 4 full */
static int chg_fcc_ma;                  /* fast-charge current now asked for, mA */
static int chg_fv_mv = -1;              /* float voltage (hardware's, unless chg_fv_mv given), mV */
static int chg_manage;                  /* 1: chg=1 and the charger probe matched: writes allowed */
static int chg_writes;                  /* SPMI writes to the charger so far */
static int chg_from_vadc;               /* 1: Darwin's battery numbers come from this driver */

SYSCTL_INT(_kern_hakios, OID_AUTO, chg_vbat_mv, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_vbat_mv, 0, "battery voltage from the PMIC voltage ADC (pm8550b_vbat_sns_qbg), in mV");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_temp_c, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_temp_c, 0, "battery pack temperature from the PMIC voltage ADC (pm8550_msm_therm), in C");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_soc, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_soc, 0, "charge from voltage against the float voltage, 0..100 percent, always estimated (-1 unknown)");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_ichg_raw, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_ichg_raw, 0, "raw ADC code of pm8550b_ichg_fb (charge-current feedback)");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_iin_raw, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_iin_raw, 0, "raw ADC code of pm8550b_iin_fb (input-current feedback)");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_plugin, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_plugin, 0, "1 when a charger is on the USB port");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_status, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_status, 0, "Linux power-supply status: 1 charging, 2 discharging, 3 not charging, 4 full");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_fcc_ma, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_fcc_ma, 0, "fast-charge current now asked for, in mA");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_fv_mv, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_fv_mv, 0, "float voltage, in mV (-1: not read)");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_manage, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_manage, 0, "1 when chg=1 and the charger probe matched, so writes are allowed");
SYSCTL_INT(_kern_hakios, OID_AUTO, chg_writes, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_writes, 0, "SPMI writes to the charger block so far");
SYSCTL_INT(_kern_hakios, OID_AUTO, battery_from_vadc, CTLFLAG_RD | CTLFLAG_LOCKED,
    &chg_from_vadc, 0, "1 when Darwin's battery numbers come from the PMIC voltage ADC (not ADSP)");

/* ---- the voltage ADC (ADC5 Gen3 on PMK8550, slave 0, 0x9000 = PPID 0x090) -------------------------
 * Linux drivers/iio/adc/qcom-spmi-adc5-gen3.c + include/linux/iio/adc/qcom-adc5-gen3-common.h.
 * One conversion on the ADC's channel 0, polled: wait for the handshake (HS READY),
 * write the 7 configuration bytes at SID, write CONV_REQ, wait for EOC on channel 0,
 * read DATA0/1, clear EOC. The only writes are the conversion's own control registers. */

#define ADC_PPID                0x090
#define ADC_HS                  0x45
#define ADC_HS_READY            0x01
#define ADC_STATUS1             0x46
#define ADC_EOC_STS             0x4a
#define ADC_EOC_CHAN0           0x01
#define ADC_EOC_CLR             0x4b
#define ADC_SID                 0x4f
#define ADC_PERPH_CH            0x50
#define ADC_CHAN_CONV_REQ       0x80
#define ADC_TIMER_SEL           0x51
#define ADC_TIME_IMMEDIATE      0x01
#define ADC_DIG_PARAM           0x52
#define ADC_CAL_ABSOLUTE        2
#define ADC_CAL_RATIOMETRIC     1
#define ADC_DECIMATION_DEFAULT  2               /* index into { 85, 340, 1360 } */
#define ADC_FAST_AVG            0x53
#define ADC_FAST_AVG_EN         0x80
#define ADC_CH_SEL              0x54
#define ADC_DELAY_CTL           0x55
#define ADC_DATA0               0x5c
#define ADC_CONV_REQ            0xe5
#define ADC_CONV_REQ_BIT        0x01
#define ADC_DATA_CHECK          0x8000
#define ADC_FULL_SCALE          0x70e4          /* codes for 1.875 V (qcom-vadc-common.h) */
#define ADC_VREF_UV             1875000

/* This phone's channels (vadc@9000 in its device tree): sid, channel, prescale num/den, calibration. */
struct adc_channel {
	const char *name;
	uint8_t sid, chan;
	uint8_t pre_num, pre_den, cal;
};
static const struct adc_channel CH_VBAT = { "pm8550b_vbat_sns_qbg", 7, 0x8f, 1, 3, ADC_CAL_ABSOLUTE };
static const struct adc_channel CH_MSM_THERM = { "pm8550_msm_therm", 1, 0x44, 1, 1, ADC_CAL_RATIOMETRIC };
static const struct adc_channel CH_IIN_FB = { "pm8550b_iin_fb", 7, 0x17, 1, 1, ADC_CAL_ABSOLUTE };
static const struct adc_channel CH_ICHG_FB = { "pm8550b_ichg_fb", 7, 0xa1, 1, 1, ADC_CAL_ABSOLUTE };

static uint16_t adc_apid = 0xffff;
static uint64_t adc_conversions, adc_failures;

static bool
adc_wait(uint8_t reg, uint8_t mask, uint8_t value, unsigned tries)
{
	for (unsigned i = 0; i < tries; i++) {
		uint8_t b = 0;

		if (hakios_spmi_read(adc_apid, reg, &b, 1) == NULL && (b & mask) == value) {
			return true;
		}
		delay_for_interval(200, NSEC_PER_USEC);
	}
	return false;
}

/* One conversion of that channel; the raw code, or -1. Single caller at a time. */
static int
adc_read(const struct adc_channel *ch)
{
	uint8_t cfg[7], data[2], one = ADC_CONV_REQ_BIT, zero = 0;
	const char *problem;

	if (!adc_wait(ADC_HS, 0xff, ADC_HS_READY, 75)) {         /* up to 15 ms, as Linux waits */
		return -1;
	}
	{
		/* no conversion in flight (Linux's poll_wait_hs also waits for CONV_REQ == 0) */
		uint8_t req = 0xff;

		for (unsigned i = 0; i < 50; i++) {
			if (hakios_spmi_read(adc_apid, ADC_CONV_REQ, &req, 1) == NULL && req == 0) {
				break;
			}
			req = 0xff;
			delay_for_interval(200, NSEC_PER_USEC);
		}
		if (req != 0) {
			return -1;
		}
	}
	cfg[0] = ch->sid & 0xf;
	cfg[1] = ADC_CHAN_CONV_REQ | 0;
	cfg[2] = ADC_TIME_IMMEDIATE;
	cfg[3] = (uint8_t)((ch->cal & 3) << 4 | (ADC_DECIMATION_DEFAULT & 3) << 2);
	cfg[4] = ADC_FAST_AVG_EN;                               /* one sample, averaged once */
	cfg[5] = ch->chan;
	cfg[6] = 0;                                             /* 15 us settle (the tree names none here) */
	if ((problem = hakios_spmi_write(adc_apid, ADC_SID, cfg, 7)) != NULL ||
	    (problem = hakios_spmi_write(adc_apid, ADC_CONV_REQ, &one, 1)) != NULL) {
		say("chg-s: ADC %s: setup: %s\n", ch->name, problem);
		return -1;
	}
	if (!adc_wait(ADC_EOC_STS, ADC_EOC_CHAN0, ADC_EOC_CHAN0, 500)) {
		say("chg-s: ADC %s: no end of conversion\n", ch->name);
		return -1;
	}
	if ((problem = hakios_spmi_read(adc_apid, ADC_DATA0, data, 2)) != NULL) {
		say("chg-s: ADC %s: data: %s\n", ch->name, problem);
		return -1;
	}
	one = ADC_EOC_CHAN0;
	(void)hakios_spmi_write(adc_apid, ADC_EOC_CLR, &one, 1); /* clear EOC, then the request, as Linux does */
	(void)hakios_spmi_write(adc_apid, ADC_PERPH_CH, &zero, 1);
	one = ADC_CONV_REQ_BIT;
	(void)hakios_spmi_write(adc_apid, ADC_CONV_REQ, &one, 1);
	adc_conversions++;
	{
		int code = data[0] | data[1] << 8;

		if (code == ADC_DATA_CHECK || code > 0x7fff) {
			adc_failures++;
			return -1;
		}
		return code;
	}
}

/* code -> uV (Linux qcom_vadc_scale_code_voltage_factor, factor 1). 64 bits: code * 1.875 V * den overflows 32. */
static int64_t
adc_uv(int code, uint8_t num, uint8_t den)
{
	return (int64_t)code * ADC_VREF_UV * den / ADC_FULL_SCALE / num;
}

/* NTC voltage (mV, 100 k pull-up, PM7 table) -> millidegrees C (Linux adcmap_100k_104ef_104fb_1875_vref). */
static const struct { int mv, mdec; } ntc_table[] = {
	{ 1831, -40000 }, { 1814, -35000 }, { 1791, -30000 }, { 1761, -25000 }, { 1723, -20000 },
	{ 1675, -15000 }, { 1616, -10000 }, { 1545, -5000 }, { 1463, 0 }, { 1370, 5000 },
	{ 1268, 10000 }, { 1160, 15000 }, { 1049, 20000 }, { 937, 25000 }, { 828, 30000 },
	{ 726, 35000 }, { 630, 40000 }, { 544, 45000 }, { 467, 50000 }, { 399, 55000 },
	{ 340, 60000 }, { 290, 65000 }, { 247, 70000 }, { 209, 75000 }, { 179, 80000 },
	{ 153, 85000 }, { 130, 90000 }, { 112, 95000 }, { 96, 100000 }, { 82, 105000 },
	{ 71, 110000 }, { 62, 115000 }, { 53, 120000 }, { 46, 125000 },
};

static int
ntc_mdec(int mv)
{
	unsigned i = 0;

	while (i < sizeof(ntc_table) / sizeof(ntc_table[0]) && ntc_table[i].mv > mv) {
		i++;
	}
	if (i == 0) {
		return ntc_table[0].mdec;
	}
	if (i >= sizeof(ntc_table) / sizeof(ntc_table[0])) {
		return ntc_table[sizeof(ntc_table) / sizeof(ntc_table[0]) - 1].mdec;
	}
	/* linear interpolation between the two bracketing rows */
	return ntc_table[i - 1].mdec + (int)((int64_t)(ntc_table[i].mdec - ntc_table[i - 1].mdec) *
	    (ntc_table[i - 1].mv - mv) / (ntc_table[i - 1].mv - ntc_table[i].mv));
}

/* Voltage (mV) -> percent, 3.40 V empty, the float voltage full (Linux has no such table: the
 * fuel gauge normally owns this; calibrate against dumpsys battery on the first runs). */
static int
soc_of(int mv, int full_mv)
{
	static const struct { int mv, pct; } curve[] = {
		{ 3400, 0 }, { 3500, 5 }, { 3600, 12 }, { 3650, 20 }, { 3700, 30 }, { 3750, 40 },
		{ 3800, 50 }, { 3850, 60 }, { 3900, 68 }, { 3950, 76 }, { 4000, 83 }, { 4050, 89 },
		{ 4100, 93 }, { 4150, 96 }, { 4200, 98 },
	};
	unsigned i = 0;

	if (mv <= curve[0].mv) {
		return 0;
	}
	if (mv >= full_mv) {
		return 100;
	}
	while (i < sizeof(curve) / sizeof(curve[0]) && curve[i].mv < mv) {
		i++;
	}
	if (i >= sizeof(curve) / sizeof(curve[0])) {     /* between the table top and full: stretch */
		return curve[sizeof(curve) / sizeof(curve[0]) - 1].pct +
		    (100 - curve[sizeof(curve) / sizeof(curve[0]) - 1].pct) *
		    (mv - curve[sizeof(curve) / sizeof(curve[0]) - 1].mv) / (full_mv - curve[sizeof(curve) / sizeof(curve[0]) - 1].mv);
	}
	return curve[i - 1].pct + (curve[i].pct - curve[i - 1].pct) * (mv - curve[i - 1].mv) / (curve[i].mv - curve[i - 1].mv);
}

/* ---- the charger core (SMB on PM8550B, slave 7, 0x1000 = PPID 0x710) --------------------------------
 * Upstream Linux drivers/power/supply/qcom_smbx.c (SMB2, same core the SMB5 discussion
 * confirmed at 0x1000 with writes to 0x42/0x61/0x70). Offsets from the charger base. */

#define CHG_PPID                0x710
#define CHG_TYPE_REG            0x04
#define CHG_SUBTYPE_REG         0x05
#define CHG_STATUS_1            0x06            /* [2:0]: 0 trickle, 1 pre, 2 fast, 3 full-on, 4 taper, 5 done, 6 inhibit, 7 off */
#define CHG_STATUS_2            0x07            /* bit 5 battery over-voltage; [3:0] JEITA temperature */
#define CHG_STATUS_7            0x0d            /* bit 7 trickle, 6 pre, 5 fast, 4 full-on running */
#define CHG_ENABLE              0x42            /* bit 0: charging enabled (software) */
#define CHG_FCC                 0x61            /* fast-charge current, 25 mA/LSB */
#define CHG_FV                  0x70            /* float voltage: 3487500 uV + 7500 uV/LSB */
#define CHG_APSD_STATUS         0x307           /* bit 0: detection done */
#define CHG_APSD_RESULT         0x308           /* bit 2 CDP, 3 DCP, 4 float, 0 SDP */
#define CHG_PLUGIN_STS          0x310           /* bit 4: USBIN_PLUGIN (a charger is attached) */
#define CHG_TYPEC_STS4          0x30e           /* bit 0: CC attached */
#define CHG_ICL_CFG             0x370           /* input-current limit, 25 mA/LSB */
#define CHG_POWER_PATH          0x60b           /* bit 4 USBIN in use, bit 0 input valid */
#define CHG_FV_MIN_UV           3487500
#define CHG_FV_STEP_UV          7500
#define CHG_FV_MAX_UV           4920000
#define CHG_CUR_LSB_UA          25000
/* STATUS_1 as Linux POWER_SUPPLY_STATUS_* (drivers/audio/hakios.adsp.c's battmgr uses the same). */
#define ST_CHARGING             1
#define ST_DISCHARGING          2
#define ST_NOT_CHARGING         3
#define ST_FULL                 4

static uint16_t chg_apid = 0xffff;
static uint32_t chg_want_type;                  /* chg_type= boot-arg: TYPE<<8|SUBTYPE that allows writes */

static bool
chg_read(uint8_t reg, uint8_t *v)
{
	const char *problem = hakios_spmi_read(chg_apid, reg, v, 1);

	if (problem != NULL) {
		say("chg-s: charger %#x: %s\n", reg, problem);
		return false;
	}
	return true;
}

/* The one way anything is written to the charger: allow-listed, read back (torch_shennong pattern). */
static bool
chg_write(uint8_t reg, uint8_t v)
{
	static const uint8_t allowed[] = { CHG_ENABLE, CHG_FCC, CHG_FV, CHG_ICL_CFG };
	const char *problem;
	uint8_t back = 0xff;

	for (unsigned i = 0; i < sizeof(allowed); i++) {
		if (allowed[i] == reg) {
			goto ok;
		}
	}
	say("chg-s: charger %#x is not in the write allow-list: refused\n", reg);
	return false;
ok:
	if (!chg_manage) {
		return false;
	}
	problem = hakios_spmi_write(chg_apid, reg, &v, 1);
	if (problem != NULL) {
		say("chg-s: charger write %#x <- %#x: %s\n", reg, v, problem);
		return false;
	}
	if (!chg_read(reg, &back) || back != v) {
		say("chg-s: charger %#x <- %#x, reads back %#x\n", reg, v, back);
		return false;
	}
	chg_writes++;
	return true;
}

static int
chg_fcc_of(uint8_t raw)
{
	return (int)raw * CHG_CUR_LSB_UA / 1000;
}

static uint8_t
chg_fcc_raw(int ma)
{
	int raw = (ma * 1000 + CHG_CUR_LSB_UA - 1) / CHG_CUR_LSB_UA;

	return (uint8_t)(raw < 0 ? 0 : raw > 255 ? 255 : raw);
}

/* ---- thermal, the way drivers/power/hakios.battguard.c asks (no link dependency) --------------------- */

static bool
sysctl_int(const char *name, int *value)
{
	size_t size = sizeof(*value);

	return kernel_sysctlbyname(name, value, &size, NULL, 0) == 0 && size == sizeof(*value);
}

/* ---- configuration ----------------------------------------------------------------------------------- */

static uint32_t want_fg, want_chg, poll_seconds = 10;
static int cfg_fcc_ma = 1000, cfg_fv_mv;        /* 0: leave the hardware's float voltage */
static int cfg_icl_ma;                          /* 0: from APSD */
static int cfg_hot = 45, cfg_max = 50, cfg_soc_hot = 95;

/* ---- one round: gauge, charger, report ----------------------------------------------------------------- */

static void
round(void)
{
	int vbat_code, therm_code, soc = -1, temp_c = -273, vbat_mv = -1;
	int plugin = 0, status = ST_DISCHARGING, usb_type = 0;
	int therm_mv;
	uint8_t s1 = 0x07, s2 = 0, s7 = 0, plug = 0, ppath = 0, apsd = 0, apsd_res = 0, typec = 0;
	bool chg_seen = chg_apid != 0xffff;

	/* the gauge */
	vbat_code = adc_read(&CH_VBAT);
	therm_code = adc_read(&CH_MSM_THERM);
	chg_iin_raw = adc_read(&CH_IIN_FB);
	chg_ichg_raw = adc_read(&CH_ICHG_FB);
	if (vbat_code >= 0) {
		vbat_mv = (int)(adc_uv(vbat_code, 1, 3) / 1000);
	}
	if (therm_code >= 0) {
		therm_mv = (int)(adc_uv(therm_code, 1, 1) / 1000);
		temp_c = ntc_mdec(therm_mv) / 1000;
	}
	if (vbat_mv > 0) {
		soc = soc_of(vbat_mv, chg_fv_mv > 0 ? chg_fv_mv : 4400);
	}
	chg_vbat_mv = vbat_mv;
	chg_temp_c = temp_c;
	chg_soc = soc;

	/* the charger: status first (read-only always) */
	if (chg_seen) {
		bool ok = chg_read(CHG_STATUS_1, &s1) & chg_read(CHG_STATUS_2, &s2) & chg_read(CHG_STATUS_7, &s7);

		ok = chg_read(CHG_PLUGIN_STS, &plug) && ok;
		ok = chg_read(CHG_POWER_PATH, &ppath) && ok;
		(void)chg_read(CHG_APSD_STATUS, &apsd);
		(void)chg_read(CHG_APSD_RESULT, &apsd_res);
		(void)chg_read(CHG_TYPEC_STS4, &typec);
		if (ok) {
			/* attached: the plugin bit, or USBIN in use and valid (upstream smb_get_prop_usb_online) */
			plugin = (plug & 0x10) || ((ppath & 0x10) && (ppath & 0x01)) || (s1 & 0x07) <= 4;
			if (!plugin) {
				status = ST_DISCHARGING;
			} else {
				switch (s1 & 0x07) {
				case 0: case 1: case 2: case 3: case 4:
					status = ST_CHARGING;
					break;
				case 5:
					status = ST_FULL;
					break;
				default:
					status = ST_NOT_CHARGING;
					break;
				}
			}
			if (apsd & 0x01) {
				if (apsd_res & 0x04) {
					usb_type = 3;       /* CDP */
				} else if (apsd_res & (0x08 | 0x02 | 0x10)) {
					usb_type = 2;       /* DCP (incl. OCP/float) */
				} else {
					usb_type = 1;       /* SDP */
				}
			}
		}
	}
	/* without the charger block, the currents still tell attachment: charge current above idle */
	if (!chg_seen && chg_ichg_raw > 0) {
		static int idle_ichg = -1;

		if (idle_ichg < 0) {
			idle_ichg = chg_ichg_raw;
		}
		if (chg_ichg_raw > idle_ichg + 50) {
			plugin = 1;
			status = soc >= 100 ? ST_FULL : ST_CHARGING;
		}
	}
	chg_plugin = plugin;
	chg_status = status;

	/* manage: enable, FCC/FV/ICL, thermal derating (writes need chg_manage) */
	if (chg_manage) {
		int fcc = cfg_fcc_ma, icl = cfg_icl_ma;
		uint8_t en = 0;

		if (icl == 0) {
			icl = usb_type == 1 ? 500 : 1500;   /* SDP 500 mA, CDP/DCP 1500 mA (qcom_smbx) */
		}
		if (temp_c != -273) {
			if (temp_c >= cfg_max) {
				fcc = 0;
			} else if (temp_c >= cfg_hot) {
				fcc /= 2;
			}
		}
		{
			int soc_mc = 0;

			if (sysctl_int("kern.hakios.thermal_max_mc", &soc_mc) && soc_mc >= cfg_soc_hot * 1000) {
				fcc = fcc < 500 ? fcc : 500;
			}
		}
		if (plugin && fcc > 0 && fcc < cfg_fcc_ma && fcc < 300) {
			fcc = 0;                /* thermal-derated below usable: stop rather than trickle from here */
		}
		chg_fcc_ma = fcc;
		(void)chg_read(CHG_ENABLE, &en);
		if (plugin && fcc > 0 && !(en & 0x01)) {
			chg_write(CHG_ENABLE, en | 0x01);
		} else if ((!plugin || fcc == 0) && (en & 0x01)) {
			chg_write(CHG_ENABLE, en & ~0x01);
		}
		if (fcc > 0) {
			chg_write(CHG_FCC, chg_fcc_raw(fcc));
			chg_write(CHG_ICL_CFG, chg_fcc_raw(icl));
		}
		if (cfg_fv_mv > 0 && chg_fv_mv != cfg_fv_mv) {
			int sel = (cfg_fv_mv * 1000 - CHG_FV_MIN_UV + CHG_FV_STEP_UV - 1) / CHG_FV_STEP_UV;

			if (sel >= 0 && cfg_fv_mv * 1000 >= CHG_FV_MIN_UV && cfg_fv_mv * 1000 <= CHG_FV_MAX_UV) {
				chg_write(CHG_FV, (uint8_t)sel);
				chg_fv_mv = cfg_fv_mv;
			}
		}
	}

	say("chg-s: %d mV%s, pack %d C, SoC ~%d%%%s; charger %s, status %d (s1 %#x s2 %#x s7 %#x apsd %#x/%#x typec %#x)%s; FCC %d mA, FV %d mV (%llu conversions, %llu failed)\n",
	    vbat_mv, vbat_code < 0 ? " (ADC failed)" : "", temp_c, soc, soc < 0 ? "" : " (voltage estimate)",
	    plugin ? "attached" : "off", status, s1, s2, s7, apsd, apsd_res, typec,
	    chg_manage ? "" : " (monitor only)", chg_fcc_ma, chg_fv_mv, adc_conversions, adc_failures);

	/* to Darwin, the way the DSP's readings go (drivers/power/hakios.battery.c) */
	if ((want_fg || want_chg) && soc >= 0) {
		struct hakios_battery_state b;

		bzero(&b, sizeof(b));
		b.size = sizeof(b);
		b.percent_hundredths = soc * 100;
		b.voltage_uv = vbat_mv * 1000;
		b.current_ua = (status == ST_CHARGING && chg_manage) ? chg_fcc_ma * 1000 : 0;
		b.temperature_hundredths = temp_c == -273 ? -27300 : temp_c * 100;
		b.present = 1;
		b.external = plugin;
		b.status = status;
		b.usb_type = usb_type;
		b.usb_voltage_uv = plugin ? 5000000 : 0;
		b.usb_current_ua = (status == ST_CHARGING && chg_manage) ? chg_fcc_ma * 1000 : 0;
		hakios_battery_report(&b);
		chg_from_vadc = 1;
	}
}

static void
chg_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t type[2] = { 0, 0 };
	uint32_t battery = 0;

	/* the loader's cache (a real fuel gauge) wins over this one, like battmgr defers to it */
	if (PE_parse_boot_argn("battery", &battery, sizeof(battery)) && battery != 0) {
		say("chg-s: battery=1: the loader's cache owns the battery; staying out\n");
		goto out;
	}
	say("chg-s: PMIC arbiter version %#x\n", hakios_spmi_version());
	if (!hakios_spmi_find(ADC_PPID, &adc_apid)) {
		say("chg-s: no APID is the voltage ADC (PPID %#x): no gauge\n", ADC_PPID);
		goto out;
	}
	say("chg-s: voltage ADC at APID %u\n", adc_apid);
	if (!hakios_spmi_find(CHG_PPID, &chg_apid)) {
		say("chg-s: no APID is the charger core (PPID %#x): gauge only, no charger status\n", CHG_PPID);
		chg_apid = 0xffff;
	} else if (!chg_read(CHG_TYPE_REG, &type[0]) || !chg_read(CHG_SUBTYPE_REG, &type[1])) {
		chg_apid = 0xffff;
	} else {
		uint8_t fv = 0;

		say("chg-s: charger core at APID %u: type %#x, subtype %#x\n", chg_apid, type[0], type[1]);
		if (chg_want_type != 0 && (type[0] << 8 | type[1]) == chg_want_type) {
			chg_manage = 1;
			say("chg-s: TYPE/SUBTYPE match chg_type=%#x: managing (FCC %d mA%s)\n",
			    chg_want_type, cfg_fcc_ma, cfg_fv_mv > 0 ? "" : ", hardware float voltage");
		} else {
			say("chg-s: %s: monitor only%s\n",
			    chg_want_type == 0 ? "no chg_type= given" : "TYPE/SUBTYPE do not match chg_type",
			    want_chg ? " (chg=1 wants managing: pass the TYPE/SUBTYPE above as chg_type=0xTTSS)" : "");
		}
		/* the hardware's float voltage is the top of the SoC curve */
		if (chg_read(CHG_FV, &fv)) {
			chg_fv_mv = (int)((CHG_FV_MIN_UV + (uint32_t)fv * CHG_FV_STEP_UV) / 1000);
			say("chg-s: hardware float voltage %d mV\n", chg_fv_mv);
		}
		{
			uint8_t fcc = 0;

			if (chg_read(CHG_FCC, &fcc)) {
				chg_fcc_ma = chg_fcc_of(fcc);   /* what the hardware holds now */
			}
		}
	}
	if (!want_fg && !want_chg) {
		goto out;
	}
	for (;;) {
		round();
		delay_for_interval(poll_seconds, NSEC_PER_SEC);
	}
out:
	thread_terminate(current_thread());
}

void
hakios_chg_init(void)
{
	uint32_t value;
	thread_t thread;

	(void)PE_parse_boot_argn("fg", &want_fg, sizeof(want_fg));
	(void)PE_parse_boot_argn("chg", &want_chg, sizeof(want_chg));
	if (!want_fg && !want_chg) {
		return;
	}
	if (PE_parse_boot_argn("fg_poll", &value, sizeof(value)) && value >= 5 && value <= 600) {
		poll_seconds = value;
	}
	(void)PE_parse_boot_argn("chg_type", &chg_want_type, sizeof(chg_want_type));
	if (PE_parse_boot_argn("chg_fcc_ma", &value, sizeof(value)) && value >= 100 && value <= 3000) {
		cfg_fcc_ma = (int)value;
	}
	if (PE_parse_boot_argn("chg_fv_mv", &value, sizeof(value)) && value >= 3400 && value <= 4800) {
		cfg_fv_mv = (int)value;
	}
	if (PE_parse_boot_argn("chg_icl_ma", &value, sizeof(value)) && value >= 100 && value <= 3000) {
		cfg_icl_ma = (int)value;
	}
	if (PE_parse_boot_argn("chg_hot", &value, sizeof(value)) && value < 100) {
		cfg_hot = (int)value;
	}
	if (PE_parse_boot_argn("chg_max", &value, sizeof(value)) && value < 100) {
		cfg_max = (int)value;
	}
	if (PE_parse_boot_argn("chg_soc_hot", &value, sizeof(value)) && value < 160) {
		cfg_soc_hot = (int)value;
	}
	/* chg_manage also needs the probe; the thread sets it. chg=1 without fg still probes. */
	say("chg-s: on (fg=%u chg=%u every %u s; FCC %d mA%s, hot %d C, max %d C, SoC-hot %d C)\n",
	    want_fg, want_chg, poll_seconds, cfg_fcc_ma, cfg_fv_mv > 0 ? "" : " (FV: hardware's)", cfg_hot, cfg_max,
	    cfg_soc_hot);
	if (kernel_thread_start(chg_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
