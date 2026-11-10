/*
 * The vibration motor of the Xiaomi 14 Pro (shennong): PM8550B hv-haptics
 * (qcom,hv-haptics@f000 on PM8550B@7: slave 7, peripheral 0xf0, PPID 0x7f0),
 * behind the PMIC arbiter (soc_arb_* from the phone's tree). Only with
 * boot-arg haptic=N: 1 = look (read-only), 2 = buzz once, 3 = buzz until
 * the run ends.
 *
 * Register layout: Xiaomi's qcom-hv-haptics.c (HAP525_HV = PM8550B) --
 * CFG block at 0xf000 (EN_CTL 0x46, SPMI_PLAY 0x4c: PLAY_EN bit 7, source
 * bits 2:0), pattern block at 0xf100 (DIRECT_PLAY 0x26: one amplitude byte).
 * DIRECT_PLAY mode: the amplitude byte drives the LRA until PLAY_EN clears.
 * Keep to reads plus this one path; the FIFO/brake/calibration paths of the
 * vendor driver are not needed to feel the motor.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern void hakios_tl(const char *format, ...) __printflike(1, 2);
extern kern_return_t thread_terminate(thread_t thread);

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

#define HAPTIC_PPID             0x7f0         /* PM8550B@7 hv-haptics@f000 */
#define HAPTIC_PTN_PPID           0x7f1         /* pattern block @f100 */
#define HAP_CFG_TYPE            0x04
#define HAP_CFG_SUBTYPE         0x05
#define HAP_CFG_EN_CTL          0x46
#define HAP_EN_BIT              0x80
#define HAP_CFG_SPMI_PLAY       0x4c
#define HAP_PLAY_EN             0x80
#define HAP_SRC_DIRECT          0x01
#define HAP_PTN_DIRECT_PLAY     0x26          /* in the 0xf100 block */

#define HAP_CFG_BASE            0xf000
#define HAP_PTN_BASE            0xf100

static uint16_t hap_apid = 0xffff, hap_ptn_apid = 0xffff;

static bool
hap_read(uint8_t reg, uint8_t *v)
{
	const char *problem = hakios_spmi_read(hap_apid, reg, v, 1);

	if (problem != NULL) {
		printf("haptic-s: read %#x: %s\n", reg, problem);
		return false;
	}
	return true;
}

static bool
hap_write(uint8_t reg, uint8_t v)
{
	const char *problem = hakios_spmi_write(hap_apid, reg, &v, 1);
	uint8_t back = 0xff;

	if (problem != NULL) {
		printf("haptic-s: write %#x <- %#x: %s\n", reg, v, problem);
		return false;
	}
	if (!hap_read(reg, &back) || back != v) {
		printf("haptic-s: %#x <- %#x, reads back %#x\n", reg, v, back);
		return false;
	}
	return true;
}

/* pattern-block register lives on a second APID (PPID 0x7f1) */
static bool
hap_ptn_write(uint8_t reg, uint8_t v)
{
	const char *problem;
	uint8_t back = 0xff, vv = v;
	uint16_t saved = hap_apid;

	if (hap_ptn_apid == 0xffff)
		return false;
	hap_apid = hap_ptn_apid;
	problem = hakios_spmi_write(hap_apid, reg, &vv, 1);
	hap_apid = saved;
	if (problem != NULL) {
		printf("haptic-s: ptn write %#x <- %#x: %s\n", reg, v, problem);
		return false;
	}
	saved = hap_apid;
	hap_apid = hap_ptn_apid;
	problem = hakios_spmi_read(hap_apid, reg, &back, 1);
	hap_apid = saved;
	if (problem != NULL || back != v) {
		printf("haptic-s: ptn %#x <- %#x, reads back %#x (%s)\n", reg, v, back,
		    problem ? problem : "mismatch");
		return false;
	}
	return true;
}

static void
haptic_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t level = 0;
	uint8_t type[2] = { 0, 0 };

	(void)PE_parse_boot_argn("haptic", &level, sizeof(level));
	if (level == 0) {
		goto out;
	}
	hakios_tl("tl: haptic-s thread running level %u\n", level);
	if (!hakios_spmi_find(HAPTIC_PPID, &hap_apid)) {
		printf("haptic-s: no APID is the hv-haptics block (PPID %#x)\n", HAPTIC_PPID);
		hakios_tl("tl: haptic-s no APID for %#x\n", HAPTIC_PPID);
		goto out;
	}
	printf("haptic-s: hv-haptics at APID %u\n", hap_apid);
	hakios_tl("tl: haptic-s APID %u\n", hap_apid);
	if (!hakios_spmi_find(HAPTIC_PTN_PPID, &hap_ptn_apid)) {
		printf("haptic-s: no APID is the pattern block (PPID %#x)\n", HAPTIC_PTN_PPID);
		hakios_tl("tl: haptic-s no APID for ptn\n");
		goto out;
	}
	hakios_tl("tl: haptic-s PTN APID %u\n", hap_ptn_apid);
	if (!hap_read(HAP_CFG_TYPE, &type[0]) || !hap_read(HAP_CFG_SUBTYPE, &type[1])) {
		goto out;
	}
	printf("haptic-s: type %#x, subtype %#x\n", type[0], type[1]);
	hakios_tl("tl: haptic-s type %#x sub %#x\n", type[0], type[1]);
	if (level == 1) {
		goto out;
	}
	if (!hap_write(HAP_CFG_EN_CTL, HAP_EN_BIT)) {
		printf("haptic-s: enable failed\n");
		goto out;
	}
	if (!hap_ptn_write(HAP_PTN_DIRECT_PLAY, 0x7f)) {
		printf("haptic-s: amplitude failed\n");
		goto out;
	}
	if (!hap_write(HAP_CFG_SPMI_PLAY, HAP_PLAY_EN | HAP_SRC_DIRECT)) {
		printf("haptic-s: play failed\n");
		goto out;
	}
	hakios_tl("tl: haptic-s buzzing\n");
	printf("haptic-s: buzzing%s\n", level >= 3 ? " until the run ends" : "");
	if (level >= 3) {
		goto out;
	}
	sleep_ms(500);
	hap_write(HAP_CFG_SPMI_PLAY, HAP_SRC_DIRECT);
	hap_write(HAP_CFG_EN_CTL, 0);
	printf("haptic-s: off again\n");
	hakios_tl("tl: haptic-s OFF\n");
out:
	thread_terminate(current_thread());
}

void
hakios_haptic_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("haptic", &enabled, sizeof(enabled)) || enabled == 0) {
		return;
	}
	if (kernel_thread_start(haptic_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

void
hakios_haptic_quiesce(void)
{
}

void
hakios_haptic_pretend(__unused uint32_t peak)
{
}
