/*
 * The flashlight of the Xiaomi 14 Pro (shennong): the PMIC flash LED block
 * (qcom,flash_led@ee00 on PM8550B@7: slave 7, peripheral 0xee, PPID 0x7ee),
 * behind the PMIC arbiter (soc_arb_* from the phone's tree). Only with
 * boot-arg torch=N: 1 = look (read-only), 2 = flash once at Android's torch
 * current, 3 = on until the run ends.
 *
 * Register layout: the common torch driver (drivers/camera/hakios.torch.c,
 * qcom,pm8350c-flash-led family) and the vendor's leds-qti-flash.c: the
 * channel-0 registers ITARGET 0x42, MODULE_EN 0x46, STROBE 0x4a, LED_CTRL 0x4e.
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

#define FLASH_PPID              0x7ee         /* PM8550B@7 flash_led@ee00 */
#define FLASH_TYPE              0x04
#define FLASH_SUBTYPE           0x05
#define FLASH_RT_STATUS         0x10
#define FLASH_SAFETY_TIMER0     0x3e
#define FLASH_ITARGET0          0x42
#define FLASH_MODULE_EN         0x46
#define FLASH_STROBE_CTRL0      0x4a
#define FLASH_EN_LED_CTRL       0x4e
#define ANDROID_ITARGET         9             /* 125 mA at 12.5 mA resolution */

static uint16_t flash_apid = 0xffff;
static uint32_t torch_secs = 2;

static bool
flash_read(uint8_t reg, uint8_t *v)
{
	const char *problem = hakios_spmi_read(flash_apid, reg, v, 1);

	if (problem != NULL) {
		printf("torch-s: read %#x: %s\n", reg, problem);
		return false;
	}
	return true;
}

static bool
flash_write(uint8_t reg, uint8_t v)
{
	const char *problem = hakios_spmi_write(flash_apid, reg, &v, 1);
	uint8_t back = 0xff;

	if (problem != NULL) {
		printf("torch-s: write %#x <- %#x: %s\n", reg, v, problem);
		return false;
	}
	if (!flash_read(reg, &back) || back != v) {
		printf("torch-s: %#x <- %#x, reads back %#x\n", reg, v, back);
		return false;
	}
	return true;
}

static void
torch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t type[2] = { 0, 0 };
	uint32_t level = 0;

	hakios_tl("tl: torch-s thread running\n");
	(void)PE_parse_boot_argn("torch", &level, sizeof(level));
	(void)PE_parse_boot_argn("torch_secs", &torch_secs, sizeof(torch_secs));
	if (level == 0) {
		goto out;
	}
	printf("torch-s: PM8550B flash (torch=%u): arbiter version %#x\n", level, hakios_spmi_version());
	hakios_tl("tl: torch-s arbiter version %#x\n", hakios_spmi_version());
	if (!hakios_spmi_find(FLASH_PPID, &flash_apid)) {
		printf("torch-s: no APID is the flash module (PPID %#x)\n", FLASH_PPID);
	hakios_tl("tl: torch-s no APID for %#x\n", FLASH_PPID);
		goto out;
	}
	printf("torch-s: flash module at APID %u\n", flash_apid);
	hakios_tl("tl: torch-s APID %u\n", flash_apid);
	if (!flash_read(FLASH_TYPE, &type[0]) || !flash_read(FLASH_SUBTYPE, &type[1])) {
		goto out;
	}
	printf("torch-s: type %#x, subtype %#x\n", type[0], type[1]);
	hakios_tl("tl: torch-s type %#x sub %#x\n", type[0], type[1]);
	if (type[0] != 0x18) {
		printf("torch-s: not a flash LED module: left alone\n");
	hakios_tl("tl: torch-s not flash\n");
		goto out;
	}
	if (level == 1) {
		goto out;
	}
	/* Android's torch: strobe to software, safety timer off, 125 mA, module + channel 0 on */
	if (!flash_write(FLASH_STROBE_CTRL0, 0) || !flash_write(FLASH_SAFETY_TIMER0, 0) ||
	    !flash_write(FLASH_ITARGET0, ANDROID_ITARGET) || !flash_write(FLASH_MODULE_EN, 0x80) ||
	    !flash_write(FLASH_EN_LED_CTRL, 0x01)) {
		printf("torch-s: setup failed: no light\n");
		goto out;
	}
	printf("torch-s: ON at Android's torch current%s\n", level >= 3 ? " until the run ends" : "");
	hakios_tl("tl: torch-s ON\n");
	if (level == 2) {
		sleep_ms(torch_secs * 1000);
	}
	if (level == 2) {
		flash_write(FLASH_EN_LED_CTRL, 0);
		flash_write(FLASH_MODULE_EN, 0);
		printf("torch-s: off again\n");
		hakios_tl("tl: torch-s OFF\n");
	}
out:
	thread_terminate(current_thread());
}

void
hakios_torch_quiesce(void)
{
	/* the LED is off already after a torch=2 run; torch=3 leaves it to the loader */
}

void
hakios_torch_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("torch", &enabled, sizeof(enabled)) || enabled == 0) {
		return;
	}
	if (kernel_thread_start(torch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
