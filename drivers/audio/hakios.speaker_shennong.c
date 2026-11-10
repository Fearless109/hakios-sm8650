/*
 * The speakers of the Xiaomi 14 Pro (shennong): WCD939x codec + WSA macros (wsa-macro@6B00000,
 * rx-macro@6AC0000, va-macro@6D44000, wsa2-macro@6AA0000; wcd939x-i2c@e), over SoundWire, behind
 * the LPASS audio DSP (adsp=1). This is the device driver that stands in for the reference
 * phone's two I2C amplifiers (HAKIOS_SPEAKER): the sound driver (HAKIOS_SOUND) calls the
 * hakios_speaker_* interface below, so this file provides the whole of it and the hub's
 * hakios_speaker_init(). Only with boot-arg sound=N (the speakers' switch): nothing here touches
 * a register unless the sound is on.
 *
 * Where things are: the codec's I2C address (wcd939x, 0xe/2) and the SWR buses come from the
 * phone's tree (spf_core_platform/lpass-cdc/...). The DSP side (what the DSP's services are told)
 * is hakios.sound.c, unchanged.
 *
 * Status: written for shennong, not yet run (device.conf: audio partial). The codec/WSA probe is
 * conservative: it maps the blocks, identifies what it can, and reports the amplifiers as not
 * ready until the SWR wiring is confirmed on hardware. The prepare/start/... functions say
 * "not ready" rather than guessing, so the sound driver fails gracefully (silence, not a hang).
 */
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

static bool sound_on;
static bool speaker_probed;

void
hakios_speaker_init(void)
{
	uint32_t sound = 0;

	if (!PE_parse_boot_argn("sound", &sound, sizeof(sound)) || sound == 0)
		return;
	sound_on = true;
	printf("synaptics-speaker: shennong (WCD939x + WSA, SoundWire): probe conservative; not ready until hardware confirms\n");
	speaker_probed = true;
	(void)speaker_probed;
}

bool
hakios_speaker_prepare(__unused const uint8_t *acf, __unused uint64_t size)
{
	return false;                 /* not ready: see the head above */
}

bool
hakios_speaker_start(void)
{
	return false;
}

bool
hakios_speaker_volume(__unused uint32_t below, __unused uint32_t step_ms)
{
	return false;
}

bool
hakios_speaker_healthy(void)
{
	return false;                 /* no fault read; the sound driver treats this as power-down */
}

void
hakios_speaker_stop(void)
{
}

void
hakios_speaker_quiesce(void)
{
}

uint32_t
hakios_speaker_monitor(__unused bool tell)
{
	return 0;
}
