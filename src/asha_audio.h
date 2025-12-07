#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASHA_G722_RING_SIZE 8u
#define ASHA_G722_RING_SIZE_MASK 7u

// Number of samples per ms
#define ASHA_PCM_PACKET_SIZE 16u

// Stereo packet is twice the size of mono
#define ASHA_PCM_STEREO_PACKET_SIZE 32u

// 16-bits per sample
#define ASHA_PCM_PACKET_SIZE_BYTES 32u

// Number of bytes of 1ms of G.722 encoded audio
#define ASHA_G722_1MS_SIZE_BYTES 8u

// Nunber of bytes of 20ms of G.722 encoded audio
#define ASHA_G722_20MS_SIZE_BYTES 160u

// Size of l2cap SDU. Includes 20ms G.722 encoded audio
// plus a sequence number byte
#define ASHA_SDU_SIZE_BYTES 161u

// Add padding to sdu for memory alignment purposes
#define ASHA_SDU_SIZE_BYTES_ALIGNED 164u

/* ASHA volume range is -48dB to 0dB, and is set as an int8_t
 * with a range of -128 to 0. This gives steps of 0.375 dB.

 * USB volume control is an int16_t, from -128dB to +128dB, in
 * steps of 1/256 dB. The ASHA resolution is 96/256dB per step 
 * which gives a range of -12192 to 0 and a resolution of 96.
 */

#define ASHA_USB_VOL_MAX   -3072 // -12dB
#define ASHA_USB_VOL_MIN  -12192 // -47.625dB
#define ASHA_USB_VOL_RES      96 // 96/256 is the step between ASHA volume levels
#define ASHA_USB_VOL_MUTE -32768

enum AshaAudioSide {
    AudioLeft,
    AudioRight,
};

void asha_audio_init();

/**
 * Get the current write index.
 * 
 * If index is greater than 0, then the latest encoded
 * audio packet is index - 1
 */
uint32_t asha_audio_get_write_index();

/**
 * Encode 1ms of 16-bit 16kHz PCM stereo audio to G.722
 */
void asha_audio_encode_1ms_pcm(int16_t *stereo_pcm);

/**
 * Get encoded audio for side at index
 */
uint8_t* asha_audio_get_encoded_at_index(enum AshaAudioSide side, uint32_t index);

/**
 * Set the current volume, as provided by USB
 */
//void asha_audio_set_curr_usb_vol(enum AshaAudioSide side, int16_t usb_volume);

void asha_audio_set_curr_usb_vol(int16_t main_vol, int16_t left_vol, int16_t right_vol);

/**
 * Get the current volume, as provided by USB
 */
int16_t asha_audio_get_curr_usb_vol(enum AshaAudioSide side);

/**
 * Set whether to enable encoding or not
 */
void asha_audio_set_encoding_enabled(bool enabled);

/**
 * Get current encoding status
 */
bool asha_audio_get_encoding_enabled();

/**
 * Set whether to encode audio as mono
 */
void asha_audio_set_encode_mono(bool mono);

/**
 * Gets if encoding audio as mono
 */
bool asha_audio_get_encode_mono();

/**
 * Sets if PCM audio is playing
 */
void asha_audio_set_pcm_streaming_enabled(bool enabled);

/**
 * Gets state of PCM audio
 */
bool asha_audio_get_pcm_streaming_enabled();

#ifdef __cplusplus
}
#endif
