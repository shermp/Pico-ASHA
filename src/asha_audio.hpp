#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace asha
{

inline constexpr std::size_t audio_ring_size = 8u;
inline constexpr std::size_t audio_ring_mask = 7u;

// Number of samples per ms
inline constexpr std::size_t pcm_packet_samples = 16u;

// Max number of samples per ms
inline constexpr std::size_t pcm_max_samples = 48u;

// Stereo packet is twice the size of mono
inline constexpr std::size_t pcm_stereo_packet_samples = 32u;

// 16-bits per sample
inline constexpr std::size_t pcm_packet_bytes = 32u;

// Number of bytes of 1ms of G.722 encoded audio
inline constexpr std::size_t g722_1ms_bytes = 8u;

// Number of bytes of 20ms of G.722 encoded audio
inline constexpr std::size_t g722_20ms_bytes = 160u;
inline constexpr std::size_t pcm_blocks_per_sdu = 20u;

// Size of l2cap SDU. Includes 20ms G.722 encoded audio
// plus a sequence number byte
inline constexpr std::size_t sdu_size_bytes = 161u;

// Add padding to sdu for memory alignment purposes
inline constexpr std::size_t sdu_aligned_size_bytes = 164u;

static_assert((audio_ring_size & (audio_ring_size - 1u)) == 0u);
static_assert(audio_ring_mask == audio_ring_size - 1u);
static_assert(pcm_stereo_packet_samples == 2u * pcm_packet_samples);
static_assert(pcm_packet_bytes == pcm_packet_samples * sizeof(int16_t));
static_assert(g722_20ms_bytes == pcm_blocks_per_sdu * g722_1ms_bytes);
static_assert(sdu_size_bytes == 1u + g722_20ms_bytes);
static_assert(sdu_aligned_size_bytes >= sdu_size_bytes);
static_assert(sdu_aligned_size_bytes % alignof(uint32_t) == 0u);

/* ASHA volume range is -48dB to 0dB, and is set as an int8_t
 * with a range of -128 to 0. This gives steps of 0.375 dB.

 * USB volume control is an int16_t, from -128dB to +128dB, in
 * steps of 1/256 dB. The ASHA resolution is 96/256dB per step
 * which gives a range of -12192 to 0 and a resolution of 96.
 */

inline constexpr int16_t usb_vol_max = 0; // 0dB
inline constexpr int16_t usb_vol_min = -12192; // -47.625dB
inline constexpr int16_t usb_vol_res = 96; // 96/256 dB per ASHA step
inline constexpr int16_t usb_vol_mute = -32768;

enum class AudioSide {
    Left = 0,
    Right = 1,
};

struct PCMStereoSample {
    int16_t left;
    int16_t right;
};

static_assert(std::is_standard_layout_v<PCMStereoSample>);
static_assert(sizeof(PCMStereoSample) == 2u * sizeof(int16_t));
static_assert(offsetof(PCMStereoSample, left) == 0u);
static_assert(offsetof(PCMStereoSample, right) == sizeof(int16_t));

void asha_audio_init();

/**
 * Get the current write index.
 *
 * If index is greater than 0, then the latest encoded
 * audio packet is index - 1
 */
[[nodiscard]] uint32_t asha_audio_get_write_index();

/**
 * On core 0, encode one millisecond of 16-bit stereo PCM to G.722.
 * Accept 16 samples at 16 kHz or 48 samples at 48 kHz; ignore other sizes.
 */
void asha_audio_encode_1ms_pcm(std::span<const PCMStereoSample> samples);

/**
 * Borrow read-only encoded audio for side at index. The producer reuses
 * ring storage; the existing publication/copy contract still applies.
 */
[[nodiscard]] std::span<const uint8_t, sdu_size_bytes>
asha_audio_get_encoded_at_index(AudioSide side, uint32_t index);

#ifdef PICO_ASHA_ENC_STATS
[[nodiscard]] std::span<const int16_t, pcm_blocks_per_sdu>
asha_audio_get_encoding_time_at_index(uint32_t index);
#endif

struct USBVolume {
    int16_t left;
    int16_t right;
};

/**
 * Set the current volume, as provided by USB.
 *
 * On core 0, select effective channel volumes and publish them together.
 * Inputs include USB mute as usb_vol_mute.
 */
void asha_audio_set_curr_usb_vol(int16_t main_vol, int16_t left_vol, int16_t right_vol);

/**
 * Get the current volume, as provided by USB.
 *
 * On core 1, read one consistent pair per audio-processing pass.
 */
[[nodiscard]] USBVolume asha_audio_get_curr_usb_vol();

/**
 * Set whether to enable encoding or not
 */
void asha_audio_set_encoding_enabled(bool enabled);

enum class AudioReset {
    Session = 0,
    Left = 1,
    Right = 2,
};

/**
 * Request from core 1; core 0 applies the reset. Returns its request ID.
 * Session resets discard partial audio and reset codecs, FIR, and sequence.
 * Side resets affect only that codec, at the next 20 ms frame boundary.
 */
[[nodiscard]] uint32_t asha_audio_request_reset(AudioReset reset);

/**
 * On core 1, return the first complete frame for this reset request ID.
 * False until that frame is published, or if a newer request superseded it.
 */
[[nodiscard]] bool asha_audio_get_reset_frame(AudioReset reset, uint32_t request_id, uint32_t& index);

/**
 * Get current encoding status
 */
[[nodiscard]] bool asha_audio_get_encoding_enabled();

/**
 * Set whether to encode audio as mono
 */
void asha_audio_set_encode_mono(bool mono);

/**
 * Gets if encoding audio as mono
 */
[[nodiscard]] bool asha_audio_get_encode_mono();

/**
 * Sets if PCM audio is playing
 */
void asha_audio_set_pcm_streaming_enabled(bool enabled);

/**
 * Gets state of PCM audio
 */
[[nodiscard]] bool asha_audio_get_pcm_streaming_enabled();

} // namespace asha
