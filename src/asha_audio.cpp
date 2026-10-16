#include <array>
#include <atomic>
#include <bit>

#ifdef PICO_ASHA_ENC_STATS
#include <pico/time.h>
#endif

#include <g722/g722_enc_dec.h>

#include "asha_audio.hpp"
#include "asha_fir.hpp"

namespace asha
{
namespace
{

static_assert(fir_input_samples == pcm_max_samples);
static_assert(fir_output_samples == pcm_packet_samples);

struct AshaAudioEncBuffer {
    alignas(uint32_t) std::array<uint8_t, sdu_aligned_size_bytes> l;
    alignas(uint32_t) std::array<uint8_t, sdu_aligned_size_bytes> r;
#ifdef PICO_ASHA_ENC_STATS
    std::array<int16_t, pcm_blocks_per_sdu> encode_times;
#endif
};

static_assert(alignof(AshaAudioEncBuffer) >= alignof(uint32_t));
static_assert(offsetof(AshaAudioEncBuffer, l) == 0u);
static_assert(offsetof(AshaAudioEncBuffer, r) == sdu_aligned_size_bytes);
#ifdef PICO_ASHA_ENC_STATS
static_assert(offsetof(AshaAudioEncBuffer, encode_times) == 2u * sdu_aligned_size_bytes);
static_assert(sizeof(AshaAudioEncBuffer) == 2u * sdu_aligned_size_bytes
              + pcm_blocks_per_sdu * sizeof(int16_t));
#else
static_assert(sizeof(AshaAudioEncBuffer) == 2u * sdu_aligned_size_bytes);
#endif

// Controls shared between core 1 and the core-0 producer.
std::atomic_bool pcm_streaming{false};
std::atomic_bool encode_audio{false};
std::atomic_bool requested_encode_mono{false};

constexpr std::size_t reset_index(AudioReset reset)
{
    return static_cast<std::size_t>(reset);
}

// Core 1 requests/reads resets; core 0 publishes first_frame via applied_id.
struct EncoderReset {
    std::atomic_uint_least32_t requested_id{0};
    std::atomic_uint_least32_t applied_id{0};
    uint32_t first_frame{0};
};

constinit std::array<EncoderReset, reset_index(AudioReset::Right) + 1u> encoder_resets;

// Only core 0 advances this index, publishing completed ring frames to core 1.
std::atomic_uint_fast32_t write_index{0};

constexpr uint32_t pack_usb_volume(int16_t left, int16_t right)
{
    return uint32_t(uint16_t(left)) | (uint32_t(uint16_t(right)) << 16u);
}

constinit std::atomic_uint32_t usb_volume{pack_usb_volume(usb_vol_min, usb_vol_min)};

std::array<AshaAudioEncBuffer, audio_ring_size> enc_ring_buff;

// All codec, FIR, input-buffer and partial-frame state belongs to core 0.
struct AudioProducer {
    uint16_t pcm_sample_count{};
    bool encode_mono{false};
    alignas(uint32_t) FirDecimator decimator_l;
    alignas(uint32_t) FirDecimator decimator_r;
    g722_encode_state_t enc_state_l{};
    g722_encode_state_t enc_state_r{};
    unsigned int g_offset{};
    unsigned int enc_time_index{};
    uint8_t seq_num{};
    alignas(uint32_t) std::array<int16_t, pcm_max_samples> pcm_buff_l{};
    alignas(uint32_t) std::array<int16_t, pcm_max_samples> pcm_buff_r{};
    alignas(uint32_t) std::array<int16_t, pcm_packet_samples> pcm_buff_16khz_l{};
    alignas(uint32_t) std::array<int16_t, pcm_packet_samples> pcm_buff_16khz_r{};

    void initialize();
    void reset_encoders();
    void reset_decimators();
    void reset_stream_encoder();
    void encode(std::span<const PCMStereoSample> samples);
};

static_assert(offsetof(AudioProducer, decimator_l) % alignof(uint32_t) == 0u);
static_assert(offsetof(AudioProducer, decimator_r) % alignof(uint32_t) == 0u);

constinit AudioProducer producer;

uint32_t ring_buff_index(uint32_t index)
{
    return index & audio_ring_mask;
}

int16_t get_mono_sample(PCMStereoSample sample)
{
    return (int16_t)(((int32_t)sample.left + (int32_t)sample.right) / 2);
}

void AudioProducer::reset_encoders()
{
    g722_encode_init(&enc_state_l, 64000, G722_PACKED);
    g722_encode_init(&enc_state_r, 64000, G722_PACKED);
}

void AudioProducer::reset_decimators()
{
    decimator_l.reset();
    decimator_r.reset();
}

void AudioProducer::reset_stream_encoder()
{
    reset_encoders();
    reset_decimators();
    g_offset = 1;
    seq_num = 0;
    enc_time_index = 0;
    encode_mono = requested_encode_mono.load(std::memory_order_acquire);
}

void AudioProducer::initialize()
{
    pcm_sample_count = 0;
    reset_stream_encoder();
}

void AudioProducer::encode(std::span<const PCMStereoSample> samples)
{
#ifdef PICO_ASHA_ENC_STATS
    absolute_time_t start_time = get_absolute_time();
#endif
    auto const count = samples.size();
    uint32_t const w_index = write_index.load(std::memory_order_relaxed);
    auto& session_reset = encoder_resets[reset_index(AudioReset::Session)];
    uint32_t const session_request_id = session_reset.requested_id.load(std::memory_order_acquire);
    if (session_request_id != session_reset.applied_id.load(std::memory_order_relaxed)) {
        // The hearing aid resets its decoder after an ACP stop/start. Reset on
        // this producer core so no partial SDU or stale predictor state leaks
        // into the next stream.
        reset_stream_encoder();
        session_reset.first_frame = w_index;
        session_reset.applied_id.store(session_request_id, std::memory_order_release);
    }

    if (count != pcm_sample_count) {
        // A USB rate change invalidates FIR history, not the aid's G.722
        // predictor or the partially encoded 20 ms frame.
        reset_decimators();
        pcm_sample_count = count;
    }

    bool enc_audio = encode_audio;
    if (!enc_audio) return;
    auto& buff = enc_ring_buff[ring_buff_index(w_index)];

    // Mode changes and per-side resets must not split an SDU.
    if (g_offset == 1) {
        encode_mono = requested_encode_mono.load(std::memory_order_acquire);
        for (auto i = reset_index(AudioReset::Left); i <= reset_index(AudioReset::Right); ++i) {
            auto& reset = encoder_resets[i];
            uint32_t const request_id = reset.requested_id.load(std::memory_order_acquire);
            if (request_id != reset.applied_id.load(std::memory_order_relaxed)) {
                g722_encode_init(i == reset_index(AudioReset::Left) ? &enc_state_l : &enc_state_r, 64000, G722_PACKED);
                reset.first_frame = w_index;
                reset.applied_id.store(request_id, std::memory_order_release);
            }
        }
    }

    // Separate interleaved stereo samples to separate channels
    bool mono = encode_mono;
    if (mono) {
        for (unsigned int i = 0; i < count; ++i) {
            int16_t sample = get_mono_sample(samples[i]);
            pcm_buff_l[i] = sample;
            pcm_buff_r[i] = sample;
        }       
    } else {
        for (unsigned int i = 0; i < count; ++i) {
            pcm_buff_l[i] = samples[i].left;
            pcm_buff_r[i] = samples[i].right;
        }
    }
    std::span<const int16_t, pcm_packet_samples> pcm_l = std::span{pcm_buff_l}.first<pcm_packet_samples>();
    std::span<const int16_t, pcm_packet_samples> pcm_r = std::span{pcm_buff_r}.first<pcm_packet_samples>();
    if (count == pcm_max_samples) {
        decimator_l.process(pcm_buff_l, pcm_buff_16khz_l);
        decimator_r.process(pcm_buff_r, pcm_buff_16khz_r);
        pcm_l = pcm_buff_16khz_l;
        pcm_r = pcm_buff_16khz_r;
    }

    g722_encode(&enc_state_l, buff.l.data() + g_offset, pcm_l.data(), pcm_packet_samples);
    g722_encode(&enc_state_r, buff.r.data() + g_offset, pcm_r.data(), pcm_packet_samples);
    
    g_offset += g722_1ms_bytes;
#ifdef PICO_ASHA_ENC_STATS
    int64_t time_diff = absolute_time_diff_us(start_time, get_absolute_time());
    buff.encode_times[enc_time_index] = (int16_t)time_diff;
    ++enc_time_index;
#endif
    if (g_offset >= sdu_size_bytes) {
        buff.l[0] = seq_num;
        buff.r[0] = seq_num;
        ++seq_num;
        g_offset = 1;
        write_index.store(w_index + 1u, std::memory_order_release);
        enc_time_index = 0;
    }
}

} // namespace

void asha_audio_init()
{
    // Initialize on core 0 before launching core 1, along with shared queues
    // and the SDK flash-lockout registration.
    for (auto& buff : enc_ring_buff) {
        buff.l.fill(0);
        buff.r.fill(0);
#ifdef PICO_ASHA_ENC_STATS
        buff.encode_times.fill(0);
#endif
    }
    pcm_streaming = false;
    encode_audio = false;
    requested_encode_mono = false;
    for (auto& reset : encoder_resets) {
        reset.requested_id = 0;
        reset.first_frame = 0;
        reset.applied_id = 0;
    }
    write_index = 0u;
    usb_volume.store(pack_usb_volume(usb_vol_min, usb_vol_min));
    producer.initialize();
}

uint32_t asha_audio_get_write_index()
{
    return write_index.load(std::memory_order_acquire);
}

void asha_audio_encode_1ms_pcm(std::span<const PCMStereoSample> samples)
{
    if (samples.size() != pcm_packet_samples && samples.size() != pcm_max_samples) return;
    producer.encode(samples);
}

std::span<const uint8_t, sdu_size_bytes>
asha_audio_get_encoded_at_index(AudioSide side, uint32_t index)
{
    auto const& buff = enc_ring_buff[ring_buff_index(index)];
    return std::span{side == AudioSide::Left ? buff.l : buff.r}.first<sdu_size_bytes>();
}

#ifdef PICO_ASHA_ENC_STATS
std::span<const int16_t, pcm_blocks_per_sdu>
asha_audio_get_encoding_time_at_index(uint32_t index)
{
    return enc_ring_buff[ring_buff_index(index)].encode_times;
}
#endif

void asha_audio_set_curr_usb_vol(int16_t main_vol, int16_t left_vol, int16_t right_vol)
{
    // If volume levels are different for each side, user likely wants this.
    if (left_vol == right_vol) {
        // Choose the lowest volume. Some OS drivers seem to prefer setting the "main"
        // channel (observed on MacOS with UAC1). Some set all the same (Windows UAC1)
        // and some set the left/right pair (Windows UAC2). Channels that are not changed
        // are often set to the loudest level - we don't want that!
        left_vol = main_vol < left_vol ? main_vol : left_vol;
        right_vol = left_vol;
    }
    usb_volume.store(pack_usb_volume(left_vol, right_vol));
}

USBVolume asha_audio_get_curr_usb_vol()
{
    auto const packed = usb_volume.load();
    return {std::bit_cast<int16_t>(uint16_t(packed)),
            std::bit_cast<int16_t>(uint16_t(packed >> 16u))};
}

void asha_audio_set_encoding_enabled(bool enabled)
{
    encode_audio = enabled;
}

uint32_t asha_audio_request_reset(AudioReset reset)
{
    if (reset_index(reset) > reset_index(AudioReset::Right)) return 0;
    // Core 1 is the sole requester, so no atomic read-modify-write is needed.
    auto& requested_id = encoder_resets[reset_index(reset)].requested_id;
    uint32_t const next = requested_id.load(std::memory_order_relaxed) + 1u;
    requested_id.store(next, std::memory_order_release);
    return next;
}

bool asha_audio_get_reset_frame(AudioReset reset, uint32_t request_id, uint32_t& index)
{
    if (reset_index(reset) > reset_index(AudioReset::Right) || request_id == 0) return false;
    auto const& state = encoder_resets[reset_index(reset)];
    if (state.requested_id.load(std::memory_order_relaxed) != request_id
        || state.applied_id.load(std::memory_order_acquire) != request_id) return false;
    index = state.first_frame;
    // The write index publishes complete SDUs; an applied reset alone is not enough.
    return write_index.load(std::memory_order_acquire) > index;
}

bool asha_audio_get_encoding_enabled()
{
    bool enabled = encode_audio;
    return enabled;
}

void asha_audio_set_encode_mono(bool mono)
{
    requested_encode_mono.store(mono, std::memory_order_release);
}

void asha_audio_set_pcm_streaming_enabled(bool enabled)
{
    pcm_streaming = enabled;
}

bool asha_audio_get_pcm_streaming_enabled()
{
    bool pcm = pcm_streaming;
    return pcm;
}

} // namespace asha
