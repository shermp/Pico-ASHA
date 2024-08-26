#pragma once
#include <array>
#include <cstdint>
#include <memory>

#include "pico/async_context.h"

#include "g722/g722_enc_dec.h"
#include "PseudoAtomic/RP2040Atomic.hpp"

using namespace patom::types;

namespace asha 
{

constexpr uint32_t g722_ringbuff_size = 8u;
constexpr uint32_t g722_ringbuff_size_mask = g722_ringbuff_size - 1;

// Number of samples per ms
constexpr int pcm_packet_size = 16;

// Stereo packet is twice the size of mono
constexpr int pcm_stereo_packet_size = pcm_packet_size * 2;

// 16-bits per sample
constexpr int pcm_packet_size_bytes = pcm_packet_size * 2;

// Number of bytes of 1ms of G.722 encoded audio
constexpr int g722_1ms_size_bytes = pcm_packet_size_bytes / 4;

// Nunber of bytes of 20ms of G.722 encoded audio
constexpr int g722_20ms_size_bytes = g722_1ms_size_bytes * 20;

// Size of l2cap SDU. Includes 20ms G.722 encoded audio
// plus a sequence number byte
constexpr int sdu_size_bytes = g722_20ms_size_bytes + 1;

// Add padding to sdu for memory alignment purposes
constexpr int sdu_size_bytes_aligned = sdu_size_bytes + 3;
static_assert(sdu_size_bytes_aligned % sizeof(int) == 0);

constexpr int8_t volume_mute = -128;

constexpr uint32_t ring_buff_index(const uint32_t index)
{
    return index & g722_ringbuff_size_mask;
}

class AudioBuffer
{
public:
    struct G722Buff {
        std::array<uint8_t, sdu_size_bytes_aligned> l = {};
        std::array<uint8_t, sdu_size_bytes_aligned> r = {};
    };
    struct Volume {
        int8_t l;
        int8_t r;
    };

    patomic_bool encode_audio;

    AudioBuffer();
    void init();
    void reset_state();
    G722Buff& get_g_buff(uint32_t index);
    Volume get_volume();
    void set_volume(Volume volume);
    uint32_t get_write_index();

    void encode_1ms_audio(int16_t* stereo_pcm);
private:
    std::array<G722Buff, g722_ringbuff_size> g_ring_buff = {};

    struct {
        std::array<int16_t, pcm_packet_size> l = {};
        std::array<int16_t, pcm_packet_size> r = {};
    } pcm_buff = {};

    uint8_t seq_num;

    struct {
        g722_encode_state_t l = {};
        g722_encode_state_t r = {};
    } enc_state = {};

    int g_offset = 1;

    struct {
        patomic_int8_t l;
        patomic_int8_t r;
    } atomic_vol;

    // Current write index. If zero, no audio packets
    // have been encoded
    patomic_uint32_t write_index;

    patomic_bool encode_mono;
};

extern AudioBuffer audio_buff;
extern async_context_t *bt_async_ctx;
extern async_when_pending_worker_t bt_audio_pending_worker;

} // namespace asha