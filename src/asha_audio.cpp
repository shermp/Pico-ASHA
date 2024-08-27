#include "asha_audio.hpp"

namespace asha
{

AudioBuffer::AudioBuffer()
{}

void AudioBuffer::init()
{
    reset_state();
    atomic_vol.l = volume_mute;
    atomic_vol.r = volume_mute;
    encode_mono = false;
    encode_audio = false;
    pcm_streaming = false;
    write_index = 0u;
    g_offset = 1;
}

void AudioBuffer::reset_state()
{
    g722_encode_init(&enc_state.l, 64000, G722_PACKED);
    g722_encode_init(&enc_state.r, 64000, G722_PACKED);
    seq_num = 0u;
}

AudioBuffer::G722Buff& AudioBuffer::get_g_buff(uint32_t index)
{
    return g_ring_buff[ring_buff_index(index)];
}

AudioBuffer::Volume AudioBuffer::get_volume()
{
    return Volume{.l = atomic_vol.l.Load(), .r = atomic_vol.r.Load()};
}

void AudioBuffer::set_volume(AudioBuffer::Volume volume)
{
    atomic_vol.l = volume.l;
    atomic_vol.r = volume.r;
}

uint32_t AudioBuffer::get_write_index()
{
    return write_index.Load();
}

void AudioBuffer::encode_1ms_audio(int16_t* stereo_pcm)
{
    if (!encode_audio.Load()) return;
    int buff_index = 0;
    uint32_t w_index = write_index.Load();
    if (encode_mono.Load()) {
        for (int i = 0; i < pcm_stereo_packet_size; i += 2) {
            pcm_buff.l[buff_index] = (int16_t)(((int32_t)stereo_pcm[i] + (int32_t)stereo_pcm[i + 1]) / 2);
            ++buff_index;
        }
        g722_encode(&enc_state.l, g_ring_buff[ring_buff_index(w_index)].l.data() + g_offset, pcm_buff.l.data(), pcm_buff.l.size());
    } else {
        for (int i = 0; i < pcm_stereo_packet_size; i += 2) {
            pcm_buff.l[buff_index] = stereo_pcm[i];
            pcm_buff.r[buff_index] = stereo_pcm[i + 1];
            ++buff_index;
        }
        g722_encode(&enc_state.l, g_ring_buff[ring_buff_index(w_index)].l.data() + g_offset, pcm_buff.l.data(), pcm_buff.l.size());
        g722_encode(&enc_state.r, g_ring_buff[ring_buff_index(w_index)].r.data() + g_offset, pcm_buff.r.data(), pcm_buff.r.size());
    }
    g_offset += g722_1ms_size_bytes;
    if (g_offset >= sdu_size_bytes) {
        g_ring_buff[ring_buff_index(w_index)].l[0] = seq_num;
        g_ring_buff[ring_buff_index(w_index)].r[0] = seq_num;
        g_offset = 1;
        ++w_index;
        ++seq_num;
        write_index = w_index;
    }
}

} // namespace asha