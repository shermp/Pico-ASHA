#include <stdio.h>
#include <pico/time.h>

#include "asha_audio.h"

void asha_audio_init(struct asha_audio *audio)
{
    audio->volume = -128;
    audio->packet_index = 0;
    audio->write_offset = 1;
    audio->seq_num = 0;
    audio->pcm_streaming = false;
    audio->encode_audio = false;

    asha_audio_g_reset(audio);
}

void asha_audio_g_l_reset(struct asha_audio *audio) 
{
    g722_encode_init(&audio->g_l_state, 64000, G722_PACKED);
}
void asha_audio_g_r_reset(struct asha_audio *audio)
{
    g722_encode_init(&audio->g_r_state, 64000, G722_PACKED);
}
void asha_audio_g_m_reset(struct asha_audio *audio)
{
    g722_encode_init(&audio->g_m_state, 64000, G722_PACKED);
}
void asha_audio_g_reset(struct asha_audio *audio)
{
    asha_audio_g_l_reset(audio);
    asha_audio_g_r_reset(audio);
    asha_audio_g_m_reset(audio);
}

void asha_audio_g_enc_1ms(struct asha_audio *audio, int16_t stereo_pcm[ASHA_PCM_STEREO_PACKET_SIZE])
{
    int16_t left, right, mono;
    int buff_index = 0;
    for (int i = 0; i < ASHA_PCM_STEREO_PACKET_SIZE; i += 2) {
        left = stereo_pcm[i];
        right = stereo_pcm[i + 1];
        mono = (int16_t) (((uint32_t)left >> 1) + ((uint32_t)right >> 1));
        audio->tmp_l_pcm[buff_index] = left;
        audio->tmp_r_pcm[buff_index] = right;
        audio->tmp_m_pcm[buff_index] = mono;
        buff_index++;        
    }
    int offset = audio->write_offset;
    uint32_t packet_index = ASHA_RING_BUFF_INDEX(audio->packet_index);
    g722_encode(&audio->g_l_state, &(audio->g_l_buff[packet_index][offset]), audio->tmp_l_pcm, ASHA_PCM_PACKET_SIZE);
    g722_encode(&audio->g_r_state, &(audio->g_r_buff[packet_index][offset]), audio->tmp_r_pcm, ASHA_PCM_PACKET_SIZE);
    //g722_encode(&audio->g_m_state, &(audio->g_m_buff[packet_index][offset]), audio->tmp_m_pcm, ASHA_PCM_PACKET_SIZE);
    
    audio->write_offset += ASHA_G722_1MS_SIZE_BYTES;
    if (audio->write_offset >= (ASHA_SDU_SIZE_BYTES)) {
        uint8_t s = audio->seq_num;
        audio->g_l_buff[packet_index][0] = s;
        audio->g_r_buff[packet_index][0] = s;
        audio->g_m_buff[packet_index][0] = s;
        audio->write_offset = 1;
        audio->seq_num++;
        audio->packet_index++;
    }
}

// uint32_t asha_audio_latest_index(struct asha_audio *audio)
// {
//     return audio->packet_index - 1;
// }

// uint8_t* asha_audio_g_get_20ms(struct asha_audio *audio, uint32_t index)
// {
    
// }