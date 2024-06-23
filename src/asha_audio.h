#ifndef ASHA_AUDIO_H
#define ASHA_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

#include <pico/util/queue.h>

#include "g722/g722_enc_dec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ASHA_G722_QUEUE_SIZE 16U
#define ASHA_COMMAND_QUEUE_SIZE 4U
#define ASHA_USB_STATUS_QUEUE_SIZE 4U

#define ASHA_PCM_PACKET_SIZE 16 // 16 samples per ms
#define ASHA_PCM_STEREO_PACKET_SIZE (ASHA_PCM_PACKET_SIZE * 2)
#define ASHA_PCM_PACKET_SIZE_BYTES (ASHA_PCM_PACKET_SIZE * 2)
#define ASHA_PCM_20MS_SIZE_BYTES (ASHA_PCM_PACKET_SIZE_BYTES * 20)
#define ASHA_G722_1MS_SIZE_BYTES (ASHA_PCM_PACKET_SIZE_BYTES / 4)
#define ASHA_G722_20MS_SIZE_BYTES (ASHA_G722_1MS_SIZE_BYTES * 20)
#define ASHA_SDU_SIZE_BYTES (ASHA_G722_20MS_SIZE_BYTES + 1)
#define ASHA_SDU_SIZE_BYTES_ALIGN (ASHA_G722_20MS_SIZE_BYTES + 4)

// Offset that G.722 encoded packet starts to allow
// for ASHA sequence number
#define ASHA_SDU_START_OFFSET 1

typedef struct asha_g722_sdu {
    uint8_t l_packet[ASHA_SDU_SIZE_BYTES_ALIGN];
    uint8_t r_packet[ASHA_SDU_SIZE_BYTES_ALIGN];
    int8_t  l_volume;
    int8_t  r_volume;
} asha_g722_sdu_t;

enum ASHAEncodeCmd {
    StartEnc,
    StopEnc,
};

enum ASHAUsbStatus {
    PCMStreaming,
    PCMNotStreaming,
};

// G722 Encoded packet and volume: left and right side
extern queue_t asha_g_queue;

// Command to USB core to start/stop encoding
extern queue_t asha_command_queue;

// Status to bluetooth to determine PCM state
extern queue_t asha_usb_status_queue;

#ifdef __cplusplus
}
#endif

#endif // ASHA_AUDIO_H