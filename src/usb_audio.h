#ifndef ASHA_USB_AUDIO_H
#define ASHA_USB_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

void init_usb(void);
void run_tud_task(void);
void audio_task(void);

#ifdef __cplusplus
}
#endif

#endif // ASHA_USB_AUDIO_H