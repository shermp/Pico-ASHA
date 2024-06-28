#ifndef PICO_ASHA_H
#define PUCO_ASHA_H

#include <pico/unique_id.h>

#ifdef ASHA_USB_SERIAL
extern char pico_uid[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1 + 4];
#else
extern char pico_uid[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
#endif

#endif // PICO_ASHA_H