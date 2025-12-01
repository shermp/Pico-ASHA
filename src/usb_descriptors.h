/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenbreg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

// #include "tusb.h"
#ifdef __cplusplus
extern "C" {
#endif
enum
{
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_AUDIO_STREAMING,
  // ITF_NUM_CDC,
  // ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

//--------------------------------------------------------------------+
// UAC1 DESCRIPTOR TEMPLATES
//--------------------------------------------------------------------+

// Defined in TUD_AUDIO10_SPEAKER_STEREO_FB_DESCRIPTOR
#define UAC1_ENTITY_INPUT_TERMINAL      0x01
#define UAC1_ENTITY_FEATURE_UNIT        0x02
#define UAC1_ENTITY_OUTPUT_TERMINAL     0x03

#define TUD_AUDIO10_SPEAKER_STEREO_FB_DESC_LEN(_nfreqs) (\
  + TUD_AUDIO10_DESC_STD_AC_LEN\
  + TUD_AUDIO10_DESC_CS_AC_LEN(1)\
  + TUD_AUDIO10_DESC_INPUT_TERM_LEN\
  + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN\
  + TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2)\
  + TUD_AUDIO10_DESC_STD_AS_LEN\
  + TUD_AUDIO10_DESC_STD_AS_LEN\
  + TUD_AUDIO10_DESC_CS_AS_INT_LEN\
  + TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(_nfreqs)\
  + TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN\
  + TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN)

#define TUD_AUDIO10_SPEAKER_STEREO_FB_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epoutsize, ...) \
  /* Standard AC Interface Descriptor(4.3.1) */\
  TUD_AUDIO10_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.3.2) */\
  TUD_AUDIO10_DESC_CS_AC(/*_bcdADC*/ 0x0100, /*_totallen*/ (TUD_AUDIO10_DESC_INPUT_TERM_LEN+TUD_AUDIO10_DESC_OUTPUT_TERM_LEN+TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2)), /*_itf*/ ((_itfnum)+1)),\
  /* Input Terminal Descriptor(4.3.2.1) */\
  TUD_AUDIO10_DESC_INPUT_TERM(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_nchannels*/ 0x02, /*_channelcfg*/ AUDIO10_CHANNEL_CONFIG_LEFT_FRONT | AUDIO10_CHANNEL_CONFIG_RIGHT_FRONT, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.3.2.2) */\
  TUD_AUDIO10_DESC_OUTPUT_TERM(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm*/ 0x00, /*_srcid*/ 0x02, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.3.2.5) */\
  TUD_AUDIO10_DESC_FEATURE_UNIT(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_stridx*/ 0x00, /*_ctrlmaster*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), /*_ctrlch1*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), /*_ctrlch2*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME)),\
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO10_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO10_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.5.2) */\
  TUD_AUDIO10_DESC_CS_AS_INT(/*_termid*/ 0x01, /*_delay*/ 0x00, /*_formattype*/ AUDIO10_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.2.5) */\
  TUD_AUDIO10_DESC_TYPE_I_FORMAT(/*_nrchannels*/ 0x02, /*_subframesize*/ _nBytesPerSample, /*_bitresolution*/ _nBitsUsedPerSample, /*_freqs*/ __VA_ARGS__),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */\
  TUD_AUDIO10_DESC_STD_AS_ISO_EP(/*_ep*/ _epout, /*_attr*/ (uint8_t) ((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ADAPTIVE), /*_maxEPsize*/ _epoutsize, /*_interval*/ 0x01, /*_sync_ep*/ 0x00),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */\
  TUD_AUDIO10_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO10_CS_AS_ISO_DATA_EP_ATT_SAMPLING_FRQ, /*_lockdelayunits*/ AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0001)

#ifdef __cplusplus
}
#endif

#endif