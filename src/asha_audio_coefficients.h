#pragma once

#include <stdint.h>

#include "asha_fir.h"

/************ Only for inclusion by asha_fir.c *****************/

/*
 * G.722-shaped FIR decimator for the fixed 48 kHz to 16 kHz audio path.
 *
 * Listening tests found that passing the whole 7 kHz to 8 kHz region into
 * G.722 made sibilants less defined.  Although that region does not alias
 * during decimation, it shares G.722's coarsely quantised two-bit upper
 * subband with useful consonant energy.  This design therefore follows the
 * original filter's codec input shaping:
 *
 *     passband:   0 Hz .. 6.8 kHz
 *     transition: 6.8 kHz .. 7.8 kHz
 *     stopband:   7.8 kHz .. 24 kHz
 *
 * Design: 253 taps, Kaiser beta 8.0, 7.28 kHz cutoff, unity DC gain.
 * The 126-sample group delay is exactly 42 output samples (2.625 ms).
 *
 * The coefficients were rounded directly to Q15 and the centre coefficient
 * was corrected so the integer coefficient sum is exactly 32768.
 *
 * Measured response of the deployed Q15 values:
 *     passband ripple, 0 Hz .. 6.8 kHz: 0.0058 dB peak-to-peak
 *     gain at 7 kHz:                    -0.4095 dB
 *     gain at 7.28 kHz:                 -6.0147 dB
 *     worst rejection, 7.8 kHz .. 24 kHz: 66.8 dB
 */

const int16_t asha_fir_coefficients[ASHA_FIR_NUM_TAPS] = {
       0,    0,    0,    0,    0,    0,    1,    0,   -1,   -1,   -1,    0,
       1,    1,    0,   -2,   -2,   -1,    2,    3,    2,   -2,   -4,   -3,
       1,    5,    5,    1,   -5,   -7,   -3,    5,    9,    6,   -3,  -11,
     -10,    0,   12,   15,    4,  -11,  -19,  -11,    8,   22,   18,   -3,
     -24,  -26,   -5,   23,   34,   16,  -18,  -40,  -29,   10,   44,   42,
       3,  -43,  -56,  -21,   37,   67,   42,  -24,  -75,  -65,    4,   75,
      87,   24,  -67, -108,  -57,   49,  122,   93,  -20, -126, -131,  -21,
     118,  166,   72,  -94, -193, -131,   53,  207,  194,    8, -203, -255,
     -87,  175,  309,  183, -119, -347, -292,   29,  362,  411,   99, -341,
    -532, -271,  273,  651,  498, -137, -760, -803, -106,  853, 1250,  557,
    -924,-2071,-1614,  968, 4920, 8500, 9942, 8500, 4920,  968,-1614,-2071,
    -924,  557, 1250,  853, -106, -803, -760, -137,  498,  651,  273, -271,
    -532, -341,   99,  411,  362,   29, -292, -347, -119,  183,  309,  175,
     -87, -255, -203,    8,  194,  207,   53, -131, -193,  -94,   72,  166,
     118,  -21, -131, -126,  -20,   93,  122,   49,  -57, -108,  -67,   24,
      87,   75,    4,  -65,  -75,  -24,   42,   67,   37,  -21,  -56,  -43,
       3,   42,   44,   10,  -29,  -40,  -18,   16,   34,   23,   -5,  -26,
     -24,   -3,   18,   22,    8,  -11,  -19,  -11,    4,   15,   12,    0,
     -10,  -11,   -3,    6,    9,    5,   -3,   -7,   -5,    1,    5,    5,
       1,   -3,   -4,   -2,    2,    3,    2,   -1,   -2,   -2,    0,    1,
       1,    0,   -1,   -1,   -1,    0,    1,    0,    0,    0,    0,    0,
       0
};
