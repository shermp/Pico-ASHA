#pragma once

#include <stdint.h>

#define ASHA_FIR_NUM_TAPS 253u
#define ASHA_FIR_INPUT_SAMPLES 48u
#define ASHA_FIR_OUTPUT_SAMPLES 16u
#define ASHA_FIR_STATE_SAMPLES (ASHA_FIR_NUM_TAPS + ASHA_FIR_INPUT_SAMPLES - 1u)

extern const int16_t asha_fir_coefficients[ASHA_FIR_NUM_TAPS];

void asha_fir_reset(int16_t state[ASHA_FIR_STATE_SAMPLES]);

void asha_fir_decimate_48_to_16_q15(
    const int16_t input[ASHA_FIR_INPUT_SAMPLES],
    int16_t output[ASHA_FIR_OUTPUT_SAMPLES],
    int16_t state[ASHA_FIR_STATE_SAMPLES]);
