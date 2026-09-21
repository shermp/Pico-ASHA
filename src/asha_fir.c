#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "asha_fir.h"
#include "asha_audio_coefficients.h"

_Static_assert((ASHA_FIR_NUM_TAPS % 4u) == 1u,
               "symmetric FIR length must be 4k+1");
_Static_assert((ASHA_FIR_INPUT_SAMPLES % 3u) == 0u,
               "input block must be divisible by the decimation factor");

static inline int16_t saturate_q15(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

void asha_fir_reset(int16_t state[ASHA_FIR_STATE_SAMPLES])
{
    memset(state, 0, ASHA_FIR_STATE_SAMPLES * sizeof(*state));
}

/*
 * Coefficient symmetry combines each sample pair into one multiplication.
 * The 253-tap design therefore requires 127 multiplications per output
 * instead of 253 in the generic CMSIS routine.
 *
 * Two signed accumulators hold alternating products.  Their worst-case Q30
 * bounds are 37830 and 36282 coefficient units, both below the signed 65536
 * limit.  Only their final addition needs 64 bits, so full-scale inputs cannot
 * invoke the wraparound behaviour of the CMSIS fast Q15 implementation.
 */
void asha_fir_decimate_48_to_16_q15(
    const int16_t input[ASHA_FIR_INPUT_SAMPLES],
    int16_t output[ASHA_FIR_OUTPUT_SAMPLES],
    int16_t state[ASHA_FIR_STATE_SAMPLES])
{
    const uint32_t half = (ASHA_FIR_NUM_TAPS - 1u) / 2u;

    for (uint32_t i = 0u; i < ASHA_FIR_INPUT_SAMPLES; ++i) {
        state[ASHA_FIR_NUM_TAPS - 1u + i] = input[i];
    }

    for (uint32_t output_index = 0u;
         output_index < ASHA_FIR_OUTPUT_SAMPLES;
         ++output_index) {
        const int16_t *samples = &state[output_index * 3u];
        int32_t accumulator_even =
            (int32_t)samples[half] * asha_fir_coefficients[half];
        int32_t accumulator_odd = 0;

        for (uint32_t coefficient = 0u;
             coefficient < half;
             coefficient += 2u) {
            int32_t pair = (int32_t)samples[coefficient] +
                           samples[ASHA_FIR_NUM_TAPS - 1u - coefficient];
            accumulator_even += pair * asha_fir_coefficients[coefficient];

            pair = (int32_t)samples[coefficient + 1u] +
                   samples[ASHA_FIR_NUM_TAPS - 2u - coefficient];
            accumulator_odd += pair * asha_fir_coefficients[coefficient + 1u];
        }

        const int64_t accumulator =
            (int64_t)accumulator_even + accumulator_odd;
        output[output_index] = saturate_q15((int32_t)(accumulator >> 15));
    }

    for (uint32_t i = 0u; i < ASHA_FIR_NUM_TAPS - 1u; ++i) {
        state[i] = state[ASHA_FIR_INPUT_SAMPLES + i];
    }
}
