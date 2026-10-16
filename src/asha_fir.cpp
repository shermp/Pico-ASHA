#include <limits.h>
#include <stdint.h>
#include "asha_fir.hpp"
#include "asha_audio_coefficients.hpp"

namespace asha
{

static_assert((fir_num_taps % 4u) == 1u,
               "symmetric FIR length must be 4k+1");
static_assert(fir_input_samples == 3u * fir_output_samples,
               "input and output blocks must match the decimation factor");

static inline int16_t saturate_q15(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

void FirDecimator::reset()
{
    history_.fill(0);
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
void FirDecimator::process(
    std::span<const int16_t, fir_input_samples> input,
    std::span<int16_t, fir_output_samples> output)
{
    const uint32_t half = (fir_num_taps - 1u) / 2u;

    for (uint32_t i = 0u; i < fir_input_samples; ++i) {
        history_[fir_num_taps - 1u + i] = input[i];
    }

    for (uint32_t output_index = 0u;
         output_index < fir_output_samples;
         ++output_index) {
        const int16_t *samples = &history_[output_index * 3u];
        int32_t accumulator_even =
            (int32_t)samples[half] * fir_coefficients[half];
        int32_t accumulator_odd = 0;

        for (uint32_t coefficient = 0u;
             coefficient < half;
             coefficient += 2u) {
            int32_t pair = (int32_t)samples[coefficient] +
                           samples[fir_num_taps - 1u - coefficient];
            accumulator_even += pair * fir_coefficients[coefficient];

            pair = (int32_t)samples[coefficient + 1u] +
                   samples[fir_num_taps - 2u - coefficient];
            accumulator_odd += pair * fir_coefficients[coefficient + 1u];
        }

        const int64_t accumulator =
            (int64_t)accumulator_even + accumulator_odd;
        output[output_index] = saturate_q15((int32_t)(accumulator >> 15));
    }

    for (uint32_t i = 0u; i < fir_num_taps - 1u; ++i) {
        history_[i] = history_[fir_input_samples + i];
    }
}

} // namespace asha
