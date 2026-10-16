#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace asha
{

inline constexpr std::size_t fir_num_taps = 253u;
inline constexpr std::size_t fir_input_samples = 48u;
inline constexpr std::size_t fir_output_samples = 16u;

class FirDecimator
{
public:
    void reset();
    void process(std::span<const int16_t, fir_input_samples> input,
                 std::span<int16_t, fir_output_samples> output);

private:
    std::array<int16_t, fir_num_taps + fir_input_samples - 1u> history_{};
};

} // namespace asha
