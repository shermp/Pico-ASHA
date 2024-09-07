#pragma once

#include <cstdint>
#include <cstring>

namespace asha 
{

/* UUID's for the ASHA service and it's characteristics */
namespace AshaUUID
{
    // 16 bit ASHA service UUID, little endian
    inline constexpr uint16_t service16 = 0xFDF0;
    // 0000FDF0-0000-1000-8000-00805F9B34FB
    inline constexpr uint8_t service[] = {0U, 0U, 253U, 240U, 0U, 0U, 16U, 0U, 128U, 0U, 0U, 128U, 95U, 155U, 52U, 251U};
    // 6333651e-c481-4a3e-9169-7c902aad37bb
    inline constexpr uint8_t readOnlyProps[] = {99U, 51U, 101U, 30U, 196U, 129U, 74U, 62U, 145U, 105U, 124U, 144U, 42U, 173U, 55U, 187U};
    // f0d4de7e-4a88-476c-9d9f-1937b0996cc0
    inline constexpr uint8_t audioControlPoint[] = {240U, 212U, 222U, 126U, 74U, 136U, 71U, 108U, 157U, 159U, 25U, 55U, 176U, 153U, 108U, 192U};
    // 38663f1a-e711-4cac-b641-326b56404837
    inline constexpr uint8_t audioStatus[] = {56U, 102U, 63U, 26U, 231U, 17U, 76U, 172U, 182U, 65U, 50U, 107U, 86U, 64U, 72U, 55U};
    // 00e4ca9e-ab14-41e4-8823-f9e70c7e91df
    inline constexpr uint8_t volume[] = {0U, 228U, 202U, 158U, 171U, 20U, 65U, 228U, 136U, 35U, 249U, 231U, 12U, 126U, 145U, 223U};
    // 2d410339-82b6-42aa-b34e-e2e01df8cc1a
    inline constexpr uint8_t psm[] = {45U, 65U, 3U, 57U, 130U, 182U, 66U, 170U, 179U, 78U, 226U, 224U, 29U, 248U, 204U, 26U};
}

/* MFI hearing aid UUID. Not all devices advertise the ASHA service as
   they should, but do advertise the MFI service.
   
   7d74f4bd-c74a-4431-862c-cce884371592 */
inline constexpr uint8_t mfiUUID[] = {125U, 116U, 244U, 189U, 199U, 74U, 68U, 49U, 134U, 44U, 204U, 232U, 132U, 55U, 21U, 146U};

namespace GapUUID
{
    constexpr uint16_t service16 = 0x1800;
    constexpr uint16_t deviceName16 = 0x2A00;
}

namespace GattUUID
{
    constexpr uint16_t service16 = 0x1801;
    constexpr uint16_t serviceChanged = 0x2A05;
}

inline bool uuid_eq(const uint8_t* u1, const uint8_t* u2) { return memcmp(u1, u2, sizeof(AshaUUID::service)) == 0; }

} // namespace asha