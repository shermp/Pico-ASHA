#pragma once

#include <cstdint>
#include <cstring>
#include <array>

#include <bluetooth_gatt.h>

namespace asha 
{

constexpr std::array<uint8_t, 16> uuid_from_str(const char uuid_str[37])
{
    auto conv_nibble = [](unsigned char n) {
        if (n >= '0' && n <= '9') {
            return n - '0';
        } else if (n >= 'a' && n <= 'f') {
            return (n - 'a') + 10;
        } else {
            return (n - 'A') + 10;
        }
    };
    std::array<uint8_t, 16> uuid = {};
    const char *u = uuid_str;
    for (auto& v : uuid) {
        if (*u == '-') {
            u += 1;
        }
        uint8_t n1 = conv_nibble(u[1]);
        uint8_t n2 = conv_nibble(u[0]);
        v = n1 | (n2 << 4);
        u += 2;
    }
    return uuid;
}

/* UUID's for the ASHA service and it's characteristics */
namespace AshaUUID
{
    // 16 bit ASHA service UUID, little endian
    inline constexpr uint16_t service16 = 0xFDF0;

    inline constexpr std::array<uint8_t, 16> service = uuid_from_str("0000FDF0-0000-1000-8000-00805F9B34FB");
    inline constexpr std::array<uint8_t, 16> readOnlyProps = uuid_from_str("6333651e-c481-4a3e-9169-7c902aad37bb");
    inline constexpr std::array<uint8_t, 16> audioControlPoint = uuid_from_str("f0d4de7e-4a88-476c-9d9f-1937b0996cc0");
    inline constexpr std::array<uint8_t, 16> audioStatus = uuid_from_str("38663f1a-e711-4cac-b641-326b56404837");
    inline constexpr std::array<uint8_t, 16> volume = uuid_from_str("00e4ca9e-ab14-41e4-8823-f9e70c7e91df");
    inline constexpr std::array<uint8_t, 16> psm = uuid_from_str("2d410339-82b6-42aa-b34e-e2e01df8cc1a");
}

/* MFI hearing aid UUID. Not all devices advertise the ASHA service as
   they should, but do advertise the MFI service.
   
   7d74f4bd-c74a-4431-862c-cce884371592 */
inline constexpr std::array<uint8_t, 16> mfiUUID = uuid_from_str("7d74f4bd-c74a-4431-862c-cce884371592");

namespace GapUUID
{
    constexpr uint16_t service16 = ORG_BLUETOOTH_SERVICE_GENERIC_ACCESS;
    constexpr uint16_t deviceName16 = ORG_BLUETOOTH_CHARACTERISTIC_GAP_DEVICE_NAME;
}

namespace DisUUID
{
    constexpr uint16_t service16 = ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION;
    constexpr uint16_t ManNameString = ORG_BLUETOOTH_CHARACTERISTIC_MANUFACTURER_NAME_STRING;
    constexpr uint16_t ModelNumString = ORG_BLUETOOTH_CHARACTERISTIC_MODEL_NUMBER_STRING;
    constexpr uint16_t FWRevString = ORG_BLUETOOTH_CHARACTERISTIC_FIRMWARE_REVISION_STRING;
}

inline bool uuid_eq(const uint8_t* u1, const uint8_t* u2) { return memcmp(u1, u2, sizeof(AshaUUID::service)) == 0; }
inline bool uuid_eq(const uint8_t* u1, const std::array<uint8_t, 16>& u2) { return memcmp(u1, u2.data(), sizeof(AshaUUID::service)) == 0; }

} // namespace asha