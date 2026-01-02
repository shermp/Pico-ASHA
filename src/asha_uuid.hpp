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

struct UUID
{
    enum class UUIDType { UUID16, UUID128 };
    const union UVar {
        uint16_t u16;
        std::array<uint8_t, 16> u128;

        constexpr UVar(const uint16_t u) : u16(u) {};
        constexpr UVar(std::array<uint8_t, 16> const& u) : u128(u) {};
    } uuid;
    const UUIDType type;

    constexpr UUID(const uint16_t uuid_16) : uuid(uuid_16), type(UUIDType::UUID16) {}

    // UUID(const uint8_t* uuid_128) {
    //     memcpy(uuid.u128.data(), uuid_128, sizeof(uuid.u128));
    //     type = UUIDType::UUID128;
    // }

    constexpr UUID(const char uuid_str[37]) : uuid(uuid_from_str(uuid_str)), type(UUIDType::UUID128) {}

    constexpr bool operator==(UUID const& rhs) const {
        switch (type) {
            case UUIDType::UUID16:
                return type == rhs.type && uuid.u16 == rhs.uuid.u16;
            case UUIDType::UUID128:
                return type == rhs.type && uuid.u128 == rhs.uuid.u128;
            default:
                return false;
        }
    }

    constexpr bool operator==(const uint16_t rhs) const {
        return type == UUIDType::UUID16 && uuid.u16 == rhs;
    }

    bool operator==(const uint8_t* rhs) const {
        return type == UUIDType::UUID128 && memcmp(uuid.u128.data(), rhs, uuid.u128.size()) == 0;
    }

    constexpr size_t size() const {
        return type == UUIDType::UUID128 ? uuid.u128.size() : sizeof(uuid.u16);
    }
};

/* UUID's for the ASHA service and it's characteristics */
namespace AshaUUID
{
    // 16 bit ASHA service UUID, little endian
    inline constexpr UUID service16(0xFDF0);
    inline constexpr UUID service("0000FDF0-0000-1000-8000-00805F9B34FB");
    inline constexpr UUID readOnlyProps("6333651e-c481-4a3e-9169-7c902aad37bb");
    inline constexpr UUID audioControlPoint("f0d4de7e-4a88-476c-9d9f-1937b0996cc0");
    inline constexpr UUID audioStatus("38663f1a-e711-4cac-b641-326b56404837");
    inline constexpr UUID volume("00e4ca9e-ab14-41e4-8823-f9e70c7e91df");
    inline constexpr UUID psm("2d410339-82b6-42aa-b34e-e2e01df8cc1a");
}

/* MFI hearing aid UUID. Not all devices advertise the ASHA service as
   they should, but do advertise the MFI service. */

namespace MfiUUID
{
    inline constexpr UUID service("7d74f4bd-c74a-4431-862c-cce884371592");
    inline constexpr UUID battery("24e1dff3-ae90-41bf-bfbd-2cf8df42bf87");
}

namespace GapUUID
{
    inline constexpr UUID service16(ORG_BLUETOOTH_SERVICE_GENERIC_ACCESS);
    inline constexpr UUID deviceName16(ORG_BLUETOOTH_CHARACTERISTIC_GAP_DEVICE_NAME);
}

namespace DisUUID
{
    inline constexpr UUID service16(ORG_BLUETOOTH_SERVICE_DEVICE_INFORMATION);
    inline constexpr UUID mfgName(ORG_BLUETOOTH_CHARACTERISTIC_MANUFACTURER_NAME_STRING);
    inline constexpr UUID modelNum(ORG_BLUETOOTH_CHARACTERISTIC_MODEL_NUMBER_STRING);
    inline constexpr UUID fwVers(ORG_BLUETOOTH_CHARACTERISTIC_FIRMWARE_REVISION_STRING);
    inline constexpr UUID swVers(ORG_BLUETOOTH_CHARACTERISTIC_SOFTWARE_REVISION_STRING);
}

} // namespace asha