#pragma once

#include <cstdint>
#include <etl/vector.h>
#include <btstack.h>

#include "hearing_aid.hpp"

namespace asha {

enum class ScanState {
    Stop,
    Scan,
    IdentityResolving,
    Connecting,
    Pairing,
    Disconnecting,
    DataLen,
    ServiceDiscovery,
    Finalizing,
    Complete,
};

constexpr uint16_t buff_size_sdu = 161;

/* Struct to hold data from a GAP advertisment report */
struct AdvertisingReport {
    bd_addr_t address;
    uint8_t   type;
    uint8_t   event_type;
    uint8_t   address_type;
    uint8_t   rssi;

    bool is_hearing_aid = false;

    AdvertisingReport() {}

    AdvertisingReport(uint8_t* packet, bool extended);

private:
    /* Check if this advertising report contains the services advertised
       by hearing aids. It checks for the existance of the ASHA service
       types, as well as the Apple MFI service, because not all hearing 
       aids advertise ASHA (such as Oticon More) */
    void check_if_ha(uint8_t length, const uint8_t * data);
};

struct ScanResult {
    bool service_found = false;
    HA ha = HA();
    AdvertisingReport report = AdvertisingReport();
    void reset();
};

} // namespace asha