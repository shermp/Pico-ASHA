#pragma once

#include <cstdint>
#include "btstack.h"

#include "hearing_aid.hpp"

namespace asha {

enum class ScanState {
    Stop,
    Scan,
    IdentityResolving,
    Connecting,
    Disconnecting,
    ServiceDiscovery,
    CharDiscovery,
    ReadROP,
    ReadPSM,
    SubscribeASPNotification,
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
    uint8_t   length;
    const uint8_t * data;

    AdvertisingReport() {}

    AdvertisingReport(uint8_t* packet);

    /* Check if this advertising report contains the services advertised
       by hearing aids. It checks for the existance of the ASHA service
       types, as well as the Apple MFI service, because not all hearing 
       aids advertise ASHA (such as Oticon More) */
    bool is_hearing_aid();
#ifdef ASHA_AD_DUMP
    void dump_ad_data();
#endif
};

struct ScanResult {
    bool service_found = false;
    HA ha = HA();
    AdvertisingReport report = AdvertisingReport();
    void reset();
};

} // namespace asha