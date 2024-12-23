#pragma once

#include "asha_common.hpp"

namespace asha 
{

/* Struct to hold data from a GAP advertisment report */
struct AdvertisingReport {
    bd_addr_t address;
    uint8_t   type;
    uint8_t   event_type;
    bd_addr_type_t  address_type;
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

} // namespace asha