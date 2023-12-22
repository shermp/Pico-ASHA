/* SPDX-License-Identifier: BSD-3-Clause */

#include <array>
#include <cstring>
#include <cinttypes>
#include "btstack.h"
#include "hci_dump_embedded_stdout.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

/* Utility functions */

/* Get bit 'pos' from value */
template<typename T>
static int get_bit(T value, const int pos)
{
    //static_assert(pos >= 0 && pos < sizeof(T) * 8);
    return (value >> pos) & static_cast<T>(1);
}

/* Get value from (sub) array of bytes */
template<typename T>
static T get_val(uint8_t *start)
{
    T val;
    std::memcpy(&val, start, sizeof(val));
    return val;
}

/* Compare equality of binary UUID's */
static bool uuid_eq(const uint8_t* u1, const uint8_t* u2)
{
    return memcmp(u1, u2, 16) == 0;
}

/* ASHA Types and variables */

/* UUID's for the ASHA service and it's characteristics */
namespace AshaUUID
{
    // 16 bit ASHA service UUID, little endian
    constexpr uint16_t service16 = 0xFDF0;
    // 0000FDF0-0000-1000-8000-00805F9B34FB
    constexpr uint8_t service[] = {0U, 0U, 253U, 240U, 0U, 0U, 16U, 0U, 128U, 0U, 0U, 128U, 95U, 155U, 52U, 251U};
    // 6333651e-c481-4a3e-9169-7c902aad37bb
    constexpr uint8_t readOnlyProps[] = {99U, 51U, 101U, 30U, 196U, 129U, 74U, 62U, 145U, 105U, 124U, 144U, 42U, 173U, 55U, 187U};
    // f0d4de7e-4a88-476c-9d9f-1937b0996cc0
    constexpr uint8_t audioControlPoint[] = {240U, 212U, 222U, 126U, 74U, 136U, 71U, 108U, 157U, 159U, 25U, 55U, 176U, 153U, 108U, 192U};
    // 38663f1a-e711-4cac-b641-326b56404837
    constexpr uint8_t audioStatus[] = {56U, 102U, 63U, 26U, 231U, 17U, 76U, 172U, 182U, 65U, 50U, 107U, 86U, 64U, 72U, 55U};
    // 00e4ca9e-ab14-41e4-8823-f9e70c7e91df
    constexpr uint8_t volume[] = {0U, 228U, 202U, 158U, 171U, 20U, 65U, 228U, 136U, 35U, 249U, 231U, 12U, 126U, 145U, 223U};
    // 2d410339-82b6-42aa-b34e-e2e01df8cc1a
    constexpr uint8_t psm[] = {45U, 65U, 3U, 57U, 130U, 182U, 66U, 170U, 179U, 78U, 226U, 224U, 29U, 248U, 204U, 26U};
}

/* MFI hearing aid UUID. Not all devices advertise the ASHA service as
   they should, but do advertise the MFI service.
   
   7d74f4bd-c74a-4431-862c-cce884371592 */
constexpr uint8_t mfiUUID[] = {125U, 116U, 244U, 189U, 199U, 74U, 68U, 49U, 134U, 44U, 204U, 232U, 132U, 55U, 21U, 146U};

enum DeviceSide { Left = 0, Right = 1 };
enum DeviceMode { Monaural = 0, Binaural = 1 };
enum DeviceStatus { None, Connected, Disconnected, Streaming };

/* Class to represent the ReadOnlyProperties characteristic

   Note, methods are used to access fields stored in an array 
   to avoid potential issues with packed structs and 
   unaligned memory access. */
struct ReadOnlyProperties
{
    ReadOnlyProperties() {}

    ReadOnlyProperties(const uint8_t* data)
    {
        std::memcpy(rop.data(), data, rop.size());
    }

    uint8_t version() { return rop[0]; }

    DeviceSide device_side() 
    { 
        return get_bit(rop[1], 0) ? DeviceSide::Right : DeviceSide::Left; 
    };

    DeviceMode device_mode() 
    { 
        return get_bit(rop[1], 1) ? DeviceMode::Binaural : DeviceMode::Monaural; 
    };

    bool supports_csis()
    {
        return get_bit(rop[1], 2) ? true : false;
    }

    uint16_t manufacture_id()
    {
        return get_val<uint16_t>(&rop[2]);
    }

    std::array<uint8_t, 6> unique_id()
    {
        return std::array<uint8_t, 6> {rop[4], rop[5], rop[6], rop[7], rop[8], rop[9]};
    }

    bool supports_le_coc_audio()
    {
        return get_bit(rop[10], 0) ? true : false;
    }

    uint16_t render_delay()
    {
        return get_val<uint16_t>(&rop[11]);
    }

    /* Check that the hearing aid supports the mandatory 
       G.722 @ 16KHz codec. */
    bool supports_16KHz()
    {
        uint16_t codecs = get_val<uint16_t>(&rop[15]);
        return get_bit(codecs, 1);
    }

    /* Check if hearing aid supports G.722 @ 24KHz sample rate.
       This is NOT in the ASHA spec, but Android checks for it. */
    bool supports_24KHz()
    {
        uint16_t codecs = get_val<uint16_t>(&rop[15]);
        return get_bit(codecs, 2);
    }

    void dump_values()
    {
        printf("Read Only Properties\n"
               "  Version:        %d\n"
               "  Side:           %s\n"
               "  Mode:           %s\n"
               "  CSIS:           %s\n"
               "  Manufacture ID: %xxu\n"
               "  LE CoC audio:   %s\n"
               "  Render delay:   %hu\n"
               "  Supports 16KHz: %s\n"
               "  Supports 24KHz: %s\n",
               version(), 
               (device_side() == DeviceSide::Left ? "Left" : "Right"),
               (device_mode() == DeviceMode::Binaural ? "Binaural" : "Monaural"),
               (supports_csis() ? "Yes" : "No"),
               manufacture_id(),
               supports_le_coc_audio() ? "Yes" : "No",
               render_delay(),
               supports_16KHz() ? "Yes" : "No",
               supports_24KHz() ? "Yes" : "No");
    }
private:
    std::array<uint8_t, 17> rop;
};

struct Device {
    DeviceStatus status = DeviceStatus::None;
    hci_con_handle_t conn_handle = 0;
    bd_addr_t addr = {};
    gatt_client_service_t service = {};
    struct Characteristics {
        gatt_client_characteristic_t rop = {};
        gatt_client_characteristic_t acp = {};
        gatt_client_characteristic_t aus = {};
        gatt_client_characteristic_t vol = {};
        gatt_client_characteristic_t psm = {};
    } chars;
    ReadOnlyProperties read_only_props;
    uint8_t psm;
};

/* Struct to hold data from a GAP advertisment report */
struct AdvertisingReport {
    uint8_t   type;
    uint8_t   event_type;
    uint8_t   address_type;
    bd_addr_t address;
    uint8_t   rssi;
    uint8_t   length;
    const uint8_t * data;

    AdvertisingReport() {}

    AdvertisingReport(uint8_t* packet)
    {
        gap_event_advertising_report_get_address(packet, address);
        event_type = gap_event_advertising_report_get_advertising_event_type(packet);
        address_type = gap_event_advertising_report_get_address_type(packet);
        rssi = gap_event_advertising_report_get_rssi(packet);
        length = gap_event_advertising_report_get_data_length(packet);
        data = gap_event_advertising_report_get_data(packet);
    }

    /* Check if this advertising report contains the services advertised
       by hearing aids. It checks for the existance of the ASHA service
       types, as well as the Apple MFI service, because not all hearing 
       aids advertise ASHA (such as Oticon More) */
    bool is_hearing_aid()
    {
        ad_context_t context;
        bd_addr_t address;
        for (ad_iterator_init(&context, length, data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)) {
            uint8_t data_type    = ad_iterator_get_data_type(&context);
            uint8_t size         = ad_iterator_get_data_len(&context);
            const uint8_t * adv_data = ad_iterator_get_data(&context);
            int i = 0;
            uint8_t tmp_uuid128[16] = {};
            switch(data_type) {
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                for (i = 0; i < size; i += sizeof(uint16_t)) {
                    if (little_endian_read_16(adv_data, i) == AshaUUID::service16) {
                        printf("ASHA 16 bit service discovered\n");
                        return true;
                    }
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
                for (i = 0; i < size; i += sizeof(AshaUUID::service)) {
                    reverse_128(adv_data + i, tmp_uuid128);
                    if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                        printf("ASHA 128 bit UUID service discovered\n");
                        return true;
                    } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                        printf("MFI UUID discovered\n");
                        return true;
                    }
                }
                break;
            default:
                break;
            }
        }
        return false;
    }
};

enum class ASHAState {
    Stop,
    // Starting state, before BT stack has initialized
    Start,
    // State when accepting GAP scan advertising reports
    Scan,
    // Resolving address of previously bonded peripheral
    IdentityResolving,
    // Service discovery, getting ASHA service
    Service,
    // Connecting to a peripheral to discover ASHA service
    Connecting,
    // Commanded disconnect from peripheral
    Disconnecting,
    // Devices are connected and ready for streaming
    Connected,
};

static ASHAState state = ASHAState::Start;
static Device left;
static Device right;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void handle_service_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_characteristic_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_rop_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_psm_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void finalise_curr_discovery();

extern "C" void le_handle_advertisement_report(uint8_t *packet, uint16_t size);

static struct ScanResult {
    bool service_found = false;
    Device device = Device();
    AdvertisingReport report = AdvertisingReport();
    void reset()
    {
        service_found = false;
        device = Device();
        report = AdvertisingReport();
    }
} curr_scan;

/* Start scanning for suitable peripherals */
static void start_scan()
{
    printf("Start scanning.\n");
    state = ASHAState::Scan;
    gap_set_scan_params(1, 0x0030, 0x0030, 0);
    gap_start_scan();
}

/* Handle discovery of ASHA service */
static void handle_service_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (state != ASHAState::Service) return;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_SERVICE_QUERY_RESULT:
        curr_scan.service_found = true;
        printf("ASHA service found\n");
        gatt_event_service_query_result_get_service(packet, &curr_scan.device.service);
        break;
    case GATT_EVENT_QUERY_COMPLETE:
    {
        // Older hearing aids may support MFI but not ASHA
        if (!curr_scan.service_found) {
            printf("ASHA service not found. Continuing scanning\n");
            state = ASHAState::Disconnecting;
            gap_disconnect(curr_scan.device.conn_handle);
            break;
        }
        // Service found. Discover characteristics
        auto res = gatt_client_discover_characteristics_for_service(
                handle_characteristic_discovery,
                curr_scan.device.conn_handle,
                &curr_scan.device.service
        );
        if (res != ERROR_CODE_SUCCESS) {
            printf("Could not register characteristics query: %d\n", static_cast<int>(res));
            state = ASHAState::Disconnecting;
            gap_disconnect(curr_scan.device.conn_handle);
        }
        break;
    }
    default:
        break;
    }
}

/* Handle discovery of ASHA characteristics */
static void handle_characteristic_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (state != ASHAState::Service) return;
    gatt_client_characteristic_t characteristic;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
        gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
        if (uuid_eq(characteristic.uuid128, AshaUUID::readOnlyProps)) {
            printf("Got ROP Characteristic\n");
            curr_scan.device.chars.rop = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioControlPoint)) {
            printf("Got ACP Characteristic\n");
            curr_scan.device.chars.acp = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioStatus)) {
            printf("Got RUS Characteristic\n");
            curr_scan.device.chars.aus = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::volume)) {
            printf("Got VOL Characteristic\n");
            curr_scan.device.chars.vol = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::psm)) {
            printf("Got PSM Characteristic\n");
            curr_scan.device.chars.psm = characteristic;
        }
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        printf("ASHA characteristic discovery complete\n");
        // Start reading the Read Only Properties characteristic
        gatt_client_read_value_of_characteristic(
            handle_rop_value_read, 
            curr_scan.device.conn_handle, 
            &curr_scan.device.chars.rop
        );
        break;
    default:
        break;
    }
}

/* Handle reading of "Read Only Properties characteristic" */
static void handle_rop_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (state != ASHAState::Service) return;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
        printf("Getting ReadOnlyProperties value\n");
        curr_scan.device.read_only_props = ReadOnlyProperties(
            gatt_event_characteristic_value_query_result_get_value(packet)
        );
        curr_scan.device.read_only_props.dump_values();
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        printf("Completed value read of ReadOnlyProperties\n");
        /* Next get the PSM value */
        gatt_client_read_value_of_characteristic(
            handle_psm_value_read,
            curr_scan.device.conn_handle,
            &curr_scan.device.chars.psm
        );
        break;
    default:
        break;
    }
}

/* Handle reading of PSM characteristic */
static void handle_psm_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (state != ASHAState::Service) return;
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
        printf("Getting PSM value\n");
        curr_scan.device.psm = gatt_event_characteristic_value_query_result_get_value(packet)[0];
        printf("PSM: %d\n", static_cast<int>(curr_scan.device.psm));
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        printf("Completed value read of PSM\n");
        finalise_curr_discovery();
        break;
    default:
        break;
    }
}

/* Once the Read Only Properties and PSM characteristics have been read,
   Determine what Device to assign it to, and whether to continue 
   searching for another aid. */
static void finalise_curr_discovery()
{
    auto side = curr_scan.device.read_only_props.device_side();
    auto mode = curr_scan.device.read_only_props.device_mode();
    if (side == DeviceSide::Left && left.status == DeviceStatus::None) {
        left = curr_scan.device;
        left.status = DeviceStatus::Connected;
    } else if (side == DeviceSide::Right && right.status == DeviceStatus::None) {
        right = curr_scan.device;
        right.status = DeviceStatus::Connected;
    }
    if (left.status != DeviceStatus::None && right.status != DeviceStatus::None) {
        printf("Both left and right hearing aids connected.\n");
        state = ASHAState::Connected;
    } else if (mode == DeviceMode::Monaural) {
        printf("Connected to Mono hearing aid.\n");
        state = ASHAState::Connected;
    } else {
        printf("Continue scanning.\n");
        state = ASHAState::Scan;
    }
}

/* Main btstack HCI event handler */
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) return;
    bd_addr_t local_addr;
    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
    /* Power on */
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s\n", bd_addr_to_str(local_addr));
            start_scan();
        } else {
            state = ASHAState::Stop;
        }
        break;
    /* Every time a GAP advertising report is received, handle it here */
    case GAP_EVENT_ADVERTISING_REPORT:
    {
        if (state != ASHAState::Scan) return;
        curr_scan.report = AdvertisingReport(packet);
        printf("Address: %s\n", bd_addr_to_str(curr_scan.report.address));
        if (bd_addr_cmp(curr_scan.report.address, left.addr) == 0 || bd_addr_cmp(curr_scan.report.address, right.addr) == 0) {
            printf("Already connected to address\n");
            return;
        }
        
        if (curr_scan.report.is_hearing_aid()) {
            state = ASHAState::Connecting;
            printf("Hearing aid discovered with addr %s. Connecting...\n", bd_addr_to_str(curr_scan.report.address));
            bd_addr_copy(curr_scan.device.addr, curr_scan.report.address);
            gap_connect(curr_scan.report.address, static_cast<bd_addr_type_t>(curr_scan.report.address_type));
        } else {
            /* Try and identify previously bonded device */
            state = ASHAState::IdentityResolving;
            sm_address_resolution_lookup(curr_scan.report.address_type, curr_scan.report.address);            
        }
        break;
    }
    case HCI_EVENT_LE_META:
        switch(hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
        {
            if (state != ASHAState::Connecting) return;
            curr_scan.device.conn_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            state = ASHAState::Service;
            printf("Device connected. Discovering ASHA service\n");
            auto res = gatt_client_discover_primary_services_by_uuid128(handle_service_discovery, curr_scan.device.conn_handle, AshaUUID::service);
            if (res != ERROR_CODE_SUCCESS) {
                printf("Could not register service query: %d\n", static_cast<int>(res));
                state = ASHAState::Disconnecting;
                gap_disconnect(curr_scan.device.conn_handle);
            }
            break;
        }
        case HCI_SUBEVENT_LE_ADVERTISING_REPORT:
            if (state != ASHAState::Scan) return;
            le_handle_advertisement_report(packet, size);
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
    {
        uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
        printf("Received disconnection event.\n");
        // Expected disconnection, reenable scanning
        if (state == ASHAState::Disconnecting) {
            printf("Expected disconnection\n");
        } else {
            printf("Disconnected with reason: %d\n", static_cast<int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        if (c == left.conn_handle) {
            printf("Left device disconnected.\n");
            left = Device();
        } else if (c == right.conn_handle) {
            printf("Right device disconnected.\n");
            right = Device();
        }
        state = ASHAState::Scan;
        curr_scan.reset();
        break;
    }
    default:
        break;
    }
}

/* Handler for security manager events */
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;
    auto ev_type = hci_event_packet_get_type(packet);
    if (state == ASHAState::IdentityResolving) {
        switch(ev_type) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            printf("Identity resolving started\n");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            printf("Identity resolving failed\n");
            state = ASHAState::Scan;
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            printf("Identity resolving succeeded\n");
            state = ASHAState::Connecting;
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.device.addr);
            auto addr_type = sm_event_identity_resolving_succeeded_get_addr_type(packet);
            printf("Connecting to address %s\n", bd_addr_to_str(curr_scan.device.addr));
            gap_connect(curr_scan.device.addr, static_cast<bd_addr_type_t>(addr_type));
            break;
        }
        default: break;
        }
    } else {
        switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("Just Works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_PAIRING_STARTED:
            printf("Pairing started\n");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)){
            case ERROR_CODE_SUCCESS:
                printf("Pairing complete, success\n");
                break;
            case ERROR_CODE_CONNECTION_TIMEOUT:
                printf("Pairing failed, timeout\n");
                break;
            case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                printf("Pairing failed, disconnected\n");
                break;
            case ERROR_CODE_AUTHENTICATION_FAILURE:
                printf("Pairing failed, authentication failure with reason: %d\n",
                        static_cast<int>(sm_event_pairing_complete_get_reason(packet)));
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
}

void asha_main()
{
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return;
    }
    //hci_dump_init(hci_dump_embedded_stdout_get_instance());
    l2cap_init();
    sm_init();
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING);
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_2);

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    hci_power_control(HCI_POWER_ON);

    while(true) {      
        sleep_ms(1000);
    }
}

int main()
{
    stdio_init_all();
    asha_main();
}
