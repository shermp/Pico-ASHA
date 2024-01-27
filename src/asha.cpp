/* SPDX-License-Identifier: BSD-3-Clause */

#include <array>
#include <cstring>
#include <cinttypes>
#include "btstack.h"
#include "hci_dump_embedded_stdout.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "g722/g722_enc_dec.h"

#include "asha-test-tone.hpp"

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
static T get_val(const uint8_t *start)
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

/* Class to represent the ReadOnlyProperties characteristic */
struct ReadOnlyProperties
{
    uint8_t version;
    DeviceSide side;
    DeviceMode mode;
    bool csis_supported;
    uint16_t manufacturer_id;
    std::array<uint8_t, 6> unique_id;
    bool le_coc_supported;
    uint16_t render_delay;
    bool codec_16khz;
    bool codec_24khz;

    ReadOnlyProperties() {}

    ReadOnlyProperties(const uint8_t* data)
    {
        version = data[0];
        side = get_bit(data[1], 0) ? DeviceSide::Right : DeviceSide::Left;
        mode = get_bit(data[1], 1) ? DeviceMode::Binaural : DeviceMode::Monaural;
        csis_supported = get_bit(data[1], 2) ? true : false;
        manufacturer_id = get_val<uint16_t>(&data[2]);
        memcpy(unique_id.data(), &data[4], unique_id.size());
        le_coc_supported = get_bit(data[10], 0) ? true : false;
        render_delay = get_val<uint16_t>(&data[11]);

        uint16_t codecs = get_val<uint16_t>(&data[15]);
        codec_16khz = get_bit(codecs, 1);
        codec_24khz = get_bit(codecs, 2);
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
               version, 
               (side == DeviceSide::Left ? "Left" : "Right"),
               (mode == DeviceMode::Binaural ? "Binaural" : "Monaural"),
               (csis_supported ? "Yes" : "No"),
               manufacturer_id,
               le_coc_supported ? "Yes" : "No",
               render_delay,
               codec_16khz ? "Yes" : "No",
               codec_24khz ? "Yes" : "No");
    }
};

struct Device {
    DeviceStatus status = DeviceStatus::None;
    hci_con_handle_t conn_handle = 0;
    bd_addr_t addr = {};
    gatt_client_service_t service = {};
    uint16_t orig_conn_interval = 0;
    uint16_t orig_conn_latency = 0;
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

enum class GATTState {
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

/* Audio buffers */
constexpr uint16_t sample_rate_16khz = 16 * 1000;
constexpr uint16_t bits_per_sample = 16;
constexpr uint16_t num_frames_16khz_20ms = sample_rate_16khz * 0.02;
constexpr uint16_t buff_size_16khz_20ms = num_frames_16khz_20ms * (bits_per_sample / 8);

constexpr uint16_t test_tone_num_20ms_chunks = asha_test_tone_raw.size() / num_frames_16khz_20ms;

static GATTState gatt_state = GATTState::Start;
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
    gatt_state = GATTState::Scan;
    gap_set_scan_params(1, 0x0030, 0x0030, 0);
    gap_start_scan();
}

/* Handle discovery of ASHA service */
static void handle_service_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (gatt_state != GATTState::Service) return;
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
            gatt_state = GATTState::Disconnecting;
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
            gatt_state = GATTState::Disconnecting;
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
    if (gatt_state != GATTState::Service) return;
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
    if (gatt_state != GATTState::Service) return;
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
    if (gatt_state != GATTState::Service) return;
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
    auto side = curr_scan.device.read_only_props.side;
    auto mode = curr_scan.device.read_only_props.mode;
    if (side == DeviceSide::Left && left.status == DeviceStatus::None) {
        left = curr_scan.device;
        left.status = DeviceStatus::Connected;
    } else if (side == DeviceSide::Right && right.status == DeviceStatus::None) {
        right = curr_scan.device;
        right.status = DeviceStatus::Connected;
    }
    if (left.status != DeviceStatus::None && right.status != DeviceStatus::None) {
        printf("Both left and right hearing aids connected.\n");
        gatt_state = GATTState::Connected;
    } else if (mode == DeviceMode::Monaural) {
        printf("Connected to Mono hearing aid.\n");
        gatt_state = GATTState::Connected;
    } else {
        printf("Continue scanning.\n");
        gatt_state = GATTState::Scan;
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
            // Set 2M PHY
            gap_set_connection_phys(2);
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s\n", bd_addr_to_str(local_addr));
            start_scan();
        } else {
            gatt_state = GATTState::Stop;
        }
        break;
    /* Every time a GAP advertising report is received, handle it here */
    case GAP_EVENT_ADVERTISING_REPORT:
    {
        if (gatt_state != GATTState::Scan) return;
        curr_scan.report = AdvertisingReport(packet);
        printf("Address: %s\n", bd_addr_to_str(curr_scan.report.address));
        if (bd_addr_cmp(curr_scan.report.address, left.addr) == 0 || bd_addr_cmp(curr_scan.report.address, right.addr) == 0) {
            printf("Already connected to address\n");
            return;
        }
        
        if (curr_scan.report.is_hearing_aid()) {
            gatt_state = GATTState::Connecting;
            printf("Hearing aid discovered with addr %s. Connecting...\n", bd_addr_to_str(curr_scan.report.address));
            bd_addr_copy(curr_scan.device.addr, curr_scan.report.address);
            gap_connect(curr_scan.report.address, static_cast<bd_addr_type_t>(curr_scan.report.address_type));
        } else {
            /* Try and identify previously bonded device */
            gatt_state = GATTState::IdentityResolving;
            sm_address_resolution_lookup(curr_scan.report.address_type, curr_scan.report.address);            
        }
        break;
    }
    case HCI_EVENT_LE_META:
        switch(hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
        {
            if (gatt_state != GATTState::Connecting) return;
            curr_scan.device.conn_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            curr_scan.device.orig_conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
            curr_scan.device.orig_conn_latency = hci_subevent_le_connection_complete_get_conn_latency(packet);
            printf("Original connection parameters: Interval: %hu Latency: %hu\n", 
                curr_scan.device.orig_conn_interval, curr_scan.device.orig_conn_latency);
            gatt_state = GATTState::Service;
            printf("Device connected. Discovering ASHA service\n");
            auto res = gatt_client_discover_primary_services_by_uuid128(handle_service_discovery, curr_scan.device.conn_handle, AshaUUID::service);
            if (res != ERROR_CODE_SUCCESS) {
                printf("Could not register service query: %d\n", static_cast<int>(res));
                gatt_state = GATTState::Disconnecting;
                gap_disconnect(curr_scan.device.conn_handle);
            }
            break;
        }
        case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
        {
            uint16_t conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
            uint16_t conn_latency = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
            printf("Connection parameter update complete: Interval: %hu Latency: %hu\n", conn_interval, conn_latency);
            break;
        }
        case HCI_SUBEVENT_LE_ADVERTISING_REPORT:
            if (gatt_state != GATTState::Scan) return;
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
        if (gatt_state == GATTState::Disconnecting) {
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
        gatt_state = GATTState::Scan;
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
    if (gatt_state == GATTState::IdentityResolving) {
        switch(ev_type) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            printf("Identity resolving started\n");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            printf("Identity resolving failed\n");
            gatt_state = GATTState::Scan;
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            printf("Identity resolving succeeded\n");
            gatt_state = GATTState::Connecting;
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
