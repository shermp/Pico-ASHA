/* SPDX-License-Identifier: BSD-3-Clause */

#ifndef ASHA_PERF_TEST

#include <stdio.h>
#include <array>
#include <algorithm>
#include <cstring>
#include <cinttypes>
#include <btstack.h>
#include <pico/cyw43_arch.h>
#include <pico/time.h>
#ifdef ASHA_HCI_DUMP
#include <hci_dump_embedded_stdout.h>
#endif

#include "asha_audio.h"

#if defined (ASHA_LOG_ERROR)
    #define LOG_ERROR(...) printf(__VA_ARGS__)
    #define LOG_INFO(...)
    #define LOG_AUDIO(...)
#elif defined (ASHA_LOG_INFO)
    #define LOG_ERROR(...) printf(__VA_ARGS__)
    #define LOG_INFO(...) printf(__VA_ARGS__)
    #define LOG_AUDIO(...)
#elif defined (ASHA_LOG_AUDIO)
    #define LOG_ERROR(...) printf(__VA_ARGS__)
    #define LOG_INFO(...) printf(__VA_ARGS__)
    #define LOG_AUDIO(...) printf(__VA_ARGS__)
#else
    #define LOG_ERROR(...)
    #define LOG_INFO(...)
    #define LOG_AUDIO(...)
#endif


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
enum DeviceStatus { 
    None, 
    Connected, 
    ConnParam, 
    ConnParamChanged, 
    L2Connecting, 
    L2Connected, 
    StreamStarting, 
    StreamReady, 
    Streaming, 
    StreamStopping 
};

namespace AshaVol
{
    constexpr int8_t mute = -128;
    constexpr int8_t min  = -127;
    constexpr int8_t max  = 0;
}

namespace ASPStatus
{
    constexpr int8_t unkown_command = -1;
    constexpr int8_t illegal_params = -2;
    constexpr int8_t ok = 0;
}

namespace ACPOpCode
{
    constexpr uint8_t start = 1;
    constexpr uint8_t stop = 2;
    constexpr uint8_t status = 3;
}

namespace ACPStatus
{
    constexpr uint8_t other_disconnected = 0;
    constexpr uint8_t other_connected = 1;
    constexpr uint8_t conn_param_updated = 2;
}

enum EncodeState { Reset, Idle, Encode, ResetEncode };

/* Audio buffers */
// constexpr uint16_t num_frames_16khz_20ms = ASHA_PCM_AUDIO_FRAME_SIZE * 20;
// constexpr uint16_t buff_size_16khz_20ms = num_frames_16khz_20ms * 2;
// constexpr uint16_t buff_size_g722_20ms = buff_size_16khz_20ms / 4; // 4:1 compression ratio
constexpr int num_pdu_to_buffer = 8;
constexpr uint16_t buff_size_sdu = ASHA_SDU_SIZE_BYTES;
constexpr int ms_20 = 20;

#ifdef ASHA_DELETE_PAIRINGS
constexpr bool delete_pairings = true;
#else
constexpr bool delete_pairings = false;
#endif

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
         LOG_INFO("Read Only Properties\n"
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

    bool operator==(const ReadOnlyProperties& other) const = default;
    bool operator!=(const ReadOnlyProperties& other) const
    {
        return !(operator==(other));
    }
};

struct Device {
    DeviceStatus status = DeviceStatus::None;
    hci_con_handle_t conn_handle = 0;
    bd_addr_t addr = {};
    gatt_client_service_t service = {};
    uint16_t cid = 0;
    uint16_t orig_conn_interval = 0;
    uint16_t orig_conn_latency = 0;
    uint16_t supervision_timeout = 0;
    struct Characteristics {
        gatt_client_characteristic_t rop = {};
        gatt_client_characteristic_t acp = {};
        gatt_client_characteristic_t aus = {};
        gatt_client_characteristic_t vol = {};
        gatt_client_characteristic_t psm = {};
    } chars;
    bool listener_registered = false;
    ReadOnlyProperties read_only_props;
    uint8_t psm;
    std::array<uint8_t ,5> acp_command_packet = {};
    std::array<uint8_t, buff_size_sdu> recv_buff = {};
    bool audio_send_pending = false;
    uint8_t *audio_packet = nullptr;
    int8_t curr_vol = -128;

    bool operator==(const Device& other) const
    {
        return read_only_props == other.read_only_props &&
               bd_addr_cmp(addr, other.addr) == 0;
    }
    bool operator!=(const Device& other) const
    {
        return !(operator==(other));
    }
    const char* get_side_str() const
    {
        return (read_only_props.side == DeviceSide::Left) ? "Left" : "Right";
    }
    void set_volume(int8_t volume)
    {
        if (volume > 0) { return; }
        auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle, 
                                                                              chars.vol.value_handle, 
                                                                              sizeof(int8_t), 
                                                                              (uint8_t*)&volume);
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: Writing to volume char failed with %d\n", get_side_str(), res);
        }
    }
    void send_status_update(uint8_t update)
    {
        if (status != DeviceStatus::Streaming) {
            LOG_INFO("%s: Device not streaming. Not sending status update\n", get_side_str());
            return;
        }
        if (update != ACPStatus::conn_param_updated && 
            update != ACPStatus::other_connected && 
            update != ACPStatus::other_disconnected) {
                LOG_ERROR("%s: Unknown status command\n", get_side_str());
                return;
            }
        std::array<uint8_t,2> cmd = {ACPOpCode::status, update};
        auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                              chars.acp.value_handle,
                                                                              cmd.size(),
                                                                              cmd.data());
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: Updating status via ACP failed with %d\n", get_side_str(), res);
        }
        return;
    }
};

struct DeviceManager {
    Device* get_by_address(bd_addr_t addr)
    {
        if (bd_addr_cmp(leftOrMono.addr, addr) == 0) {
            return &leftOrMono;
        } else if (bd_addr_cmp(right.addr, addr) == 0) {
            return &right;
        }
        return nullptr;
    }
    Device* get_by_conn_handle(hci_con_handle_t conn_handle)
    {
        if (leftOrMono.conn_handle == conn_handle) {
            return &leftOrMono;
        } else if (right.conn_handle == conn_handle) {
            return &right;
        }
        return nullptr;
    }
    Device* get_by_cid(uint16_t cid)
    {
        if (leftOrMono.cid == cid) {
            return &leftOrMono;
        } else if (right.cid == cid) {
            return &right;
        }
        return nullptr;
    }
    Device* get_other(Device& d) 
    {
        if (device_exists(d) && set_is_binaural() && have_complete_set()) {
            return (d.read_only_props.side == DeviceSide::Left) ? &right : &leftOrMono;
        }
        return nullptr;
    }
    Device* get_left_or_mono() {
        if (leftOrMono.status != DeviceStatus::None) {
            return &leftOrMono;
        }
        return nullptr;
    }
    Device* get_right() {
        if (right.status != DeviceStatus::None) {
            return &right;
        }
        return nullptr;
    }
    bool have_complete_set()
    {
        if (num_devices == 0) { return false; }
        if (num_devices == 1 && !set_is_binaural()) { return true; }
        if (num_devices == 2 && leftOrMono.read_only_props.unique_id == right.read_only_props.unique_id) { return true; }
        return false;
    }
    bool set_is_binaural() {
        return set_mode == DeviceMode::Binaural;
    }
    bool device_exists(const Device& dev)
    {
        return dev == leftOrMono || dev == right;
    }
    
    Device* add_connected_device(const Device& dev)
    {
        if (have_complete_set()) {
            LOG_INFO("Already have complete set\n");
            return nullptr;
        }
        if (dev.status == DeviceStatus::None) {
            LOG_INFO("%s: Cannot add non-connected device\n", dev.get_side_str());
            return nullptr;
        }
        if (dev.read_only_props.mode == DeviceMode::Monaural) {
            if (num_devices > 0) {
                LOG_INFO("%s: Cannot add mono device when a device has been added\n", dev.get_side_str());
                return nullptr;
            }
            set_mode = DeviceMode::Monaural;
            leftOrMono = dev;
            num_devices++;
            return &leftOrMono;
        }
        Device& same = (dev.read_only_props.side == DeviceSide::Left) ? leftOrMono : right;
        Device& other = (dev.read_only_props.side == DeviceSide::Left) ? right : leftOrMono;
        if (dev == same) {
            LOG_INFO("%s: Identical device already in DeviceManager\n", dev.get_side_str());
            return nullptr;
        }
        if (same.status != DeviceStatus::None) {
            LOG_INFO("%s: Another device device already in DeviceManager\n", dev.get_side_str());
            return nullptr;
        }
        if (other.status != DeviceStatus::None && dev.read_only_props.unique_id != other.read_only_props.unique_id) {
            LOG_INFO("%s: Device not part of same set\n", dev.get_side_str());
            return nullptr; 
        }
        num_devices++;
        if (dev.read_only_props.side == DeviceSide::Left) {
            LOG_INFO("%s: Adding device\n", dev.get_side_str());
            leftOrMono = dev;
            return &leftOrMono;
        } else {
            LOG_INFO("%s: Adding device\n", dev.get_side_str());
            right = dev;
            return &right;
        }
        return nullptr;
    }
    bool remove_device(const Device& dev)
    {
        if (dev == leftOrMono) {
            LOG_INFO("%s: Removing device\n", dev.get_side_str());
            leftOrMono = Device();
            num_devices--;
            return true;
        } else if (dev == right) {
            LOG_INFO("%s: Removing device\n", dev.get_side_str());
            right = Device();
            num_devices--;
            return true;
        }
        if (num_devices == 0) {
            LOG_INFO("All devices removed\n");
            set_mode = DeviceMode::Binaural;
        }
        return false;
    }
    bool audio_send_pending()
    {
        return  (leftOrMono.status == DeviceStatus::Streaming && leftOrMono.audio_send_pending) ||
                (right.status == DeviceStatus::Streaming && right.audio_send_pending);
    }
private:
    Device leftOrMono;
    Device right;
    DeviceMode set_mode = DeviceMode::Binaural;
    size_t num_devices = 0;
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
                        LOG_INFO("ASHA 16 bit service discovered\n");
                        return true;
                    }
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
                for (i = 0; i < size; i += sizeof(AshaUUID::service)) {
                    reverse_128(adv_data + i, tmp_uuid128);
                    if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                        LOG_INFO("ASHA 128 bit UUID service discovered\n");
                        return true;
                    } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                        LOG_INFO("MFI UUID discovered\n");
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
    // AudioStatusPoint notification registration
    ASPNotification,
    // Finalizing connection
    Finalizing,
    // Connecting to a peripheral to discover ASHA service
    Connecting,
    // Commanded disconnect from peripheral
    Disconnecting,
    // Complete set are connected and ready for streaming
    CompleteConnected,
};



/* Connection parameters for ASHA
   Note, connection interval is in units of 1.25ms */
constexpr uint16_t asha_conn_interval = 20 / 1.25f;
constexpr uint16_t asha_conn_latency  = 10;

static uint32_t pre_buff = num_pdu_to_buffer;
static uint32_t curr_packet_index = 0;

static GATTState gatt_state = GATTState::Start;
//static Device left;
//static Device right;
static DeviceManager device_mgr;
static std::array<Device*, 2> prepared_send = {};

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static gatt_client_notification_t notification_listener = {};

static void audio_starter();

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void delete_paired_devices();

static void handle_service_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_characteristic_discovery(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_rop_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_psm_value_read(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_char_config_write_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_char_notification_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_cbm_l2cap_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void finalise_curr_discovery();
static void change_conn_params(Device& dev);
static void create_l2cap_conn(Device& dev);
static void write_acp(Device& dev, uint8_t opcode);
static void handle_acp_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void enable_asp_notification();
static void send_audio_packet(Device& dev);
static void prep_audio_packets();
static void send_audio_packets();

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
    LOG_INFO("Start scanning.\n");
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
        LOG_INFO("ASHA service found\n");
        gatt_event_service_query_result_get_service(packet, &curr_scan.device.service);
        break;
    case GATT_EVENT_QUERY_COMPLETE:
    {
        // Older hearing aids may support MFI but not ASHA
        if (!curr_scan.service_found) {
            LOG_INFO("ASHA service not found. Continuing scanning\n");
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
            LOG_ERROR("Could not register characteristics query: %d\n", static_cast<int>(res));
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
            LOG_INFO("Got ROP Characteristic\n");
            curr_scan.device.chars.rop = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioControlPoint)) {
            LOG_INFO("Got ACP Characteristic\n");
            curr_scan.device.chars.acp = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioStatus)) {
            LOG_INFO("Got AUS Characteristic\n");
            curr_scan.device.chars.aus = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::volume)) {
            LOG_INFO("Got VOL Characteristic\n");
            curr_scan.device.chars.vol = characteristic;
        } else if (uuid_eq(characteristic.uuid128, AshaUUID::psm)) {
            LOG_INFO("Got PSM Characteristic\n");
            curr_scan.device.chars.psm = characteristic;
        }
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        LOG_INFO("ASHA characteristic discovery complete\n");
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
        LOG_INFO("Getting ReadOnlyProperties value\n");
        curr_scan.device.read_only_props = ReadOnlyProperties(
            gatt_event_characteristic_value_query_result_get_value(packet)
        );
        curr_scan.device.read_only_props.dump_values();
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        LOG_INFO("%s: Completed value read of ReadOnlyProperties\n", curr_scan.device.get_side_str());
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
        LOG_INFO("%s: Getting PSM value\n", curr_scan.device.get_side_str());
        curr_scan.device.psm = gatt_event_characteristic_value_query_result_get_value(packet)[0];
        LOG_INFO("%s: PSM: %d\n", curr_scan.device.get_side_str(), static_cast<int>(curr_scan.device.psm));
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        LOG_INFO("%s: Completed value read of PSM\n", curr_scan.device.get_side_str());
        gatt_state = GATTState::ASPNotification;
        enable_asp_notification();
        break;
    default:
        break;
    }
}

/* Subscribe to AudioStatusPoint notification */
static void enable_asp_notification()
{
    if (gatt_state != GATTState::ASPNotification) { return; }
    Device& dev = curr_scan.device;
    dev.listener_registered = true;

    gatt_client_write_client_characteristic_configuration(
        handle_char_config_write_packet,
        dev.conn_handle,
        &dev.chars.aus,
        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
    );
}

/* Handle completion of ASP notification subscription */
static void handle_char_config_write_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{   
    if (gatt_state != GATTState::ASPNotification) { return; }
    switch(hci_event_packet_get_type(packet)) {
    case GATT_EVENT_QUERY_COMPLETE: 
    {
        auto att_status = gatt_event_query_complete_get_att_status(packet);
        if (att_status != ATT_ERROR_SUCCESS) {
            LOG_ERROR("%s: Enabling AudioStatusPoint notifications failed with error code: 0x%02x\n"
                   "Disconnecting\n", curr_scan.device.get_side_str(), att_status);
            gatt_state = GATTState::Disconnecting;
            gap_disconnect(curr_scan.device.conn_handle);
        }
        LOG_INFO("%s: AudioStatusPoint notification enabled.\n", curr_scan.device.get_side_str());
        gatt_state = GATTState::Finalizing;
        finalise_curr_discovery();
        break;
    }
    default:
        break;
    }
}

/* Once the Read Only Properties and PSM characteristics have been read,
   and the AudioStatusPoint notification subscribed to, determine what Device 
   to assign it to, and whether to continue searching for another aid. */
static void finalise_curr_discovery()
{
    // This shouldn't be the case, but best be sure.
    if (device_mgr.have_complete_set()) {
        LOG_INFO("%s: Already have complete set. Not adding current device.\n", curr_scan.device.get_side_str());
        gatt_state = GATTState::CompleteConnected;
        return;
    }
    if (device_mgr.device_exists(curr_scan.device)) {
        LOG_INFO("%s: Already connected to this device.\n", curr_scan.device.get_side_str());
        gatt_state = GATTState::Scan;
        return;
    }
    curr_scan.device.status = DeviceStatus::Connected;
    Device *d = device_mgr.add_connected_device(curr_scan.device);
    if (!d) {
        LOG_ERROR("%s: Error adding this device.\n", curr_scan.device.get_side_str());
        gatt_state = GATTState::Scan;
        return;
    }
    if (device_mgr.have_complete_set()) {
        LOG_INFO("Connected to all aid(s) in set.\n");
        gatt_state = GATTState::CompleteConnected;
    } else {
        gatt_state = GATTState::Scan;
    }
}

static void audio_starter()
{
    if (asha_shared.pcm_streaming) {
        Device* l = device_mgr.get_left_or_mono();
        Device* r = device_mgr.get_right();
        uint32_t packet_index = (asha_shared.packet_index > 0) ? asha_shared.packet_index - 1 : 0;
        for (Device* d : {l, r}) {
            if (d) {
                const char* side_str = d->get_side_str();
                Device *o = device_mgr.get_other(*d);
                switch (d->status) {
                case DeviceStatus::Connected:
                    LOG_INFO("%s: Setting connection params.\n", side_str);
                    d->status = DeviceStatus::ConnParam;
                    change_conn_params(*d);
                    break;
                case DeviceStatus::ConnParamChanged:
                    LOG_INFO("%s: Connecting to L2CAP\n", side_str);
                    d->status = DeviceStatus::L2Connecting;
                    create_l2cap_conn(*d);
                    break;
                case DeviceStatus::L2Connected:
                    LOG_INFO("%s: Start streaming on left side.\n", side_str);
                    d->status = DeviceStatus::StreamStarting;
                    write_acp(*d, ACPOpCode::start);
                    break;
                case DeviceStatus::StreamReady:
                {
                    uint32_t p_index = (asha_shared.packet_index > 0) ? asha_shared.packet_index - 1 : 0;
                    if (!o) {
                        d->status = DeviceStatus::Streaming;
                        curr_packet_index = p_index;
                        pre_buff = num_pdu_to_buffer;
                        asha_shared.encode_audio = true;
                    } else {
                        if (o->status == DeviceStatus::StreamReady) {
                            d->status = DeviceStatus::Streaming;
                            d->send_status_update(ACPStatus::other_connected);

                            o->status = DeviceStatus::Streaming;
                            o->send_status_update(ACPStatus::other_connected);

                            curr_packet_index = p_index;
                            pre_buff = num_pdu_to_buffer;
                            asha_shared.encode_audio = true;
                        }
                    }
                    break;
                }
                default:
                    break;
                }
                
            }
        }
    } else {
        LOG_AUDIO("pcm_streaming: false\n");
    }
}

/* Change connection parameters for device */
static void change_conn_params(Device& dev)
{
    if (dev.status != DeviceStatus::ConnParam) { return; }
    // Note see the hci_event_handler callback for the completion
    // of this update
    gap_update_connection_parameters(
        dev.conn_handle,
        asha_conn_interval,
        asha_conn_interval,
        asha_conn_latency,
        dev.supervision_timeout
    );
}

/* Create a L2CAP CoC connection to HA */
static void create_l2cap_conn(Device& dev)
{
    if (dev.status != DeviceStatus::L2Connecting) { return; }
    auto res = l2cap_cbm_create_channel(&handle_cbm_l2cap_packet, 
                                        dev.conn_handle, 
                                        dev.psm, 
                                        dev.recv_buff.data(), 
                                        dev.recv_buff.size(),
                                        L2CAP_LE_AUTOMATIC_CREDITS,
                                        LEVEL_2,
                                        &dev.cid);
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Failure creating l2cap channel with error code: %d", dev.get_side_str(), res);
    }
}

static void handle_cbm_l2cap_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    switch (packet_type) {
    case HCI_EVENT_PACKET:
        switch (hci_event_packet_get_type(packet)) {
        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
        {
            bd_addr_t event_address;
            l2cap_event_cbm_channel_opened_get_address(packet, event_address);
            uint16_t psm = l2cap_event_cbm_channel_opened_get_psm(packet);
            uint16_t cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
            hci_con_handle_t handle = l2cap_event_cbm_channel_opened_get_handle(packet);
            uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
            Device *d = device_mgr.get_by_conn_handle(handle);
            if (!d || d->status != DeviceStatus::L2Connecting) { return; }
            if (status != ERROR_CODE_SUCCESS) {
                LOG_ERROR("%s: L2CAP CoC for device %s failed with status code: 0x%02x\n", d->get_side_str(), bd_addr_to_str(event_address), status);
                // Try again
                create_l2cap_conn(*d);
                return;
            }
            if (cid != d->cid || handle != d->conn_handle) {
                LOG_ERROR("%s: l2CAP CoC: connected device does not match current device!\n", d->get_side_str());
            }
            LOG_INFO("%s: L2CAP CoC for device %s succeeded\n", d->get_side_str(), bd_addr_to_str(event_address));
            d->status = DeviceStatus::L2Connected;
            break;
        }
        // case L2CAP_EVENT_CAN_SEND_NOW:
        // {
        //     auto cid = l2cap_event_can_send_now_get_local_cid(packet);
        //     Device* d = device_mgr.get_by_conn_handle(cid);
        //     if (!d) return;
        //     LOG_AUDIO("%s: Can send now event\n", d->get_side_str());
        //     auto res = l2cap_send(d->cid, d->audio_packet, buff_size_sdu);
        //     if (res != ERROR_CODE_SUCCESS) {
        //         d->audio_send_pending = false;
        //         LOG_ERROR("%s: Failed to send l2cap data with reason: %d\n", d->get_side_str(), res);
        //     }
        //     break;
        // }
        case L2CAP_EVENT_PACKET_SENT:
        {
            auto cid = l2cap_event_packet_sent_get_local_cid(packet);
            Device *d = device_mgr.get_by_cid(cid);
            if (!d) {
                LOG_ERROR("Could not find device with cid '%hd'\n", cid);
                return;
            }
            LOG_AUDIO("%s: Packet sent event\n", d->get_side_str());
            d->audio_send_pending = false;
            break;
        }
        case L2CAP_EVENT_CHANNEL_CLOSED:
        {
            auto cid = l2cap_event_channel_closed_get_local_cid(packet);
            Device *dev = device_mgr.get_by_cid(cid);
            if (!dev) {
                LOG_ERROR("Could not find device with cid '%hd'\n", cid);
                return;
            }
            if (dev->status == DeviceStatus::Streaming) {
                LOG_ERROR("%s: Unexpected close of L2CAP channel\n", dev->get_side_str());
                dev->status = DeviceStatus::L2Connecting;
            } else {
                LOG_INFO("%s: L2CAP channel closed\n", dev->get_side_str());
            }
            break;
        }
        default:
            break;
        }
    default:
        break;
    }
}

/* Handle characteristic notification packet */
static void handle_char_notification_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    LOG_INFO("Got Notification packet.\n");
    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) { 
        LOG_ERROR("Not GATT notification event.\n");
        return; 
    }
    auto device_handle = gatt_event_notification_get_handle(packet);
    auto dev = device_mgr.get_by_conn_handle(device_handle);
    if (!dev) {
        LOG_ERROR("ASP notification: cannot get device\n");
        return;
    }
    if (dev->status != DeviceStatus::StreamStarting && dev->status != DeviceStatus::StreamStopping) {
        LOG_INFO("%s: Device Status is not stopping or starting. It is: %d\n", dev->get_side_str(), dev->status);
        return; 
    }
    
    auto status_val = gatt_event_notification_get_value(packet);
    int8_t status = (int8_t)status_val[0];
    switch(status) {
    case ASPStatus::ok:
    {
        LOG_INFO("%s: ASP Ok.\n", dev->get_side_str());

        if (dev->status == DeviceStatus::StreamStarting) {
            dev->status = DeviceStatus::StreamReady;
        } else if (dev->status == DeviceStatus::StreamStopping) {
            dev->status = DeviceStatus::L2Connected;
        }
        break;
    }
    case ASPStatus::unkown_command:
        LOG_INFO("%s: ASP: Unknown command\n", dev->get_side_str());
        break;
    case ASPStatus::illegal_params:
        LOG_INFO("%s: ASP: Illegal parameters\n", dev->get_side_str());
        break;
    default:
        LOG_INFO("%s: ASP: Status: %d\n", dev->get_side_str(), status);
        break;
    }
}

static void write_acp(Device& dev, uint8_t opcode)
{
    uint8_t res = ERROR_CODE_SUCCESS;
    switch (opcode) {
    case ACPOpCode::start:
    {
        if (dev.status != DeviceStatus::StreamStarting) {
            LOG_INFO("%s: Device not in stream starting mode.\n", dev.get_side_str());
            return;
        }
        Device* o = device_mgr.get_other(dev);
        dev.acp_command_packet[0] = opcode; // Start
        dev.acp_command_packet[1] = 1U;     // Codec
        dev.acp_command_packet[2] = 0U;     // Audio type
        dev.acp_command_packet[3] = (uint8_t)dev.curr_vol; // Volume
        dev.acp_command_packet[4] = (o && (o->status == DeviceStatus::StreamReady || o->status == DeviceStatus::Streaming)) ? 1 : 0; // Other state
        res = gatt_client_write_value_of_characteristic(&handle_acp_packet, 
                                                        dev.conn_handle, 
                                                        dev.chars.acp.value_handle,
                                                        (uint16_t)dev.acp_command_packet.size(),
                                                        dev.acp_command_packet.data());
        break;
    }
    case ACPOpCode::stop:
    {
        if (dev.status != DeviceStatus::StreamStopping) {
            LOG_INFO("%s: Device not in stream stopping mode.\n", dev.get_side_str());
            return;
        }
        dev.acp_command_packet[0] = opcode;
        res = gatt_client_write_value_of_characteristic(&handle_acp_packet, 
                                                        dev.conn_handle, 
                                                        dev.chars.acp.value_handle,
                                                        1U,
                                                        dev.acp_command_packet.data());
        break;
    }
    default:
        break;
    }
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Error registering ACP write.\n", dev.get_side_str());
        dev.status = DeviceStatus::L2Connected;
    }
}

static void handle_acp_packet(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_QUERY_COMPLETE:
    {
        auto handle = gatt_event_query_complete_get_handle(packet);
        Device *dev = device_mgr.get_by_conn_handle(handle);
        if (!dev) {
            LOG_ERROR("Could not get device with connection handle %hu.\n", handle);
            return;
        }
        auto status = gatt_event_query_complete_get_att_status(packet);
        if (status != ATT_ERROR_SUCCESS) {
            LOG_ERROR("%s: Write to ACP failed with error: %d", dev->get_side_str(), status);
            dev->status = DeviceStatus::L2Connected;
        }
        LOG_INFO("%s: ACP write succeeded.\n", dev->get_side_str());
        break;
    }
    }
}

static void send_audio_packet(Device& dev)
{
    LOG_AUDIO("%s: Send audio packet\n", dev.get_side_str());
    auto res = l2cap_send(dev.cid, dev.audio_packet, buff_size_sdu);
    if (res != ERROR_CODE_SUCCESS) {
        dev.audio_send_pending = false;
        LOG_ERROR("%s: Failed to send l2cap data with reason: %d\n", dev.get_side_str(), res);
    }
}

static void prep_audio_packets()
{
    if (!asha_shared.encode_audio || device_mgr.audio_send_pending()) {
        LOG_AUDIO("Not sending audio. Encode: %s  Send Pending: %s\n", 
                        asha_shared.encode_audio ? "true" : "false", 
                        device_mgr.audio_send_pending() ? "true" : "false");
        return;
    }
    Device *l = device_mgr.get_left_or_mono();
    Device *r = device_mgr.get_right();
    uint32_t packet_index = asha_shared.packet_index;
    if ((curr_packet_index + pre_buff) < packet_index)
    {
        LOG_AUDIO("Wanted packet index: %d\n", curr_packet_index + pre_buff);
        ++curr_packet_index;
        pre_buff = 0;
        if (l && l->status == DeviceStatus::Streaming) {
            LOG_AUDIO("Left: Sending audio packet\n");
            if (l->curr_vol != asha_shared.l_volume) {
                l->curr_vol = asha_shared.l_volume;
                l->set_volume(l->curr_vol);
            }
            l->audio_packet = asha_audio_get_l_buff_at_index(&asha_shared, curr_packet_index);
            l->audio_send_pending = true;
            prepared_send[0] = l;
        }
        if (r && r->status == DeviceStatus::Streaming) {
            LOG_AUDIO("Right: Sending audio packet\n");
            if (r->curr_vol != asha_shared.r_volume) {
                r->curr_vol = asha_shared.r_volume;
                r->set_volume(r->curr_vol);
            }
            r->audio_packet = asha_audio_get_r_buff_at_index(&asha_shared, curr_packet_index);
            r->audio_send_pending = true;
            prepared_send[1] = r;
        }
    }
    return;
}

static void send_audio_packets()
{
    while (prepared_send[0] || prepared_send[1]) {
        for (int i = 0; i < prepared_send.size(); ++i) {
            Device* d = prepared_send[i];
            if (d) {
                if (l2cap_can_send_packet_now(d->cid)) {
                    send_audio_packet(*d);
                    prepared_send[i] = nullptr;
                }
            }
        }
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
            // Set defaults, except min_ce_length & max_ce_length
            gap_set_connection_parameters(0x0030, 0x0030, 8, 24, 4, 72, 12, 12);
            gap_local_bd_addr(local_addr);
            LOG_INFO("BTstack up and running on %s\n", bd_addr_to_str(local_addr));
            if (delete_pairings) {
                delete_paired_devices();
            }
            start_scan();
            // btstack_run_loop_add_timer(&audio_timer);
        } else {
            gatt_state = GATTState::Stop;
        }
        break;
    /* Every time a GAP advertising report is received, handle it here */
    case GAP_EVENT_ADVERTISING_REPORT:
    {
        if (gatt_state != GATTState::Scan) return;
        curr_scan.report = AdvertisingReport(packet);
        if (device_mgr.get_by_address(curr_scan.report.address) != nullptr) {
            return;
        }
        // printf("Address: %s\n", bd_addr_to_str(curr_scan.report.address));
        if (curr_scan.report.is_hearing_aid()) {
            gatt_state = GATTState::Connecting;
            LOG_INFO("Hearing aid discovered with addr %s. Connecting...\n", bd_addr_to_str(curr_scan.report.address));
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
            curr_scan.device.supervision_timeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
            LOG_INFO("Original connection parameters: Interval: %hu, Latency: %hu, Supervision Timeout: %hu\n", 
                curr_scan.device.orig_conn_interval, curr_scan.device.orig_conn_latency, curr_scan.device.supervision_timeout);
            gatt_state = GATTState::Service;
            LOG_INFO("Device connected. Discovering ASHA service\n");
            auto res = gatt_client_discover_primary_services_by_uuid128(handle_service_discovery, curr_scan.device.conn_handle, AshaUUID::service);
            if (res != ERROR_CODE_SUCCESS) {
                LOG_ERROR("Could not register service query: %d\n", static_cast<int>(res));
                gatt_state = GATTState::Disconnecting;
                gap_disconnect(curr_scan.device.conn_handle);
            }
            break;
        }
        case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
        {
            uint16_t conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
            uint16_t conn_latency = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
            uint16_t supervision_timeout = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
            hci_con_handle_t handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            Device *d = device_mgr.get_by_conn_handle(handle);
            if (d) {
                LOG_INFO("%s: Connection parameter update complete: Interval: %hu Latency: %hu, Supervision Timeout: %hu\n", 
                          d->get_side_str(), conn_interval, conn_latency, supervision_timeout);
                d->status = DeviceStatus::ConnParamChanged;
            }
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
        LOG_INFO("Received disconnection event.\n");
        // Expected disconnection, reenable scanning
        if (gatt_state == GATTState::Disconnecting) {
            LOG_INFO("Expected disconnection\n");
        } else {
            LOG_ERROR("Disconnected with reason: %d\n", static_cast<int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        auto d = device_mgr.get_by_conn_handle(c);
        if (d != nullptr) {
            LOG_INFO("%s device disconnected.\n", (d->read_only_props.side == DeviceSide::Left) ? "Left" : "Right");
            device_mgr.remove_device(*d);
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
            //printf("Identity resolving started\n");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            //printf("Identity resolving failed\n");
            gatt_state = GATTState::Scan;
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            LOG_INFO("Identity resolving succeeded\n");
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.device.addr);
            if (device_mgr.get_by_address(curr_scan.device.addr)) {
                LOG_INFO("Device already connected.\n");
                return;
            }
            gatt_state = GATTState::Connecting;
            auto addr_type = sm_event_identity_resolving_succeeded_get_addr_type(packet);
            LOG_INFO("Connecting to address %s\n", bd_addr_to_str(curr_scan.device.addr));
            gap_connect(curr_scan.device.addr, static_cast<bd_addr_type_t>(addr_type));
            break;
        }
        default: break;
        }
    } else {
        switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            LOG_INFO("Just Works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_PAIRING_STARTED:
            LOG_INFO("Pairing started\n");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)){
            case ERROR_CODE_SUCCESS:
                LOG_INFO("Pairing complete, success\n");
                break;
            case ERROR_CODE_CONNECTION_TIMEOUT:
                LOG_ERROR("Pairing failed, timeout\n");
                break;
            case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                LOG_ERROR("Pairing failed, disconnected\n");
                break;
            case ERROR_CODE_AUTHENTICATION_FAILURE:
                LOG_ERROR("Pairing failed, authentication failure with reason: %d\n",
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

static void delete_paired_devices()
{
    LOG_INFO("Removing paired devices\n");
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            LOG_INFO("Removing: %s\n", bd_addr_to_str(addr));
            le_device_db_remove(i);
        }
    }
}

extern "C" void asha_main()
{
    LOG_INFO("BT ASHA starting.\n");
    if (cyw43_arch_init()) {
        LOG_ERROR("failed to initialise cyw43_arch\n");
        return;
    }
#ifdef ASHA_HCI_DUMP
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif
    LOG_INFO("L2CAP Init.\n");
    l2cap_init();
    LOG_INFO("SM Init.\n");
    l2cap_set_max_le_mtu(buff_size_sdu);
    sm_init();
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING);
    LOG_INFO("GATT Client Init.\n");
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_2);

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    gatt_client_listen_for_characteristic_value_updates(
        &notification_listener, 
        &handle_char_notification_packet,
        GATT_CLIENT_ANY_CONNECTION,
        NULL
    );

    absolute_time_t ble_start_time = get_absolute_time();
    LOG_INFO("HCI power on.\n");
    hci_power_control(HCI_POWER_ON);

    // Wait for all devices to connect if possible
    const int64_t max_delay_us = 10'000'000LL;
    while (!device_mgr.have_complete_set() && absolute_time_diff_us(ble_start_time, get_absolute_time()) < max_delay_us) {
        sleep_ms(500);
    }
    // If all devices are connected, or the max delay has been
    // reached, connect to L2CAP and start streaming audio
    while(1) {
        audio_starter();
        prep_audio_packets();
        send_audio_packets();
    }
}

#endif // ASHA_PERF_TEST