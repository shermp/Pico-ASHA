/* SPDX-License-Identifier: BSD-3-Clause */

#ifndef ASHA_PERF_TEST

#include <stdio.h>
#include <array>
#include <algorithm>
#include <cstring>
#include <cinttypes>
#include "btstack.h"
#include "hci_dump_embedded_stdout.h"
#include "pico/cyw43_arch.h"
#include "g722/g722_enc_dec.h"

#include "pico_asha.h"

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
enum DeviceStatus { None, Connected, ConnParam, L2Connecting, L2Connected, StreamStarting, StreamStopping, Streaming };

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
constexpr uint16_t num_frames_16khz_20ms = ASHA_PCM_AUDIO_FRAME_SIZE * 20;
constexpr uint16_t buff_size_16khz_20ms = num_frames_16khz_20ms * 2;
constexpr uint16_t buff_size_g722_20ms = buff_size_16khz_20ms / 4; // 4:1 compression ratio
constexpr uint16_t buff_size_pdu = buff_size_g722_20ms + 1;
constexpr int ms_20 = 20;

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

    bool operator==(const ReadOnlyProperties& other) const
    {
        return version == other.version &&
               side == other.side &&
               mode == other.mode &&
               csis_supported == other.csis_supported &&
               manufacturer_id == other.manufacturer_id &&
               unique_id == other.unique_id &&
               le_coc_supported == other.le_coc_supported &&
               render_delay == other.render_delay &&
               codec_16khz == other.codec_16khz &&
               codec_24khz == other.codec_24khz;
    }
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
    gatt_client_notification_t notification_listener = {};
    bool listener_registered = false;
    ReadOnlyProperties read_only_props;
    uint8_t psm;
    std::array<uint8_t ,5> acp_command_packet = {};
    std::array<uint8_t, buff_size_pdu> recv_buff = {};
    bool send_audio_packet_pending = false;

    bool operator==(const Device& other) const
    {
        return read_only_props == other.read_only_props &&
               bd_addr_cmp(addr, other.addr);
    }
    bool operator!=(const Device& other) const
    {
        return !(operator==(other));
    }
    void set_volume(int8_t volume)
    {
        if (volume > 0) { return; }
        auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle, 
                                                                              chars.vol.value_handle, 
                                                                              sizeof(int8_t), 
                                                                              (uint8_t*)&volume);
        if (res != ERROR_CODE_SUCCESS) {
            printf("Writing to volume char failed with %d\n", res);
        }
    }
    void send_status_update(int8_t update)
    {
        if (status != DeviceStatus::Streaming) {
            printf("Device not streaming. Not sending status update\n");
            return;
        }
        if (update != ACPStatus::conn_param_updated && 
            update != ACPStatus::other_connected && 
            update != ACPStatus::other_disconnected) {
                printf("Unknown status command\n");
                return;
            }
        std::array<uint8_t,2> cmd = {ACPOpCode::status, update};
        auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                              chars.acp.value_handle,
                                                                              cmd.size(),
                                                                              cmd.data());
        if (res != ERROR_CODE_SUCCESS) {
            printf("Updating status via ACP failed with %d\n", res);
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
    bool audio_send_pending()
    {
        return (leftOrMono.send_audio_packet_pending || right.send_audio_packet_pending);
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
            printf("Already have complete set\n");
            return nullptr;
        }
        if (dev.status == DeviceStatus::None) {
            printf("Cannot add non-connected device\n");
            return nullptr;
        }
        if (dev.read_only_props.mode == DeviceMode::Monaural) {
            if (num_devices > 0) {
                printf("Cannot add mono device when a device has been added\n");
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
            printf("Identical device already in DeviceManager\n");
            return nullptr;
        }
        if (same.status != DeviceStatus::None) {
            printf("Another device device already in DeviceManager\n");
            return nullptr;
        }
        if (other.status != DeviceStatus::None && dev.read_only_props.unique_id != other.read_only_props.unique_id) {
            printf("Device not part of same set\n");
            return nullptr; 
        }
        num_devices++;
        if (dev.read_only_props.side == DeviceSide::Left) {
            printf("Adding left device\n");
            leftOrMono = dev;
            return &leftOrMono;
        } else {
            printf("Adding right device\n");
            right = dev;
            return &right;
        }
        return nullptr;
    }
    bool remove_device(const Device& dev)
    {
        if (dev == leftOrMono) {
            printf("Removing left device\n");
            leftOrMono = Device();
            num_devices--;
            return true;
        } else if (dev == right) {
            printf("Right left device\n");
            right = Device();
            num_devices--;
            return true;
        }
        if (num_devices == 0) {
            printf("All devices removed\n");
            set_mode = DeviceMode::Binaural;
        }
        return false;
    }
private:
    Device leftOrMono;
    Device right;
    DeviceMode set_mode = DeviceMode::Binaural;
    size_t num_devices = 0;
};

struct Encoder 
{
    std::array<uint8_t, buff_size_pdu> left_g722;
    std::array<uint8_t, buff_size_pdu> right_g722;
    std::array<uint8_t, buff_size_pdu> mono_g722;

    void audio_init() 
    {
        seq_num = 0;
        first = true;
        g722_encode_init(&left_enc_state, 64000, G722_PACKED);
        g722_encode_init(&right_enc_state, 64000, G722_PACKED);
        g722_encode_init(&mono_enc_state, 64000, G722_PACKED);
        std::fill(left_g722.begin(), left_g722.end(), 0);
        std::fill(right_g722.begin(), right_g722.end(), 0);
        std::fill(mono_g722.begin(), mono_g722.end(), 0);
    }

    void encode_next_20ms()
    {
        left_g722[0] = seq_num;
        right_g722[0] = seq_num;
        mono_g722[0] = seq_num;

        if (first) {
            if (pcm_buff.write_index < 40) {
                pcm_index = 40;
            } else {
                pcm_index = pcm_buff.write_index - 5 - ms_20;
            }
            first = false;
        }
        while (pcm_buff.write_index < pcm_index) {
            ;
        }
        //printf("A:%d U:%d ", pcm_index, pcm_buff.write_index);
        
        // asha_pcm_copy_audio(pcm_buff.left, pcm_index, tmp_pcm.data(), ms_20);
        // g722_encode(&left_enc_state, left_g722.data() + 1, tmp_pcm.data(), tmp_pcm.size());

        asha_pcm_copy_audio(pcm_buff.right, pcm_index, tmp_pcm.data(), ms_20);
        g722_encode(&right_enc_state, right_g722.data() + 1, tmp_pcm.data(), tmp_pcm.size());

        // asha_pcm_copy_audio(pcm_buff.mono, pcm_index, tmp_pcm.data(), ms_20);
        // g722_encode(&mono_enc_state, mono_g722.data() + 1, tmp_pcm.data(), tmp_pcm.size());

        pcm_index += ms_20;
        ++seq_num;
    }

private:
    g722_encode_state_t left_enc_state = {};
    g722_encode_state_t right_enc_state = {};
    g722_encode_state_t mono_enc_state = {};
    uint8_t seq_num = 0;
    uint32_t pcm_index = 0;
    bool first = false;

    std::array<int16_t, ASHA_PCM_AUDIO_FRAME_SIZE * 20> tmp_pcm;
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
constexpr uint16_t asha_conn_latency  = 0;

static GATTState gatt_state = GATTState::Start;
//static Device left;
//static Device right;
static DeviceManager device_mgr;

static Encoder encoder;
static volatile EncodeState encode_state = EncodeState::Reset;

static int8_t curr_volume = -127;
static std::array<uint8_t, buff_size_pdu> curr_left_buff = {};
static std::array<uint8_t, buff_size_pdu> curr_right_buff = {};
static std::array<uint8_t, buff_size_pdu> curr_mono_buff = {};

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static btstack_timer_source_t audio_timer;
static void audio_starter();

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

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
            printf("Got AUS Characteristic\n");
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
    gatt_client_listen_for_characteristic_value_updates(
        &dev.notification_listener, 
        handle_char_notification_packet,
        dev.conn_handle,
        &dev.chars.aus
    );

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
            printf("Enabling AudioStatusPoint notifications failed with error code: 0x%02x\n"
                   "Disconnecting\n", att_status);
            gatt_state = GATTState::Disconnecting;
            gap_disconnect(curr_scan.device.conn_handle);
        }
        printf("AudioStatusPoint notification enabled.\n");
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
        printf("Already have complete set. Not adding current device.\n");
        gatt_state = GATTState::CompleteConnected;
        return;
    }
    if (device_mgr.device_exists(curr_scan.device)) {
        printf("Already connected to this device.\n");
        gatt_state = GATTState::Scan;
        return;
    }
    curr_scan.device.status = DeviceStatus::Connected;
    Device *d = device_mgr.add_connected_device(curr_scan.device);
    if (!d) {
        printf("Error adding this device.\n");
        gatt_state = GATTState::Scan;
        return;
    }
    if (device_mgr.have_complete_set()) {
        printf("Connected to all aid(s) in set.\n");
        gatt_state = GATTState::CompleteConnected;
    } else {
        gatt_state = GATTState::Scan;
    }
}

static void audio_starter()
{
    if (pcm_buff.streaming) {
        auto l = device_mgr.get_left_or_mono();
        auto r = device_mgr.get_right();
        if (l) {
            if (l->status == DeviceStatus::Connected) {
                printf("Setting connection params for left side.\n");
                l->status = DeviceStatus::ConnParam;
                change_conn_params(*l);
            } else if (l->status == DeviceStatus::L2Connected) {
                printf("Start streaming on left side.\n");
                l->status = DeviceStatus::StreamStarting;
                write_acp(*l, ACPOpCode::start);
            }
        }
        if (r) {
            if (r->status == DeviceStatus::Connected) {
                printf("Setting connection params for right side.\n");
                r->status = DeviceStatus::ConnParam;
                change_conn_params(*r);
            } else if (r->status == DeviceStatus::L2Connected) {
                printf("Start streaming on right side.\n");
                r->status = DeviceStatus::StreamStarting;
                write_acp(*r, ACPOpCode::start);
            }
        }
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
    printf("Connecting to L2CAP\n");
    auto res = l2cap_cbm_create_channel(&handle_cbm_l2cap_packet, 
                                        dev.conn_handle, 
                                        dev.psm, 
                                        dev.recv_buff.data(), 
                                        dev.recv_buff.size(),
                                        L2CAP_LE_AUTOMATIC_CREDITS,
                                        LEVEL_2,
                                        &dev.cid);
    if (res != ERROR_CODE_SUCCESS) {
        printf("Failure creating l2cap channel with error code: %d", res);
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
                printf("L2CAP CoC for device %s failed with status code: 0x%02x\n", bd_addr_to_str(event_address), status);
                // Try again
                create_l2cap_conn(*d);
                return;
            }
            if (cid != d->cid || handle != d->conn_handle) {
                printf("l2CAP CoC: connected device does not match current device!\n");
            }
            printf("L2CAP CoC for device %s succeeded\n", bd_addr_to_str(event_address));
            d->status = DeviceStatus::L2Connected;
            break;
        }
        case L2CAP_EVENT_CAN_SEND_NOW:
        {
            auto cid = l2cap_event_can_send_now_get_local_cid(packet);
            Device *dev = device_mgr.get_by_cid(cid);
            if (!dev) {
                printf("Could not find device with cid '%hd'\n", cid);
                return;
            }
            uint8_t *data;
            if (dev->read_only_props.mode == DeviceMode::Monaural) {
                data = curr_mono_buff.data();
            } else {
                data = (dev->read_only_props.side == DeviceSide::Left) ? curr_left_buff.data() : curr_right_buff.data();
            }
            auto res = l2cap_send(dev->cid, data, curr_left_buff.size());
            if (res != ERROR_CODE_SUCCESS) {
                printf("Failed to send l2cap data with reason: %d\n", res);
            }
            //printf("%hu ", (uint16_t)(data[0]));
            break;
        }
        // case L2CAP_EVENT_PACKET_SENT:
        // {
        //     printf("|");
        //     break;
        // }
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
    printf("Got Notification packet.\n");
    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) { 
        printf("Not GATT notification event.\n");
        return; 
    }
    auto device_handle = gatt_event_notification_get_handle(packet);
    auto dev = device_mgr.get_by_conn_handle(device_handle);
    if (!dev) {
        printf("ASP notification: cannot get device\n");
        return;
    }
    if (dev->status != DeviceStatus::StreamStarting && dev->status != DeviceStatus::StreamStopping) {
        printf("Device Status is not stopping or starting. It is: %d\n", dev->status);
        return; 
    }
    
    auto status_val = gatt_event_notification_get_value(packet);
    int8_t status = (int8_t)status_val[0];
    switch(status) {
    case ASPStatus::ok:
    {
        printf("ASP Ok.\n");
        auto es = EncodeState::Encode;
        Device *other = device_mgr.get_other(*dev);
        if (other && other->status != DeviceStatus::Streaming) {
            es = EncodeState::ResetEncode;
        }
        if (dev->status == DeviceStatus::StreamStarting) {
            dev->status = DeviceStatus::Streaming;
        }
        encode_state = es;
        break;
    }
    case ASPStatus::unkown_command:
        printf("ASP: Unknown command\n");
        break;
    case ASPStatus::illegal_params:
        printf("ASP: Illegal parameters\n");
        break;
    default:
        printf("ASP: Status: %d\n", status);
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
            printf("Device not in stream starting mode.\n");
            return;
        }
        curr_volume = pcm_buff.volume;
        dev.acp_command_packet[0] = opcode;
        dev.acp_command_packet[1] = 1U;
        dev.acp_command_packet[2] = 0U;
        dev.acp_command_packet[3] = (uint8_t)curr_volume;
        dev.acp_command_packet[4] = device_mgr.get_other(dev) ? 1 : 0;
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
            printf("Device not in stream stopping mode.\n");
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
        printf("Error registering ACP write.\n");
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
            printf("Could not get device with connection handle %hu.\n", handle);
            return;
        }
        auto status = gatt_event_query_complete_get_att_status(packet);
        if (status != ATT_ERROR_SUCCESS) {
            printf("Write to ACP failed with error: %d", status);
            dev->status = DeviceStatus::L2Connected;
        }
        printf("ACP write succeeded.\n");
        break;
    }
    }
}

static void send_audio_packets()
{
    Device *l = device_mgr.get_left_or_mono();
    Device *r = device_mgr.get_right();
    curr_left_buff = encoder.left_g722;
    curr_right_buff = encoder.right_g722;
    curr_mono_buff = encoder.mono_g722;
    bool change_vol = false;
    if (curr_volume != pcm_buff.volume) {
        curr_volume = pcm_buff.volume;
        change_vol = true;
    }
    if (l && l->status == DeviceStatus::Streaming) {
        if (change_vol) {
            l->set_volume(curr_volume);
        }
        l2cap_request_can_send_now_event(l->cid);
    }
    if (r && r->status == DeviceStatus::Streaming) {
        if (change_vol) {
            r->set_volume(curr_volume);
        }
        l2cap_request_can_send_now_event(r->cid);
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
            // gap_set_connection_parameters(0x0030, 0x0030, 8, 24, 4, 72, 10, 16);
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s\n", bd_addr_to_str(local_addr));
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
            curr_scan.device.supervision_timeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
            printf("Original connection parameters: Interval: %hu, Latency: %hu, Supervision Timeout: %hu\n", 
                curr_scan.device.orig_conn_interval, curr_scan.device.orig_conn_latency, curr_scan.device.supervision_timeout);
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
            uint16_t supervision_timeout = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
            hci_con_handle_t handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            printf("Connection parameter update complete: Interval: %hu Latency: %hu, Supervision Timeout: %hu\n", 
                conn_interval, conn_latency, supervision_timeout);
            Device *d = device_mgr.get_by_conn_handle(handle);
            if (d) {
                d->status = DeviceStatus::L2Connecting;
                create_l2cap_conn(*d);
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
        printf("Received disconnection event.\n");
        // Expected disconnection, reenable scanning
        if (gatt_state == GATTState::Disconnecting) {
            printf("Expected disconnection\n");
        } else {
            printf("Disconnected with reason: %d\n", static_cast<int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        auto d = device_mgr.get_by_conn_handle(c);
        if (d != nullptr) {
            printf("%s device disconnected.\n", (d->read_only_props.side == DeviceSide::Left) ? "Left" : "Right");
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
            printf("Identity resolving succeeded\n");
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.device.addr);
            if (device_mgr.get_by_address(curr_scan.device.addr)) {
                printf("Device already connected.\n");
                return;
            }
            gatt_state = GATTState::Connecting;
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

extern "C" void asha_main()
{
    printf("BT ASHA starting.\n");
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return;
    }
    //hci_dump_init(hci_dump_embedded_stdout_get_instance());
    //audio_mgr.init();
    printf("L2CAP Init.\n");
    l2cap_init();
    printf("SM Init.\n");
    sm_init();
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING);
    printf("GATT Client Init.\n");
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_2);

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    // btstack_run_loop_set_timer(&audio_timer, 20);
    // btstack_run_loop_set_timer_handler(&audio_timer, &handle_audio_start_timer);
    // btstack_run_loop_add_timer(&audio_timer);

    printf("HCI power on.\n");
    hci_power_control(HCI_POWER_ON);

    // Run the encoder in the main loop to avoid blocking BTStack
    while(1) {
        if (encode_state == EncodeState::Reset) {
            encoder.audio_init();
            encode_state = EncodeState::Idle;
        } else if (encode_state == EncodeState::ResetEncode) {
            encoder.audio_init();
            encoder.encode_next_20ms();
            send_audio_packets();
            encode_state = EncodeState::Encode;
        } else if (encode_state == EncodeState::Encode) {
            encoder.encode_next_20ms();
            send_audio_packets();
        }
        audio_starter();
    }
}

#endif // ASHA_PERF_TEST