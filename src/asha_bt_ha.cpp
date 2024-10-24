#include <cstring>

#include <pico/async_context.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio_uart.h>
#include <pico/stdio_usb.h>
#include <hardware/watchdog.h>

#include <ArduinoJson.h>
#include <etl/vector.h>
#include <etl/algorithm.h>
#include <etl/bitset.h>
#include <etl/delegate.h>

#include "asha_bt_ha.hpp"
#include "asha_audio.hpp"
#include "asha_led.hpp"
#include "asha_logging.h"
#include "asha_usb_serial.hpp"
#include "pico_bt.hpp"
#include "runtime_settings.hpp"
#include "util.hpp"

namespace asha
{

using namespace picobt;
/* Constants */

/* AudioStatusPoint notification value */
namespace ASPStatus
{
    constexpr int8_t unkown_command = -1;
    constexpr int8_t illegal_params = -2;
    constexpr int8_t ok = 0;
}

/* AudioControlPoint other side status */
namespace ACPStatus
{
    constexpr uint8_t other_disconnected = 0;
    constexpr uint8_t other_connected = 1;
    constexpr uint8_t conn_param_updated = 2;
}

/* Data length variables */

static constexpr uint16_t pdu_len = 167u;
static constexpr uint16_t max_tx_time = 1064;

/* Static function declarations */

static void handle_bt_audio_pending_worker(async_context_t *context, async_when_pending_worker_t *worker);
static void handle_stdin_line_worker(async_context_t *context, async_when_pending_worker_t *worker);

static void on_bt_started(bool started, uint8_t bt_state);
static void on_ad_report(AdReport &report);
static void on_bt_connected(uint8_t status, BT::Remote* remote);
static void on_bt_disconnected(uint8_t reason, BT::Remote* remote);

static auto on_bt_started_delegate = etl::delegate<void(bool, uint8_t)>::create<on_bt_started>();
static auto on_ad_report_delegate = etl::delegate<void(picobt::AdReport&)>::create<on_ad_report>();
static auto on_bt_connected_delegate = etl::delegate<void(uint8_t, BT::Remote*)>::create<on_bt_connected>();
static auto on_bt_disconnected_delegate = etl::delegate<void(uint8_t, BT::Remote*)>::create<on_bt_disconnected>();

static void restart_pico();

static inline void log_bt_res(BT::Result res, uint8_t err, const char* msg);

/* Utility functions */
/* Get value from (sub) array of bytes */
template<typename T>
static T get_val(const uint8_t *start)
{
    T val;
    std::memcpy(&val, start, sizeof(val));
    return val;
}

static inline void log_bt_res(BT::Result res, uint8_t err, const char* msg)
{
    switch (res) {
        case BT::Result::Ok: break;
        case BT::Result::BTError: LOG_ERROR("%s: BT error 0x%02x", msg, (err)); break;
        case BT::Result::WrongState: LOG_ERROR("%s: In wrong state", msg); break;
        case BT::Result::MaxConnections: LOG_ERROR("%s: max num connections", msg); break;
        case BT::Result::InternalError: LOG_ERROR("%s: internal picobt error", msg); break;
        default: LOG_ERROR("%s: unknown error", msg); break;
    }
}

/* UUID definitions */

/* UUID's for the ASHA service and it's characteristics */
namespace AshaUUID
{
    // 16 bit ASHA service UUID, little endian
    constexpr UUID service16(0xFDF0);
    constexpr UUID service("0000fdf0-0000-1000-8000-00805f9b34fb");
    constexpr UUID readOnlyProps("6333651e-c481-4a3e-9169-7c902aad37bb");
    constexpr UUID audioControlPoint("f0d4de7e-4a88-476c-9d9f-1937b0996cc0");
    constexpr UUID audioStatus("38663f1a-e711-4cac-b641-326b56404837");
    constexpr UUID volume("00e4ca9e-ab14-41e4-8823-f9e70c7e91df");
    constexpr UUID psm("2d410339-82b6-42aa-b34e-e2e01df8cc1a");
}

/* MFI hearing aid UUID. Not all devices advertise the ASHA service as
   they should, but do advertise the MFI service.
   
   7d74f4bd-c74a-4431-862c-cce884371592 */
namespace MfiUUID
{
    constexpr UUID service("7d74f4bd-c74a-4431-862c-cce884371592");
}

namespace GapUUID
{
    constexpr UUID service16(0x1800);
    constexpr UUID deviceName16(0x2A00);
}

/* Type definitions */

HearingAid::HearingAid(BT::Remote* r) : remote(r) 
{
    /* Set delegates */

    on_services_discovered_d.set<HearingAid, &HearingAid::on_services_discovered>(*this);
    service_filter_d.set<HearingAid, &HearingAid::service_filter>(*this);
    on_data_length_set_d.set<HearingAid, &HearingAid::on_data_length_set>(*this);
    on_paired_and_bonded_d.set<HearingAid, &HearingAid::on_paired_and_bonded>(*this);
    on_chars_discovered_d.set<HearingAid, &HearingAid::on_chars_discovered>(*this);
    discover_chars_filter_d.set<HearingAid, &HearingAid::discover_chars_filter>(*this);
    on_read_char_value_d.set<HearingAid, &HearingAid::on_read_char_value>(*this);
    on_char_values_read_d.set<HearingAid, &HearingAid::on_char_values_read>(*this);
    on_l2cap_connected_d.set<HearingAid, &HearingAid::on_l2cap_connected>(*this);
    on_l2cap_write_d.set<HearingAid, &HearingAid::on_l2cap_write>(*this);
    on_asp_notification_enabled_d.set<HearingAid, &HearingAid::on_asp_notification_enabled>(*this);
    on_asp_notification_received_d.set<HearingAid, &HearingAid::on_asp_notification_received>(*this);
    on_audio_start_written_d.set<HearingAid, &HearingAid::on_audio_start_written>(*this);
    on_audio_stop_written_d.set<HearingAid, &HearingAid::on_audio_stop_written>(*this);
}

bool HearingAid::is_streaming()
{
    return state == State::AudioStarted || state == State::AudioWorking;
}

void HearingAid::set_characteristic_data()
{
    if (remote) {
        for (auto& s : remote->services) {
            for (auto& c : s.chars) {
                UUID u16(c.uuid16);
                UUID u128(c.uuid128);
                uint16_t val_handle = c.value_handle;
                if (u16 == GapUUID::deviceName16) {
                    LOG_INFO("%s: Got device name char", bd_addr_to_str(remote->addr));
                    read_val_handles[CharValRead::DeviceName] = val_handle;
                } else if (u128 == AshaUUID::readOnlyProps) {
                    LOG_INFO("%s: Got ROP char", bd_addr_to_str(remote->addr));
                    read_val_handles[CharValRead::ReadOnlyProps] = val_handle;
                } else if (u128 == AshaUUID::psm) {
                    LOG_INFO("%s: Got PSM char", bd_addr_to_str(remote->addr));
                    read_val_handles[CharValRead::PSM] = val_handle;
                } else if (u128 == AshaUUID::audioControlPoint) {
                    LOG_INFO("%s: Got ACP char", bd_addr_to_str(remote->addr));
                    asha_chars.acp = &c;
                } else if (u128 == AshaUUID::audioStatus) {
                    LOG_INFO("%s: Got ASP char", bd_addr_to_str(remote->addr));
                    asha_chars.asp = &c;
                } else if (u128 == AshaUUID::volume) {
                    LOG_INFO("%s: Got Volume char", bd_addr_to_str(remote->addr));
                    asha_chars.vol = &c;
                }
            }
        }
    }
}

void HearingAid::disconnect() 
{
    state = State::Disconnecting;
    remote->disconnect(&bt_err); //TODO: add error handling
}

void HearingAid::on_services_discovered(uint8_t status, BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Service discovery failed with ATT error 0x%02x", bd_addr_to_str(remote->addr), status);
        delay_counter = 500;
        state = State::Connected;
        return;
    }
    bool asha_found = false;
    for (auto& s : remote->services) {
        UUID u16(s.service.uuid16);
        UUID u128(s.service.uuid128);
        if (u16 == AshaUUID::service16 || u128 == AshaUUID::service) {
            asha_found = true;
            break;
        }
    }
    if (asha_found) {
        LOG_INFO("%s: ASHA service found", bd_addr_to_str(remote->addr));
        state = State::ServicesDiscovered;
    } else {
        LOG_ERROR("%s: ASHA service not found", bd_addr_to_str(remote->addr));
        disconnect();
    }
    return;
}

bool HearingAid::service_filter(BT::Service const& service)
{
    UUID u16(service.service.uuid16);
    UUID u128(service.service.uuid128);
    return (u16 == AshaUUID::service16 || 
            u128 == AshaUUID::service || 
            u16 == GapUUID::service16);
}

void HearingAid::discover_services() 
{
    LOG_INFO("%s: Discovering services", bd_addr_to_str(remote->addr));
    state = State::DiscoveringServices;
    BT::Result res = remote->discover_services(
        on_services_discovered_d,
        service_filter_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Discover services");
    if (res != BT::Result::Ok) {
        state = State::Connected;
    }
    return;
}

void HearingAid::on_data_length_set(BT::Remote*)
{
    LOG_INFO("%s: Data length set", bd_addr_to_str(remote->addr));
    state = State::DataLengthSet;
}

void HearingAid::set_data_length()
{
    LOG_INFO("%s: Setting data length", bd_addr_to_str(remote->addr));
    state = State::SetDataLength;
    BT::Result res = remote->set_data_length(pdu_len, max_tx_time, on_data_length_set_d);
    log_bt_res(res, bt_err, "Set data length");
    if (res != BT::Result::Ok) {
        LOG_ERROR("%s: Failed to set data length", bd_addr_to_str(remote->addr));
        disconnect();
    }
}

void HearingAid::on_paired_and_bonded(uint8_t status, uint8_t reason, BT::Remote*)
{
    if (status != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Pairing and bonding failed with status 0x%02x and reson 0x%02x", 
                    bd_addr_to_str(remote->addr), status, reason);
        disconnect();
        return;
    }
    LOG_INFO("%s: Paired and bonded", bd_addr_to_str(remote->addr));
    state = State::PairedAndBonded;
    return;
}

void HearingAid::pair_and_bond()
{
    state = State::PairAndBonding;
    BT::Result res = remote->bond(on_paired_and_bonded_d);
    log_bt_res(res, bt_err, "Pair and bond");
    if (res != BT::Result::Ok) {
        state = State::ServicesDiscovered;
    }
    return;
}

void HearingAid::on_chars_discovered(uint8_t status, picobt::BT::Remote*)
{
    if (status != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Characteristic discovery failed with status 0x%02x",
                    bd_addr_to_str(remote->addr), status);
        delay_counter = 500;
        state = State::PairedAndBonded;
        return;
    }
    set_characteristic_data();
    LOG_INFO("%s: Characteristics discovered", bd_addr_to_str(remote->addr));
    state = State::CharsDiscovered;
    return;
}

bool HearingAid::discover_chars_filter(gatt_client_characteristic_t*)
{
    return true;
}

void HearingAid::discover_characteristics()
{
    state = State::DiscoverChars;
    BT::Result res = remote->discover_characteristics(
        on_chars_discovered_d,
        discover_chars_filter_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Discover characteristics");
    if (res != BT::Result::Ok) {
        state = State::PairedAndBonded;
    }
    return;
}

void HearingAid::on_read_char_value(BT::Remote*, uint16_t val_handle, const uint8_t *data, uint16_t len)
{
    if (val_handle == read_val_handles[CharValRead::DeviceName]) {
        LOG_INFO("%s: Read device name", bd_addr_to_str(remote->addr));
        device_name.append((const char*)data, len);
    } else if (val_handle == read_val_handles[CharValRead::ReadOnlyProps]) {
        LOG_INFO("%s: Read ROP", bd_addr_to_str(remote->addr));
        rop.read(data);
        side_str = rop.side == Side::Left ? "Left" : "Right";
        rop.print_values();
    } else if (val_handle == read_val_handles[CharValRead::PSM]) {
        LOG_INFO("%s: Read PSM", bd_addr_to_str(remote->addr));
        psm = get_val<uint16_t>(data);
    }
    return;
}

void HearingAid::on_char_values_read(uint8_t status, picobt::BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Read characteristic values failed with status 0x%02x",
                    bd_addr_to_str(remote->addr), status);
        delay_counter = 500;
        state = State::CharsDiscovered;
        return;
    }
    LOG_INFO("%s: Characteristics Read", side_str);
    state = State::CharsRead;
    return;
}

void HearingAid::read_characteristics()
{
    state = State::ReadChars;
    BT::Result res = remote->read_characteristic_values(
        etl::span<uint16_t>(read_val_handles),
        on_read_char_value_d,
        on_char_values_read_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Read characteristics");
    if (res != BT::Result::Ok) {
        state = State::CharsDiscovered;
    }
    return;
}

void HearingAid::on_l2cap_connected(uint8_t status, BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Create l2cap channel failed with status 0x%02x",
                    side_str, status);
        delay_counter = 500;
        state = State::CharsRead;
        return;
    }
    LOG_INFO("%s: L2CAP CoC created", side_str);
    state = State::L2CAPConnected;
    return;
}

void HearingAid::on_l2cap_write(BT::Remote*)
{
    state = State::AudioStarted;
}

void HearingAid::connect_l2cap()
{
    state = State::ConnectL2AP;
    BT::Result res = remote->create_l2cap_cbm_conn(
        psm,
        receive_buff.data(),
        receive_buff.size(),
        on_l2cap_connected_d,
        on_l2cap_write_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Connect L2CAP");
    if (res != BT::Result::Ok) {
        state = State::CharsRead;
    }
    return;
}

void HearingAid::on_asp_notification_enabled(uint8_t status, BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Enable ASP notification failed with status 0x%02x",
                    side_str, status);
        delay_counter = 500;
        state = State::L2CAPConnected;
        return;
    }
    LOG_INFO("%s: ASP Notification enabled", side_str);
    state = State::ASPNotEnabled;
    return;
}

void HearingAid::on_asp_notification_received(BT::Remote*, const uint8_t *data, uint16_t len)
{
    int8_t asp_status = (int8_t)data[0];
    if (asp_status == ASPStatus::ok) {
        LOG_INFO("%s: ASP Ok", side_str);
        if (state == State::AudioStarting) {
            state = State::AudioStarted;
        } else if (state == State::AudioStopping) {
            if (other_ha) {
                other_ha->write_status(ACPStatus::other_disconnected);
            }
            state = State::AudioReady;
        }
    } else {
        LOG_ERROR("%s: ASP: %s", side_str, asp_status == ASPStatus::unkown_command ? "Unknown command" : "Illegal param");
    }
    return;
}

void HearingAid::enable_asp_notification()
{
    state = State::EnableASPNot;
    BT::Result res = remote->enable_notification(
        asha_chars.asp,
        on_asp_notification_received_d,
        on_asp_notification_enabled_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Enable ASP notification");
    if (res != BT::Result::Ok) {
        state = State::L2CAPConnected;
    }
    return;
}

void HearingAid::write_status(uint8_t acp_status)
{
    if (is_streaming()) {
        acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Status);
        acp_cmd_packet[1] = acp_status;
        BT::Result res = remote->write_characteristic_value_no_resp(
            asha_chars.acp->value_handle,
            acp_cmd_packet.data(),
            2U,
            &bt_err
        );
        log_bt_res(res, bt_err, "Write status");
    }
}

void HearingAid::on_audio_start_written(uint8_t status, BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Start audio failed with status 0x%02x",
                    side_str, status);
        delay_counter = 500;
        state = State::AudioReady;
        return;
    }
}

void HearingAid::start_audio()
{
    state = State::AudioStarting;
    LOG_INFO("%s: Starting audio", side_str);
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Start); // Opcode
    acp_cmd_packet[1] = 1u; // G.722 codec at 16KHz
    acp_cmd_packet[2] = 0u; // Unkown audio type
    acp_cmd_packet[3] = (uint8_t)volume; // Volume
    acp_cmd_packet[4] = (other_ha && other_ha->is_streaming()) ? 1 : 0; // Otherstate
    BT::Result res = remote->write_characteristic_value(
        asha_chars.acp->value_handle,
        acp_cmd_packet.data(),
        acp_cmd_packet.size(),
        on_audio_start_written_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Start audio");
    if (res != BT::Result::Ok) {
        state = State::AudioReady;
    }
    return;
}

void HearingAid::on_audio_stop_written(uint8_t status, BT::Remote*)
{
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: Stop audio failed with status 0x%02x",
                    side_str, status);
        return;
    }
}

void HearingAid::stop_audio()
{
    state = State::AudioStopping;
    LOG_INFO("%s: Stopping audio", side_str);
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Stop);
    BT::Result res = remote->write_characteristic_value(
        asha_chars.acp->value_handle,
        acp_cmd_packet.data(),
        1U,
        on_audio_stop_written_d,
        &bt_err
    );
    log_bt_res(res, bt_err, "Stop audio");
}

void HearingAid::set_volume(int8_t new_volume)
{
    if (new_volume != volume) {
        volume = new_volume;
        LOG_INFO("%s: Setting volume to %d", side_str, volume);
        if (is_streaming()) {
            remote->write_characteristic_value_no_resp(
                asha_chars.vol->value_handle,
                (uint8_t*)&volume,
                1U,
                &bt_err
            );
        }
    }
}

void HearingAid::send_audio(uint32_t write_index)
{
    if (write_index > audio_w_index) {
        audio_w_index = write_index;
    }
    if (outgoing_credits == 0) {
        LOG_INFO("%s: Available credits: 0, restarting stream", side_str);
        stop_audio();
        return;
    }
    if (audio_r_index >= audio_w_index) {
        return;
    }
    state = State::AudioWorking;
    if ((audio_w_index - audio_r_index) >= 2) {
        audio_r_index = audio_w_index - 1;
    }
    AudioBuffer::G722Buff& packet = audio_buff.get_g_buff(audio_r_index);
    audio_data = rop.side == Side::Left ? packet.l.data() : packet.r.data();
    ++audio_r_index;
    remote->send_l2cap_data(audio_data, sdu_size_bytes, &bt_err);
}

struct HearingAidMgr
{
etl::vector<HearingAid, 2> hearing_aids = {};

HearingAid* get_ha_from_remote(BT::Remote* remote) 
{
    for (auto& ha : hearing_aids) {
        if (ha.remote == remote) {
            return &ha;
        }
    }
    return nullptr;
}

bool add_ha_from_remote(BT::Remote* remote)
{
    if (!hearing_aids.full()) {
        hearing_aids.emplace_back(remote);
        if (hearing_aids.size() > 1) {
            hearing_aids[0].other_ha = &hearing_aids[1];
            hearing_aids[1].other_ha = &hearing_aids[0];
        }
        return true;
    }
    LOG_ERROR("hearing_aids vector full");
    return false;
}

void remove_ha(HearingAid* hearing_aid)
{
    etl::erase_if(hearing_aids, [hearing_aid](HearingAid const& ha) { return bd_addr_cmp(ha.remote->addr, hearing_aid->remote->addr) == 0; });
}

/**
 * Test if a complete set of hearing aids is connected
 */
bool set_complete()
{
    switch (hearing_aids.size()) {
        case 1:
            return hearing_aids[0].rop.id.manufacturer_id > 0U &&
                    hearing_aids[0].rop.mode == Mode::Mono;
        case 2:
            return hearing_aids[0].rop.id.manufacturer_id > 0U &&
                    hearing_aids[1].rop.id.manufacturer_id > 0U &&
                    hearing_aids[0].rop.id == hearing_aids[1].rop.id;
        default:
            return false;
    }
}

void set_led(LEDManager& led)
{
    switch (hearing_aids.size()) {
        case 0:
            return led.set_led_pattern({.len = 2, .interval_ms = 250, .delay_ms = 0, .pattern = 0b10});
        case 1:
            return led.set_led_pattern({.len = 2, .interval_ms = 100, .delay_ms = 0, .pattern = 0b10});
        case 2:
            return led.set_led(LEDManager::State::On);
        default:
            return led.set_led(LEDManager::State::Off);
    }
}
};

/* Global variables */

static HearingAidMgr ha_mgr = {};

static LEDManager led_mgr = {};

static constexpr size_t json_resp_str_size = 4096;

static etl::string<json_resp_str_size> response_json = {};

/* Class member functions */
void ROP::read(const uint8_t* data)
{
    etl::bitset<8>  device_cap{data[1]};
    etl::bitset<8>  feat_map{data[10]};
    etl::bitset<16> codecs{get_val<uint16_t>(&data[15])};

    version = data[0];
    side = device_cap[0] ? Side::Right : Side::Left;
    mode = device_cap[1] ? Mode::Binaural : Mode::Mono;
    csis_supported = device_cap[2];
    id.manufacturer_id = get_val<uint16_t>(&data[2]);
    memcpy(id.unique_id.data(), &data[4], id.unique_id.size());
    le_coc_supported = feat_map[0];
    render_delay = get_val<uint16_t>(&data[11]);
    codec_16khz = codecs[1];
    codec_24khz = codecs[2];
}

void ROP::print_values()
{
    LOG_INFO("ROP -"
    " Side: %s,"
    " Mode: %s,"
    " M. ID: %04hx,"
    " Delay: %hu,"
    " 16KHz: %s,"
    " 24KHz: %s",
    (side == Side::Left ? "L" : "R"),
    (mode == Mode::Binaural ? "B" : "M"),
    id.manufacturer_id,
    render_delay,
    codec_16khz ? "Y" : "N",
    codec_24khz ? "Y" : "N");
}

/* Static function definitions */

static void restart_pico()
{
    LOG_INFO("Restarting Pico-ASHA");
    watchdog_enable(250, true);
}

static void handle_bt_audio_pending_worker([[maybe_unused]] async_context_t *context, 
                                           [[maybe_unused]] async_when_pending_worker_t *worker)
{
    using enum HearingAid::State;
    BT& bt = BT::instance();

    uint32_t write_index = audio_buff.get_write_index();
    AudioBuffer::Volume vol = audio_buff.get_volume();
    bool pcm_is_streaming = audio_buff.pcm_streaming.Load();

    for (auto& ha : ha_mgr.hearing_aids) {
        if (ha.delay_counter > 0) {
            ha.delay_counter--;
            continue;
        }
        switch (ha.state) {
            case Connected:
                LOG_INFO("%s: Discover services", bd_addr_to_str(ha.remote->addr));
                ha.discover_services();
                break;
            case ServicesDiscovered:
                LOG_INFO("%s: Set Data Length", bd_addr_to_str(ha.remote->addr));
                ha.set_data_length();
                break;
            case DataLengthSet:
                LOG_INFO("%s: Pair and bond", bd_addr_to_str(ha.remote->addr));
                ha.pair_and_bond();
                break;
            case PairedAndBonded:
                LOG_INFO("%s: Discover characteristics", bd_addr_to_str(ha.remote->addr));
                ha.discover_characteristics();
                break;
            case CharsDiscovered:
                LOG_INFO("%s: Read characteristics", bd_addr_to_str(ha.remote->addr));
                ha.read_characteristics();
                break;
            case CharsRead:
                LOG_INFO("%s: Create L2CAP CoC", ha.side_str);
                ha.connect_l2cap();
                break;
            case L2CAPConnected:
                LOG_INFO("%s: Enable ASP notification", ha.side_str);
                ha.enable_asp_notification();
                break;
            case ASPNotEnabled:
                if (ha_mgr.set_complete()) {
                    LOG_INFO("Set complete");
                    runtime_settings.set_full_set_paired(true);
                } else {
                    bt.continue_scan();
                }
                ha_mgr.set_led(led_mgr);
                ha.state = AudioReady;
                break;
            case AudioReady:
                ha.outgoing_credits = l2cap_cbm_available_credits(ha.remote->local_cid);
                /* Ensure sufficient credits are available to (re)start
                audio streaming */
                if (pcm_is_streaming && ha.outgoing_credits >= 8) {
                    ha.start_audio();
                }
                break;
            case AudioStarted:
                ha.outgoing_credits = l2cap_cbm_available_credits(ha.remote->local_cid);
                if (!pcm_is_streaming) {
                    LOG_INFO("%s: USB audio no longer streaming. Stopping ASHA stream", ha.side_str);
                    ha.stop_audio();
                    audio_buff.encode_audio = false;
                    continue;
                }
                audio_buff.encode_audio = true;

                if (write_index == 0) continue;

                ha.set_volume(ha.rop.side == Side::Left ? vol.l : vol.r);
                ha.send_audio(write_index);
            default:
                break;
        }
    }
}

static void handle_stdin_line_worker([[maybe_unused]] async_context_t *context, [[maybe_unused]] async_when_pending_worker_t *worker)
{
    BT& bt = BT::instance();
    JsonDocument cmd_doc;
    JsonDocument resp_doc;
    auto err = deserializeJson(cmd_doc, complete_std_line);
    if (err != DeserializationError::Ok) {
        // Dump the log if the user started monitoring USB serial late
        if (err == DeserializationError::EmptyInput) {
            if (logging_ctx) {
                async_context_set_work_pending(logging_ctx, &logging_pending_worker);
            }
        } else {
            printf("%s\r\n", R"("{"cmd":"error"}")");
        }
        return;
    }
    const char* cmd = cmd_doc["cmd"];
    resp_doc["cmd"] = cmd;

    auto cmd_is = [&](const char* command) {return str_eq(cmd, command);};

    if (cmd_is(SerCmd::AshaFWVers)) {
        resp_doc[SerCmd::AshaFWVers] = PICO_ASHA_VERS;

    } else if (cmd_is(SerCmd::Status)) {
        resp_doc["num_conn"] = ha_mgr.hearing_aids.size();
        resp_doc["full_set"] = ha_mgr.set_complete();
        JsonObject settings = resp_doc["settings"].to<JsonObject>();
        settings["log_level"] = log_level_to_str(runtime_settings.log_level);
        settings["hci_dump_enabled"] = runtime_settings.hci_dump_enabled;
        settings["uart_enabled"] = runtime_settings.serial_uart_enabled;
        settings["full_set_paired"] = runtime_settings.full_set_paired;

    } else if (cmd_is(SerCmd::ConnDevices)) {
        JsonArray devices = resp_doc["devices"].to<JsonArray>();
        for (auto& ha : ha_mgr.hearing_aids) {
            JsonObject dev = devices.add<JsonObject>();
            dev["addr"] = bd_addr_to_str(ha.remote->addr);
            dev["name"] = ha.device_name;
            dev["side"] = ha.side_str;
            dev["mono"] = ha.rop.mode == Mode::Mono;
            dev["streaming"] = ha.is_streaming();
            dev["paused"] = false;
        }

    } else if (cmd_is(SerCmd::ClearDevDb)) {
        bt.clear_bonding_data();
        runtime_settings.set_full_set_paired(false);
        resp_doc["success"] = true;

    } else if (cmd_is(SerCmd::UartSerial)) {
        bool uart_enabled = cmd_doc["enabled"];
        if (runtime_settings.set_uart_enabled(uart_enabled)) {
            stdio_set_driver_enabled(&stdio_uart, runtime_settings.serial_uart_enabled);
        }
        resp_doc["success"] = true;

    } else if (cmd_is(SerCmd::LogLevel)) {
        const char* log_level = cmd_doc["level"];
        enum LogLevel ll = str_to_log_level(log_level);
        if (ll != LogLevel::None) {
            runtime_settings.log_level = ll;
            resp_doc["success"] = true;
        } else {
            resp_doc["success"] = false;
        }
    
    } else if (cmd_is(SerCmd::HCIDump)) {
        bool hci_dump_enabled = cmd_doc["enabled"];
        if (runtime_settings.set_hci_dump_enabled(hci_dump_enabled)) {
            restart_pico();
        }
        resp_doc["success"] = true;
    } else if (cmd_is(SerCmd::Restart)) {
        restart_pico();
        resp_doc["success"] = true;
    }
    // else {
    //     resp_doc["cmd"] = "unknown";
    // }
    size_t len = serializeJson(resp_doc, response_json.data(), response_json.capacity() - 1);
    response_json.uninitialized_resize(len);
    printf("%s\r\n", response_json.c_str());
}

static void on_bt_started(bool started, uint8_t bt_state)
{
    if (started) {
        LOG_INFO("BT Started")
        ha_mgr.set_led(led_mgr);
        auto& bt = BT::instance();
        bt.enable_scan(on_ad_report_delegate, 
                       [](const AdReport &report) {
                            for (auto& s : report.services) {
                                UUID uuid16(s.uuid_16);
                                UUID uuid128(s.uuid_128);
                                if (uuid16 == AshaUUID::service16 || 
                                    uuid128 == AshaUUID::service || 
                                    uuid128 == MfiUUID::service) {
                                        return true;
                                }
                            }
                            return false;
                       });
    } else {
        LOG_ERROR("BT did not start: Error code: 0x%02x", bt_state);
    }
}

static void on_ad_report(AdReport &report)
{
    if (ha_mgr.hearing_aids.full()) {
        LOG_INFO("Cannot add new HA");
        return;
    }
    LOG_INFO("HA with address %s discovered. Connecting", bd_addr_to_str(report.address));
    auto& bt = BT::instance();
    uint8_t bt_err = 0U;
    auto res = bt.connect(report.address, 
                          report.address_type, 
                          on_bt_connected_delegate, 
                          on_bt_disconnected_delegate,
                          &bt_err);
    log_bt_res(res, bt_err, "On AD Report");
    if (res != BT::Result::Ok) {
        LOG_INFO("Continue scanning");
        bt.continue_scan();
    }
}

static void on_bt_connected(uint8_t status, BT::Remote* remote)
{
    auto& bt = BT::instance();
    if (status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("Error connecting to remote");
        bt.continue_scan();
        return;
    }
    if (!ha_mgr.add_ha_from_remote(remote)) {
        uint8_t bt_err;
        auto res = remote->disconnect(&bt_err);
        log_bt_res(res, bt_err, "On BT Connected");
        return;
    }
    LOG_INFO("%s: Connected", bd_addr_to_str(remote->addr));
}

static void on_bt_disconnected(uint8_t reason, BT::Remote* remote)
{
    auto& bt = BT::instance();
    HearingAid *ha = ha_mgr.get_ha_from_remote(remote);
    if (ha) {
        if (ha->state != HearingAid::State::Disconnecting) {
            LOG_ERROR("%s: Unexpected disconnection with reason 0x%02x", ha->side_str, reason);
        } else {
            LOG_INFO("%s: Disconnected", ha->side_str);
        }
        if (ha->other_ha) {
            ha->other_ha->write_status(ACPStatus::other_disconnected);
            ha->other_ha->other_ha = nullptr;
        }
        ha_mgr.remove_ha(ha);
    } else {
        LOG_ERROR("Remote %s disconnected with reason 0x%02x", bd_addr_to_str(remote->addr), reason);
    }
    ha_mgr.set_led(led_mgr);
    bt.continue_scan();
}

extern "C" void bt_main()
{
    if (cyw43_arch_init()) {
        return;
    }
    runtime_settings.init();
    runtime_settings.get_settings();

    stdio_set_driver_enabled(&stdio_uart, runtime_settings.serial_uart_enabled);

    async_context_t *ctx = cyw43_arch_async_context();

    bt_audio_pending_worker.do_work = handle_bt_audio_pending_worker;
    async_context_add_when_pending_worker(ctx, &bt_audio_pending_worker);
    bt_async_ctx = ctx;

    stdin_pending_worker.do_work = handle_stdin_line_worker;
    async_context_add_when_pending_worker(ctx, &stdin_pending_worker);
    usb_ser_ctx = ctx;

    logging_pending_worker.do_work = handle_logging_pending_worker;
    async_context_add_when_pending_worker(ctx, &logging_pending_worker);
    logging_ctx = ctx;

    led_mgr.set_ctx(ctx);

    if (runtime_settings.hci_dump_enabled) {
        // Allow time for USB serial to connect before proceeding
        while (!stdio_usb_connected()) {
            sleep_ms(250);
        }
        sleep_ms(250);
    }

    if (!runtime_settings) {
        LOG_ERROR("Runtime settings not initialised");
    }

    auto& bt = BT::instance();
    BT::Config cfg = {};
    bt.configure(cfg);

    auto on_bt_started_delegate = etl::delegate<void (bool, uint8_t)>::create<on_bt_started>();
    bt.start(on_bt_started_delegate);

    while(true) {
        sleep_ms(10000);
    }
}

} // namespace asha