#include <bitset>
#include <pico/assert.h>

#include "hearing_aid.hpp"
#include "asha_uuid.hpp"
#include "bt_status_err.hpp"

namespace asha
{

/* Data length variables */
static constexpr uint16_t pdu_len = 167u;

static constexpr uint16_t max_tx_time = 1064;

static constexpr size_t ev_packet_str_size = sizeof comm::EventPacket::data.str;

namespace ASPStatus
{
    constexpr int8_t unkown_command = -1;
    constexpr int8_t illegal_params = -2;
    constexpr int8_t ok = 0;
}

namespace ACPOpCode
{
    constexpr uint8_t start  = 1;
    constexpr uint8_t stop   = 2;
    constexpr uint8_t status = 3;
}

namespace ACPStatus
{
    constexpr uint8_t other_disconnected = 0;
    constexpr uint8_t other_connected = 1;
    constexpr uint8_t conn_param_updated = 2;
}

static bool gatt_service_valid(gatt_client_service_t* service);

static void delete_paired_device(const bd_addr_t address);

static bool gatt_service_valid(gatt_client_service_t* service)
{
    return service->end_group_handle > 0U;
}

/* Get value from (sub) array of bytes */
template<typename T>
static T get_val(const uint8_t *start)
{
    T val;
    std::memcpy(&val, start, sizeof(val));
    return val;
}

ROP::ROP()
{}

void ROP::read(const uint8_t* data)
{
    std::bitset<8>  device_cap{data[1]};
    std::bitset<8>  feat_map{data[10]};
    std::bitset<16> codecs{get_val<uint16_t>(&data[15])};

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

    memcpy(raw_rop_data, data, sizeof raw_rop_data);
}

// void ROP::print_values()
// {
//     LOG_INFO("ROP -"
//         " Side: %s,"
//         " Mode: %s,"
//         " M. ID: %04hx,"
//         " Delay: %hu,"
//         " 16KHz: %s,"
//         " 24KHz: %s",
//         (side == Side::Left ? "L" : "R"),
//         (mode == Mode::Binaural ? "B" : "M"),
//         id.manufacturer_id,
//         render_delay,
//         codec_16khz ? "Y" : "N",
//         codec_24khz ? "Y" : "N");
// }

/* Public methods */

HearingAid::HearingAid()
{
    auth_req = SM_AUTHREQ_BONDING | SM_AUTHREQ_SECURE_CONNECTION;
    if (hearing_aids[0] == nullptr) {
        hearing_aids[0] = this;
    } else if (hearing_aids[1] == nullptr) {
        hearing_aids[1] = this;
    } else {
        hard_assert(false);
    }
    next_conn_id = comm::unset_conn_id;
}

void __not_in_flash_func(HearingAid::process)()
{
    using enum ProcessState;
    using namespace comm;

    for (auto ha : hearing_aids) {
        uint8_t res = ERROR_CODE_SUCCESS;
        if (!ha->is_connected()) { continue; }
        if (ha->process_delay_ticks > 0) {
            --ha->process_delay_ticks;
            continue;
        }
        EventType ev_type;
        switch (ha->process_state)
        {
            case DiscoverServices:
                ev_type = EventType::DiscServices;
                //LOG_INFO("%s: Discovering services", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_discover_primary_services(&HearingAid::handle_service_discovery, ha->conn_handle);
                break;
            case PairBond:
                ev_type = EventType::PairAndBond;
                //LOG_INFO("%s: Pairing and bonding",  ha->get_side_str())
                ha->set_process_busy();
                sm_request_pairing(ha->conn_handle);
                break;
            case DataLength:
                ev_type = EventType::DLE;
                //LOG_INFO("%s: Setting data length",  ha->get_side_str())
                ha->set_process_busy();
                ha->set_data_langth();
                break;
            case DiscoverASHAChar:
                //LOG_INFO("%s: Discovering ASHA characteristics", ha->get_side_str());
                ev_type = EventType::DiscASHAChar;
                ha->set_process_busy();
                res = gatt_client_discover_characteristics_for_service(&HearingAid::handle_char_discovery, ha->conn_handle, &ha->services.asha.service);
                break;
            case ReadROP:
                ev_type = EventType::ROPRead;
                //LOG_INFO("%s: Read ROP characteristic", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.asha.rop);
                break;
            case ReadPSM:
                ev_type = EventType::PSMRead;
                //LOG_INFO("%s: Read PSM", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.asha.psm);
                break;
            case DiscoverGAPChar:
                ev_type = EventType::DiscGAPChar;
                //LOG_INFO("%s: Discover GAP characteristics", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_discover_characteristics_for_service(&HearingAid::handle_char_discovery, ha->conn_handle, &ha->services.gap.service);
                break;
            case ReadDeviceName:
                ev_type = EventType::DevNameRead;
                //LOG_INFO("%s: Read device name", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.gap.device_name);
                break;
            case DiscoverDISChar:
                ev_type = EventType::DiscDISChar;
                //LOG_INFO("%s: Discover DIS characteristics", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_discover_characteristics_for_service(&HearingAid::handle_char_discovery, ha->conn_handle, &ha->services.dis.service);
                break;
            case ReadMfgName:
                ev_type = EventType::MfgRead;
                //LOG_INFO("%s: Read manufacturer name", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.dis.manufacture_name);
                break;
            case ReadModelNum:
                ev_type = EventType::ModelRead;
                //LOG_INFO("%s: Read model number", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.dis.model_num);
                break;
            case ReadFWVers:
                ev_type = EventType::FWRead;
                //LOG_INFO("%s: Read FW version", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, &ha->services.dis.fw_vers);
                break;
            case ConnectL2CAP:
                ev_type = EventType::L2CAPCon;
                //LOG_INFO("%s: Create L2CAP CoC", ha->get_side_str());
                ha->set_process_busy();
                res = l2cap_cbm_create_channel(&HearingAid::handle_l2cap_cbm, 
                                               ha->conn_handle, 
                                               ha->psm, 
                                               ha->recv_buff.data(), 
                                               ha->recv_buff.size(), 
                                               L2CAP_LE_AUTOMATIC_CREDITS, 
                                               LEVEL_2, 
                                               &ha->cid);
                break;
            case EnASPNotification:
                ev_type = EventType::ASPNotEnable;
                //LOG_INFO("%s: Enable ASP notification", ha->get_side_str());
                ha->set_process_busy();
                res = gatt_client_write_client_characteristic_configuration(&HearingAid::handle_asp_notification_reg, 
                                                                            ha->conn_handle, 
                                                                            &ha->services.asha.asp, 
                                                                            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                break;
            case Finalize:
                ev_type = EventType::StreamReady;
                //LOG_INFO("%s: Ready for audio streaming", ha->get_side_str());
                ha->set_process_busy();
                if (full_set_connected()) {
                    runtime_settings.set_full_set_paired(true);
                    led_mgr.set_led(LEDManager::State::On);
                } else {
                    led_mgr.set_led_pattern(one_connected);
                    start_scan();
                }
                ha->cached = true;
                ha->process_state = Audio;
                ha->audio_state = AudioState::Ready;
            default:
                break;
        }
        if (res != ERROR_CODE_SUCCESS) {
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::BtstackStatus, res));
            //LOG_ERROR("%s: BTStack error: %s", ha->get_side_str(), bt_err_str(res));
            ha->unset_process_busy();
            ha->process_delay_ticks = ha_process_delay_ticks;
        }
    }
}

/* Public static methods */

int HearingAid::num_connected()
{
    int num = 0;
    for (auto ha : hearing_aids) {
        if (ha->is_connected()) {
            ++num;
        }
    }
    return num;
}

std::array<HearingAid*,2> HearingAid::connected_has()
{
    std::array<HearingAid*,2> has = {nullptr, nullptr};
    size_t i = 0;
    for (auto ha : hearing_aids) {
        if (ha->is_connected()) {
            has[i] = ha;
        }
    }
    return has;
}

void HearingAid::on_serial_host_connected()
{
    using namespace comm;
    RemoteInfo info;
    send_intro_packet((int8_t)num_connected());
    for (auto ha : hearing_aids) {
        info.conn_id = ha->conn_id;
        info.hci_handle = ha->conn_handle;
        bd_addr_copy(info.addr, ha->addr);
        info.connected = true;
        info.paired = ha->paired_and_bonded;
        info.psm = ha->psm;
        info.l2cap_id = ha->cid;
        memcpy(info.dev_name, ha->device_name.data(), sizeof(info.dev_name));
        memcpy(info.mfg_name, ha->manufacturer.data(), sizeof(info.mfg_name));
        memcpy(info.model_name, ha->model.data(), sizeof(info.model_name));
        memcpy(info.fw_vers, ha->fw_vers.data(), sizeof(info.fw_vers));
        info.side = (ha->rop.side == Side::Left)  ? CSide::Left : 
                    (ha->rop.side == Side::Right) ? CSide::Right
                                                  : CSide::Unset;
        info.mode = (ha->rop.mode == Mode::Binaural) ? CMode::Binaural :
                    (ha->rop.mode == Mode::Mono)     ? CMode::Mono
                                                     : CMode::Unset;
        info.audio_streaming = ha->is_streaming();
        info.curr_vol = ha->curr_vol;

        send_remote_info_packet(info);
    }
}

bool HearingAid::is_addr_connected(const bd_addr_t addr)
{
    for (auto ha : hearing_aids) {
        if (bd_addr_cmp(addr, ha->addr) == 0 && ha->is_connected()) {
            return true;
        }
    }
    return false;
}

void HearingAid::start_scan()
{
    gap_set_scan_params(1, 0x0030, 0x0030, runtime_settings.full_set_paired ? 1 : 0);
    gap_start_scan();
}

void HearingAid::on_ad_report(const AdvertisingReport& report)
{
    if (is_addr_connected(report.address)) { return; }
    gap_stop_scan();
    if (full_set_connected()) { 
        return; 
    }
    if (runtime_settings.full_set_paired || report.is_hearing_aid) {
        gap_connect(report.address, report.address_type);
    } else {
        start_scan();
    }
}

void HearingAid::on_connected(bd_addr_t addr, hci_con_handle_t handle)
{
    using namespace comm;

    auto ha_cached = get_by_cached_addr(addr);
    if (ha_cached) {
        //LOG_INFO("%s: Connected to cached HA", bd_addr_to_str(addr));
        ha_cached->connected = true;
        ha_cached->conn_handle = handle;
        set_other_side_ptrs();
        ha_cached->assign_next_conn_id();

        EventPacket ev_pkt(EventType::RemoteConnected);
        ev_pkt.data.conn_info.hci_handle = handle;
        bd_addr_copy(ev_pkt.data.conn_info.addr, ha_cached->addr);
        add_event_to_buffer(ha_cached->conn_id, ev_pkt);

        ha_cached->process_state = ProcessState::PairBond;
        return;
    }
    for (auto ha : hearing_aids) {
        if (ha->is_connected()) { continue; }
        //LOG_INFO("%s: Connected to HA", bd_addr_to_str(addr))
        ha->connected = true;
        ha->cached = false;
        bd_addr_copy(ha->addr, addr);
        ha->conn_handle = handle;
        set_other_side_ptrs();
        ha->assign_next_conn_id();

        EventPacket ev_pkt(EventType::RemoteConnected);
        ev_pkt.data.conn_info.hci_handle = handle;
        bd_addr_copy(ev_pkt.data.conn_info.addr, ha->addr);
        add_event_to_buffer(ha->conn_id, ev_pkt);

        ha->process_state = ProcessState::DiscoverServices;
        return;
    }
    EventPacket err_pkt(EventType::RemoteConnected, StatusType::PAStatus, PAError::PAMaxConnected);
    add_event_to_buffer(unset_conn_id, err_pkt);
    //LOG_ERROR("%s: Unable to connect - array full", bd_addr_to_str(addr));
    gap_disconnect(handle);
}

void HearingAid::on_disconnected(hci_con_handle_t handle, uint8_t status, uint8_t reason)
{
    using namespace comm;
    HearingAid* ha = get_by_con_handle(handle);
    // if (ha->process_state == ProcessState::Disconnect) {
    //     LOG_INFO("%s: Disconnected", ha->get_side_str());
    // } else {
    //     LOG_ERROR("%s: Disconnected with status code: %s and reason: %s", 
    //                ha->get_side_str(), 
    //                bt_err_str(status), bt_err_str(reason));
    // }
    EventPacket ev_pkt(EventType::RemoteDisconnected, StatusType::ATTStatus, status, reason);
    add_event_to_buffer(ha->conn_id, ev_pkt);
    if (ha->other && ha->other->is_streaming()) {
        ha->other->send_acp_status(ACPStatus::other_disconnected);
    }
    ha->process_state = ProcessState::Disconnect;
    ha->connected = false;
    set_other_side_ptrs();
    ha->reset();
    int num_c = num_connected();
    if (num_c == 1) {
        led_mgr.set_led_pattern(one_connected);
    } else {
        led_mgr.set_led_pattern(none_connected);
    }
    start_scan();
}

void HearingAid::on_data_len_set(hci_con_handle_t handle, 
    [[maybe_unused]] uint16_t rx_octets, 
    [[maybe_unused]] uint16_t rx_time, 
    [[maybe_unused]] uint16_t tx_octets, 
    [[maybe_unused]] uint16_t tx_time)
{
    HearingAid* ha = get_by_con_handle(handle);

    // The data length is automatically changed when DLE is enabled, so ignore that case
    if (ha->process_state != (ProcessState::DataLength | ProcessState::ProcessBusy)) { return; }

    // LOG_INFO("%s: DL set to: RX Octets: %hu, RX Time: %hu us, TX Octets: %hu, TX Time: %hu us",
    //                     ha->get_side_str(),
    //                     rx_octets, rx_time, tx_octets, tx_time);    
    
    ha->process_state = ha->cached ? ProcessState::ReadPSM : ProcessState::DiscoverASHAChar;
}

void HearingAid::handle_sm(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    HearingAid *ha = nullptr;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    uint8_t reason = 0U;

    uint8_t ev_type = hci_event_packet_get_type(packet);

    EventPacket short_log_pkt(EventType::ShortLog);

    switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            handle = sm_event_just_works_request_get_handle(packet);
            ha = get_by_con_handle(handle);
            short_log_pkt.set_data_str("%s", "Just Works Req");
            add_event_to_buffer(ha->conn_id, short_log_pkt);
            //LOG_INFO("%s: Just Works requested", ha->get_side_str());
            sm_just_works_confirm(handle);
            break;

        case SM_EVENT_PAIRING_STARTED:
            handle = sm_event_pairing_started_get_handle(packet);
            ha = get_by_con_handle(handle);
            short_log_pkt.set_data_str("%s", "Pairing started");
            add_event_to_buffer(ha->conn_id, short_log_pkt);
            //LOG_INFO("%s: Pairing started", ha->get_side_str());
            break;

        case SM_EVENT_PAIRING_COMPLETE: {
            handle = sm_event_pairing_complete_get_handle(packet);
            ha = get_by_con_handle(handle);
            att_status = sm_event_pairing_complete_get_status(packet);
            reason = sm_event_pairing_complete_get_reason(packet);

            EventPacket ev_pkt(EventType::PairAndBond, StatusType::SMStatus, att_status, reason);
            switch (att_status) {
                case ERROR_CODE_SUCCESS:
                    //LOG_INFO("%s: Pairing complete", ha->get_side_str());;
                    ha->paired_and_bonded = true;
                    ha->process_state = ha->cached ? ProcessState::ReadPSM : ProcessState::DiscoverASHAChar;
                    add_event_to_buffer(ha->conn_id, EventPacket(EventType::PairAndBond));
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    //LOG_ERROR("%s: Pairing failed - timeout", ha->get_side_str());
                    //LOG_ERROR("%s: Pairing failed - remote user terminated connection", ha->get_side_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    ha->disconnect();
                    break;
                case ERROR_CODE_AUTHENTICATION_FAILURE:
                    if (reason == SM_REASON_AUTHENTHICATION_REQUIREMENTS) {
                        //LOG_ERROR("%s: Auth requirements not met. Attempting downgrade", ha->get_side_str());
                        short_log_pkt.set_data_str("%s", "Attempt auth downgrade");
                        add_event_to_buffer(ha->conn_id, short_log_pkt);
                        auth_req = SM_AUTHREQ_BONDING;
                        sm_set_authentication_requirements(auth_req);
                        sm_request_pairing(handle);
                    } else {
                        //LOG_ERROR("%s: Pairing failed, auth failure with reason: %s", ha->get_side_str(), sm_reason_str(reason));
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                        ha->disconnect();
                    }
                    break;
                default:
                    //LOG_ERROR("%s: Unhandled SM_EVENT_PAIRING_COMPLETE error. Reason: %s", ha->get_side_str(), sm_reason_str(reason))
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
            }
            break;
        }
        case SM_EVENT_REENCRYPTION_STARTED:
            handle = sm_event_reencryption_started_get_handle(packet);
            ha = get_by_con_handle(handle);
            short_log_pkt.set_data_str("%s", "Reencryption started");
            add_event_to_buffer(ha->conn_id, short_log_pkt);
            //LOG_INFO("%s: Reencryption started", ha->get_side_str());
            break;

        case SM_EVENT_REENCRYPTION_COMPLETE: {
            handle = sm_event_reencryption_complete_get_handle(packet);
            att_status = sm_event_reencryption_complete_get_status(packet);
            ha = get_by_con_handle(handle);

            EventPacket ev_pkt(EventType::PairAndBond, StatusType::SMStatus, att_status);

            switch (att_status) {
                case ERROR_CODE_SUCCESS:
                    //LOG_INFO("%s: Reencryption succeeded", ha->get_side_str());
                    ha->paired_and_bonded = true;
                    ha->process_state = ha->cached ? ProcessState::ReadPSM : ProcessState::DiscoverASHAChar;
                    comm::add_event_to_buffer(ha->conn_id, EventPacket(EventType::PairAndBond));
                    break;
                case ERROR_CODE_PIN_OR_KEY_MISSING:
                    //LOG_ERROR("%s: Reencryption failed with ERROR_CODE_PIN_OR_KEY_MISSING", ha->get_side_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    delete_paired_device(ha->addr);
                    ha->disconnect();
                    break;
                default:
                    //LOG_ERROR("%s: Reencryption error: %s", ha->get_side_str(), bt_err_str(att_status))
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    ha->disconnect();
                    break;
            }
            break;
        }
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        case SM_EVENT_IDENTITY_CREATED:
            break;
        default:
            //LOG_ERROR("Unhandled SM event: 0x%02x", ev_type);
            short_log_pkt.set_data_str("Unhandled SM Ev: 0x%02x", ev_type);
            add_event_to_buffer(unset_conn_id, short_log_pkt);
            break;
    }
}

void HearingAid::handle_service_discovery(PACKET_HANDLER_PARAMS)
{
    using namespace comm;
    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;
    gatt_client_service_t s = {};
    switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            handle = gatt_event_service_query_result_get_handle(packet);
            ha = get_by_con_handle(handle);

            EventPacket short_log_pkt(EventType::ShortLog);

            gatt_event_service_query_result_get_service(packet, &s);
            if (uuid_eq(s.uuid128, AshaUUID::service) || s.uuid16 == AshaUUID::service16) {
                //LOG_INFO("%s: Discovered ASHA service", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "ASHA service discovered");
                ha->services.asha.service = s;
            } else if (s.uuid16 == GapUUID::service16) {
                //LOG_INFO("%s: Discovered GAP service", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "GAP service discovered");
                ha->services.gap.service = s;
            } else if (s.uuid16 == DisUUID::service16) {
                //LOG_INFO("%s: Discovered DIS service", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "DIS service discovered");
                ha->services.dis.service = s;
            }
            add_event_to_buffer(ha->conn_id, short_log_pkt);
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);

            if (att_status != ATT_ERROR_SUCCESS) {
                //LOG_ERROR("%s: Error discovering services with status: %s", ha->get_side_str(), att_err_str(att_status));
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::DiscServices, StatusType::ATTStatus, att_status));
                ha->disconnect();
                break;
            } else if (!gatt_service_valid(&ha->services.asha.service)) {
                //LOG_ERROR("%s: ASHA service not discovered", ha->get_side_str());
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::DiscServices, StatusType::PAStatus, PAError::PAASHAServiceNotFound));
                ha->disconnect();
                break;
            }
            ha->process_state = ProcessState::PairBond;
            break;
        default:
            break;
    }
}

void HearingAid::handle_char_discovery(PACKET_HANDLER_PARAMS)
{
    using enum ProcessState;
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;
    gatt_client_characteristic_t c = {};

    switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            handle = gatt_event_characteristic_query_result_get_handle(packet);
            ha = get_by_con_handle(handle);
            gatt_event_characteristic_query_result_get_characteristic(packet, &c);

            EventPacket short_log_pkt(EventType::ShortLog);

            if (uuid_eq(c.uuid128, AshaUUID::readOnlyProps)) {
                //LOG_INFO("%s: Discovered ReadOnlyProperties characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "ROP char discovered");
                ha->services.asha.rop = c;
            } else if (uuid_eq(c.uuid128, AshaUUID::audioControlPoint)) {
                //LOG_INFO("%s: Discovered AudioControlPoint characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "ACP char discovered");
                ha->services.asha.acp = c;
            } else if (uuid_eq(c.uuid128, AshaUUID::audioStatus)) {
                //LOG_INFO("%s: Discovered AudioStatusPoint characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "ASP char discovered");
                ha->services.asha.asp = c;
            } else if (uuid_eq(c.uuid128, AshaUUID::volume)) {
                //LOG_INFO("%s: Discovered Volume characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "Volume char discovered");
                ha->services.asha.vol = c;
            } else if (uuid_eq(c.uuid128, AshaUUID::psm)) {
                //LOG_INFO("%s: Discovered PSM characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "PSM char discovered");
                ha->services.asha.psm = c;
            } else if (c.uuid16 == GapUUID::deviceName16) {
                //LOG_INFO("%s: Discovered Device Name characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "Device name char discovered");
                ha->services.gap.device_name = c;
            } else if (c.uuid16 == DisUUID::mfgName) {
                //LOG_INFO("%s: Discovered Manufacturer Name characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "Mfg name char discovered");
                ha->services.dis.manufacture_name = c;
            } else if (c.uuid16 == DisUUID::modelNum) {
                //LOG_INFO("%s: Discovered Model Number characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "Model num char discovered");
                ha->services.dis.model_num = c;
            } else if (c.uuid16 == DisUUID::fwVers) {
                //LOG_INFO("%s: Discovered FW Version characteristic", ha->get_side_str());
                short_log_pkt.set_data_str("%s", "FW vers char discovered");
                ha->services.dis.fw_vers = c;
            }
            add_event_to_buffer(ha->conn_id, short_log_pkt);
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);

            switch (ha->process_state) {
                case DiscoverASHAChar | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error discovering ASHA characteristics: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::DiscASHAChar, StatusType::ATTStatus, att_status));
                        ha->disconnect();
                    } else {
                        ha->process_state = ReadROP;
                    }
                    break;
                case DiscoverGAPChar | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error discovering GAP characteristics: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::DiscGAPChar, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    }
                    ha->process_state = ReadDeviceName;
                    break;
                case DiscoverDISChar | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error discovering DIS characteristics: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::DiscDISChar, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    }
                    ha->process_state = ReadMfgName;
                    break;
                default:
                    break;
            }
            break;
    }
}

void HearingAid::handle_char_read(PACKET_HANDLER_PARAMS)
{
    using enum ProcessState;
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;
    uint16_t val_len = 0U;
    uint16_t val_handle = 0U;
    const uint8_t* val = nullptr;

    switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
            handle = gatt_event_characteristic_value_query_result_get_handle(packet);
            val_handle = gatt_event_characteristic_value_query_result_get_value_handle(packet);
            val_len = gatt_event_characteristic_value_query_result_get_value_length(packet);
            val = gatt_event_characteristic_value_query_result_get_value(packet);
            ha = get_by_con_handle(handle);

            if (val_handle == ha->services.asha.rop.value_handle) {
                //LOG_INFO("%s: ROP characteristic read", ha->get_side_str());
                ha->rop.read(val);
                ha->side_str = ha->rop.side == Side::Left ? "Left" : "Right";
            } else if (val_handle == ha->services.asha.psm.value_handle) {
                ha->psm = val[0];
                //LOG_INFO("%s: PSM characteristic read: %d", ha->get_side_str(), ha->psm);
            } else if (val_handle == ha->services.gap.device_name.value_handle) {
                //LOG_INFO("%s: Device name read", ha->get_side_str());
                ha->device_name.clear();
                ha->device_name.append((const char*)val, val_len);
            } else if (val_handle == ha->services.dis.manufacture_name.value_handle) {
                //LOG_INFO("%s: Manufacterer name read", ha->get_side_str());
                ha->manufacturer.clear();
                ha->manufacturer.append((const char*)val, val_len);
            } else if (val_handle == ha->services.dis.model_num.value_handle) {
                //LOG_INFO("%s: Model number read", ha->get_side_str());
                ha->model.clear();
                ha->model.append((const char*)val, val_len);
            } else if (val_handle == ha->services.dis.fw_vers.value_handle) {
                //LOG_INFO("%s: FW version read", ha->get_side_str());
                ha->fw_vers.clear();
                ha->fw_vers.append((const char*)val, val_len);
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);

            switch (ha->process_state) {
                case ReadROP | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading ROP characteristic: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::ROPRead, StatusType::ATTStatus, att_status));
                        ha->disconnect();
                    } else {
                        EventPacket ev_pkt(EventType::ROPRead);
                        memcpy(ev_pkt.data.rop, ha->rop.raw_rop_data, sizeof ev_pkt.data.rop);
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                        ha->process_state = ReadPSM;
                    }
                    break;
                case ReadPSM | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading PSM characteristics %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::PSMRead, StatusType::ATTStatus, att_status));
                        ha->disconnect();
                    } else {
                        EventPacket ev_pkt(EventType::PSMRead);
                        ev_pkt.data.psm = ha->psm;
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                        ha->process_state = ha->cached ? ConnectL2CAP : DiscoverGAPChar;
                    }
                    break;
                case ReadDeviceName | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading device name characteristic: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::DevNameRead, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    } else {
                        EventPacket ev_pkt(EventType::DevNameRead);
                        ev_pkt.set_data_str("%s", ha->device_name.c_str());
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                    }
                    ha->process_state = DiscoverDISChar;
                    break;
                case ReadMfgName | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading manufacturer name characteristic: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::MfgRead, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    } else {
                        EventPacket ev_pkt(EventType::MfgRead);
                        ev_pkt.set_data_str("%s", ha->manufacturer.c_str());
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                    }
                    ha->process_state = ReadModelNum;
                    break;
                case ReadModelNum | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading model number characteristic: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::ModelRead, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    } else {
                        EventPacket ev_pkt(EventType::ModelRead);
                        ev_pkt.set_data_str("%s", ha->model.c_str());
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                    }
                    ha->process_state = ReadFWVers;
                    break;
                case ReadFWVers | ProcessBusy:
                    if (att_status != ATT_ERROR_SUCCESS) {
                        //LOG_ERROR("%s: Error reading FW version characteristic: %s", ha->get_side_str(), att_err_str(att_status));
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::FWRead, StatusType::ATTStatus, att_status));
                        // Not a fatal error
                    } else {
                        EventPacket ev_pkt(EventType::FWRead);
                        ev_pkt.set_data_str("%s", ha->fw_vers.c_str());
                        add_event_to_buffer(ha->conn_id, ev_pkt);
                    }
                    ha->process_state = ConnectL2CAP;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void HearingAid::handle_acp_write(PACKET_HANDLER_PARAMS)
{
    using namespace comm;
    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;

    if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
        handle = gatt_event_query_complete_get_handle(packet);
        att_status = gatt_event_query_complete_get_att_status(packet);
        ha = get_by_con_handle(handle);

        EventType ev_type = ha->audio_state == AudioState::Start ? EventType::ACPStart 
                                                                 : EventType::ACPStop;
        if (att_status != ATT_ERROR_SUCCESS) {
            //LOG_ERROR("%s: ACP write failed with status: %s", ha->get_side_str(), att_err_str(att_status));
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::ATTStatus, att_status));
            ha->audio_state = AudioState::Ready;
        } else {
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type));
            if (ha->audio_state == AudioState::Stop) {
                if (ha->other && ha->other->is_streaming()) {
                    ha->other->send_acp_status(ACPStatus::other_disconnected);
                }
            }
        }
    }
}

void HearingAid::handle_l2cap_cbm(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint16_t cid = 0U;
    uint8_t bt_status = ERROR_CODE_SUCCESS;
    HearingAid *ha = nullptr;

    switch (hci_event_packet_get_type(packet)) {
        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
            handle = l2cap_event_cbm_channel_opened_get_handle(packet);
            bt_status = l2cap_event_cbm_channel_opened_get_status(packet);
            ha = get_by_con_handle(handle);
            if (bt_status != ATT_ERROR_SUCCESS) {
                //LOG_ERROR("%s: Error creating L2CAP cbm connection: %s", ha->get_side_str(), bt_err_str(att_status));
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::L2CAPCon, StatusType::L2CapStatus, bt_status));
                // Try again later
                ha->unset_process_busy();
                ha->process_delay_ticks = ha_process_delay_ticks * 3;
            } else {
                //LOG_INFO("%s: L2CAP cbm connection created", ha->get_side_str());
                EventPacket ev_pkt(EventType::L2CAPCon);
                ev_pkt.data.cid = ha->cid;
                add_event_to_buffer(ha->conn_id, ev_pkt);
                ha->process_state = ProcessState::EnASPNotification;
            }
            break;
        case L2CAP_EVENT_CAN_SEND_NOW:
            cid = l2cap_event_can_send_now_get_local_cid(packet);
            ha = get_by_cid(cid);
            l2cap_send(cid, ha->audio_data, sdu_size_bytes);
            break;
        case L2CAP_EVENT_PACKET_SENT:
            cid = l2cap_event_packet_sent_get_local_cid(packet);
            ha = get_by_cid(cid);
            ha->unset_audio_busy();
            break;
        default:
            break;
    }
}

void HearingAid::handle_asp_notification_reg(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;

    if (hci_event_packet_get_type(packet) == GATT_EVENT_QUERY_COMPLETE) {
        handle = gatt_event_query_complete_get_handle(packet);
        att_status = gatt_event_query_complete_get_att_status(packet);
        ha = get_by_con_handle(handle);
        if (att_status != ATT_ERROR_SUCCESS) {
            //LOG_ERROR("%s: Failed to enable ASP notifications with status: %s", ha->get_side_str(), att_err_str(att_status));
            // Try again later
            ha->unset_process_busy();
            ha->process_delay_ticks = ha_process_delay_ticks;
            add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPNotEnable, StatusType::ATTStatus, att_status));
        } else {
            //LOG_INFO("%s: ASP notifications enabled", ha->get_side_str());
            ha->process_state = ProcessState::Finalize;
            add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPNotEnable));
        }
    }
}

void HearingAid::handle_gatt_notification(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint16_t val_handle = 0U;
    int8_t asp_status = 0;
    HearingAid *ha = nullptr;

    if (hci_event_packet_get_type(packet) == GATT_EVENT_NOTIFICATION) {
        handle = gatt_event_notification_get_handle(packet);
        val_handle = gatt_event_notification_get_value_handle(packet);
        ha = get_by_con_handle(handle);
        if (val_handle == ha->services.asha.asp.value_handle) {
            asp_status = (int8_t)gatt_event_notification_get_value(packet)[0];
            EventPacket err_ev_pkt(EventType::ASPError);
            err_ev_pkt.data.asp_not = asp_status;

            switch (asp_status) {
                case ASPStatus::ok:
                    if (ha->audio_state == AudioState::Start) {
                        //LOG_INFO("%s: Audio start OK", ha->get_side_str());
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPStart));
                        ha->audio_state = AudioState::Streaming;
                        audio_buff.encode_audio = true;
                        if (ha->other && ha->other->is_streaming()) {
                            ha->other->send_acp_status(ACPStatus::other_connected);
                        }
                    } else if (ha->audio_state == AudioState::Stop) {
                        //LOG_INFO("%s: Audio stop OK", ha->get_side_str());
                        add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPStop));
                        ha->audio_state = AudioState::Ready;
                    }
                    break;
                case ASPStatus::unkown_command:
                case ASPStatus::illegal_params:
                    add_event_to_buffer(ha->conn_id, err_ev_pkt);
                    break;
                default:
                    //LOG_ERROR("%s: ASP: Unknown command", ha->get_side_str());
                    //LOG_ERROR("%s: ASP: Illegal parameters", ha->get_side_str());
                    //LOG_ERROR("%s: ASP: Unknown status: %d", ha->get_side_str(), asp_status);
                    add_event_to_buffer(ha->conn_id, err_ev_pkt);
                    break;
            }
        }
    }
}

void __not_in_flash_func(HearingAid::process_audio)()
{
    using namespace comm;

    uint32_t w_index = audio_buff.get_write_index();
    AudioBuffer::Volume vol = audio_buff.get_volume();
    bool pcm_is_streaming = audio_buff.pcm_streaming.Load();

    EventPacket short_log_pkt(EventType::ShortLog);

    for (auto ha : hearing_aids) {
        if (ha->process_state != ProcessState::Audio) { continue; }
        ha->credits = l2cap_cbm_available_credits(ha->cid);
        switch (ha->audio_state) {
            case AudioState::Ready:
                if (pcm_is_streaming && ha->credits >= 8) {
                    ha->set_audio_busy();
                    ha->send_acp_start();
                }
                break;
            case AudioState::Streaming:
                if (!pcm_is_streaming || ha->credits == 0) {
                    if (!pcm_is_streaming) {
                        audio_buff.encode_audio = false;
                    }
                    // LOG_INFO("%s: Stopping audio stream. PCM Streaming: %d, Credits: %d", 
                    //           ha->get_side_str(), (int)pcm_is_streaming, (int)ha->credits);
                    short_log_pkt.set_data_str("PCM: %d - Cr: %d", (int)pcm_is_streaming, (int)ha->credits);
                    add_event_to_buffer(ha->conn_id, short_log_pkt);
                    ha->send_acp_stop();
                } else {
                    // Send volume update if volume has changed
                    int8_t v = ha->rop.side == Side::Left ? vol.l : vol.r;
                    if (ha->curr_vol != v) {
                        ha->curr_vol = v;
                        ha->send_volume(ha->curr_vol);
                    }
                    if (w_index == 0) {
                        break;
                    }
                    if (w_index > ha->curr_write_index) {
                        ha->set_audio_busy();
                        ha->curr_write_index = w_index;

                        if ((ha->curr_write_index - ha->curr_read_index) >= 2) {
                            ha->curr_read_index = ha->curr_write_index - 1;
                        }
                        AudioBuffer::G722Buff& packet = audio_buff.get_g_buff(ha->curr_read_index);
                        ha->audio_data = ha->rop.side == Side::Left ? packet.l.data() : packet.r.data();
                        ++(ha->curr_read_index);
                        l2cap_request_can_send_now_event(ha->cid);
                    }
                }
                break;
            default:
                break;
        }
    }
}

/* Private methods */

HearingAid* HearingAid::get_by_con_handle(hci_con_handle_t handle)
{
    for (auto ha : hearing_aids) {
        if (ha->conn_handle == handle) {
            return ha;
        }
    }
    return nullptr;
}

HearingAid* HearingAid::get_by_cid(uint16_t cid)
{
    for (auto ha : hearing_aids) {
        if (ha->cid == cid) {
            return ha;
        }
    }
    return nullptr;
}

HearingAid* HearingAid::get_by_cached_addr(bd_addr_t addr)
{
    for (auto ha : hearing_aids) {
        if (ha->cached && bd_addr_cmp(ha->addr, addr) == 0) {
            return ha;
        }
    }
    return nullptr;
}

bool HearingAid::full_set_connected()
{
    bool have_left = false;
    bool have_right = false;
    for (auto ha : hearing_aids) {
        if (ha->is_connected() && ha->rop.side != Side::Unset) {
            if (ha->rop.mode == Mode::Mono) {
                have_left = true;
                have_right = true;
                break;
            } else if (ha->rop.side == Side::Left) {
                have_left = true;
            } else if (ha->rop.side == Side::Right) {
                have_right = true;
            }
        }
    }
    return have_left && have_right;
}

void HearingAid::set_other_side_ptrs()
{
    if (hearing_aids[0]->is_connected() && hearing_aids[1]->is_connected()) {
        hearing_aids[0]->other = hearing_aids[1];
        hearing_aids[1]->other = hearing_aids[0];
    } else {
        hearing_aids[0]->other = nullptr;
        hearing_aids[1]->other = nullptr;
    }
}

void HearingAid::assign_next_conn_id()
{
    ++next_conn_id;
    conn_id = next_conn_id;
}

bool HearingAid::is_connected()
{
    return connected && conn_handle != HCI_CON_HANDLE_INVALID;
}

bool HearingAid::is_streaming()
{
    return  audio_state == AudioState::Streaming || 
            audio_state == (AudioState::Streaming | AudioState::AudioBusy);
}

void HearingAid::set_process_busy()
{
    process_state |= ProcessState::ProcessBusy;
}

void HearingAid::unset_process_busy()
{
    process_state &= ~ProcessState::ProcessBusy;
}

void HearingAid::set_audio_busy()
{
    audio_state |= AudioState::AudioBusy;
}

void HearingAid::unset_audio_busy()
{
    audio_state &= ~AudioState::AudioBusy;
}

void HearingAid::set_data_langth()
{
    while (1) {
        if (hci_can_send_command_packet_now()) {
            hci_send_cmd(&hci_le_set_data_length, conn_handle, pdu_len, max_tx_time);
            break;
        }
    }
}

void HearingAid::send_acp_start()
{
    using namespace comm;

    audio_state = AudioState::Start;
    acp_cmd_packet[0] = ACPOpCode::start; // Opcode
    acp_cmd_packet[1] = 1u; // G.722 codec at 16KHz
    acp_cmd_packet[2] = 0u; // Unkown audio type
    acp_cmd_packet[3] = (uint8_t)curr_vol; // Volume
    acp_cmd_packet[4] = (other && other->is_streaming()) ? 1 : 0; // Otherstate

    uint8_t res = gatt_client_write_value_of_characteristic(&HearingAid::handle_acp_write, 
                                                            conn_handle, 
                                                            services.asha.acp.value_handle,
                                                            (uint16_t)acp_cmd_packet.size(),
                                                            acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        //LOG_ERROR("%s: ACP Write: Start error %s", get_side_str(), bt_err_str(res));
        add_event_to_buffer(conn_id, EventPacket(EventType::ACPStart, StatusType::BtstackStatus, res));
        audio_state = AudioState::Ready;
    }
}

void HearingAid::send_acp_stop()
{
    using namespace comm;

    audio_state = AudioState::Stop;
    acp_cmd_packet[0] = ACPOpCode::stop;
    uint8_t res = gatt_client_write_value_of_characteristic(&HearingAid::handle_acp_write,
                                                            conn_handle,
                                                            services.asha.acp.value_handle,
                                                            1u,
                                                            acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        //LOG_ERROR("%s: ACP Write: Stop error %s", get_side_str(), bt_err_str(res));
        add_event_to_buffer(conn_id, EventPacket(EventType::ACPStop, StatusType::BtstackStatus, res));
        disconnect();
    }
}

void HearingAid::send_acp_status(uint8_t status)
{
    using namespace comm;

    acp_cmd_packet[0] = ACPOpCode::status;
    acp_cmd_packet[1] = status;
    auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                          services.asha.acp.value_handle,
                                                                          2u,
                                                                          acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        //LOG_ERROR("%s: Updating status via ACP failed with %s", get_side_str(), bt_err_str(res));
        add_event_to_buffer(conn_id, EventPacket(EventType::ACPStatus, StatusType::BtstackStatus, res));
    }
}

void HearingAid::send_volume(int8_t volume)
{
    using namespace comm;

    uint8_t res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                             services.asha.vol.value_handle,
                                                                             sizeof(volume),
                                                                             (uint8_t*)&volume);
    if (res != ERROR_CODE_SUCCESS) {
        //LOG_ERROR("%s: Sending volume failed with %s", get_side_str(), bt_err_str(res));
        add_event_to_buffer(conn_id, EventPacket(EventType::AudioVolume, StatusType::BtstackStatus, res));
    } else {
        //LOG_INFO("%s: Volume changed to %d", get_side_str(), (int)volume);
        EventPacket ev_pkt(EventType::AudioVolume);
        ev_pkt.data.volume = volume;
        add_event_to_buffer(conn_id, ev_pkt);
    }
}

void HearingAid::disconnect()
{
    //LOG_INFO("%s: Disconnect requested", get_side_str());
    process_state = ProcessState::Disconnect;
    gap_disconnect(conn_handle);
}

void HearingAid::reset()
{
    conn_id = comm::unset_conn_id;
    conn_handle = HCI_CON_HANDLE_INVALID;
    cid = 0U;
    connected = false;
    process_state = ProcessState::ProcessUnset;
    audio_state = AudioState::AudioUnset;
    other = nullptr;
    psm = 0;
    credits = 0;
    paired_and_bonded = false;
    process_delay_ticks = 0;
    if (!cached) {
        memset(addr, 0U, sizeof(bd_addr_t));
        device_name.clear();
        manufacturer.clear();
        model.clear();
        fw_vers.clear();

        rop = {};
        side_str = "Unknown";
        services = {};
    }
}

const char* HearingAid::get_side_str()
{
    return (rop.side == Side::Unset) ? bd_addr_to_str(addr) : side_str;
}

static void delete_paired_device(const bd_addr_t address)
{
    using namespace comm;
    //LOG_INFO("%s: Removing paired device", bd_addr_to_str(address));
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    EventPacket ev_pkt(EventType::DeletePair);
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN && bd_addr_cmp(addr, address) == 0) {
            //LOG_INFO("Found. Removing");
            le_device_db_remove(i);
            bd_addr_copy(ev_pkt.data.conn_info.addr, address);
            add_event_to_buffer(unset_conn_id, ev_pkt);
        }
    }
}

} // namespace asha