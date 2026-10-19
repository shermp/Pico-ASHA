#include <bitset>
#include <pico/assert.h>

#include "hearing_aid.hpp"
#include "asha_uuid.hpp"
#include "bt_status_err.hpp"
#include "usb_common.hpp"

namespace asha
{

/* Data length variables */
static constexpr uint16_t pdu_len = 167u;

static constexpr uint16_t max_tx_time = (pdu_len + 14) * 8;

static constexpr size_t ev_packet_str_size = sizeof comm::EventPacket::data.str;

static constexpr int max_error_count = 10;

static int8_t volume_mute = -128;

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

static void delete_paired_devices();
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

/* Public methods */

HearingAid::AudioSession HearingAid::audio_session;

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
    connections_allowed = true;
    auto_pair_enabled = false;
}

void HearingAid::process()
{
    using enum ProcessState;
    using namespace comm;

    for (auto ha : hearing_aids) {
        uint8_t res = ERROR_CODE_SUCCESS;
        bool advanced_service_index = false;
        bool advanced_char_index = false;
        if (!ha->is_connected()) { continue; }
        if (ha->l2cap_open_pending && ha->process_state == ConnectL2CAPInProgress
            && btstack_run_loop_get_time_ms() - ha->l2cap_open_started_ms >= l2cap_open_timeout_ms) {
            short_log(ha->conn_id, "%s", "CoC open timeout: ACL reset");
            // BTstack owns the pending channel and receive buffer. ACL teardown
            // cancels/frees it; do not clear the CID and allocate another one.
            ha->disconnect();
            continue;
        }
        if (ha->process_delay_ticks > 0) {
            --ha->process_delay_ticks;
            continue;
        }
        // Just disconnect the hearing aid if we get too many errors
        if (ha->error_count >= max_error_count) {
            ha->disconnect();
            continue;
        }
        EventType ev_type;
        ProcessState revert_process_state = ha->process_state;
        switch (ha->process_state)
        {
            case DiscoverServices:
                ev_type = EventType::DiscServices;
                //LOG_INFO("%s: Discovering services", ha->get_side_str());
                ha->process_state = DiscoverServicesInProgress;
                res = gatt_client_discover_primary_services(&HearingAid::handle_service_discovery, ha->conn_handle);
                break;
            case PairBond:
                ev_type = EventType::PairAndBond;
                //LOG_INFO("%s: Pairing and bonding",  ha->get_side_str())
                ha->process_state = PairBondInProgress;
                sm_request_pairing(ha->conn_handle);
                break;
            case DataLength:
                ev_type = EventType::DLE;
                //LOG_INFO("%s: Setting data length",  ha->get_side_str())

                // Advance to discovery right after sending: the controller only emits
                // HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE when the effective length
                // actually changes, so an aid already at the requested size would
                // never fire it and we'd stall waiting.
                gap_le_set_data_length(ha->conn_handle, pdu_len, max_tx_time);
                ha->process_state = DiscoverChars;
                break;
            case DiscoverChars: {
                ha->process_state = DiscoverCharsInProgress;
                auto i = ha->service_index++; // Yes, the post-increment is on purpose
                advanced_service_index = true;
                auto s = ha->service_arr[i];
                ev_type = ha->service_ev_arr[i];
                res = gatt_client_discover_characteristics_for_service(&HearingAid::handle_char_discovery, ha->conn_handle, s);
                break;
            }
            case ReadChars: {
                ha->process_state = ReadCharsInProgress;
                auto i = ha->chars_index++; // Yes, the post-increment is on purpose
                advanced_char_index = true;
                auto c = ha->chars_arr[i];
                ev_type = ha->chars_ev_arr[i];
                res = gatt_client_read_value_of_characteristic(&HearingAid::handle_char_read, ha->conn_handle, c);
                break;
            }
            case ConnectL2CAP:
                if (ha->cid != 0 || ha->l2cap_open_pending) {
                    short_log(ha->conn_id, "%s", "CoC still owned: ACL reset");
                    ha->disconnect();
                    continue;
                }
                ev_type = EventType::L2CAPCon;
                //LOG_INFO("%s: Create L2CAP CoC", ha->get_side_str());
                ha->process_state = ConnectL2CAPInProgress;
                ha->l2cap_open_started_ms = btstack_run_loop_get_time_ms();
                ha->l2cap_open_pending = true;
                res = l2cap_cbm_create_channel(&HearingAid::handle_l2cap_cbm, 
                                               ha->conn_handle, 
                                               ha->psm, 
                                               ha->recv_buff.data(), 
                                               ha->recv_buff.size(), 
                                               L2CAP_LE_AUTOMATIC_CREDITS, 
                                               LEVEL_2, 
                                               &ha->cid);
                break;
            case EnBattNotification:
                ev_type = EventType::MFIBatteryNotEnable;
                ha->process_state = EnBattNotificationInProgress;
                res = gatt_client_write_client_characteristic_configuration(&HearingAid::handle_notification_reg, 
                                                                            ha->conn_handle, 
                                                                            &ha->services.mfi.battery, 
                                                                            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                break;
            case EnASPNotification:
                ev_type = EventType::ASPNotEnable;
                //LOG_INFO("%s: Enable ASP notification", ha->get_side_str());
                ha->process_state = EnASPNotificationInProgress;
                res = gatt_client_write_client_characteristic_configuration(&HearingAid::handle_notification_reg, 
                                                                            ha->conn_handle, 
                                                                            &ha->services.asha.asp, 
                                                                            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                break;
            case Finalize:
                ev_type = EventType::StreamReady;
                //LOG_INFO("%s: Ready for audio streaming", ha->get_side_str());
                ha->process_state = FinalizeInProgress;
                if (full_set_connected()) {
                    runtime_settings.set_full_set_paired(true);
                    led_mgr.set_led(LEDManager::State::On);
                } else {
                    led_mgr.set_led_pattern(one_connected);
                    start_scan();
                }
                ha->cached = true;
                ha->error_count = 0;
                ha->process_state = Audio;
                ha->audio_state = AudioState::Ready;
                if (ha->other && ha->other->is_streaming()) {
                    audio_session.request_stream_restart();
                }
            default:
                break;
        }
        if (res != ERROR_CODE_SUCCESS) {
            if (revert_process_state == ConnectL2CAP) {
                // A synchronous API error allocated no channel and produces
                // no callback. Never retain a freed/failed CID for routing.
                ha->cid = 0;
                ha->l2cap_open_pending = false;
                ha->l2cap_open_started_ms = 0;
            }
            if (advanced_service_index) {
                // A failed call produces no completion callback. Keep the
                // index on the same item for the delayed retry.
                --ha->service_index;
            } else if (advanced_char_index) {
                --ha->chars_index;
            }
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::BtstackStatus, res));
            if (revert_process_state == ConnectL2CAP && ha->l2cap_recovery_pending) {
                ha->disconnect();
                continue;
            }
            //LOG_ERROR("%s: BTStack error: %s", ha->get_side_str(), bt_err_str(res));
            ha->process_state = revert_process_state;
            ++ha->error_count;
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
    RemoteInfo info = {};
    uint16_t intro_flags = 0x00;
    if (connections_allowed) {
        intro_flags |= IntroFlags::conn_allowed;
    }
    if (audio_session.enabled) {
        intro_flags |= IntroFlags::streaming_enabled;
    }
    send_intro_packet((int8_t)num_connected(), intro_flags);
    auto const settings = runtime_settings.get_usb_settings();
    USBInfo usb_info = {
        .uac_vers = settings.uac_version,
        .min_vol = settings.min_vol,
        .max_vol = settings.max_vol,
        .reserved = 0
    };
    send_usb_info_packet(usb_info);
    for (auto ha : hearing_aids) {
        if (!ha->is_connected()) {
            continue;
        }
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
        memcpy(info.sw_vers, ha->sw_vers.data(), sizeof(info.sw_vers));
        if (ha->rop) {
            info.side = (ha->rop.side() == Side::Left) ? CSide::Left : CSide::Right;
            info.mode = (ha->rop.mode() == Mode::Binaural) ? CMode::Binaural : CMode::Mono;
        } else {
            info.side = CSide::Unset;
            info.mode = CMode::Unset;
        }
        info.audio_streaming = ha->is_streaming();
        info.curr_vol = ha->curr_vol;
        info.curr_battery = ha->battery_level;

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

void HearingAid::set_connections_allowed(bool allowed)
{
    if (connections_allowed == allowed) {
        return;
    }
    if (connections_allowed && !allowed) {
        connections_allowed = false;
        gap_stop_scan();
        for (auto ha : hearing_aids) {
            if (ha->is_connected()) {
                ha->disconnect();
            }
        }
    } else if (!connections_allowed && allowed) {
        connections_allowed = true;
        start_scan();
    }
}

void HearingAid::set_audio_streaming_enabled(bool enabled)
{
    audio_session.enabled = enabled;
}

void HearingAid::set_auto_pair_enabled(bool enabled)
{
    auto_pair_enabled = enabled;
}

void HearingAid::start_scan()
{
    if (connections_allowed) {
        gap_set_scan_params(1, 0x0030, 0x0030, runtime_settings.get_full_set_paired() ? 1 : 0);
        gap_start_scan();
    }
}

void HearingAid::on_ad_report(const AdvertisingReport& report)
{
    if (is_addr_connected(report.address)) { return; }
    gap_stop_scan();
    if (full_set_connected()) { 
        return; 
    }
    if (runtime_settings.get_full_set_paired() || (auto_pair_enabled && report.is_hearing_aid)) {
        connect(report.address, report.address_type);
    } else {
        comm::AdvertisingPacket ad_pkt = {};
        bd_addr_copy(ad_pkt.addr, report.address);
        ad_pkt.addr_type = report.address_type;
        ad_pkt.is_ha = report.is_hearing_aid;
        ad_pkt.rssi = report.rssi;
        size_t name_size = report.name.size() < sizeof(ad_pkt.name) ? report.name.size()
                                                                    : sizeof(ad_pkt.name);
        strncpy(ad_pkt.name, report.name.data(), name_size);
        ad_pkt.name[sizeof(ad_pkt.name)-1] = '\0';
        comm::send_advertising_packet(ad_pkt);
        start_scan();
    }
}

void HearingAid::connect(const bd_addr_t addr, bd_addr_type_t addr_type)
{
    gap_stop_scan();
    gap_connect(addr, addr_type);
}

void HearingAid::on_connected(bd_addr_t addr, hci_con_handle_t handle, uint8_t status)
{
    using namespace comm;

    if (status != ERROR_CODE_SUCCESS || handle == HCI_CON_HANDLE_INVALID) {
        uint8_t const error = status != ERROR_CODE_SUCCESS ? status : ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER;
        add_event_to_buffer(unset_conn_id, EventPacket(EventType::RemoteConnected, StatusType::BtstackStatus, error));
        start_scan();
        return;
    }
    if (get_by_con_handle(handle)) { return; }
    auto ha_cached = get_by_cached_addr(addr);
    if (ha_cached) {
        if (ha_cached->is_connected()) {
            // Do not overwrite a live ACL/CID with a duplicate connection.
            gap_disconnect(handle);
            return;
        }
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
    if (!ha) {
        // A connection can be rejected after both slots are occupied. Its
        // disconnect event has no corresponding slot to reset.
        return;
    }
    // if (ha->process_state == ProcessState::Disconnect) {
    //     LOG_INFO("%s: Disconnected", ha->get_side_str());
    // } else {
    //     LOG_ERROR("%s: Disconnected with status code: %s and reason: %s", 
    //                ha->get_side_str(), 
    //                bt_err_str(status), bt_err_str(reason));
    // }
    EventPacket ev_pkt(EventType::RemoteDisconnected, StatusType::ATTStatus, status, reason);
    add_event_to_buffer(ha->conn_id, ev_pkt);
    ha->audio_state = AudioState::AudioUnset;
    audio_session.on_stream_stopped(*ha);
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

    // DLE changes are handled asynchronously: the DataLength process state issues
    // the set-data-length command and advances to DiscoverChars itself, rather than
    // waiting on this event (which the controller doesn't emit when the effective
    // data length is unchanged). This handler is now informational only.
    (void)ha;
    // LOG_INFO("%s: DL set to: RX Octets: %hu, RX Time: %hu us, TX Octets: %hu, TX Time: %hu us",
    //                     ha->get_side_str(),
    //                     rx_octets, rx_time, tx_octets, tx_time);
}

void HearingAid::delete_pair()
{
    delete_paired_devices();
    runtime_settings.set_full_set_paired(false);
    for (auto ha : hearing_aids) {
        if (ha->is_connected()) {
            ha->disconnect();
        }
    }
}

void HearingAid::delete_pair(uint16_t conn_id)
{
    if (conn_id != comm::unset_conn_id) {
        auto ha = get_by_conn_id(conn_id);
        if (ha) {
            delete_paired_device(ha->addr);
            runtime_settings.set_full_set_paired(false);
            ha->disconnect();
        }
    }
}

void HearingAid::handle_sm(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    HearingAid *ha = nullptr;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    uint8_t reason = 0U;

    uint8_t ev_type = hci_event_packet_get_type(packet);

    switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            handle = sm_event_just_works_request_get_handle(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                break;
            }
            short_log(ha->conn_id, "%s", "Just Works Req");
            //LOG_INFO("%s: Just Works requested", ha->get_side_str());
            sm_just_works_confirm(handle);
            break;

        case SM_EVENT_PAIRING_STARTED:
            handle = sm_event_pairing_started_get_handle(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                break;
            }
            short_log(ha->conn_id, "%s", "Pairing started");
            //LOG_INFO("%s: Pairing started", ha->get_side_str());
            break;

        case SM_EVENT_PAIRING_COMPLETE: {
            handle = sm_event_pairing_complete_get_handle(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                break;
            }
            att_status = sm_event_pairing_complete_get_status(packet);
            reason = sm_event_pairing_complete_get_reason(packet);

            EventPacket ev_pkt(EventType::PairAndBond, StatusType::SMStatus, att_status, reason);
            switch (att_status) {
                case ERROR_CODE_SUCCESS:
                    //LOG_INFO("%s: Pairing complete", ha->get_side_str());;
                    ha->paired_and_bonded = true;
                    ha->process_state = ProcessState::DataLength;
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
                        short_log(ha->conn_id, "%s", "Attempt auth downgrade");
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
            if (!ha) {
                break;
            }
            short_log(ha->conn_id, "%s", "Reencryption started");
            //LOG_INFO("%s: Reencryption started", ha->get_side_str());
            break;

        case SM_EVENT_REENCRYPTION_COMPLETE: {
            handle = sm_event_reencryption_complete_get_handle(packet);
            att_status = sm_event_reencryption_complete_get_status(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                break;
            }

            EventPacket ev_pkt(EventType::PairAndBond, StatusType::SMStatus, att_status);

            switch (att_status) {
                case ERROR_CODE_SUCCESS:
                    //LOG_INFO("%s: Reencryption succeeded", ha->get_side_str());
                    ha->paired_and_bonded = true;
                    ha->process_state = ProcessState::DataLength;
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
            short_log(unset_conn_id, "Unhandled SM Ev: 0x%02x", ev_type);
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
            if (!ha) {
                return;
            }

            gatt_event_service_query_result_get_service(packet, &s);
            if (AshaUUID::service == s.uuid128 || AshaUUID::service16 == s.uuid16) {
                //LOG_INFO("%s: Discovered ASHA service", ha->get_side_str());
                short_log(ha->conn_id, "%s", "ASHA service discovered");
                ha->services.asha.service = s;
            } else if (GapUUID::service16 == s.uuid16) {
                //LOG_INFO("%s: Discovered GAP service", ha->get_side_str());
                short_log(ha->conn_id, "%s", "GAP service discovered");
                ha->services.gap.service = s;
            } else if (DisUUID::service16 == s.uuid16) {
                //LOG_INFO("%s: Discovered DIS service", ha->get_side_str());
                short_log(ha->conn_id, "%s", "DIS service discovered");
                ha->services.dis.service = s;
            } else if (MfiUUID::service == s.uuid128) {
                short_log(ha->conn_id, "%s", "MFI service discovered");
                ha->services.mfi.service = s;
            }
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                return;
            }

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
            if (!ha) {
                return;
            }
            gatt_event_characteristic_query_result_get_characteristic(packet, &c);

            if (AshaUUID::readOnlyProps == c.uuid128) {
                //LOG_INFO("%s: Discovered ReadOnlyProperties characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "ROP char discovered");
                ha->services.asha.rop = c;
            } else if (AshaUUID::audioControlPoint == c.uuid128) {
                //LOG_INFO("%s: Discovered AudioControlPoint characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "ACP char discovered");
                ha->services.asha.acp = c;
            } else if (AshaUUID::audioStatus == c.uuid128) {
                //LOG_INFO("%s: Discovered AudioStatusPoint characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "ASP char discovered");
                ha->services.asha.asp = c;
            } else if (AshaUUID::volume == c.uuid128) {
                //LOG_INFO("%s: Discovered Volume characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "Volume char discovered");
                ha->services.asha.vol = c;
            } else if (AshaUUID::psm == c.uuid128) {
                //LOG_INFO("%s: Discovered PSM characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "PSM char discovered");
                ha->services.asha.psm = c;
            } else if (GapUUID::deviceName16 == c.uuid16) {
                //LOG_INFO("%s: Discovered Device Name characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "Device name char discovered");
                ha->services.gap.device_name = c;
            } else if (DisUUID::mfgName == c.uuid16) {
                //LOG_INFO("%s: Discovered Manufacturer Name characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "Mfg name char discovered");
                ha->services.dis.manufacture_name = c;
            } else if (DisUUID::modelNum == c.uuid16) {
                //LOG_INFO("%s: Discovered Model Number characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "Model num char discovered");
                ha->services.dis.model_num = c;
            } else if (DisUUID::fwVers == c.uuid16) {
                //LOG_INFO("%s: Discovered FW Version characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "FW vers char discovered");
                ha->services.dis.fw_vers = c;
            } else if (DisUUID::swVers == c.uuid16) {
                //LOG_INFO("%s: Discovered FW Version characteristic", ha->get_side_str());
                short_log(ha->conn_id, "%s", "SW vers char discovered");
                ha->services.dis.sw_vers = c;
            } else if (MfiUUID::battery == c.uuid128) {
                short_log(ha->conn_id, "%s", "MFI battery char discovered");
                ha->services.mfi.battery = c;
            }
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                return;
            }

            if (att_status != ATT_ERROR_SUCCESS) {
                auto ev_type = ha->service_ev_arr[ha->service_index - 1];
                add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::ATTStatus, att_status));
                if (ev_type == EventType::DiscASHAChar) {
                    ha->disconnect();
                    break;
                }
            }
            if (ha->service_index < ha->service_arr.size()) {
                ha->process_state = DiscoverChars;
            } else {
                ha->process_state = ReadChars;
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
            if (!ha) {
                return;
            }

            if (val_handle == ha->services.asha.rop.value_handle) {
                //LOG_INFO("%s: ROP characteristic read", ha->get_side_str());
                ha->rop.read(val);
                ha->side_str = ha->rop.side() == Side::Left ? "Left" : "Right";
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
            } else if (val_handle == ha->services.dis.sw_vers.value_handle) {
                //LOG_INFO("%s: FW version read", ha->get_side_str());
                ha->sw_vers.clear();
                ha->sw_vers.append((const char*)val, val_len);
            } else if (val_handle == ha->services.mfi.battery.value_handle) {
                ha->battery_level = val[0];
            }
            break;

        case GATT_EVENT_QUERY_COMPLETE: {
            handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);
            ha = get_by_con_handle(handle);
            if (!ha) {
                return;
            }

            auto ev_type = ha->chars_ev_arr[ha->chars_index - 1];

            if (att_status != ATT_ERROR_SUCCESS) {
                add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::ATTStatus, att_status));
                if (ev_type == EventType::ROPRead || ev_type == EventType::PSMRead) {
                    ha->disconnect();
                    break;
                }
            }
            EventPacket ev_pkt(ev_type);
            switch (ev_type) {
                case EventType::ROPRead:
                    memcpy(ev_pkt.data.rop, ha->rop.raw_data, sizeof ev_pkt.data.rop);
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::PSMRead:
                    ev_pkt.data.psm = ha->psm;
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::DevNameRead:
                    ev_pkt.set_data_str("%s", ha->device_name.c_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::MfgRead:
                    ev_pkt.set_data_str("%s", ha->manufacturer.c_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::ModelRead:
                    ev_pkt.set_data_str("%s", ha->model.c_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::FWRead:
                    ev_pkt.set_data_str("%s", ha->fw_vers.c_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::SWRead:
                    ev_pkt.set_data_str("%s", ha->sw_vers.c_str());
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                case EventType::MfiBatteryRead:
                    ev_pkt.data.battery_level = ha->battery_level;
                    add_event_to_buffer(ha->conn_id, ev_pkt);
                    break;
                default:
                    break;
            }

            if (ha->chars_index < ha->chars_arr.size()) {
                ha->process_state = ReadChars;
            } else {
                ha->process_state = ConnectL2CAP;
            }
            break;
        }
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
        if (!ha || ha->acp_operation == ACPOperation::None || !ha->acp_write_pending) {
            return;
        }

        auto const operation = ha->acp_operation;
        uint32_t const now_ms = btstack_run_loop_get_time_ms();
        ha->acp_write_pending = false;
        // Teardown may have started while BTstack still owned the command.
        if (ha->process_state != ProcessState::Audio) {
            return;
        }
        EventType ev_type = operation == ACPOperation::Start ? EventType::ACPStart
                                                            : EventType::ACPStop;
        if (att_status != ATT_ERROR_SUCCESS) {
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type, StatusType::ATTStatus, att_status));
            ha->disconnect();
        } else if (!ha->acp_timed_out(now_ms)) {
            add_event_to_buffer(ha->conn_id, EventPacket(ev_type));
            ha->acp_phase_started_ms = now_ms;
            ha->complete_acp();
        }
    }
}

void HearingAid::handle_l2cap_cbm(PACKET_HANDLER_PARAMS)
{
    using namespace comm;

    if (packet_type != HCI_EVENT_PACKET || size < 2 || packet[1] + 2u > size) { return; }
    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint16_t cid = 0U;
    uint8_t bt_status = ERROR_CODE_SUCCESS;
    HearingAid *ha = nullptr;

    switch (hci_event_packet_get_type(packet)) {
        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
            if (size < 23) { break; }
            handle = l2cap_event_cbm_channel_opened_get_handle(packet);
            cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
            bt_status = l2cap_event_cbm_channel_opened_get_status(packet);
            ha = get_by_con_handle(handle);
            if (!ha || cid == 0 || ha->cid != cid) {
                // A successful unclaimed completion still owns a BTstack
                // channel. Close it, but never touch another aid's live CID.
                if (bt_status == ERROR_CODE_SUCCESS && cid != 0 && !get_by_cid(cid)) {
                    l2cap_disconnect(cid);
                }
                break;
            }
            if (!ha->l2cap_open_pending) { break; } // Duplicate/stale completion.
            ha->l2cap_open_pending = false;
            ha->l2cap_open_started_ms = 0;
            if (bt_status != ERROR_CODE_SUCCESS) {
                // BTstack discards failed opens after this callback. Drop the
                // local reference before this ID can be reused by the other aid.
                ha->cid = 0;
                ha->credits = 0;
            }
            if (ha->process_state != ProcessState::ConnectL2CAPInProgress) {
                if (bt_status == ERROR_CODE_SUCCESS) { l2cap_disconnect(cid); }
                break;
            }
            if (bt_status != ERROR_CODE_SUCCESS) {
                //LOG_ERROR("%s: Error creating L2CAP cbm connection: %s", ha->get_side_str(), bt_err_str(att_status));
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::L2CAPCon, StatusType::L2CapStatus, bt_status));
                if (ha->l2cap_recovery_pending) {
                    ha->disconnect();
                    break;
                }
                // Try again later
                ha->process_state = ProcessState::ConnectL2CAP;
                ++ha->error_count;
                ha->process_delay_ticks = ha_process_delay_ticks * 3;
            } else {
                //LOG_INFO("%s: L2CAP cbm connection created", ha->get_side_str());
                EventPacket ev_pkt(EventType::L2CAPCon);
                ev_pkt.data.cid = ha->cid;
                add_event_to_buffer(ha->conn_id, ev_pkt);
                ha->process_state = ProcessState::EnASPNotification;
            }
            break;
        case L2CAP_EVENT_PACKET_SENT:
            if (size < 4) { break; }
            cid = l2cap_event_packet_sent_get_local_cid(packet);
            ha = get_by_cid(cid);
            if (!ha || !ha->audio_tx_pending) {
                break;
            }
#ifdef PICO_ASHA_L2CAP_TX_TRACE
            short_log(ha->conn_id, "%s", "L2CAP Tx sent");
#endif
            ha->clear_audio_send_state();
            if (ha->process_state == ProcessState::Audio
                && ha->is_streaming()) {
                ha->l2cap_recovery_pending = false;
            }
            break;
        case L2CAP_EVENT_CHANNEL_CLOSED:
            if (size < 4) { break; }
            cid = l2cap_event_channel_closed_get_local_cid(packet);
            ha = get_by_cid(cid);
            if (!ha) {
                break;
            }
            add_event_to_buffer(ha->conn_id, EventPacket(EventType::L2CAPDiscon));
            ha->cid = 0U;
            ha->l2cap_open_pending = false;
            ha->l2cap_open_started_ms = 0;
            ha->credits = 0U;
            ha->clear_audio_send_state();
            ha->l2cap_recovery_ticks = 0U;
            ha->audio_state = AudioState::AudioUnset;
            ha->credit_average = {};
            audio_session.on_stream_stopped(*ha);
            if (ha->process_state == ProcessState::Disconnect) { break; }
            if (ha->acp_operation != ACPOperation::None) {
                ha->disconnect();
                break;
            }
            ha->l2cap_recovery_pending = true;
            // Keep the ACL connection, then create a fresh CoC with the
            // existing PSM and service discovery data.
            ha->process_state = ProcessState::ConnectL2CAP;
            break;
        default:
            break;
    }
}

void HearingAid::handle_notification_reg(PACKET_HANDLER_PARAMS)
{
    using namespace comm;
    using enum ProcessState;

    if (hci_event_packet_get_type(packet) != GATT_EVENT_QUERY_COMPLETE) {
        return;
    }

    hci_con_handle_t handle = HCI_CON_HANDLE_INVALID;
    uint8_t att_status = ATT_ERROR_SUCCESS;
    HearingAid *ha = nullptr;

    handle = gatt_event_query_complete_get_handle(packet);
    att_status = gatt_event_query_complete_get_att_status(packet);
    ha = get_by_con_handle(handle);
    if (!ha) {
        return;
    }

    switch (ha->process_state) {
        case EnASPNotificationInProgress:
            if (att_status != ATT_ERROR_SUCCESS) {
                //LOG_ERROR("%s: Failed to enable ASP notifications with status: %s", ha->get_side_str(), att_err_str(att_status));
                // Try again later
                ha->process_state = EnASPNotification;
                ++ha->error_count;
                ha->process_delay_ticks = ha_process_delay_ticks;
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPNotEnable, StatusType::ATTStatus, att_status));
            } else {
                //LOG_INFO("%s: ASP notifications enabled", ha->get_side_str());
                ha->process_state = EnBattNotification;
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::ASPNotEnable));
            }
            break;
        case EnBattNotificationInProgress:
            if (att_status != ATT_ERROR_SUCCESS) {
                // Not fatal - just continue
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::MFIBatteryNotEnable, StatusType::ATTStatus, att_status));
            } else {
                add_event_to_buffer(ha->conn_id, EventPacket(EventType::MFIBatteryNotEnable));
            }
            ha->process_state = Finalize;
            break;
        default:
            break;
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
        if (!ha) {
            return;
        }
        if (val_handle == ha->services.asha.asp.value_handle) {
            // Stop status is optional. ASP has no command ID, so a delayed
            // Stop status during a new Start remains ambiguous, as on Android.
            if (ha->process_state != ProcessState::Audio
                || ha->acp_operation != ACPOperation::Start) { return; }
            if (gatt_event_notification_get_value_length(packet) != 1) {
                short_log(ha->conn_id, "%s", "Invalid ASP length");
                return;
            }
            if (ha->acp_timed_out(btstack_run_loop_get_time_ms())) { return; }
            asp_status = (int8_t)gatt_event_notification_get_value(packet)[0];
            if (asp_status == ASPStatus::ok) {
                ha->acp_start_acked = true;
                ha->complete_acp();
            } else {
                EventPacket err_ev_pkt(EventType::ASPError);
                err_ev_pkt.data.asp_not = asp_status;
                add_event_to_buffer(ha->conn_id, err_ev_pkt);
                ha->disconnect();
            }
        } else if (val_handle == ha->services.mfi.battery.value_handle) {
            ha->battery_level = gatt_event_notification_get_value(packet)[0];
            EventPacket ev_pkt(EventType::MfiBatteryRead);
            ev_pkt.data.battery_level = ha->battery_level;
            add_event_to_buffer(ha->conn_id, ev_pkt);
        }
    }
}

bool HearingAid::process_audio()
{
    using namespace comm;
#ifdef PICO_ASHA_ENC_STATS
    bool send_enc_times = false;
#endif

    uint32_t w_index = asha_audio_get_write_index();
    auto const usb_volume = asha_audio_get_curr_usb_vol();

    // Dividing the USB volume by usb_vol_res gives a volume that
    // matches the ASHA volume. Volume is clamped to the min/max volume
    // set in runtime settings
    auto const settings = runtime_settings.get_usb_settings();
    auto min_vol = settings.min_vol;
    auto max_vol = settings.max_vol;
    int8_t vol_l = (usb_volume.left == usb_vol_mute) ? volume_mute
                          : (int8_t)(std::clamp(usb_volume.left, min_vol, max_vol) / usb_vol_res);
    int8_t vol_r = (usb_volume.right == usb_vol_mute) ? volume_mute
                          : (int8_t)(std::clamp(usb_volume.right, min_vol, max_vol) / usb_vol_res);
    
    bool const audio_requested = audio_session.process_streaming_state();

    bool enable_process_delay = false;
    uint32_t const now_ms = btstack_run_loop_get_time_ms();

    for (auto ha : hearing_aids) {
        if (ha->process_state == ProcessState::CloseL2CAP
            || (ha->l2cap_recovery_pending && ha->process_state != ProcessState::Audio
                && ha->process_state != ProcessState::Disconnect)) {
            if (++ha->l2cap_recovery_ticks >= l2cap_recovery_timeout_ticks) {
                short_log(ha->conn_id, "%s", "CoC recovery t/o, reconnecting");
                ha->disconnect();
            }
            continue;
        }
        if (ha->process_state != ProcessState::Audio) { continue; }
        if (ha->acp_operation != ACPOperation::None) {
            ha->acp_timed_out(now_ms);
            continue;
        }
        ha->credits = l2cap_cbm_available_credits(ha->cid);
        // Sample before scheduling audio, even while BTstack owns an SDU.
        ha->sample_audio_credits();
        if (ha->audio_tx_pending) {
            if (++ha->audio_tx_pending_ticks >= audio_tx_stuck_timeout_ticks) {
                short_log(ha->conn_id, "L2 Tx t/o:%lu Cr:%u",
                          (unsigned long)ha->audio_tx_pending_ticks,
                          (unsigned)ha->credits);
                ha->close_l2cap_for_recovery();
            }
            continue;
        }
        switch (ha->audio_state) {
            case AudioState::Ready:
                // After a zero-credit stop, wait up to the cooldown window for
                // credits to fully replenish before restarting; starting at low
                // credit counts immediately re-drains and produces audible
                // cycling. Falls through on timeout so we don't deadlock on aids
                // that never grant back to the ceiling.
                if (ha->zero_credits_cooldown > 0) {
                    --ha->zero_credits_cooldown;
                    if (ha->credits < audio_restart_credits) {
                        break;
                    }
                }
                if (!audio_session.restarting
                    && audio_requested
                    && ha->credits >= audio_start_credits) {
                        // Sync curr_vol from the host volume so the start payload
                        // reflects what the host is currently asking for. Without
                        // this, curr_vol's class default of -128 (the ASHA mute
                        // sentinel) gets baked into the first start after boot and
                        // some aids latch onto it, ignoring later volume writes.
                        ha->curr_vol = ha->rop.side() == Side::Left ? vol_l : vol_r;
                        ha->send_acp_start();
                } else if (audio_requested && ha->credits < audio_start_credits) {
                    // Aid wedged with credits stuck below the start gate. Only a
                    // fresh L2CAP CoC channel resets the credit window, so force
                    // a reconnect once the stuck timeout elapses.
                    if (++ha->ready_stuck_ticks >= ready_stuck_timeout_ticks) {
                        ha->ready_stuck_ticks = 0;
                        short_log(ha->conn_id, "%s", "Ready stuck, recreating L2CAP");
                        ha->close_l2cap_for_recovery();
                    }
                } else {
                    ha->ready_stuck_ticks = 0;
                }
                break;
            case AudioState::Streaming: {
                // Healthy streaming produces and hands an ASHA SDU to the
                // controller about every 20 ms. If no completed send occurs
                // for a sustained interval, credits alone cannot reveal the
                // failure: there may simply be no fresh encoded frame to
                // schedule. Stop cleanly so the hearing aid does not time
                // out and disconnect while it is left in streaming state.
                if (++ha->audio_sdu_gap_ticks >= audio_sdu_gap_timeout_ticks) {
                    short_log(ha->conn_id, "Tx gap: %lums Cr:%u",
                              (unsigned long)ha->audio_sdu_gap_ticks,
                              (unsigned)ha->credits);
                    ha->audio_sdu_gap_ticks = 0U;
                    audio_session.request_stream_restart();
                    break;
                }
                // Retry rejected writes using the latest host volume.
                int8_t v = ha->rop.side() == Side::Left ? vol_l : vol_r;
                if (ha->curr_vol != v) {
                    // short_log(ha->conn_id, "USB L:%d R:%d", (int)usb_volume.left, (int)usb_volume.right);
                    ha->send_volume(v);
                }
                if (w_index == 0) {
                    break;
                }
                if (ha->first_audio_send) {
                    if (!asha_audio_get_reset_frame(ha->audio_reset_type,
                                                    ha->audio_reset_request_id,
                                                    ha->curr_read_index)) {
                        // Wait for the producer's first complete post-reset SDU.
                        break;
                    }

                    // Ensure the volume is sent over CDC in case the control
                    // app does not have the current value
                    auto vol_ev = EventPacket(EventType::AudioVolume);
                    vol_ev.data.volume = ha->curr_vol;
                    add_event_to_buffer(ha->conn_id, vol_ev);

                    // add_event_to_buffer(ha->conn_id, )
                    ha->first_audio_send = false;
                }
                if (ha->curr_read_index < w_index) {
                    // A stale predictor cannot be repaired by skipping frames.
                    if (w_index - ha->curr_read_index >= audio_backlog_frame_limit) {
                        short_log(ha->conn_id, "%s", "Stream fell behind: restart");
                        ha->restart_stream_for_recovery();
                        break;
                    }
                    if (ha->credits == 0) {
                        if (ha->audio_tx_pending_ticks == 0) {
                            short_log(ha->conn_id, "%s", "Zero credits");
                        }
                        if (++ha->audio_tx_pending_ticks >= audio_tx_stuck_timeout_ticks) {
                            short_log(ha->conn_id, "%s", "Credit stall: restart");
                            ha->restart_stream_for_recovery();
                        }
                        break;
                    }
                    if (ha->audio_tx_pending_ticks != 0) {
                        short_log(ha->conn_id, "Credit wait: %lu ticks",
                                  (unsigned long)ha->audio_tx_pending_ticks);
                    }

                    // For LE CoC this checks the open channel and its free SDU
                    // slot; BTstack handles credits/controller capacity after submission.
                    if (!l2cap_can_send_packet_now(ha->cid)) {
                        short_log(ha->conn_id, "%s", "L2CAP Tx slot unavailable");
                        ha->close_l2cap_for_recovery();
                        break;
                    }
                    AudioSide audio_side = ha->rop.side() == Side::Left ? AudioSide::Left
                                                                      : AudioSide::Right;
                    auto encoded_audio = asha_audio_get_encoded_at_index(audio_side, ha->curr_read_index);
                    memcpy(ha->audio_tx_buffer.data(), encoded_audio.data(), encoded_audio.size());
                    // PACKET_SENT can arrive inside l2cap_send().
                    ha->audio_tx_pending = true;
                    ha->audio_tx_pending_ticks = 0;

#ifdef PICO_ASHA_L2CAP_TX_TRACE
                    short_log(ha->conn_id, "L2CAP Tx send S:%u Cr:%u",
                              (unsigned)ha->audio_tx_buffer[0], (unsigned)ha->credits);
#endif
                    uint8_t const tx_status = l2cap_send(ha->cid, ha->audio_tx_buffer.data(), sdu_size_bytes);
                    if (tx_status != ERROR_CODE_SUCCESS) {
                        // Rejection does not transfer ownership or consume a frame.
                        ha->audio_tx_pending = false;
                        short_log(ha->conn_id, "L2CAP Tx err: 0x%02X Cr:%u",
                                  (unsigned)tx_status, (unsigned)ha->credits);
                        ha->close_l2cap_for_recovery();
                        break;
                    }
                    if (ha->process_state == ProcessState::Audio && ha->is_streaming()) {
                        ++ha->curr_read_index;
                        ha->credit_average.active = true;
                    }
                    enable_process_delay = true;
#ifdef PICO_ASHA_ENC_STATS
                    send_enc_times = true;
#endif
                }
                break;
            }
            default:
                break;
        }
    }
#ifdef PICO_ASHA_ENC_STATS
    if (send_enc_times) {
        auto enc_times = asha_audio_get_encoding_time_at_index(w_index == 0 ? 0 : w_index - 1);
        EventPacket pkt1(EventType::G722EncTimings);
        EventPacket pkt2(EventType::G722EncTimings);
        static_assert(2u * sizeof(pkt1.data.encode_timings) == pcm_blocks_per_sdu * sizeof(int16_t));
        memcpy(pkt1.data.encode_timings, enc_times.data(), sizeof(pkt1.data.encode_timings));
        memcpy(pkt2.data.encode_timings, enc_times.data() + 10, sizeof(pkt2.data.encode_timings));
        add_event_to_buffer(unset_conn_id, pkt1);
        add_event_to_buffer(unset_conn_id, pkt2);
    }
#endif
    return enable_process_delay;
}

/* Private methods */

bool HearingAid::AudioSession::process_streaming_state()
{
    bool const want_audio = enabled && asha_audio_get_pcm_streaming_enabled();
    // Keep a brief host pause or disable latched until all Stops finish.
    if (!want_audio) { restarting = true; }
    bool busy = false;
    for (auto ha : hearing_aids) {
        if (ha->process_state != ProcessState::Audio) { continue; }
        // Wait for PACKET_SENT before Stop; the pending SDU still owns its buffer.
        if ((restarting || ha->credit_average.restart_pending)
            && ha->is_streaming() && !ha->audio_tx_pending) {
            if (restarting) {
                comm::short_log(ha->conn_id, "%s", want_audio ? "Restarting shared stream" : "Stopping shared stream");
            }
            ha->send_acp_stop();
        }
        busy |= ha->is_streaming() || ha->acp_operation != ACPOperation::None;
    }
    // A Start already in flight must finish and Stop before the restart.
    if (!busy) { restarting = false; }
    bool const first_streaming = hearing_aids[0]->is_streaming();
    bool const second_streaming = hearing_aids[1]->is_streaming();
    asha_audio_set_encode_mono(!(first_streaming && second_streaming));
    if ((!want_audio || !(first_streaming || second_streaming))
        && asha_audio_get_encoding_enabled()) {
        asha_audio_set_encoding_enabled(false);
        (void)asha_audio_request_reset(AudioReset::Session);
    }
    return want_audio;
}

void HearingAid::AudioSession::request_stream_restart()
{
    restarting = true;
    process_streaming_state();
}

void HearingAid::AudioSession::request_stream_restart(HearingAid& ha)
{
    ha.credit_average.restart_pending = true;
    process_streaming_state();
}

void HearingAid::AudioSession::on_stream_started(HearingAid& ha)
{
    bool const other_streaming = ha.other && ha.other->is_streaming();
    asha_audio_set_encode_mono(!other_streaming);
    ha.audio_reset_type = !other_streaming ? AudioReset::Session :
                         ha.rop.side() == Side::Left ? AudioReset::Left : AudioReset::Right;
    ha.audio_reset_request_id = asha_audio_request_reset(ha.audio_reset_type);
    asha_audio_set_encoding_enabled(true);
    if (other_streaming) {
        ha.other->send_acp_status(ACPStatus::other_connected);
    }
}

void HearingAid::AudioSession::on_stream_stopped(HearingAid& ha)
{
    if (ha.other && ha.other->is_streaming()) {
        ha.other->send_acp_status(ACPStatus::other_disconnected);
    }
    process_streaming_state();
}

HearingAid* HearingAid::get_by_con_handle(hci_con_handle_t handle)
{
    if (handle == HCI_CON_HANDLE_INVALID) { return nullptr; }
    for (auto ha : hearing_aids) {
        if (ha->is_connected() && ha->conn_handle == handle) {
            return ha;
        }
    }
    return nullptr;
}

HearingAid* HearingAid::get_by_cid(uint16_t cid)
{
    if (cid == 0) { return nullptr; }
    for (auto ha : hearing_aids) {
        if (ha->is_connected() && ha->cid == cid) {
            return ha;
        }
    }
    return nullptr;
}

HearingAid* HearingAid::get_by_conn_id(uint16_t conn_id)
{
    for (auto ha : hearing_aids) {
        if (ha->conn_id == conn_id) {
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
        if (ha->is_connected() && ha->rop) {
            if (ha->rop.mode() == Mode::Mono) {
                have_left = true;
                have_right = true;
                break;
            } else if (ha->rop.side() == Side::Left) {
                have_left = true;
            } else if (ha->rop.side() == Side::Right) {
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
    return audio_state == AudioState::Streaming;
}

void HearingAid::send_acp_start()
{
    using namespace comm;

    if (acp_operation != ACPOperation::None || acp_write_pending) { return; }
    acp_operation = ACPOperation::Start;
    acp_write_pending = true;
    acp_start_acked = false;
    acp_phase_started_ms = btstack_run_loop_get_time_ms();
    audio_state = AudioState::Start;
    clear_audio_send_state();
    ready_stuck_ticks = 0;
    zero_credits_cooldown = 0;
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
        acp_write_pending = false;
        disconnect();
    }
}

void HearingAid::send_acp_stop()
{
    using namespace comm;

    if (acp_operation != ACPOperation::None || acp_write_pending) { return; }
    acp_operation = ACPOperation::Stop;
    acp_write_pending = true;
    acp_phase_started_ms = btstack_run_loop_get_time_ms();
    audio_state = AudioState::Stop;
    credit_average = {};
    audio_sdu_gap_ticks = 0U;
    acp_cmd_packet[0] = ACPOpCode::stop;
    uint8_t res = gatt_client_write_value_of_characteristic(&HearingAid::handle_acp_write,
                                                            conn_handle,
                                                            services.asha.acp.value_handle,
                                                            1u,
                                                            acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        //LOG_ERROR("%s: ACP Write: Stop error %s", get_side_str(), bt_err_str(res));
        add_event_to_buffer(conn_id, EventPacket(EventType::ACPStop, StatusType::BtstackStatus, res));
        acp_write_pending = false;
        disconnect();
    }
}

bool HearingAid::acp_timed_out(uint32_t now_ms)
{
    if (now_ms - acp_phase_started_ms < acp_timeout_ms) {
        return false;
    }
    comm::short_log(conn_id, "%s", "ACP timeout, reconnecting");
    disconnect();
    return true;
}

void HearingAid::complete_acp()
{
    if (acp_write_pending || acp_operation == ACPOperation::None) { return; }
    if (acp_operation == ACPOperation::Start && !acp_start_acked) { return; }
    auto const operation = acp_operation;
    acp_operation = ACPOperation::None;
    if (operation == ACPOperation::Start) {
        comm::add_event_to_buffer(conn_id, comm::EventPacket(comm::EventType::ASPStart));
        audio_state = AudioState::Streaming;
        audio_session.on_stream_started(*this);
        first_audio_send = true;
    } else {
        // Report Stop to the control app at GATT completion; ASP is optional.
        comm::add_event_to_buffer(conn_id, comm::EventPacket(comm::EventType::ASPStop));
        audio_state = AudioState::Ready;
        audio_session.on_stream_stopped(*this);
    }
}

void HearingAid::send_acp_status(uint8_t status)
{
    using namespace comm;

    // Write-without-response copies its bytes before returning. Keep this
    // separate from the buffer retained by a pending Start/Stop GATT write.
    std::array<uint8_t, 2> status_packet = {ACPOpCode::status, status};
    auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                          services.asha.acp.value_handle,
                                                                          2u,
                                                                          status_packet.data());
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
        curr_vol = volume;
        //LOG_INFO("%s: Volume changed to %d", get_side_str(), (int)volume);
        EventPacket ev_pkt(EventType::AudioVolume);
        ev_pkt.data.volume = volume;
        add_event_to_buffer(conn_id, ev_pkt);
    }
}

void HearingAid::sample_audio_credits()
{
    if (!is_streaming() || !credit_average.active || credit_average.restart_pending
        || audio_session.restarting) { return; }
    credit_average.window_sum += credits;
    if (++credit_average.samples != audio_credit_window_ticks) { return; }

    if (!credit_average.baseline_ready) {
        credit_average.baseline_sum = credit_average.window_sum;
        credit_average.baseline_ready = true;
    }
    comm::EventPacket event(comm::EventType::AudioCreditStats);
    event.data.credit_stats = {credit_average.baseline_sum, credit_average.window_sum, credit_average.samples};
    comm::add_event_to_buffer(conn_id, event);

    int32_t const difference = (int32_t)credit_average.window_sum - (int32_t)credit_average.baseline_sum;
    uint32_t const magnitude = difference < 0 ? -difference : difference;
    if ((uint64_t)magnitude * 10 >= (uint64_t)audio_credit_window_ticks * audio_credit_drift_threshold_tenths) {
        comm::short_log(conn_id, "%s", "Credit drift: restart");
        audio_session.request_stream_restart();
    }
    credit_average.window_sum = 0;
    credit_average.samples = 0;
}

void HearingAid::clear_audio_send_state()
{
    // Called when ownership is released, or before starting an idle stream.
    audio_tx_pending = false;
    audio_tx_pending_ticks = 0U;
    audio_sdu_gap_ticks = 0U;
}

void HearingAid::restart_stream_for_recovery()
{
    if (!is_streaming() || audio_tx_pending || acp_operation != ACPOperation::None) {
        close_l2cap_for_recovery();
        return;
    }
    if (credits == 0) { zero_credits_cooldown = audio_credit_cooldown_ticks; }
    clear_audio_send_state();
    send_acp_stop();
}

void HearingAid::close_l2cap_for_recovery()
{
    using namespace comm;

    if (process_state == ProcessState::CloseL2CAP || process_state == ProcessState::Disconnect) { return; }
    if (l2cap_recovery_pending || acp_operation != ACPOperation::None || cid == 0) {
        short_log(conn_id, "%s", "CoC recovery failed: ACL reset");
        disconnect();
        return;
    }
    // Stop scheduling audio immediately. The old CoC may still hold a pointer
    // to audio_tx_buffer until its close completes; retain its ownership flag.
    audio_state = AudioState::AudioUnset;
    credit_average = {};
    l2cap_recovery_ticks = 0U;
    l2cap_recovery_pending = true;
    process_state = ProcessState::CloseL2CAP;

    uint8_t const result = l2cap_disconnect(cid);
    if (result == ERROR_CODE_SUCCESS) {
        return;
    }

    // The channel no longer exists or cannot be closed locally. In that case,
    // the ACL reconnect is the only recovery path.
    short_log(conn_id, "L2CAP close err: 0x%02X", (unsigned)result);
    disconnect();
}

void HearingAid::disconnect()
{
    if (process_state == ProcessState::Disconnect) { return; }
    //LOG_INFO("%s: Disconnect requested", get_side_str());
    audio_state = AudioState::AudioUnset;
    credit_average = {};
    // Keep ACP storage and ownership intact until completion or ACL teardown.
    process_state = ProcessState::Disconnect;
    uint8_t const result = gap_disconnect(conn_handle);
    if (result == ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER) {
        // The stack already has no ACL for this slot, so no completion event
        // will follow. Clear stale application state and resume scanning.
        on_disconnected(conn_handle, ERROR_CODE_SUCCESS, result);
    }
}

void HearingAid::reset()
{
    conn_id = comm::unset_conn_id;
    conn_handle = HCI_CON_HANDLE_INVALID;
    cid = 0U;
    connected = false;
    process_state = ProcessState::ProcessUnset;
    audio_state = AudioState::AudioUnset;
    acp_operation = ACPOperation::None;
    acp_write_pending = false;
    acp_start_acked = false;
    acp_phase_started_ms = 0;
    other = nullptr;
    psm = 0;
    battery_level = 0;
    curr_vol = -128;
    credits = 0;
    credit_average = {};
    zero_credits_cooldown = 0;
    ready_stuck_ticks = 0;
    paired_and_bonded = false;
    process_delay_ticks = 0;
    error_count = 0;
    service_index = 0;
    chars_index = cached_chars_index;
    curr_read_index = 0U;
    audio_reset_type = AudioReset::Session;
    audio_reset_request_id = 0U;
    first_audio_send = false;
    clear_audio_send_state();
    l2cap_recovery_ticks = 0U;
    l2cap_recovery_pending = false;
    l2cap_open_pending = false;
    l2cap_open_started_ms = 0;

    if (!cached) {
        memset(addr, 0U, sizeof(bd_addr_t));
        device_name.clear();
        manufacturer.clear();
        model.clear();
        fw_vers.clear();
        sw_vers.clear();

        rop = {};
        side_str = "Unknown";
        services = {};
        chars_index = 0;
    }
}

const char* HearingAid::get_side_str()
{
    return (rop) ? bd_addr_to_str(addr) : side_str;
}

static void delete_paired_devices()
{
    using namespace comm;
    //LOG_INFO("Removing paired devices");
    EventPacket ev_pkt(EventType::DeletePair);

    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            //LOG_INFO("Removing: %s", bd_addr_to_str(addr));
            le_device_db_remove(i);
            bd_addr_copy(ev_pkt.data.conn_info.addr, addr);
            add_event_to_buffer(unset_conn_id, ev_pkt);
        }
    }
    gap_whitelist_clear();
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
