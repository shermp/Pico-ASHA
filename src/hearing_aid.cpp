#include <bitset>
#include <cassert>
#include <cstring>

#include "asha_logging.h"
#include "hearing_aid.hpp"
#include "asha_uuid.hpp"

#include "util.hpp"

namespace asha 
{

namespace ASPStatus
{
    constexpr int8_t unkown_command = -1;
    constexpr int8_t illegal_params = -2;
    constexpr int8_t ok = 0;
}

namespace ACPStatus
{
    constexpr uint8_t other_disconnected = 0;
    constexpr uint8_t other_connected = 1;
    constexpr uint8_t conn_param_updated = 2;
}

/* Get value from (sub) array of bytes */
template<typename T>
static T get_val(const uint8_t *start)
{
    T val;
    std::memcpy(&val, start, sizeof(val));
    return val;
}

static const char* state_to_str(HA::State state)
{
    using enum HA::State;
    switch(state) {
        case Invalid: return "";
        case Cached: return "";
        case ServicesDiscovered: return "";
        case DiscoverGAPChars: return "Discover GAP Chars";
        case GAPCharsDiscovered: return "";
        case DiscoverDISChars: return "Discover DIS Chars";
        case DISCharsDiscovered: return "";
        case DiscoverASHAChars: return "Discover ASHA Chars";
        case ASHACharsDiscovered: return "";
        case ReadDeviceName: return "Read Device Name";
        case DeviceNameRead: return "";
        case ReadManufacturerName: return "Read Manufacture Name";
        case ManufacturerNameRead: return "";
        case ReadModelNum: return "Read Model Num";
        case ModelNumRead: return "";
        case ReadFWVers: return "Read FW Version";
        case FWVersRead: return "";
        case ReadROP: return "Read ROP";
        case ROPRead: return "";
        case ReadPSM: return "Read PSM";
        case PSMRead: return "";
        case SubscribeASPNotification: return "Subscribe to ASP";
        case ASPNotificationSubscribed: return "";
        case GAPDisconnect: return "";
        case L2Connecting: return "";
        case L2Connected: return "";
        case ACPStart: return "ACP Start";
        case ASPStartOk: return "";
        case ASPStopOK: return "";
        case AudioPacketReady: return "";
        case AudioPacketSending: return "";
        case AudioPacketSent: return "";
        case ACPStop: return "ACP Stop";
        default: return "";
    }
}

HA::HA()
{}

HA::operator bool() const
{
    return state != State::Invalid;
}

HA::Side HA::side() const
{ 
    return rop.side;
}

HA::Mode HA::mode() const
{
    return rop.mode;
}

void HA::dec_state_enum()
{
    state = State(static_cast<int>(state) - 1);
}

void HA::inc_state_enum()
{
    state = State(static_cast<int>(state) + 1);
}

void HA::discover_chars()
{
    using enum State;
    gatt_client_service_t* service = nullptr;
    const char *serv_name = "";
    if (state == DiscoverGAPChars) {
        serv_name = "GAP";
        service = &gap_service.service;
    } else if (state == DiscoverDISChars) {
        serv_name = "DIS";
        service = &dis_service.service;
    } else if (state == DiscoverASHAChars) {
        serv_name = "ASHA";
        service = &asha_service.service;
    } else {
        LOG_ERROR("discover_chars: incorrect state");
        return;
    }
    LOG_INFO("%s: Discovering characteristics for %s service", side_str, serv_name);
    uint8_t res = gatt_client_discover_characteristics_for_service(
        gatt_packet_handler,
        conn_handle,
        service
    );
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Error registering char discovery for service %s 0x%02x", side_str, serv_name, static_cast<unsigned int>(res));
        state = GAPDisconnect;
        gap_disconnect(conn_handle);
        return;
    }
}
void HA::on_char_discovered(uint8_t* char_query_res_packet)
{
    gatt_client_characteristic_t characteristic;
    gatt_event_characteristic_query_result_get_characteristic(char_query_res_packet, &characteristic);
    if (characteristic.uuid16 == GapUUID::deviceName16) {
        LOG_INFO("%s: Got Device Name Characteristic", side_str);
        gap_service.device_name = characteristic;
    } else if (characteristic.uuid16 == DisUUID::ManNameString) {
        LOG_INFO("%s: Got Manufacture Name Characteristic", side_str);
        dis_service.manufacture_name = characteristic;
    } else if (characteristic.uuid16 == DisUUID::ModelNumString) {
        LOG_INFO("%s: Got Model Number Characteristic", side_str);
        dis_service.model_num = characteristic;
    } else if (characteristic.uuid16 == DisUUID::FWRevString) {
        LOG_INFO("%s: Got FW Version Characteristic", side_str);
        dis_service.fw_vers = characteristic;
    } else if (uuid_eq(characteristic.uuid128, AshaUUID::readOnlyProps)) {
        LOG_INFO("%s: Got ROP Characteristic", side_str);
        asha_service.rop = characteristic;
    } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioControlPoint)) {
        LOG_INFO("%s: Got ACP Characteristic", side_str);
        asha_service.acp = characteristic;
    } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioStatus)) {
        LOG_INFO("%s: Got AUS Characteristic", side_str);
        asha_service.asp = characteristic;
    } else if (uuid_eq(characteristic.uuid128, AshaUUID::volume)) {
        LOG_INFO("%s: Got VOL Characteristic", side_str);
        asha_service.vol = characteristic;
    } else if (uuid_eq(characteristic.uuid128, AshaUUID::psm)) {
        LOG_INFO("%s: Got PSM Characteristic", side_str);
        asha_service.psm = characteristic;
    }
}

// void HA::on_char_discovery_complete([[maybe_unused]] uint8_t status)
// {}

void HA::read_char()
{
    using enum State;
    auto gatt_query = [&](gatt_client_characteristic_t* c) {
        if (is_valid(c)) {
            uint8_t res = gatt_client_read_value_of_characteristic(
                gatt_packet_handler, conn_handle, c
            );
            if (res != ERROR_CODE_SUCCESS) {
                LOG_ERROR("%s: %s error", side_str, state_to_str(state));
                dec_state_enum();
            }
        } else {
            LOG_INFO("%s: Skipping %s", side_str, state_to_str(state));
            inc_state_enum();
        }
    };

    // Note: order is same as enum declaration
    switch(state) {
    case ReadROP:
        gatt_query(&asha_service.rop);
        break;
    case ReadPSM:
        gatt_query(&asha_service.psm);
        break;
    case ReadDeviceName:
        gatt_query(&gap_service.device_name);
        break;
    case ReadManufacturerName:
        gatt_query(&dis_service.manufacture_name);
        break;
    case ReadModelNum:
        gatt_query(&dis_service.model_num);
        break;
    case ReadFWVers:
        gatt_query(&dis_service.fw_vers);
        break;
    default:
        break;
    }
}
void HA::on_read_char_value(uint8_t* char_val_query_res_packet)
{
    using enum State;
    size_t len = static_cast<size_t>(gatt_event_characteristic_value_query_result_get_value_length(char_val_query_res_packet));
    const uint8_t *val = gatt_event_characteristic_value_query_result_get_value(char_val_query_res_packet);
    switch(state) {
    case ReadROP:
        rop.read(val);
        side_str = rop.side == Side::Left ? "Left" : "Right";
        break;
    case ReadPSM:
        psm = val[0];
        LOG_INFO("%s: PSM: %u", side_str, static_cast<unsigned int>(psm));
        break;
    case ReadDeviceName:
        name.clear();
        name.append((const char*)val, len);
        break;
    case ReadManufacturerName:
        manufacturer.clear();
        manufacturer.append((const char*)val, len);
        break;
    case ReadModelNum:
        model.clear();
        model.append((const char*)val, len);
        break;
    case ReadFWVers:
        fw_vers.clear();
        fw_vers.append((const char*)val, len);
        break;
    default:
        break;
    }
}
// void HA::on_read_char_complete([[maybe_unused]] uint8_t status)
// {}

void HA::subscribe_to_asp_notification()
{
    auto res = gatt_client_write_client_characteristic_configuration(
        gatt_packet_handler,
        conn_handle,
        &asha_service.asp,
        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
    );
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Error writing ASP notification configuration: 0x%02x", side_str, (unsigned int)res);
        state = State::L2Connected;
    }
}

void HA::create_l2cap_channel()
{
    uint8_t res = ERROR_CODE_SUCCESS;
        LOG_INFO("%s: Connecting to L2CAP", side_str);
        res = l2cap_cbm_create_channel(l2cap_packet_handler, 
                                        conn_handle, 
                                        psm, 
                                        recv_buff.data(), 
                                        recv_buff.size(),
                                        L2CAP_LE_AUTOMATIC_CREDITS,
                                        LEVEL_2,
                                        &cid);
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: Failure creating l2cap channel with error code: 0x%02x", side_str, (unsigned int)res);
            state = State::GAPDisconnect;
            gap_disconnect(conn_handle);
        }
}

void HA::on_l2cap_channel_created(uint8_t status) 
{
    if (status != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: L2CAP CoC failed with status code: 0x%02x", side_str, status);
        state = State::FWVersRead;
        gatt_wait_count = 1000;
        return;
    }
    LOG_INFO("%s: L2CAP CoC channel created", side_str);
    state = State::L2Connected;
}

void HA::write_acp_start()
{
    state = State::ACPStart;
    change_vol = false;
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Start); // Opcode
    acp_cmd_packet[1] = 1u; // G.722 codec at 16KHz
    acp_cmd_packet[2] = 0u; // Unkown audio type
    acp_cmd_packet[3] = (uint8_t)volume; // Volume
    acp_cmd_packet[4] = (other_ha && other_ha->is_streaming_audio()) ? 1 : 0; // Otherstate

    uint8_t res = gatt_client_write_value_of_characteristic(gatt_packet_handler, 
                                                            conn_handle, 
                                                            asha_service.acp.value_handle,
                                                            (uint16_t)acp_cmd_packet.size(),
                                                            acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: ACP Write: Start error 0x%02x", side_str, (unsigned int)res);
        state = State::ASPNotificationSubscribed;
    }
}

void HA::write_acp_stop()
{
    state = State::ACPStop;
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Stop);
    uint8_t res = gatt_client_write_value_of_characteristic(gatt_packet_handler,
                                                            conn_handle,
                                                            asha_service.acp.value_handle,
                                                            1u,
                                                            acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: ACP Write: Stop error 0x%02x", side_str, (unsigned int)res);
        state = State::GAPDisconnect;
        gap_disconnect(conn_handle);
    }
}

void HA::write_acp_status(uint8_t status)
{
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Status);
    acp_cmd_packet[1] = status;
    auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                          asha_service.acp.value_handle,
                                                                          2u,
                                                                          acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Updating status via ACP failed with 0x%02x", side_str, (unsigned int)res);
    }
    return;
}

void HA::on_gatt_event_query_complete(uint8_t* query_complete_packet)
{
    using enum State;
    unsigned int att_status = gatt_event_query_complete_get_att_status(query_complete_packet);
    if (att_status != ATT_ERROR_SUCCESS) {
        LOG_ERROR("%s: %s failed with error code: 0x%02x", side_str, state_to_str(state), att_status);
    }
    auto set_state = [&](State if_success) {
        if (att_status != ATT_ERROR_SUCCESS) {
            dec_state_enum();
        } else {
            LOG_INFO("%s: %s success", side_str, state_to_str(state));
            state = if_success;
        }
    };
    switch (state) {
    case DiscoverASHAChars:
        set_state(ASHACharsDiscovered);
        break;
    case ReadROP:
        set_state(ROPRead);
        if (state == ROPRead && other_ha) {
            if (rop.id != other_ha->rop.id) {
                LOG_ERROR("%s: HiSyncID does not match", side_str);
                state = GAPDisconnect;
                gap_disconnect(conn_handle);
            }
        }
        break;
    case ReadPSM:
        rop.print_values();
        LOG_INFO("%s: %u", side_str, static_cast<unsigned int>(psm));
        set_state(PSMRead);
        break;
    case DiscoverGAPChars:
        set_state(GAPCharsDiscovered);
        break;
    case DiscoverDISChars:
        set_state(DISCharsDiscovered);
        break;
    case ReadDeviceName:
        set_state(DeviceNameRead);
        break;
    case ReadManufacturerName:
        set_state(ManufacturerNameRead);
        break;
    case ReadModelNum:
        set_state(ModelNumRead);
        break;
    case ReadFWVers:
        LOG_INFO("%s: HA: %s %s - FW %s", side_str, manufacturer.c_str(), model.c_str(), fw_vers.c_str());
        set_state(FWVersRead);
        break;
    // If this case is hit, it means the connection attempt probably timed out
    case L2Connecting:
        set_state(L2Connected);
        break;
    case SubscribeASPNotification:
        set_state(ASPNotificationSubscribed);
        gatt_wait_count = 1000;
        break;
    case ACPStart:
        if (att_status != ATT_ERROR_SUCCESS) {
            state = L2Connected;
            return;
        }
        break;
    case ACPStop:
        if (att_status != ATT_ERROR_SUCCESS) {
            state = L2Connected;
        }
        break;
    default:
        break;
    }
}

void HA::on_asp_notification(int8_t asp_status)
{
    if (state != State::ACPStart && state != State::ACPStop) {
        LOG_ERROR("%s: Unexpected ASP notification", side_str);
    }
    switch (asp_status) {
    case ASPStatus::ok:
        if (state == State::ACPStart) {
            LOG_INFO("%s: ASP Start Ok.", side_str);
            if (other_ha && other_ha->is_streaming_audio()) {
                other_ha->write_acp_status(ACPStatus::other_connected);
                curr_read_index = other_ha->curr_read_index;
            }
            state = State::AudioPacketReady;
        } else if (state == State::ACPStop) {
            LOG_INFO("%s: ASP Stop Ok.", side_str);
            if (other_ha && other_ha->is_streaming_audio()) {
                other_ha->write_acp_status(ACPStatus::other_disconnected);
            }
            state = State::ASPNotificationSubscribed;
        }
        break;
    case ASPStatus::unkown_command:
        LOG_ERROR("%s: ASP: Unknown command", side_str);
        break;
    case ASPStatus::illegal_params:
        LOG_ERROR("%s: ASP: Illegal parameters", side_str);
        break;
    default:
        LOG_ERROR("%s: ASP: Unknown status: %d", side_str, asp_status);
        break;
    }
}

void HA::set_volume(AudioBuffer::Volume& vol)
{
    int8_t v = side() == Side::Left ? vol.l : vol.r;
    if (volume != v) {
        volume = v;
        change_vol = true;
        LOG_INFO("%s: Volume changed to %d", side_str, (int)volume);
    }    
}

void HA::write_volume()
{
    if (change_vol) {
        change_vol = false;
        uint8_t res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                                asha_service.vol.value_handle,
                                                                                sizeof(volume),
                                                                                (uint8_t*)&volume);
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: Volume: write failed with error: 0x%02x", side_str, (unsigned int)res);
        }
    }
}

void HA::set_write_index(uint32_t write_index)
{
    if (write_index > curr_write_index) {
        curr_write_index = write_index;
    }
}

void HA::send_audio_packet()
{
    switch (state) {
    case State::AudioPacketReady:
    {
        // LOG_INFO("%s: Available credits: %hu", side_str, avail_credits);
        if (curr_read_index >= curr_write_index) {
            break;
        }
        if ((curr_write_index - curr_read_index) >= 2) {
            curr_read_index = curr_write_index - 1;
        }
        AudioBuffer::G722Buff& packet = audio_buff.get_g_buff(curr_read_index);
        audio_packet = side() == Side::Left ? packet.l.data() : packet.r.data();
        ++curr_read_index;
        write_volume();
        state = State::AudioPacketSending;
    }
        [[fallthrough]];
    case State::AudioPacketSending:
        if (avail_credits == 0) {
            LOG_INFO("%s: Available credits: 0, restarting stream", side_str);
            write_acp_stop();
        } else if (l2cap_can_send_packet_now(cid)) {
            LOG_AUDIO("%s: Sending audio packet. Seq Num: %d", side_str, (int)audio_packet[0]);
            state = State::AudioPacketSent;
            zero_credit_count = 0;
            l2cap_send(cid, audio_packet, sdu_size_bytes);
        }
    default:
        break;
    }
}

void HA::on_audio_packet_sent()
{
    if (state == State::AudioPacketSent) {
        state = State::AudioPacketReady;
        LOG_AUDIO("%s: Sent audio packet. State: %d. Seq Num: %d", side_str, static_cast<int>(state), (int)audio_packet[0]);
    }
}

bool HA::is_streaming_audio()
{
    using enum State;
    return is_any_of(state, AudioPacketReady, AudioPacketSending, AudioPacketSent, ASPStartOk);
}

void HA::reset_uncached_vars()
{
    state = HA::State::Cached;
    conn_handle = HCI_CON_HANDLE_INVALID;
    cid = 0;
    other_ha = nullptr;
    audio_packet = nullptr;
    curr_read_index = 0;
    curr_write_index = 0;
    zero_credit_count = 0;
}

HA::ROP::ROP()
{}

void HA::ROP::read(const uint8_t* data)
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
}

void HA::ROP::print_values()
{
    LOG_INFO("ROP -"
        " Side: %s,"
        " Mode: %s,"
        " M. ID: %04hx,"
        " Delay: %hu,"
        " 16KHz: %s,"
        " 24KHz: %s",
        (side == HA::Side::Left ? "L" : "R"),
        (mode == HA::Mode::Binaural ? "B" : "M"),
        id.manufacturer_id,
        render_delay,
        codec_16khz ? "Y" : "N",
        codec_24khz ? "Y" : "N");
}

HAManager::HAManager()
{}

HA& HAManager::get_by_addr(bd_addr_t const addr)
{
    for (auto& ha : hearing_aids) {
        if (bd_addr_cmp(addr, ha.addr) == 0) {
            return ha;
        }
    }
    return invalid_ha;
}

HA& HAManager::get_by_conn_handle(hci_con_handle_t const handle)
{
    for (auto& ha : hearing_aids) {
        if (handle == ha.conn_handle) {
            return ha;
        }
    }
    return invalid_ha;
}

HA& HAManager::get_by_cid(uint16_t const cid)
{
    for (auto& ha : hearing_aids) {
        if (cid == ha.cid) {
            return ha;
        }
    }
    return invalid_ha;
}

bool HAManager::exists(HA const& ha)
{
    return get_by_addr(ha.addr) || get_by_conn_handle(ha.conn_handle);
}

bool HAManager::set_complete()
{
    return  (hearing_aids.size() == 1 && hearing_aids[0].mode() == HA::Mode::Mono) ||
            (hearing_aids.size() == 2 && 
                hearing_aids[0].rop.side != HA::Side::Unset && 
                hearing_aids[1].rop.side != HA::Side::Unset && 
                hearing_aids[0].rop.id == hearing_aids[1].rop.id);
}

HA& HAManager::add(HA const& new_ha)
{
    if (exists(new_ha)) {
        LOG_INFO("Add: HA with address '%s' already exists", bd_addr_to_str(new_ha.addr));
        return invalid_ha;
    }
    if (hearing_aids.size() >= max_num_ha) {
        LOG_INFO("Add: Already hold max number of hearing aids");
        return invalid_ha;
    }

    hearing_aids.emplace_back(new_ha);
    assert(hearing_aids.size() <= 2);
    if (hearing_aids.size() == 2) {
        hearing_aids[0].other_ha = &hearing_aids[1];
        hearing_aids[1].other_ha = &hearing_aids[0];
    }
    HA& added_ha = hearing_aids.back();
    // added_ha.side_str = (added_ha.side() == HA::Side::Left) ? "Left" : "Right";
    return added_ha;
}

void HAManager::remove_by_conn_handle(hci_con_handle_t handle)
{
    // Erase all even numbers
    for (auto it = hearing_aids.begin(); it != hearing_aids.end();)
    {
        if ((*it).conn_handle == handle) {
            it = hearing_aids.erase(it);
            if (hearing_aids.size() == 1) {
                hearing_aids[0].other_ha = nullptr;
            }
            return;
        } else {
            ++it;
        }
    }
    LOG_INFO("Remove: HA not found");
}

} // namespace asha