#include <bitset>
#include <cassert>
#include <cstring>

#include "asha_logging.h"
#include "hearing_aid.hpp"

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

void HA::create_l2cap_channel()
{
    if (state == State::GATTConnected) {
        state = State::L2ConnStart;
        LOG_INFO("%s: Connecting to L2CAP\n", side_str);
        auto res = l2cap_cbm_create_channel(l2cap_packet_handler, 
                                            conn_handle, 
                                            psm, 
                                            recv_buff.data(), 
                                            recv_buff.size(),
                                            L2CAP_LE_AUTOMATIC_CREDITS,
                                            LEVEL_2,
                                            &cid);
        if (res != ERROR_CODE_SUCCESS) {
            state = State::GATTConnected;
            LOG_ERROR("%s: Failure creating l2cap channel with error code: 0x%02x", side_str, (unsigned int)res);
        }
    }
}

void HA::on_l2cap_channel_created(uint8_t status) 
{
    if (state == State::L2ConnStart) {
        if (status != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: L2CAP CoC failed with status code: 0x%02x\n", side_str, status);
            // Try again
            state = State::GATTConnected;
            return;
        }
        LOG_INFO("%s: L2CAP CoC channel created\n", side_str);
        state = State::L2ConnCompleted;
    }
}

void HA::subscribe_to_asp_notification()
{
    if (state != State::L2ConnCompleted) return;
    state = State::SubscribeASPNotification;
    gatt_client_write_client_characteristic_configuration(
        gatt_packet_handler,
        conn_handle,
        &chars.asp,
        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
    );
}

void HA::write_acp_cmd(ACPOpCode opcode)
{
    uint8_t res = ERROR_CODE_SUCCESS;
    State last_state = state;
    switch (opcode) {
    case ACPOpCode::Start:
        if (state != State::ASPNotificationSubscribed) {
            LOG_ERROR("%s: Not in L2CAP connected state\n", side_str);
            return;
        }
        state = State::ACPStartWrite;
        change_vol = false;
        acp_cmd_packet[0] = static_cast<uint8_t>(opcode); // Opcode
        acp_cmd_packet[1] = 1u; // G.722 codec at 16KHz
        acp_cmd_packet[2] = 0u; // Unkown audio type
        acp_cmd_packet[3] = (uint8_t)volume; // Volume mute
        acp_cmd_packet[4] = (other_ha && other_ha->is_streaming_audio()) ? 1 : 0; // Otherstate
        res = gatt_client_write_value_of_characteristic(gatt_packet_handler, 
                                                        conn_handle, 
                                                        chars.acp.value_handle,
                                                        (uint16_t)acp_cmd_packet.size(),
                                                        acp_cmd_packet.data());
        if (res != ERROR_CODE_SUCCESS) {
             LOG_ERROR("%s: ACP Write: Start error 0x%02x\n", side_str, (unsigned int)res);
             state = last_state;
        }
        break;
    case ACPOpCode::Stop:
        if (!is_streaming_audio()) {
            LOG_ERROR("%s: Not streaming audio\n", side_str);
            return;
        }
        state = State::ACPStopWrite;
        acp_cmd_packet[0] = static_cast<uint8_t>(opcode);
        res = gatt_client_write_value_of_characteristic(gatt_packet_handler,
                                                        conn_handle,
                                                        chars.acp.value_handle,
                                                        1u,
                                                        acp_cmd_packet.data());
        if (res != ERROR_CODE_SUCCESS) {
             LOG_ERROR("%s: ACP Write: Stop error 0x%02x\n", side_str, (unsigned int)res);
             state = last_state;
        }
        break;
    default: 
        break;
    }
}

void HA::write_acp_status(uint8_t status)
{
    if (!is_streaming_audio()) {
        LOG_INFO("%s: not streaming. Not sending status update\n", side_str);
        return;
    }
    if (status != ACPStatus::conn_param_updated && 
        status != ACPStatus::other_connected && 
        status != ACPStatus::other_disconnected) {
            LOG_ERROR("%s: Unknown status command\n", side_str);
            return;
        }
    acp_cmd_packet[0] = static_cast<uint8_t>(ACPOpCode::Status);
    acp_cmd_packet[1] = status;
    auto res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                          chars.acp.value_handle,
                                                                          2u,
                                                                          acp_cmd_packet.data());
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Updating status via ACP failed with 0x%02x\n", side_str, (unsigned int)res);
    }
    return;
}

void HA::on_gatt_event_query_complete(uint8_t att_status)
{
    switch (state) {
    case State::SubscribeASPNotification:
        if (att_status != ATT_ERROR_SUCCESS) {
            LOG_ERROR("Enabling AudioStatusPoint notifications failed with error code: 0x%02x\n", att_status);
            state = State::L2ConnCompleted;
        }
        state = State::ASPNotificationSubscribed;
        LOG_INFO("%s: Subscribed to ASP Notification\n", side_str);
        break;
    case State::ACPStartWrite:
        if (att_status != ATT_ERROR_SUCCESS) {
            LOG_ERROR("%s: ACP Start: write failed with error: 0x%02x\n", side_str, (unsigned int)att_status);
            state = State::ASPNotificationSubscribed;
            return;
        }
        state = State::ACPStartWritten;
        break;
    case State::ACPStopWrite:
        if (att_status != ATT_ERROR_SUCCESS) {
            LOG_ERROR("%s: ACP Stop: write failed with error: 0x%02x\n", side_str, (unsigned int)att_status);
            state = State::ASPNotificationSubscribed;
        }
        state = State::ACPStopWritten;
        break;
    default:
        break;
    }
}

void HA::on_asp_notification(int8_t asp_status)
{
    if (state != State::ACPStartWritten && state != State::ACPStopWritten) {
        LOG_ERROR("%s: Unexpected ASP notification\n", side_str);
    }
    switch (asp_status) {
    case ASPStatus::ok:
        if (state == State::ACPStartWritten) {
            LOG_INFO("%s: ASP Start Ok.\n", side_str);
            state = State::ASPStartOk;
            if (other_ha && other_ha->is_streaming_audio()) {
                other_ha->write_acp_status(ACPStatus::other_connected);
            }
        } else if (state == State::ACPStopWritten) {
            LOG_INFO("%s: ASP Stop Ok.\n", side_str);
            state = State::ASPNotificationSubscribed;
            if (other_ha && other_ha->is_streaming_audio()) {
                other_ha->write_acp_status(ACPStatus::other_disconnected);
            }
        }
        break;
    case ASPStatus::unkown_command:
        LOG_ERROR("%s: ASP: Unknown command\n", side_str);
        break;
    case ASPStatus::illegal_params:
        LOG_ERROR("%s: ASP: Illegal parameters\n", side_str);
        break;
    default:
        LOG_ERROR("%s: ASP: Unknown status: %d\n", side_str, asp_status);
        break;
    }
}

void HA::set_volume(AudioBuffer::Volume& vol)
{
    int8_t v = side() == Side::Left ? vol.l : vol.r;
    if (volume != v) {
        volume = v;
        change_vol = true;
    }    
}

void HA::write_volume()
{
    if (change_vol && is_streaming_audio()) {
        uint8_t res = gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                                chars.vol.value_handle,
                                                                                sizeof(volume),
                                                                                (uint8_t*)&volume);
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("%s: Volume: write failed with error: 0x%02x\n", side_str, (unsigned int)res);
        }
    }
}

void HA::send_audio_packet(AudioBuffer::G722Buff& packet)
{
    if (state == State::AudioPacketReady) {
        audio_packet = side() == Side::Left ? packet.l.data() : packet.r.data();
        LOG_AUDIO("%s: Setting audio packet. Seq Num: %d\n", side_str, (int)audio_packet[0]);
        ++audio_index;
        state = State::AudioPacketSending;
        l2cap_request_can_send_now_event(cid);
    }
}

void HA::on_can_send_audio_packet_now()
{
    LOG_AUDIO("%s: Sending audio packet. Seq Num: %d\n", side_str, (int)audio_packet[0]);
    state = State::AudioPacketSent;
    uint8_t res = l2cap_send(cid, audio_packet, sdu_size_bytes);
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("%s: Error sending audio packet with error code: 0x%02x\n", side_str, (unsigned int)res);
        state = State::AudioPacketReady;
    }

}

void HA::on_audio_packet_sent()
{
    LOG_AUDIO("%s: Sent audio packet. State: %d. Seq Num: %d\n", side_str, static_cast<int>(state), (int)audio_packet[0]);
    state = State::AudioPacketReady;
}

bool HA::is_creating_l2cap_channel()
{
    using enum State;
    return is_any_of(state,
                     GATTConnected,
                     L2ConnStart,
                     L2ConnCompleted,
                     SubscribeASPNotification,
                     ASPNotificationSubscribed);
}

bool HA::is_streaming_audio()
{
    using enum State;
    return is_any_of(state, AudioPacketReady, AudioPacketSending, AudioPacketSent, ASPStartOk);
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
    LOG_INFO("Read Only Properties\n"
        "  Version:        %d\n"
        "  Side:           %s\n"
        "  Mode:           %s\n"
        "  CSIS:           %s\n"
        "  Manufacture ID: %04hx\n"
        "  LE CoC audio:   %s\n"
        "  Render delay:   %hu\n"
        "  Supports 16KHz: %s\n"
        "  Supports 24KHz: %s\n",
        version, 
        (side == HA::Side::Left ? "Left" : "Right"),
        (mode == HA::Mode::Binaural ? "Binaural" : "Monaural"),
        (csis_supported ? "Yes" : "No"),
        id.manufacturer_id,
        le_coc_supported ? "Yes" : "No",
        render_delay,
        codec_16khz ? "Yes" : "No",
        codec_24khz ? "Yes" : "No");
}

HAManager::HAManager()
{
    // We should only ever have at most two 
    // hearing aids so reserve capacity up front
    hearing_aids.reserve(max_num_ha);
}

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
            (hearing_aids.size() == 2 && hearing_aids[0].rop.id == hearing_aids[1].rop.id);
}

HA& HAManager::add(HA const& new_ha)
{
    if (exists(new_ha)) {
        LOG_INFO("Add: HA with address '%s' already exists\n", bd_addr_to_str(new_ha.addr));
        return invalid_ha;
    }
    if (hearing_aids.size() >= max_num_ha) {
        LOG_INFO("Add: Already hold max number of hearing aids\n");
        return invalid_ha;
    }
    if (hearing_aids.size() == 1) {
        if (hearing_aids[0].mode() == HA::Mode::Mono || new_ha.mode() == HA::Mode::Mono) {
            LOG_INFO("Add: Mono HA\n");
            return invalid_ha;
        }
        if (hearing_aids[0].rop.id != new_ha.rop.id) {
            LOG_INFO("Add: ID does not match other hearing aid\n");
            return invalid_ha;
        }
    }
    hearing_aids.emplace_back(new_ha);
    assert(hearing_aids.size() <= 2);
    if (hearing_aids.size() == 2) {
        hearing_aids[0].other_ha = &hearing_aids[1];
        hearing_aids[1].other_ha = &hearing_aids[0];
    }
    HA& added_ha = hearing_aids.back();
    added_ha.side_str = (added_ha.side() == HA::Side::Left) ? "Left" : "Right";
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
    LOG_INFO("Remove: HA not found\n");
}

} // namespace asha