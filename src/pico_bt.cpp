#include <etl/pool.h>
#include <etl/algorithm.h>

#include <pico/async_context.h>
#include "pico_bt.hpp"

#include <hci_dump_embedded_stdout.h>

namespace picobt {

constexpr size_t service_pool_size = 16U;

static etl::pool<BT::Service, service_pool_size> service_pool = {};

constexpr uint8_t authreq_le_sec = SM_AUTHREQ_BONDING | SM_AUTHREQ_SECURE_CONNECTION;
constexpr uint8_t authreq_no_le_sec = SM_AUTHREQ_BONDING;

AdReport::AdReport(uint8_t* packet, bool extended)
{
    uint8_t         length;
    const uint8_t * data;
    if (extended) {
        gap_event_extended_advertising_report_get_address(packet, address);
        event_type = gap_event_extended_advertising_report_get_advertising_event_type(packet);
        address_type = (bd_addr_type_t)gap_event_extended_advertising_report_get_address_type(packet);
        rssi = gap_event_extended_advertising_report_get_rssi(packet);
        length = gap_event_extended_advertising_report_get_data_length(packet);
        data = gap_event_extended_advertising_report_get_data(packet);
    } else {
        gap_event_advertising_report_get_address(packet, address);
        event_type = gap_event_advertising_report_get_advertising_event_type(packet);
        address_type = (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet);
        rssi = gap_event_advertising_report_get_rssi(packet);
        length = gap_event_advertising_report_get_data_length(packet);
        data = gap_event_advertising_report_get_data(packet);
    }

    ad_context_t context;
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
                if (!services.full()) {
                    services.insert(UUID(little_endian_read_16(adv_data, i)));
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += sizeof(tmp_uuid128)) {
                reverse_128(adv_data + i, tmp_uuid128);
                if (!services.full()) {
                    services.insert(UUID(tmp_uuid128));
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
            complete_name.append((const char*)adv_data, size);
            break;
        case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            short_name.append((const char*)adv_data, size);
            break;
        default:
            break;
        }
    }
}

BT::Remote::~Remote()
{
    for (Service* s : services) {
        service_pool.destroy(s);
    }
}

void BT::configure(Config const& config)
{
    if (p_configured) { return; }
    p_config = config;
    if (config.hci_dump) {
        hci_dump_init(hci_dump_embedded_stdout_get_instance());
    }
    l2cap_init();
    l2cap_set_max_le_mtu(config.max_le_mtu);

    /* Init the security manager. This takes care of pairing
       with hearing aids, and also allows reconnecting to already
       paired hearing aids */
    sm_init();
    sm_set_secure_connections_only_mode(false);
    sm_set_io_capabilities(config.auth_io_capabilities);
    auth_req = authreq_le_sec;
    sm_set_authentication_requirements(auth_req);
    sm_allow_ltk_reconstruction_without_le_device_db_entry(0);

    /* Init GATT client. Required to read/write GATT characteristics */
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_0);

    /* Register BTStack callbacks */
    hci_cb_reg.callback = &BT::hci_handler;
    sm_cb_reg.callback = &BT::sm_handler;
    hci_add_event_handler(&hci_cb_reg);
    sm_add_event_handler(&sm_cb_reg);

    /* Register the listener for GATT notifications and indications */
    gatt_client_listen_for_characteristic_value_updates(
        &notification_listener, 
        &BT::gatt_handler,
        GATT_CLIENT_ANY_CONNECTION,
        NULL
    );

    p_configured = true;
}

bool BT::start(etl::delegate<void(bool started, uint8_t state)> start_cb)
{
    if (p_configured && start_cb) {
        p_start_cb = start_cb;
        p_base_state = BaseState::Starting;
        return hci_power_control(HCI_POWER_ON) == 0;
    }
    return false;
}

void BT::enable_scan(etl::delegate<void(AdReport& report)> ad_report_cb, 
                     etl::delegate<bool(AdReport const& report)> filter,
                     bool only_connectable)
{
    if (!scan_enabled) {
        scan_enabled = true;
        p_only_show_connectable = only_connectable;
        p_ad_report_cb = ad_report_cb;
        p_scan_filter = filter;
        gap_set_scan_params(1, p_config.scan_interval, p_config.scan_window, 0);
        p_base_state = BaseState::Scan;
        gap_start_scan();
    }
}

BT::Result BT::continue_scan()
{
    if (scan_enabled && p_base_state == BaseState::Idle) {
        if (remotes.full()) {
            return Result::MaxConnections;
        }
        p_base_state = BaseState::Scan;
        return Result::Ok;
    }
    return Result::InternalError;
}

BT::Result BT::connect(bd_addr_t addr, 
                    bd_addr_type_t addr_type,
                    etl::delegate<void(uint8_t status, Remote* remote)> connect_cb,
                    etl::delegate<void(uint8_t reason, Remote* remote)> disconnect_cb,
                    uint8_t* bt_err)
{
    if (p_base_state == BaseState::Idle) {
        if (remotes.full()) {
            return Result::MaxConnections;
        }
        p_connect_cb = connect_cb;
        p_disconnect_cb = disconnect_cb;
        p_base_state = BaseState::Connect;
        uint8_t err = gap_connect(addr, addr_type);
        if (err != ERROR_CODE_SUCCESS) {
            set_idle_state();
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    set_idle_state();
    return Result::InternalError;
}

void BT::clear_bonding_data()
{
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            le_device_db_remove(i);
        }
    }
}

BT::Result BT::Remote::disconnect(uint8_t* bt_err)
{
    if (state == RemoteState::Connected) {
        state = RemoteState::Disconnect;
        uint8_t err = gap_disconnect(con_handle);
        if (err != ERROR_CODE_SUCCESS) {
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::bond(etl::delegate<void(uint8_t status, uint8_t reason, Remote* remote)> bond_cb)
{
    p_bond_cb = bond_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::Bonding;
        sm_request_pairing(con_handle);
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::discover_services(etl::delegate<void(uint8_t status, Remote* remote)> services_cb,
        etl::delegate<bool(Service const& filter)> filter_cb,
        uint8_t* bt_err)
{
    p_services_cb = services_cb;
    p_service_filter_cb = filter_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::ServiceDiscovery;
        uint8_t err = gatt_client_discover_primary_services(&BT::gatt_handler, con_handle);
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::discover_characteristics(etl::delegate<void(uint8_t status, Remote* remote)> char_cb,
        etl::delegate<bool(gatt_client_characteristic_t* filter)> filter_cb,
        uint8_t* bt_err)
{
    p_char_cb = char_cb;
    p_char_filter_cb = filter_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::CharDiscovery;
        uint8_t err = gatt_client_discover_characteristics_for_service(
            &BT::gatt_handler, 
            con_handle, 
            &services[curr_service_char_index]->service);
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::read_characteristic_values(etl::span<uint16_t> value_handles,
        etl::delegate<void(Remote* remote, uint16_t val_handle, uint8_t const* data, uint16_t len)> char_val_cb,
        etl::delegate<void(uint8_t status, Remote* remote)> char_val_complete_cb,
        uint8_t* bt_err)
{
    p_char_val_cb = char_val_cb;
    p_char_val_complete_cb = char_val_complete_cb;
    char_val_handles = value_handles;
    curr_char_val_handle_index = 0;
    return read_characteristic_value(bt_err);
}

BT::Result BT::Remote::write_characteristic_value_no_resp(uint16_t val_handle,
        uint8_t* data,
        uint16_t len,
        uint8_t* bt_err)
{
    if (state == RemoteState::Connected) {
        uint8_t err = gatt_client_write_value_of_characteristic_without_response(
            con_handle,
            val_handle,
            len,
            data
        );
        if (err != ERROR_CODE_SUCCESS) {
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::write_characteristic_value(uint16_t val_handle,
        uint8_t* data,
        uint16_t len,
        etl::delegate<void(uint8_t status, Remote* remote)> char_write_cb,
        uint8_t* bt_err)
{
    p_char_write_cb = char_write_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::WriteCharVal;
        uint8_t err = gatt_client_write_value_of_characteristic(
            &BT::gatt_handler,
            con_handle,
            val_handle,
            len,
            data
        );
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::enable_notification(gatt_client_characteristic_t* characteristic,
        etl::delegate<void(Remote* remote, uint8_t const* data, uint16_t len)> char_not_val_cb,
        etl::delegate<void(uint8_t status, Remote* remote)> char_not_en_cb,
        uint8_t* bt_err)
{
    p_char_not_val_cb = char_not_val_cb;
    p_char_not_en_cb = char_not_en_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::EnableNotification;
        uint8_t err = gatt_client_write_client_characteristic_configuration(
            &BT::gatt_handler,
            con_handle,
            characteristic,
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
        );
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::create_l2cap_cbm_conn(uint16_t psm,
        uint8_t* receive_buff,
        uint16_t receive_len,
        etl::delegate<void(uint8_t status, Remote* remote)> l2cap_created_cb,
        etl::delegate<void(Remote* remote)> l2cap_write_cb,
        uint8_t* bt_err)
{
    p_l2cap_created_cb = l2cap_created_cb;
    p_l2cap_write_cb = l2cap_write_cb;
    if (state == RemoteState::Connected) {
        state = RemoteState::CreateL2CAP;
        uint8_t err = l2cap_cbm_create_channel(
            &BT::l2cap_handler,
            con_handle,
            psm,
            receive_buff,
            receive_len,
            L2CAP_LE_AUTOMATIC_CREDITS,
            LEVEL_2,
            &local_cid
        );
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

BT::Result BT::Remote::send_l2cap_data(uint8_t const* data,
        uint16_t len,
        uint8_t* bt_err)
{
    if (state == RemoteState::Connected) {
        state = RemoteState::SendL2CAPData;
        l2cap_data = data;
        l2cap_len = len;
        uint8_t err = l2cap_request_can_send_now_event(local_cid);
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

/************************************
 * Private methods
 ************************************/

BT::Result BT::Remote::read_characteristic_value(uint8_t* bt_err)
{
    if (state == RemoteState::Connected) {
        state = RemoteState::ReadCharVal;
        uint8_t err = gatt_client_read_value_of_characteristic_using_value_handle(
            &BT::gatt_handler,
            con_handle,
            char_val_handles[curr_char_val_handle_index]
        );
        if (err != ERROR_CODE_SUCCESS) {
            state = RemoteState::Connected;
            *bt_err = err;
            return Result::BTError;
        }
        return Result::Ok;
    }
    return Result::WrongState;
}

AdReport& BT::add_or_merge_ad_report(AdReport const& report)
{
    for (auto& ad : p_ad_reports) {
        if (bd_addr_cmp(ad.address, report.address) == 0) {
            ad.short_name = report.short_name;
            ad.complete_name = report.complete_name;
            ad.services.insert(report.services.begin(), report.services.end());
            return ad;
        }
    }
    p_ad_reports.push(report);
    return p_ad_reports.back();
}

void BT::set_idle_state()
{
    p_base_state = BaseState::Idle;
}

BT::Remote* BT::get_by_con_handle(hci_con_handle_t handle)
{
    for (Remote& r : remotes) {
        if (r.con_handle == handle) {
            return &r;
        }
    }
    return nullptr;
}

BT::Remote* BT::get_by_local_cid (uint16_t local_cid)
{
    for (Remote& r : remotes) {
        if (r.local_cid == local_cid) {
            return &r;
        }
    }
    return nullptr;
}

bool BT::address_connected(bd_addr_t addr) {
    for (auto& r : remotes) {
        if (bd_addr_cmp(addr, r.addr) == 0) {
            return true;
        }
    }
    return false;
}

void BT::hci_handler(uint8_t packet_type,  
                     [[maybe_unused]] uint16_t channel, 
                     uint8_t *packet, 
                     [[maybe_unused]] uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) { return; }

    BT& bt = BT::instance();

    uint8_t hci_ev_type = hci_event_packet_get_type(packet);
    switch(hci_ev_type) {
        /* BTStack sends this event at startup */
        case BTSTACK_EVENT_STATE:
        {
            if (bt.p_base_state != BaseState::Starting) { return; }
            uint8_t start_state = btstack_event_state_get_state(packet);
            if (start_state == HCI_STATE_WORKING) {
                gap_local_bd_addr(bt.p_local_addr);
                gap_set_connection_parameters(
                    bt.p_config.scan_interval,
                    bt.p_config.scan_window,
                    bt.p_config.min_conn_interval,
                    bt.p_config.max_conn_interval,
                    bt.p_config.conn_latency,
                    bt.p_config.supervision_timeout,
                    bt.p_config.min_ce,
                    bt.p_config.max_ce
                );
                bt.p_base_state = BaseState::Idle;
                return bt.p_start_cb(true, 0U);
            }
            break;
        }
        /* Handle advertising reports here */
        case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        case GAP_EVENT_ADVERTISING_REPORT:
        {
            if (bt.p_base_state != BaseState::Scan) { return; }
            AdReport ad_report(packet, hci_ev_type == GAP_EVENT_EXTENDED_ADVERTISING_REPORT);
            AdReport& ad = bt.add_or_merge_ad_report(ad_report);
            if (bt.address_connected(ad.address)) { return; }
            bt.p_curr_add_report = &ad;
            bt.p_base_state = BaseState::IdentityResolve;
            // Result returned in sm_handler()
            sm_address_resolution_lookup(bt.p_curr_add_report->address_type, bt.p_curr_add_report->address);
            break;
        }
        case HCI_EVENT_META_GAP:
        {
            if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE) { return; }
            if (bt.p_base_state != BaseState::Connect) { return; }
            Remote remote;
            uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
            remote.state = status == ATT_ERROR_SUCCESS ? RemoteState::Connected : RemoteState::Invalid;
            if (remote.state == RemoteState::Connected) {
                remote.con_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                gap_subevent_le_connection_complete_get_peer_address(packet, remote.addr);
                bt.remotes.push_back(remote);
                bt.p_base_state = BaseState::Idle;
                bt.p_connect_cb(status, &bt.remotes.back());
                return;
            }
            bt.p_base_state = BaseState::Idle;
            bt.p_connect_cb(status, nullptr);
            break;
        }
        case HCI_EVENT_DISCONNECTION_COMPLETE:
        {
            Remote* r = bt.get_by_con_handle(hci_event_disconnection_complete_get_connection_handle(packet));
            if (!r) { return; }
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            bt.p_disconnect_cb(reason, r);
            etl::erase_if(bt.remotes, [r](Remote& remote) { return r == &remote; });
            break;
        }
        default:
            break;

    }
}

void BT::gatt_handler(uint8_t packet_type, 
                      [[maybe_unused]] uint16_t channel, 
                      uint8_t *packet, 
                      [[maybe_unused]] uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) { return; }

    BT& bt = BT::instance();

    uint8_t ev_type = hci_event_packet_get_type(packet);
    switch (ev_type) {
        case GATT_EVENT_SERVICE_QUERY_RESULT:
        {
            Remote* r = bt.get_by_con_handle(gatt_event_service_query_result_get_handle(packet));
            if (!r) { return; }
            if (r->state != RemoteState::ServiceDiscovery) { return; }
            if (r->services.full()) { return; }
            Service* s = service_pool.create();
            if (!s) { return; }
            gatt_event_service_query_result_get_service(packet, &s->service);
            if (r->p_service_filter_cb(*s)) {
                r->services.push_back(s);
            } else {
                service_pool.destroy(s);
            }
            break;
        }
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
        {
            Remote* r = bt.get_by_con_handle(gatt_event_characteristic_query_result_get_handle(packet));
            if (!r) { return; }
            if (r->state != RemoteState::CharDiscovery) { return; }
            gatt_client_characteristic_t c;
            gatt_event_characteristic_query_result_get_characteristic(packet, &c);
            if (r->p_char_filter_cb(&c)) {
                auto& chars = r->services[r->curr_service_char_index]->chars;
                if (!chars.full()) {
                    chars.push_back(c);
                }
            }
            break;
        }
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
        {
            Remote* r = bt.get_by_con_handle(gatt_event_characteristic_value_query_result_get_handle(packet));
            if (!r) { return; }
            if (r->state != RemoteState::ReadCharVal) { return; }
            r->p_char_val_cb(r,
                             gatt_event_characteristic_value_query_result_get_value_handle(packet),
                             gatt_event_characteristic_value_query_result_get_value(packet),
                             gatt_event_characteristic_value_query_result_get_value_length(packet));
            break;
        }
        case GATT_EVENT_NOTIFICATION:
        {
            Remote* r = bt.get_by_con_handle(gatt_event_notification_get_handle(packet));
            if (!r) { return; }
            r->p_char_not_val_cb(
                r,
                gatt_event_notification_get_value(packet),
                gatt_event_notification_get_value_length(packet)
            );
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
        {
            Remote* r = bt.get_by_con_handle(gatt_event_query_complete_get_handle(packet));
            if (!r) { return; }
            uint8_t status = gatt_event_query_complete_get_att_status(packet);
            switch (r->state) {
                case RemoteState::ServiceDiscovery:
                    r->state = RemoteState::Connected;
                    r->p_services_cb(status, r);
                    break;
                case RemoteState::CharDiscovery:
                    if (status != ATT_ERROR_SUCCESS) {
                        r->state = RemoteState::Connected;
                        r->p_char_cb(status, r);
                        return;
                    }
                    r->curr_service_char_index++;
                    if (r->curr_service_char_index < r->services.size()) {
                        uint8_t err = gatt_client_discover_characteristics_for_service(&BT::gatt_handler, 
                            r->con_handle, 
                            &r->services[r->curr_service_char_index]->service);
                        if (err != ERROR_CODE_SUCCESS) {
                            r->state = RemoteState::Connected;
                            r->p_char_cb(err, r);
                            return;
                        }
                        return;
                    }
                    r->curr_service_char_index = 0;
                    r->state = RemoteState::Connected;
                    r->p_char_cb(status, r);
                    break;
                case RemoteState::ReadCharVal:
                    r->state = RemoteState::Connected;
                    r->curr_char_val_handle_index++;
                    if (r->curr_char_val_handle_index < r->char_val_handles.size()) {
                        uint8_t err = ERROR_CODE_SUCCESS;
                        r->read_characteristic_value(&err);
                        if (err != ERROR_CODE_SUCCESS) {
                            r->state = RemoteState::Connected;
                            r->p_char_val_complete_cb(err, r);
                            return;
                        }
                        return;
                    }
                    r->p_char_val_complete_cb(status, r);
                    break;
                case RemoteState::WriteCharVal:
                    r->state = RemoteState::Connected;
                    r->p_char_write_cb(status, r);
                    break;
                case RemoteState::EnableNotification:
                    r->state = RemoteState::Connected;
                    r->p_char_not_en_cb(status, r);
                    break;
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }
}

void BT::l2cap_handler(uint8_t packet_type, 
                    [[maybe_unused]] uint16_t channel, 
                    uint8_t *packet, 
                    [[maybe_unused]] uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) { return; }

    BT& bt = BT::instance();

    uint8_t ev_type = hci_event_packet_get_type(packet);
    switch (ev_type) {
        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
        {
            Remote* r = bt.get_by_con_handle(l2cap_event_cbm_channel_opened_get_handle(packet));
            if (!r) { return; }
            if (r->state != RemoteState::CreateL2CAP) { return; }
            uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
            r->state = RemoteState::Connected;
            r->p_l2cap_created_cb(status, r);
            break;
        }
        case L2CAP_EVENT_CAN_SEND_NOW:
        {
            Remote* r = bt.get_by_local_cid(l2cap_event_can_send_now_get_local_cid(packet));
            if (!r) { return; }
            if (r->state != RemoteState::SendL2CAPData) { return; }
            l2cap_send(r->local_cid, r->l2cap_data, r->l2cap_len);
            break;
        }
        case L2CAP_EVENT_PACKET_SENT:
        {
            Remote* r = bt.get_by_local_cid(l2cap_event_packet_sent_get_local_cid(packet));
            if (!r) { return; }
            if (r->state != RemoteState::SendL2CAPData) { return; }
            r->state = RemoteState::Connected;
            r->p_l2cap_write_cb(r);
            break;
        }
        default:
            break;
    }
}

void BT::sm_handler(uint8_t packet_type, 
                    [[maybe_unused]] uint16_t channel, 
                    uint8_t *packet, 
                    [[maybe_unused]] uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) { return; }

    BT& bt = BT::instance();

    uint8_t ev_type = hci_event_packet_get_type(packet);
    if (bt.p_base_state == BaseState::IdentityResolve) { 
        switch(ev_type) {
            case SM_EVENT_IDENTITY_RESOLVING_STARTED:
                break;
            /* Identity resolved to a bonded remote device */
            case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
                bt.p_curr_add_report->identity_resolved = true;
                sm_event_identity_resolving_succeeded_get_address(packet, 
                                                                  bt.p_curr_add_report->address);
                bt.p_curr_add_report->address_type = (bd_addr_type_t)sm_event_identity_resolving_succeeded_get_addr_type(packet);
                if (bt.address_connected(bt.p_curr_add_report->address)) {
                    bt.p_base_state = BaseState::Scan;
                    return;
                }
                bt.p_base_state = BaseState::Idle;
                bt.p_ad_report_cb(*bt.p_curr_add_report);
                break;
            /* Identity not resolved to a bonded remote device.
               Send Ad report to caller using provided callback,
               only if filter returns true */
            case SM_EVENT_IDENTITY_RESOLVING_FAILED:
                if (bt.address_connected(bt.p_curr_add_report->address)) {
                    bt.p_base_state = BaseState::Scan;
                    return;
                }
                if (bt.p_scan_filter(*bt.p_curr_add_report)) {
                    bt.p_base_state = BaseState::Idle;
                    bt.p_ad_report_cb(*bt.p_curr_add_report);
                } else {
                    bt.p_base_state = BaseState::Scan;
                }
                break;
            default:
                break;
        }
    } else {
        switch(ev_type) {
            case SM_EVENT_JUST_WORKS_REQUEST:
                sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
                break;
            case SM_EVENT_PAIRING_STARTED:
                break;
            case SM_EVENT_PAIRING_COMPLETE:
            {
                uint8_t status = sm_event_pairing_complete_get_status(packet);
                uint8_t reason = sm_event_pairing_complete_get_reason(packet);
                Remote* r = bt.get_by_con_handle(sm_event_pairing_complete_get_handle(packet));
                if (!r) { return; }
                if (status == ERROR_CODE_AUTHENTICATION_FAILURE) {
                    if (reason == SM_REASON_AUTHENTHICATION_REQUIREMENTS && bt.auth_req == authreq_le_sec) {
                        bt.auth_req = authreq_no_le_sec;
                        sm_set_authentication_requirements(bt.auth_req);
                        sm_request_pairing(r->con_handle);
                        return;
                    }
                }
                r->state = RemoteState::Connected;
                r->p_bond_cb(status, reason, r);
                break;
            }
            case SM_EVENT_REENCRYPTION_STARTED:
                break;
            case SM_EVENT_REENCRYPTION_COMPLETE:
            {
                uint8_t status = sm_event_reencryption_complete_get_status(packet);
                Remote* r = bt.get_by_con_handle(sm_event_reencryption_complete_get_handle(packet));
                if (!r) { return; }
                if (status == ERROR_CODE_PIN_OR_KEY_MISSING && bt.auth_req == authreq_le_sec) {
                    bt.auth_req = authreq_no_le_sec;
                    sm_set_authentication_requirements(bt.auth_req);
                    sm_request_pairing(r->con_handle);
                    return;
                }
                r->state = RemoteState::Connected;
                r->p_bond_cb(status, 0U, r);
                break;
            }
            default:
                break;
        }
    }
    
}

} // namespace picobt