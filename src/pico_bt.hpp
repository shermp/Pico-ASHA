#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <tuple>

#include <etl/array.h>
#include <etl/flat_map.h>
#include <etl/set.h>
#include <etl/span.h>
#include <etl/string.h>
#include <etl/circular_buffer.h>
#include <etl/vector.h>

#include <btstack.h>

namespace picobt {

constexpr size_t max_num_services = 4U;
constexpr size_t max_chars_for_service = 8U;

struct UUID {
    etl::array<uint8_t, 16> uuid_128;
    uint16_t uuid_16;
    constexpr UUID(const char uuid_str[37]);
    constexpr UUID(const char uuid_str[37], uint16_t uuid16);
    constexpr UUID(uint16_t uuid16);
    UUID(const uint8_t* bytes);
    bool operator==(const UUID&) const = default;
    bool operator<(const UUID& u) const 
    { 
        return std::tie((*this).uuid_128, (*this).uuid_16) < std::tie(u.uuid_128, u.uuid_16); 
    }
    bool operator>(const UUID& u) const
    {
        return std::tie((*this).uuid_128, (*this).uuid_16) > std::tie(u.uuid_128, u.uuid_16);
    }
};

struct AdReport {
    bd_addr_t address;
    uint8_t   type;
    uint8_t   event_type;
    uint8_t   address_type;
    uint8_t   rssi;
    etl::string<16> short_name;
    etl::string<16> complete_name;
    etl::set<UUID, 32> services;
    bool identity_resolved = false;
    bool connected = false;

    AdReport() {}
    AdReport(uint8_t* packet, bool extended);
};

class BT {
public:
    /* States for non-connected situations such as
       - scanning
       - Identity resolving
       - Connecting */
    enum class BaseState {
        Starting,
        Stopped,
        Idle,
        Scan,
        IdentityResolve,
        Connect,
    };

    /* States for each connected remote device */
    enum class RemoteState {
        Invalid,
        Connected,
        Disconnect,
        Bonding,
        ServiceDiscovery,
        CharDiscovery,
        ReadCharVal,
        WriteCharVal,
        EnableNotification,
        CreateL2CAP,
        SendL2CAPData,
    };

    enum class Result {
        Ok,
        InternalError,
        WrongState,
        MaxConnections,
        BTError,
    };

    struct Config {
        uint16_t max_le_mtu = 512U;
        io_capability_t auth_io_capabilities = IO_CAPABILITY_NO_INPUT_NO_OUTPUT;
        bool hci_dump = false;
        uint16_t scan_interval = 0x0030;
        uint16_t scan_window = 0x0030;
        uint16_t min_conn_interval = 16U;
        uint16_t max_conn_interval = 16U;
        uint16_t conn_latency = 10U;
        uint16_t supervision_timeout = 100U;
        uint16_t min_ce = 12U;
        uint16_t max_ce = 12U;
    };

    struct Service {
        gatt_client_service_t service = {};
        etl::vector<gatt_client_characteristic_t, max_chars_for_service> chars = {};
    };

    struct Remote {
        RemoteState state = RemoteState::Invalid;
        hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
        uint16_t  local_cid = 0U;
        bd_addr_t addr = {};
        etl::vector<Service*, max_num_services> services = {};

        /**
         * Disconnect from remote.
         * The disconnect callback set in connect will be called 
         * when the disconnect completes.
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code 
         */ 
        Result disconnect(
            uint8_t* bt_err
        );

        /**
         * Pair and bond to remote.
         * bond_cb will be called on completion. Pairing and bonding
         * failed if status is not ERROR_CODE_SUCCESS, and reason 
         * may be said to provide more context
         */
        Result bond(
            std::function<void(uint8_t status, uint8_t reason, Remote* remote)> bond_cb
        );

        /**
         * Discover services availabe on remote.
         * This may be called after connecting and before pairing. 
         * services_cb will be call at the completion of service discovery.
         * filter_cb will be called for each service found.
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result discover_services(
            std::function<void(uint8_t status, Remote* remote)> services_cb,
            std::function<bool(Service const& filter)> filter_cb,
            uint8_t* bt_err
        );

        /**
         * Discover characteristics for discovered services on remote.
         * char_cb will be called when all characteristics are discovered, 
         * or an occurrs during discovery.
         * Characteristics can be filtered using filter_cb
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         * 
         * Note: the status variable in char_cb may be set to either a 
         * btstack error code, or an ATT status
         */
        Result discover_characteristics(
            std::function<void(uint8_t status, Remote* remote)> char_cb,
            std::function<bool(gatt_client_characteristic_t* filter)> filter_cb,
            uint8_t* bt_err
        );

        /**
         * Read multiple characteristic values from remote
         * value_handles is the list of characteristic value handles 
         * to read data from.
         * char_val_cb will be called when a characteristic value has 
         * been read.
         * char_val_complete_cb will be called when all characteristic 
         * values has been read.
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result read_characteristic_values(
            etl::span<uint16_t> value_handles,
            std::function<void(Remote* remote, uint8_t const* data, uint16_t len)> char_val_cb,
            std::function<void(uint8_t status, Remote* remote)> char_val_complete_cb,
            uint8_t* bt_err
        );

        /**
         * Write value to characteristic with no rwsponse
         * val_handle is the handle of the characteristic to write to
         * data is the value to write
         * len is the length of the value
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result write_characteristic_value_no_resp(
            uint16_t val_handle,
            uint8_t* data,
            uint16_t len,
            uint8_t* bt_err
        );

        /**
         * Write value to characteristic
         * val_handle is the handle of the characteristic to write to
         * data is the value to write
         * len is the length of the value
         * char_write_cb will be called when the write completes
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result write_characteristic_value(
            uint16_t val_handle,
            uint8_t* data,
            uint16_t len,
            std::function<void(uint8_t status, Remote* remote)> char_write_cb,
            uint8_t* bt_err
        );

        /**
         * Enable notification for characteristic
         * char_not_val_cb will be called when a notification is received
         * char_not_en_cb will be called when the notification has been enabled
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result enable_notification(
            gatt_client_characteristic_t* characteristic,
            std::function<void(Remote* remote, uint8_t const* data, uint16_t len)> char_not_val_cb,
            std::function<void(uint8_t status, Remote* remote)> char_not_en_cb,
            uint8_t* bt_err
        );

        /**
         * Create a L2CAP CoC channel
         * psm is the address to connect to on remote
         * receive_buff will store incoming packets
         * receive_len is the size of the incoming buffer
         * l2cap_created_cb will be called when the channel is created (or failed)
         * l2cap_write_cb will be called when a write using send_l2cap_data completes
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result create_l2cap_cbm_conn(
            uint16_t psm,
            uint8_t* receive_buff,
            uint16_t receive_len,
            std::function<void(uint8_t status, Remote* remote)> l2cap_created_cb,
            std::function<void(Remote* remote)> l2cap_write_cb,
            uint8_t* bt_err
        );

        /**
         * Send data on currently open L2CAP CoC channel
         * data is the data to send. It must remain available until
         * the write complete callback is called
         * len is the size of the data to send
         * If Result is BTError, then bt_err will be set to the 
         * underlying btstack error code
         */
        Result send_l2cap_data(
            uint8_t const* data,
            uint16_t len,
            uint8_t* bt_err
        );

        ~Remote();
        explicit operator bool() const { return con_handle != HCI_CON_HANDLE_INVALID; }
        friend class BT;
        
    private:
        /* Callbacks */
        std::function<void(uint8_t, uint8_t, Remote*)> p_bond_cb;
        std::function<void(uint8_t, Remote*)> p_services_cb;
        std::function<void(uint8_t, Remote*)> p_char_cb;
        std::function<void(Remote*, uint8_t const*, uint16_t)> p_char_val_cb;
        std::function<void(uint8_t, Remote*)> p_char_val_complete_cb;
        std::function<void(uint8_t, Remote*)> p_char_write_cb;
        std::function<void(Remote*, uint8_t const*, uint16_t)> p_char_not_val_cb;
        std::function<void(uint8_t, Remote*)> p_char_not_en_cb;
        std::function<void(uint8_t, Remote*)> p_l2cap_created_cb;
        std::function<void(Remote*)> p_l2cap_write_cb;

        /* Filters */
        std::function<bool(Service const&)> p_service_filter_cb;
        std::function<bool(gatt_client_characteristic_t*)> p_char_filter_cb;

        /* Variables */
        size_t curr_service_char_index = 0;

        /* Pending L2CAP data */
        uint8_t const* l2cap_data = nullptr;
        uint16_t l2cap_len = 0U;
    };

    /**
     * Get the BT singleton. This singleton wraps btstack 
     */
    static BT& instance() 
    {
        static BT bt;
        return bt;
    }

    /**
     * Configure BT singleton. Should be called once at startup 
     */
    void configure(Config const& config);

    /**
     * Start BT processing. start_cb will be invoked when
     * the bt stack has started its event loop, or it has
     * failed to start.
     */
    bool start(std::function<void(bool started)> start_cb);

    /** 
     * Enable scanning for advertisement packets.
     * ad_report_cb is called for every AdReport that has
     * had its identity resolved, or matches filter 
     */
    void enable_scan(
        std::function<void(AdReport const& report)> ad_report_cb,
        std::function<bool(AdReport const& report)> filter,
        bool only_connectable = true
    );
    
    /* Continue scanning for advertisement packets.
       If Result is not Ok, then BT was not in the correct 
       state to continue scanning */
    Result continue_scan();

    /**
     * Connect to remote with the specified addr and addr_type.
     * connect_cb will be called on connection completion. If 
     * callback status is not ATT_ERROR_SUCCESS, then an error 
     * has occurred and the connection has failed. disconnect_cb 
     * will be called on disconnect. NOTE: connect_cb and disconnect_cb 
     * are global callbacks for all connections.
     * If Result is BTError, then bt_err will be set to the 
     * underlying btstack error code 
     */
    Result connect(
        bd_addr_t addr, 
        bd_addr_type_t addr_type,
        std::function<void(uint8_t status, Remote* remote)> connect_cb,
        std::function<void(uint8_t reason, Remote* remote)> disconnect_cb,
        uint8_t* bt_err
    );

    BT(BT&) = delete;
    BT(BT&&) = delete;
    BT(const BT&) = delete;
    BT(const BT&&) = delete;
    void operator=(const BT&) = delete;

private:
    bool p_configured = false;
    Config p_config = {};
    bd_addr_t p_local_addr = {};
    BaseState p_base_state = BaseState::Stopped;
    
    etl::circular_buffer<AdReport, 16> p_ad_reports = {};
    AdReport *p_curr_add_report = nullptr;
    bool scan_enabled = false;
    bool p_only_show_connectable = false;
    uint8_t auth_req;

    gatt_client_notification_t notification_listener = {};

    etl::vector<Remote, MAX_NR_HCI_CONNECTIONS> remotes;

    /* Callbacks */
    std::function<void(bool)> p_start_cb;
    std::function<void(AdReport const&)> p_ad_report_cb;
    std::function<void(uint8_t, Remote*)> p_connect_cb;
    std::function<void(uint8_t, Remote*)> p_disconnect_cb;

    /* Filters */
    std::function<bool(AdReport const&)> p_scan_filter;

    /* Private methods */
    AdReport& add_or_merge_ad_report(AdReport const& report);
    void set_idle_state();
    Remote* get_by_con_handle(hci_con_handle_t handle);
    Remote* get_by_local_cid (uint16_t local_cid);

    /* BTStack packet handlers */
    static btstack_packet_callback_registration_t hci_cb_reg;
    static btstack_packet_callback_registration_t sm__cb_reg;
    static void   hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void    sm_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void  gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void l2cap_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    
    BT() {};
    ~BT() {};
};

} // namespace picobt