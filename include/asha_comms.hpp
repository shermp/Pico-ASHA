#pragma once

#include <cstdint>

namespace asha
{

namespace comm
{
    constexpr uint16_t unset_conn_id = 0;

    enum class Type : uint8_t
    {
        Event = 0,
        HCI,
    };

    enum class StatusType : uint8_t
    {
        StatusSuccess = 0,
        PAStatus,
        BtstackStatus,
        ATTStatus,
        SMStatus,
    };

    enum PAError : uint8_t
    {
        PASuccess = 0,
        PARuntimeSettingsErr,
        PAMaxConnected,
        PAASHAServiceNotFound,
    };

    enum class EventType : uint8_t
    {
        ShortLog = 0,
        PicoASHAInit,
        DeletePair,
        RemoteConnected,
        RemoteDisconnected,
        DiscServices,
        PairAndBond,
        DLE,
        DiscASHAChar,
        // Data is 17 byte ROP
        ROPRead,
        // Data is 1 byte PSM
        PSMRead,
        DiscGAPChar,
        // Data is up to 32 byte null terminated string
        DevNameRead,
        DiscDISChar,
        // Data is up to 32 byte null terminated string
        MfgRead,
        // Data is up to 32 byte null terminated string
        ModelRead,
        // Data is up to 32 byte null terminated string
        FWRead,
        L2CAPCon,
        L2CAPDiscon,
        ASPNotEnable,
        ACPStart,
        ACPStop,
        ACPStatus,
        ASPStart,
        ASPStop,
        ASPError,
        StreamReady,
        StreamPause,
        AudioVolume,
    };

    struct HeaderPacket
    {
        Type  type;
        uint8_t  len;
        uint16_t conn_id;
        uint32_t ts_ms;
    };

    static_assert(sizeof(HeaderPacket) == 8);

    struct EventPacket
    {
        EventType ev_type = {};
        StatusType status_type = {};
        uint8_t status = {};
        uint8_t reason = {};
        union {
            uint8_t addr[6];
            uint8_t psm;
            int8_t asp_not;
            int8_t volume;
            uint16_t cid;
            uint16_t credits;
            uint8_t rop[17];
            char    str[32];
        } data = {};

        EventPacket();

        EventPacket(EventType type) : ev_type(type), 
                                      status_type(StatusType::StatusSuccess),
                                      status(0), 
                                      reason(0) {};
        
        EventPacket(EventType type, StatusType st_type, uint8_t stat) : ev_type(type),
                                                                        status_type(st_type),
                                                                        status(stat), 
                                                                        reason(0) {};

        EventPacket(EventType type, StatusType st_type, uint8_t stat, uint8_t reas) : ev_type(type),
                                                                                      status_type(st_type),
                                                                                      status(stat), 
                                                                                      reason(reas) {};

        template<typename ...Fmt>
        void set_data_str(const char* format, Fmt&&... fmt) 
        {
            sniprintf(data.str, sizeof data.str, format, fmt...);
        }
    };

    static_assert(sizeof(EventPacket) == 36);

    void add_event_to_buffer(uint16_t const conn_id, EventPacket const& event);

    void try_send_events();

} // namespace comm

} // namespace asha