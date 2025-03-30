#pragma once

namespace asha
{

namespace comm
{
    enum class Type : uint8_t
    {
        Event = 0,
        HCI,
    };

    enum class PAError : uint8_t
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
        uint16_t hci_handle;
        uint32_t ts_ms;
    };

    static_assert(sizeof(HeaderPacket) == 8);

    struct EventPacket
    {
        EventType ev_type = {};
        uint8_t status = {};
        uint8_t reason = {};
        PAError pa_error = {};
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

        EventPacket(EventType type) : ev_type(type), 
                                      status(0), 
                                      reason(0),
                                      pa_error(PAError::PASuccess) {};
        
        EventPacket(EventType type, uint8_t stat) : ev_type(type),
                                                    status(stat), 
                                                    reason(0),
                                                    pa_error(PAError::PASuccess) {};

        EventPacket(EventType type, uint8_t stat, uint8_t reas) : ev_type(type),
                                                                  status(stat), 
                                                                  reason(reas),
                                                                  pa_error(PAError::PASuccess) {};

        EventPacket(EventType type, PAError pa) : ev_type(type), 
                                                  status(0), 
                                                  reason(0),
                                                  pa_error(pa) {};

        template<typename ...Fmt>
        void set_data_str(const char* format, Fmt&&... fmt) 
        {
            sniprintf(data.str, sizeof data.str, format, fmt...);
        }
    };

    static_assert(sizeof(EventPacket) == 36);

    void add_event_to_buffer(uint16_t const handle, EventPacket const& event);

    void try_send_events();

} // namespace comm

} // namespace asha