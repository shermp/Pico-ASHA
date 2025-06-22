#include <QDateTime>
#include <QDebug>
#include <QObject>
#include <QMap>
#include <QSerialPortInfo>
#include <QUrl>
#include <QtEndian>

#include <nanocobs/cobs.h>

#include "bt_status_err.hpp"

#include "picoashacomm.h"

constexpr int timer_interval = 1000;

QMap<QByteArray, RemoteDevice::CachedProps> cached_remote_props;

PicoAshaComm::PicoAshaComm(QObject *parent)
    : QObject{parent}, m_serial(this), connect_timer(this), m_hciLoggingEnabled(false)
{
    m_ui = new PicoAshaMainWindow;
    m_ui->setHciActionBtnStart(false);

    m_serial.setBaudRate(QSerialPort::Baud115200);
    m_serial.setDataBits(QSerialPort::Data8);
    m_serial.setStopBits(QSerialPort::OneStop);
    m_serial.setParity(QSerialPort::NoParity);
    m_serial.setFlowControl(QSerialPort::HardwareControl);

    QObject::connect(&connect_timer, &QTimer::timeout, this, &PicoAshaComm::onConnectTimer);
    QObject::connect(&m_serial, &QSerialPort::errorOccurred, this, &PicoAshaComm::onSerialError);
    QObject::connect(&m_serial, &QSerialPort::readyRead, this, &PicoAshaComm::onSerialReadyRead);

    QObject::connect(m_ui, &PicoAshaMainWindow::hciLogPathChanged, this, &PicoAshaComm::onHciLogPathChanged);
    QObject::connect(m_ui, &PicoAshaMainWindow::hciLogActionBtnClicked, this, &PicoAshaComm::onHciLogActionBtnClicked);
    QObject::connect(m_ui, &PicoAshaMainWindow::cmdRestartBtnClicked, this, &PicoAshaComm::onCmdRestartBtnClicked);
    QObject::connect(m_ui, &PicoAshaMainWindow::cmdConnAllowedBtnClicked, this, &PicoAshaComm::onCmdConnAllowedBtnClicked);
    QObject::connect(m_ui, &PicoAshaMainWindow::cmdStreamingEnabledBtnClicked, this, &PicoAshaComm::onCmdStreamingEnabledBtnClicked);
    QObject::connect(m_ui, &PicoAshaMainWindow::cmdRemoveBondBtnClicked, this, &PicoAshaComm::onCmdRemoveBondBtnClicked);

    connect_timer.start(timer_interval);

}

PicoAshaComm::~PicoAshaComm()
{
    closeSerial();
    delete m_ui;
}

void PicoAshaComm::onConnectTimer()
{
    auto serial_ports = QSerialPortInfo::availablePorts();
    for (auto& p : serial_ports) {
        if (p.description().contains("ASHA")) {
            m_serial.setPort(p);
            if (m_serial.open(QIODevice::ReadWrite)) {
                connect_timer.stop();
                qDebug() << "Serial port opened to " << p.description();
                m_serialConnected = true;
                m_serial.setDataTerminalReady(true);
                m_ui->onSerialConnected(true);
                sendCommandPacket(
                    {
                        .cmd = asha::comm::Command::IntroPacket,
                        .cmd_status = asha::comm::CmdStatus::CmdOk,
                        .data = {}
                    }
                );
            }
        }
    }
}

void PicoAshaComm::onSerialError(QSerialPort::SerialPortError error)
{
    using enum QSerialPort::SerialPortError;
    switch(error) {
    case NoError:
        break;
    case ResourceError:
        m_serialConnected = false;
        m_serial.close();
        m_ui->onSerialConnected(false);
        m_ui->removeRemotes();
        qDebug() << "Serial port disconnected";
        connect_timer.start(timer_interval);
        break;
    default:
        qDebug() << "Serial port error: " << error;
        break;
    }
}

void PicoAshaComm::onSerialReadyRead()
{
    QByteArray const data = m_serial.readAll();
    for (const auto& b : data) {
        if (b == '\0') {
            if (m_currPacket.size() >= 4) {
                QByteArray decoded(COBS_TINYFRAME_SAFE_BUFFER_SIZE, '\0');
                size_t dec_len;
                cobs_decode(m_currPacket.data(), m_currPacket.size(), decoded.data(), decoded.capacity(), &dec_len);
                decoded.resize(dec_len);
                handleDecodedData(decoded);
            }
            m_currPacket.clear();
        } else {
            m_currPacket.append(b);
        }
    }
}

void PicoAshaComm::onHciLogPathChanged(const QString &path)
{
    m_hciLoggingPath = path;
    if (!m_hciLoggingEnabled) {
        m_ui->setHciActionBtnStart(true);
    }
}

void PicoAshaComm::onHciLogActionBtnClicked()
{
    using namespace asha::comm;
    if (m_hciLoggingEnabled) {
        if (m_hciLogFile.isOpen()) {
            m_hciLogFile.close();
        }
        m_ui->setHciActionBtnStart(!m_hciLoggingPath.isEmpty());
        sendCommandPacket(
            {
                .cmd = Command::HCIDump,
                .cmd_status = CmdStatus::CmdOk,
                .data = {.enable_hci = false}
            }
        );
    } else {
        if (m_hciLoggingPath.isEmpty()) {
            setErrMsg("Log path not set");
            return;
        }
        if (m_hciLogFile.isOpen()) {
            m_hciLogFile.close();
        }
        m_hciLogFile.setFileName(m_hciLoggingPath);
        if (!m_hciLogFile.open(QIODevice::WriteOnly)) {
            setErrMsg(m_hciLogFile.errorString());
            return;
        }

        struct {
            char id_pattern[8] = "btsnoop";
            uint32_t version = 1;
            uint32_t data_type = 1002; // H4
        } btsnoop_file_header;

        btsnoop_file_header.version = qToBigEndian(btsnoop_file_header.version);
        btsnoop_file_header.data_type = qToBigEndian(btsnoop_file_header.data_type);

        if (m_hciLogFile.write((const char*)&btsnoop_file_header, sizeof(btsnoop_file_header)) < 0) {
            setErrMsg(m_hciLogFile.errorString());
            return;
        }
        bool res = sendCommandPacket(
            {
                .cmd = Command::HCIDump,
                .cmd_status = CmdStatus::CmdOk,
                .data = {.enable_hci = true}
            }
        );
        if (res) {
            m_hciLoggingEnabled = true;
            m_ui->setHciActionBtnStop(true);
        }
    }
}

void PicoAshaComm::onCmdRestartBtnClicked()
{
    using namespace asha::comm;
    bool res = sendCommandPacket(
        {
            .cmd = Command::Restart,
            .cmd_status = CmdStatus::CmdOk,
            .data = {}
        }
        );
    if (res) {
        m_ui->onSerialConnected(false);
    }
}

void PicoAshaComm::onCmdConnAllowedBtnClicked(bool allowed)
{
    using namespace asha::comm;
    bool res = sendCommandPacket(
        {
            .cmd = Command::AllowConnect,
            .cmd_status = CmdStatus::CmdOk,
            .data = {.allow_connect = allowed}
        }
        );
    if (res) {
        m_ui->setConnectionsAllowed(allowed);
    }
}

void PicoAshaComm::onCmdStreamingEnabledBtnClicked(bool enabled)
{
    using namespace asha::comm;
    bool res = sendCommandPacket(
        {
            .cmd = Command::AudioStreaming,
            .cmd_status = CmdStatus::CmdOk,
            .data = {.audio_streaming_enabled = enabled}
        }
        );
    if (res) {
        m_ui->setAudioStreamingEnabled(enabled);
    }
}

void PicoAshaComm::onCmdRemoveBondBtnClicked()
{
    using namespace asha::comm;
    sendCommandPacket(
        {
            .cmd = Command::DeletePair,
            .cmd_status = CmdStatus::CmdOk,
            .data = {}
        }
    );
}

template<typename T>
bool assert_packet_size(size_t dec_size, const char* pkt_type, T const& pkt)
{
    size_t dec_pkt_size = dec_size - sizeof(asha::comm::HeaderPacket);
    if (dec_pkt_size < sizeof pkt) {
        qDebug() << pkt_type << ": Size mismatch! Decoded size is "
                 << dec_size << ", which is too small for the expected: " << sizeof(pkt)
                 << ", got: " << dec_pkt_size;
        return false;
    } else if (dec_pkt_size > sizeof pkt) {
        qDebug() << pkt_type << ": Warning: decoded size of: "
                 << dec_pkt_size << " is larger than the expected size of: "
                 << sizeof(pkt);
    }
    return true;
}

void PicoAshaComm::handleDecodedData(QByteArray const& decoded)
{
    using namespace asha::comm;
    HeaderPacket header(Type::Event);
    memcpy(&header, decoded.constData(), sizeof header);
    if (header.len != decoded.size()) {
        qDebug() << "Decoded packet does not match length!";
        return;
    }

    // qDebug() << "\nHeader Type: " << static_cast<uint8_t>(header.type)
    //          << "\nHeader Conn ID: " << header.conn_id
    //          << "\nHeader Len: " << header.len
    //          << "\nHeader TS: " << header.ts_ms
    //          << "\n";

    switch(header.type) {
    case Type::Intro: {
        IntroPacket intro;
        if (!assert_packet_size(decoded.size(), "IntroPacket", intro)) {
            return;
        }
        memcpy(&intro, decoded.constData() + sizeof header, sizeof intro);
        setPaFirmwareVers(QString("%1.%2.%3").arg(
                                                  QString::number(intro.pa_version.major),
                                                  QString::number(intro.pa_version.minor),
                                                  QString::number(intro.pa_version.patch))
                          );

        qDebug() << "Num connected devices: " << intro.num_connected;
        m_ui->setPicoAshaVerStr(paFirmwareVers());
        m_ui->setConnectionsAllowed(intro.test_flag(IntroFlags::conn_allowed));
        m_ui->setAudioStreamingEnabled(intro.test_flag(IntroFlags::streaming_enabled));
        m_ui->setCmdBtnsEnabled(true);
        break;
    }
    case Type::Event: {
        EventPacket pkt(EventType::ShortLog);
        if (!assert_packet_size(decoded.size(), "EventPacket", pkt)) {
            return;
        }
        memcpy(&pkt, decoded.constData() + sizeof header, sizeof pkt);
        handleEventPacket(header, pkt);
        break;
    }
    case Type::HCI:
        writeHciPacket(decoded.constData() + sizeof header, decoded.size() - sizeof header);
        break;
    case Type::RemInfo:
        RemoteInfo info;
        if (!assert_packet_size(decoded.size(), "RemoteInfo", info)) {
            return;
        }
        qDebug() << "\nGot RemoteInfo packet";
        memcpy(&info, decoded.constData() + sizeof header, sizeof info);
        m_ui->addRemote(info);
        break;
    }
}

void PicoAshaComm::handleEventPacket(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt)
{
    using namespace asha::comm;
    auto rem = m_ui->getRemote(header.conn_id);

    switch (pkt.ev_type) {

    case EventType::ShortLog:
        appendLog(QString("%1 - %2").arg(logHeader(header)).arg(pkt.data.str));
        break;
    case EventType::PicoASHAInit:
        if (checkError(header, pkt, "PicoASHAInit")) {
            return;
        }
        break;
    case EventType::DeletePair:
        if (checkError(header, pkt, "DeletePair")) {
            return;
        }
        break;
    case EventType::RemoteConnected: {
        if (checkError(header, pkt, "RemoteConnected")) {
            return;
        }
        if (rem) {
            qDebug() << "Remote with this ID already exists";
            return;
        }
        qDebug() << "Remote Connected";
        auto r = m_ui->addRemote(header.conn_id);
        if (r) {
            QByteArray addr((const char*)pkt.data.conn_info.addr, sizeof(pkt.data.conn_info.addr));
            r->setHCIHandle(pkt.data.conn_info.hci_handle);
            r->setAddr(pkt.data.conn_info.addr);
            if (cached_remote_props.contains(addr)) {
                auto const& props = cached_remote_props.value(addr);
                r->setCachedProps(props);
            }
        }
        break;
    }
    case EventType::RemoteDisconnected:
        checkError(header, pkt, "RemoteDisonnected");
        qDebug() << "Removing Conn ID: " << header.conn_id;
        if (rem) {
            auto props = rem->cachedProps();
            cached_remote_props.insert(props.addr, props);
        }
        m_ui->removeRemote(header.conn_id);
        break;
    case EventType::DiscServices:
        if (checkError(header, pkt, "DiscoverServices")) {
            return;
        }
        break;
    case EventType::PairAndBond:
        if (checkError(header, pkt, "PairAndBond")) {
            return;
        }
        if (rem) rem->setPairedBonded(true);
        break;
    case EventType::DLE:
        if (checkError(header, pkt, "DataLenExtension")) {
            return;
        }
        break;
    case EventType::DiscASHAChar:
        if (checkError(header, pkt, "DiscoverASHAChar")) {
            return;
        }
        break;
    case EventType::ROPRead:
        if (checkError(header, pkt, "ROPRead")) {
            return;
        }
        if (rem) {
            rem->setROPInfo((const char*)pkt.data.rop);
            m_ui->setRemoteOrder();
        }
        break;
    case EventType::PSMRead:
        if (checkError(header, pkt, "PSMRead")) {
            return;
        }
        if (rem) rem->setPsm(pkt.data.psm);
        break;
    case EventType::DiscGAPChar:
        if (checkError(header, pkt, "DiscoverGAP")) {
            return;
        }
        break;
    case EventType::DevNameRead:
        if (checkError(header, pkt, "DeviceNameRead")) {
            return;
        }
        if (rem) rem->setDeviceName(pkt.data.str);
        break;
    case EventType::DiscDISChar:
        if (checkError(header, pkt, "DiscoverDIS")) {
            return;
        }
        break;
    case EventType::MfgRead:
        if (checkError(header, pkt, "MfgRead")) {
            return;
        }
        if (rem) rem->setMfgName(pkt.data.str);
        break;
    case EventType::ModelRead:
        if (checkError(header, pkt, "ModelRead")) {
            return;
        }
        if (rem) rem->setModelName(pkt.data.str);
        break;
    case EventType::FWRead:
        if (checkError(header, pkt, "FirmwareRead")) {
            return;
        }
        if (rem) rem->setFwVersion(pkt.data.str);
        break;
    case EventType::L2CAPCon:
        if (checkError(header, pkt, "L2CAPConnected")) {
            return;
        }
        if (rem) rem->setL2CID(pkt.data.cid);
        break;
    case EventType::L2CAPDiscon:
        if (checkError(header, pkt, "L2CAPDisconnected")) {
            return;
        }
        if (rem) rem->setL2CID(0);
        break;
    case EventType::ASPNotEnable:
        if (checkError(header, pkt, "ASPNotificationEnable")) {
            return;
        }
        break;
    case EventType::ACPStart:
        if (checkError(header, pkt, "ACPStart")) {
            return;
        }
        break;
    case EventType::ACPStop:
        if (checkError(header, pkt, "ACPStop")) {
            return;
        }
        break;
    case EventType::ACPStatus:
        if (checkError(header, pkt, "ACPStatus")) {
            return;
        }
        break;
    case EventType::ASPStart:
        if (checkError(header, pkt, "ASPStart")) {
            return;
        }
        if (rem) rem->setAudioStreaming(true);
        break;
    case EventType::ASPStop:
        if (checkError(header, pkt, "ASPStop")) {
            return;
        }
        if (rem) rem->setAudioStreaming(false);
        break;
    case EventType::ASPError:
        if (checkError(header, pkt, "ASPError")) {
            return;
        }
        break;
    case EventType::StreamReady:
        if (checkError(header, pkt, "StreamReady")) {
            return;
        }
        break;
    case EventType::StreamPause:
        if (checkError(header, pkt, "StreamPaused")) {
            return;
        }
        break;
    case EventType::AudioVolume:
        if (checkError(header, pkt, "AudioVolume")) {
            return;
        }
        if (rem) rem->setCurrVolume(pkt.data.volume);
        break;
    }
}

bool PicoAshaComm::checkError(const asha::comm::HeaderPacket header, const asha::comm::EventPacket &pkt, QString const& state)
{
    using namespace asha::comm;
    if (pkt.status_type == StatusType::StatusSuccess) {
        appendLog(QString("%1 - [%2] Success").arg(logHeader(header)).arg(state));
        return false;
    }

    QString err = QStringLiteral("%1 - [%2] Error with status: %3");
    QString err_with_reason = QStringLiteral("%1 - [%2] Error with status: %3 and reason %4");

    QString status = "";
    QString reason = "";

    switch(pkt.status_type) {
    case StatusType::BtstackStatus:
        status = asha::bt_err_str(pkt.status);
        reason = asha::bt_err_str(pkt.reason);
        break;
    case StatusType::ATTStatus:
        status = asha::att_err_str(pkt.status);
        reason = asha::bt_err_str(pkt.reason);
        break;
    case StatusType::SMStatus:
        status = asha::att_err_str(pkt.status);
        reason = asha::sm_reason_str(pkt.reason);
        break;
    case StatusType::L2CapStatus:
        status = asha::l2cap_err_str(pkt.status);
        reason = asha::bt_err_str(pkt.reason);
        if (status == "unknown") {
            status = asha::bt_err_str(pkt.status);
        }
        break;
    case StatusType::PAStatus:
        switch(pkt.status) {
        case PAError::PAMaxConnected:
            status = "PA_MAX_CONNECTED";
            break;
        case PAError::PAASHAServiceNotFound:
            status = "PA_ASHA_SERVICE_NOT_FOUND";
            break;
        case PAError::PARuntimeSettingsErr:
            status = "PA_RUNTIME_SETTINGS_ERR";
            break;
        default:
            status = "";
        }
    case StatusType::StatusSuccess:
        break;
    }

    QString err_str;


    if (pkt.reason > 0) {
        err_str = err_with_reason.arg(logHeader(header))
                                 .arg(state)
                                 .arg(status)
                                 .arg(reason);
    } else {
        err_str = err.arg(logHeader(header))
                     .arg(state)
                     .arg(status);
    }

    appendLog(err_str);
    setRemoteError(err_str);

    return true;
}

QString PicoAshaComm::logHeader(const asha::comm::HeaderPacket header)
{
    QDateTime dt = QDateTime::fromMSecsSinceEpoch((int64_t)header.ts_ms);
    return QString("[%1] [%2]").arg(dt.toString("dd:hh:mm:ss:zzz")).arg(header.conn_id);
}

bool PicoAshaComm::appendLog(const QString &logLine)
{
    m_ui->appendLog(logLine);
    return true;
}

void PicoAshaComm::closeSerial()
{
    if (m_serial.isOpen()) {
        m_serial.setDataTerminalReady(false);
        m_serial.close();
    }
}

bool PicoAshaComm::sendCommandPacket(asha::comm::CmdPacket const& cmd_pkt)
{
    using namespace asha::comm;
    struct {
        HeaderPacket head;
        CmdPacket cmd;
    } pkt;

    pkt.head.type = Type::Cmd;
    pkt.head.conn_id = unset_conn_id;
    pkt.head.len = sizeof(pkt);
    pkt.head.ts_ms = 0;
    pkt.cmd = cmd_pkt;


    static_assert(sizeof(pkt) == sizeof(HeaderPacket) + sizeof(CmdPacket));

    uint8_t enc[1 + COBS_ENCODE_MAX(sizeof(pkt))] = {};
    enc[0] = 0;
    size_t enc_len = 0;
    cobs_encode(&pkt, sizeof(pkt), enc + 1, sizeof(enc) - 1, &enc_len);
    ++enc_len;
    if (m_serialConnected) {
        if (m_serial.write((const char*)enc, enc_len) < 0) {
            setErrMsg(m_serial.errorString());
            return false;
        }
    }
    return true;
}

void PicoAshaComm::writeHciPacket(const char *data, size_t len)
{
    if (!m_hciLoggingEnabled) {
        m_hciLoggingEnabled = true;
        m_ui->setHciActionBtnStop(true);
    } else if (m_hciLoggingEnabled && m_hciLogFile.isOpen()) {
        m_hciLogFile.write(data, len);
    }
}

void PicoAshaComm::showUI() const
{
    m_ui->show();
}

QString PicoAshaComm::paFirmwareVers() const
{
    return m_paFirmwareVers;
}

void PicoAshaComm::setPaFirmwareVers(const QString &newPaFirmwareVers)
{
    if (m_paFirmwareVers == newPaFirmwareVers)
        return;
    m_paFirmwareVers = newPaFirmwareVers;
    emit paFirmwareVersChanged();
}

QString PicoAshaComm::remoteError() const
{
    return m_remoteError;
}

void PicoAshaComm::setRemoteError(const QString &newRemoteError)
{
    if (m_remoteError == newRemoteError)
        return;
    m_remoteError = newRemoteError;
    emit remoteErrorChanged();
}

QString PicoAshaComm::errMsg() const
{
    return m_errMsg;
}

void PicoAshaComm::setErrMsg(const QString &newErrMsg)
{
    if (m_errMsg == newErrMsg)
        return;
    m_errMsg = newErrMsg;
    emit errMsgChanged();
}
