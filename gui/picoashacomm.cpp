#include <QDateTime>
#include <QDebug>
#include <QObject>
#include <QMap>
#include <QSerialPortInfo>

#include <nanocobs/cobs.h>

#include "bt_status_err.hpp"

#include "picoashacomm.h"

constexpr int timer_interval = 1000;

QMap<QByteArray, RemoteDevice::CachedProps> cached_remote_props;

PicoAshaComm::PicoAshaComm(QObject *parent)
    : QObject{parent}, m_serial(this), connect_timer(this)
{
    m_serial.setBaudRate(QSerialPort::Baud115200);
    m_serial.setDataBits(QSerialPort::Data8);
    m_serial.setStopBits(QSerialPort::OneStop);
    m_serial.setParity(QSerialPort::NoParity);
    m_serial.setFlowControl(QSerialPort::HardwareControl);

    QObject::connect(&connect_timer, &QTimer::timeout, this, &PicoAshaComm::onConnectTimer);
    QObject::connect(&m_serial, &QSerialPort::errorOccurred, this, &PicoAshaComm::onSerialError);
    QObject::connect(&m_serial, &QSerialPort::readyRead, this, &PicoAshaComm::onSerialReadyRead);

    connect_timer.start(timer_interval);
}

PicoAshaComm::~PicoAshaComm()
{
    closeSerial();
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
                m_serial.setDataTerminalReady(true);
                //m_serial.write("\r\n");
                setSerialConnected(true);
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
        m_serial.close();
        setSerialConnected(false);
        qDebug() << "Serial port disconnected";
        m_remoteModelList.reset();
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

template<typename T>
bool assert_packet_size(size_t dec_size, const char* pkt_type, T const& pkt)
{
    if (dec_size - sizeof(asha::comm::HeaderPacket) != sizeof pkt) {
        qDebug() << pkt_type << ": Size mismatch! Decoded size is "
                 << dec_size << ", expected: " << sizeof(pkt)
                 << ", got: " << dec_size - sizeof(asha::comm::HeaderPacket);
        return false;
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
        setPaFirmwareVers(QString("%1.%2.%3").arg(intro.pa_version.major).arg(intro.pa_version.minor).arg(intro.pa_version.patch));
        qDebug() << "Num connected devices: " << intro.num_connected;
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
        break;
    case Type::RemInfo:
        RemoteInfo info;
        if (!assert_packet_size(decoded.size(), "RemoteInfo", info)) {
            return;
        }
        qDebug() << "\nGot RemoteInfo packet";
        memcpy(&info, decoded.constData() + sizeof header, sizeof info);
        RemoteDevice* dev = new RemoteDevice(info);
        if (!remoteModelList()->insert(dev)) {
            delete dev;
        }
        break;
    }
}

void PicoAshaComm::handleEventPacket(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt)
{
    using namespace asha::comm;
    auto rem = remoteModelList()->getByConnID(header.conn_id);

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
        qDebug() << "Remote Connected";
        RemoteDevice* dev = new RemoteDevice();
        QByteArray addr((const char*)pkt.data.conn_info.addr, sizeof(pkt.data.conn_info.addr));
        dev->setConnID(header.conn_id);
        dev->setConnected(true);
        dev->setHciConnHandle(pkt.data.conn_info.hci_handle);
        dev->setAddr(addr);
        if (cached_remote_props.contains(addr)) {
            RemoteDevice::CachedProps const& props = cached_remote_props.value(addr);
            dev->setCachedProps(props);
        }
        if (!remoteModelList()->insert(dev)) {
            delete dev;
        }
        break;
    }
    case EventType::RemoteDisconnected:
        checkError(header, pkt, "RemoteDisonnected");
        qDebug() << "Removing Conn ID: " << header.conn_id;
        if (rem) {
            RemoteDevice::CachedProps props = rem->cachedProps();
            QByteArray addr = rem->addr();
            cached_remote_props.insert(addr, props);
        }
        remoteModelList()->remove(header.conn_id);
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
        if (rem) rem->setPairedAndBonded(true);
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
        if (rem) rem->setROPInfo((const char*)pkt.data.rop);
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
        if (rem) rem->setL2capCID(pkt.data.cid);
        break;
    case EventType::L2CAPDiscon:
        if (checkError(header, pkt, "L2CAPDisconnected")) {
            return;
        }
        if (rem) rem->setL2capCID(0);
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
    if (m_logModelList.insertRow(m_logModelList.rowCount())) {
        QModelIndex index = m_logModelList.index(m_logModelList.rowCount() - 1, 0);
        return m_logModelList.setData(index, logLine);
    }
    return false;
}

void PicoAshaComm::closeSerial()
{
    if (m_serial.isOpen()) {
        m_serial.setDataTerminalReady(false);
        m_serial.close();
    }
}

bool PicoAshaComm::serialConnected() const
{
    return m_serialConnected;
}

void PicoAshaComm::setSerialConnected(bool newSerialConnected)
{
    if (m_serialConnected == newSerialConnected)
        return;
    m_serialConnected = newSerialConnected;
    emit serialConnectedChanged();
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

RemoteDeviceModel* PicoAshaComm::remoteModelList()
{
    return &m_remoteModelList;
}

QStringListModel *PicoAshaComm::logModelList()
{
    return &m_logModelList;
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
