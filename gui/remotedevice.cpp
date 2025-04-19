#include <QBitArray>
#include "remotedevice.h"

RemoteDevice::RemoteDevice(QObject *parent)
    : QObject{parent}
{}

RemoteDevice::RemoteDevice(const asha::comm::RemoteInfo &remote, QObject *parent)
    : QObject{parent}
{
    using namespace asha::comm;

    setConnID(remote.conn_id);
    setHciConnHandle(remote.hci_handle);
    setAddr(QByteArray((const char*)remote.addr, sizeof(remote.addr)));
    setConnected(remote.connected);
    setPairedAndBonded(remote.paired);
    setPsm(remote.psm);
    setL2capCID(remote.l2cap_id);
    setDeviceName(QString(remote.dev_name));
    setMfgName(QString(remote.mfg_name));
    setModelName(QString(remote.model_name));
    setFwVersion(QString(remote.fw_vers));
    setSide(static_cast<RemoteDevice::Side>(remote.side));
    setMode(static_cast<RemoteDevice::Mode>(remote.mode));
    setAudioStreaming(remote.audio_streaming);
    setCurrVolume(remote.curr_vol);
}

bool RemoteDevice::connected() const
{
    return m_connected;
}

void RemoteDevice::setConnected(bool newConnected)
{
    if (m_connected == newConnected)
        return;
    m_connected = newConnected;
    emit connectedChanged();
}

unsigned int RemoteDevice::hciConnHandle() const
{
    return m_hciConnHandle;
}

void RemoteDevice::setHciConnHandle(unsigned int newHciConnHandle)
{
    if (m_hciConnHandle == newHciConnHandle)
        return;
    m_hciConnHandle = newHciConnHandle;
    emit hciConnHandleChanged();
}

QByteArray RemoteDevice::addr() const
{
    return m_addr;
}

void RemoteDevice::setAddr(const QByteArray &newAddr)
{
    if (m_addr == newAddr)
        return;
    m_addr = newAddr;
    emit addrChanged();
}

bool RemoteDevice::pairedAndBonded() const
{
    return m_pairedAndBonded;
}

void RemoteDevice::setPairedAndBonded(bool newPairedAndBonded)
{
    if (m_pairedAndBonded == newPairedAndBonded)
        return;
    m_pairedAndBonded = newPairedAndBonded;
    emit pairedAndBondedChanged();
}

QString RemoteDevice::deviceName() const
{
    return m_deviceName;
}

void RemoteDevice::setDeviceName(const QString &newDeviceName)
{
    if (m_deviceName == newDeviceName)
        return;
    m_deviceName = newDeviceName;
    emit deviceNameChanged();
}

QString RemoteDevice::mfgName() const
{
    return m_mfgName;
}

void RemoteDevice::setMfgName(const QString &newMfgName)
{
    if (m_mfgName == newMfgName)
        return;
    m_mfgName = newMfgName;
    emit mfgNameChanged();
}

QString RemoteDevice::modelName() const
{
    return m_modelName;
}

void RemoteDevice::setModelName(const QString &newModelName)
{
    if (m_modelName == newModelName)
        return;
    m_modelName = newModelName;
    emit modelNameChanged();
}

QString RemoteDevice::fwVersion() const
{
    return m_fwVersion;
}

void RemoteDevice::setFwVersion(const QString &newFwVersion)
{
    if (m_fwVersion == newFwVersion)
        return;
    m_fwVersion = newFwVersion;
    emit fwVersionChanged();
}

RemoteDevice::Side RemoteDevice::side() const
{
    return m_side;
}

void RemoteDevice::setSide(const RemoteDevice::Side &newSide)
{
    if (m_side == newSide)
        return;
    m_side = newSide;
    emit sideChanged();
}

RemoteDevice::Mode RemoteDevice::mode() const
{
    return m_mode;
}

void RemoteDevice::setMode(const RemoteDevice::Mode &newMode)
{
    if (m_mode == newMode)
        return;
    m_mode = newMode;
    emit modeChanged();
}

bool RemoteDevice::g16kHZ() const
{
    return m_g16kHZ;
}

void RemoteDevice::setG16kHZ(bool newG16kHZ)
{
    if (m_g16kHZ == newG16kHZ)
        return;
    m_g16kHZ = newG16kHZ;
    emit g16kHZChanged();
}

bool RemoteDevice::g24kHZ() const
{
    return m_g24kHZ;
}

void RemoteDevice::setG24kHZ(bool newG24kHZ)
{
    if (m_g24kHZ == newG24kHZ)
        return;
    m_g24kHZ = newG24kHZ;
    emit g24kHZChanged();
}

unsigned int RemoteDevice::psm() const
{
    return m_psm;
}

void RemoteDevice::setPsm(unsigned int newPsm)
{
    if (m_psm == newPsm)
        return;
    m_psm = newPsm;
    emit psmChanged();
}

unsigned int RemoteDevice::l2capCID() const
{
    return m_l2capCID;
}

void RemoteDevice::setL2capCID(unsigned int newL2capCID)
{
    if (m_l2capCID == newL2capCID)
        return;
    m_l2capCID = newL2capCID;
    emit l2capCIDChanged();
}

bool RemoteDevice::audioStreaming() const
{
    return m_audioStreaming;
}

void RemoteDevice::setAudioStreaming(bool newAudioStreaming)
{
    if (m_audioStreaming == newAudioStreaming)
        return;
    m_audioStreaming = newAudioStreaming;
    emit audioStreamingChanged();
}

int RemoteDevice::currVolume() const
{
    return m_currVolume;
}

void RemoteDevice::setCurrVolume(int newCurrVolume)
{
    if (m_currVolume == newCurrVolume)
        return;
    m_currVolume = newCurrVolume;
    emit currVolumeChanged();
}

unsigned int RemoteDevice::connID() const
{
    return m_connID;
}

void RemoteDevice::setConnID(unsigned int newConnID)
{
    if (m_connID == newConnID)
        return;
    m_connID = newConnID;
    emit connIDChanged();
}

void RemoteDevice::setROPInfo(const char* rop)
{
    QBitArray device_cap = QBitArray::fromBits(rop + 1, 8);
    QBitArray codecs = QBitArray::fromBits(rop + 15, 16);

    setSide(device_cap.testBit(0) ? Side::Left : Side::Right);
    setMode(device_cap.testBit(1) ? Mode::Binaural : Mode::Mono);
    setG16kHZ(codecs.testBit(1));
    setG24kHZ(codecs.testBit(2));
}

RemoteDevice::CachedProps RemoteDevice::cachedProps()
{
    CachedProps props;
    props.deviceName = deviceName();
    props.modelName = modelName();
    props.mfgName = mfgName();
    props.fwVersion = fwVersion();
    props.g16kHZ = g16kHZ();
    props.g24kHZ = g24kHZ();
    props.mode = mode();
    props.side = side();
    return props;
}

void RemoteDevice::setCachedProps(CachedProps const& props)
{
    setDeviceName(props.deviceName);
    setModelName(props.modelName);
    setMfgName(props.mfgName);
    setFwVersion(props.fwVersion);
    setG16kHZ(props.g16kHZ);
    setG24kHZ(props.g24kHZ);
    setMode(props.mode);
    setSide(props.side);
}

