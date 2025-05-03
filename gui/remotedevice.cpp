#include <QBitArray>
#include <QByteArray>
#include "remotedevice.h"

RemoteDevice::RemoteDevice(QWidget *parent)
    : QGroupBox{parent}
{
    setupUI();
}

RemoteDevice::RemoteDevice(asha::comm::RemoteInfo const& remote, QWidget *parent)
    : QGroupBox{parent}
{
    using namespace asha::comm;
    setupUI();

    setConnID(remote.conn_id);
    setHCIHandle(remote.hci_handle);
    setAddr(remote.addr);
    setPairedBonded(remote.paired);
    setPsm(remote.psm);
    setL2CID(remote.l2cap_id);
    setDeviceName(remote.dev_name);
    setMfgName(remote.mfg_name);
    setModelName(remote.model_name);
    setFwVersion(remote.fw_vers);
    setSide(remote.side);
    setMode(remote.mode);
    setAudioStreaming(remote.audio_streaming);
    setCurrVolume(remote.curr_vol);
}

void RemoteDevice::setupUI()
{
    auto addRowToLayout = [this] (const char* header, const char* defaultVal, QLabel* val) {
        val->setText(defaultVal);
        val->setStyleSheet("font-weight: normal");
        this->m_formLayout.addRow(header, val);
    };

    setStyleSheet("font-weight: bold");
    setAlignment(Qt::AlignHCenter);
    setTitle("Side Unknown");

    addRowToLayout("Connection ID", "0", &m_connIDLabel);
    addRowToLayout("HCI Conn Handle", "0x00", &m_hciConnHandleLabel);
    addRowToLayout("Address", "00:00:00:00:00:00", &m_addrLabel);
    addRowToLayout("Paired and Bonded", "No", &m_pairedAndBondedLabel);
    addRowToLayout("Device Name", "", &m_deviceNameLabel);
    addRowToLayout("Make", "", &m_mfgNameLabel);
    addRowToLayout("Model", "", &m_modelNameLabel);
    addRowToLayout("FW Version", "", &m_fwVersionLabel);
    addRowToLayout("Mode", "Unknown", &m_modeLabel);
    addRowToLayout("24 kHz Support", "No", &m_g72224Label);
    addRowToLayout("PSM", "0", &m_psmLabel);
    addRowToLayout("L2CAP CID", "0", &m_l2CIDLabel);
    addRowToLayout("Audio Streaming", "No", &m_audioStreamingLabel);
    addRowToLayout("Current Volume", "-128", &m_currVolumeLabel);

    setLayout(&m_formLayout);
}

void RemoteDevice::setROPInfo(const char* rop)
{
    QBitArray device_cap = QBitArray::fromBits(rop + 1, 8);
    QBitArray codecs = QBitArray::fromBits(rop + 15, 16);

    setSide(device_cap.testBit(0) ? Side::Left : Side::Right);
    setMode(device_cap.testBit(1) ? Mode::Binaural : Mode::Mono);
    setG24KHz(codecs.testBit(2));
}

RemoteDevice::CachedProps RemoteDevice::cachedProps()
{
    return m_cachedProps;
}

void RemoteDevice::setCachedProps(CachedProps const& props)
{
    setDeviceName(props.deviceName);
    setModelName(props.modelName);
    setMfgName(props.mfgName);
    setFwVersion(props.fwVersion);
    setG24KHz(props.g24kHZ);
    setMode(props.mode);
    setSide(props.side);
}

void RemoteDevice::setConnID(uint16_t connID)
{
    m_connIDLabel.setText(QString::number(connID));
}

void RemoteDevice::setHCIHandle(uint16_t hciHandle)
{
    m_hciConnHandleLabel.setText(QString::number(hciHandle));
}

void RemoteDevice::setAddr(const uint8_t *addr)
{
    QByteArray addrBytes(reinterpret_cast<const char*>(addr), 6);
    m_addrLabel.setText(addrBytes.toHex(':'));
    m_cachedProps.addr = addrBytes;
}

void RemoteDevice::setPairedBonded(bool paired)
{
    m_pairedAndBondedLabel.setText(paired ? "Yes" : "No");
}

void RemoteDevice::setPsm(uint16_t psm)
{
    m_psmLabel.setText(QString::number(psm));
}

void RemoteDevice::setL2CID(uint16_t cid)
{
    m_l2CIDLabel.setText(QString::number(cid));
}

void RemoteDevice::setDeviceName(QString const& deviceName)
{
    m_deviceNameLabel.setText(deviceName);
    m_cachedProps.deviceName = deviceName;
}

void RemoteDevice::setMfgName(QString const& mfgName)
{
    m_mfgNameLabel.setText(mfgName);
    m_cachedProps.mfgName = mfgName;
}

void RemoteDevice::setModelName(QString const& modelName)
{
    m_modelNameLabel.setText(modelName);
    m_cachedProps.modelName = modelName;
}

void RemoteDevice::setFwVersion(QString const& fwVersion)
{
    m_fwVersionLabel.setText(fwVersion);
    m_cachedProps.fwVersion = fwVersion;
}

void RemoteDevice::setSide(Side side)
{
    QString sideStr = side == Side::Left ? "Left"
                      : side == Side::Right ? "Right" : "Side Unknown";
    setTitle(sideStr);
    m_cachedProps.side = side;
}

void RemoteDevice::setSide(asha::comm::CSide side)
{
    using namespace asha::comm;
    Side s = side == CSide::Left ? Side::Left
                : side == CSide::Right ? Side::Right : Side::SideUnset;
    setSide(s);
}

void RemoteDevice::setMode(Mode mode)
{
    QString modeStr = mode == Mode::Mono ? "Mono"
                      : mode == Mode::Binaural ? "Binaural" : "Unknown";
    m_modeLabel.setText(modeStr);
    m_cachedProps.mode = mode;
}

void RemoteDevice::setMode(asha::comm::CMode mode)
{
    using namespace asha::comm;
    Mode m = mode == CMode::Mono ? Mode::Mono
                : mode == CMode::Binaural ? Mode::Binaural : Mode::ModeUnset;
    setMode(m);
}

void RemoteDevice::setAudioStreaming(bool audioStreaming)
{
    m_audioStreamingLabel.setText(audioStreaming ? "Yes" : "No");
}

void RemoteDevice::setCurrVolume(int8_t currVol)
{
    m_currVolumeLabel.setText(QString::number(currVol));
}

void RemoteDevice::setG24KHz(bool enabled)
{
    m_g72224Label.setText(enabled ? "Supported" : "Unsupported");
    m_cachedProps.g24kHZ = enabled;
}
