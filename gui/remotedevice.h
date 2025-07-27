#ifndef REMOTEDEVICE_H
#define REMOTEDEVICE_H

#include <QWidget>
#include <QGroupBox>
#include <QByteArray>
#include <QString>
#include <QLabel>
#include <QFormLayout>
#include "asha_comms.hpp"

class RemoteDevice : public QGroupBox
{
    Q_OBJECT

public:
    enum Side {Left = static_cast<int>(asha::comm::CSide::Left), Right, SideUnset};
    enum Mode {Mono = static_cast<int>(asha::comm::CMode::Mono), Binaural, ModeUnset};

    Q_ENUM(Side);
    Q_ENUM(Mode);

    struct CachedProps {
        QByteArray addr;
        QString deviceName;
        QString mfgName;
        QString modelName;
        QString fwVersion;
        QString swVersion;
        Side side;
        Mode mode;
        bool g24kHZ;
    };

    struct LabelPair {
        QLabel* header = nullptr;
        QLabel* value = nullptr;
    };

    explicit RemoteDevice(QWidget *parent = nullptr);

    void setRemoteInfo(asha::comm::RemoteInfo const& remote);

    void setROPInfo(const char* rop);

    CachedProps cachedProps();
    void setCachedProps(CachedProps const& props);

    void setConnID(uint16_t connID);
    void setHCIHandle(uint16_t hciHandle);
    void setAddr(const uint8_t* addr);
    void setPairedBonded(bool paired);
    void setPsm(uint16_t psm);
    void setL2CID(uint16_t cid);
    void setDeviceName(QString const& deviceName);
    void setMfgName(QString const& mfgName);
    void setModelName(QString const& modelName);
    void setFwVersion(QString const& fwVersion);
    void setSwVersion(QString const& swVersion);
    void setSide(Side side);
    void setSide(asha::comm::CSide side);
    void setMode(Mode mode);
    void setMode(asha::comm::CMode mode);
    void setAudioStreaming(bool audioStreaming);
    void setCurrVolume(int8_t currVol);
    void setG24KHz(bool enabled);

    void setDefaultValues();
    bool isDefaultValues();
    void setGreyedOut(bool enabled);

signals:

private:
    void setupUI();

    CachedProps m_cachedProps;
    QFormLayout m_formLayout;

    QLabel m_connIDLabel;
    QLabel m_hciConnHandleLabel;
    QLabel m_addrLabel;
    QLabel m_pairedAndBondedLabel;
    QLabel m_deviceNameLabel;
    QLabel m_mfgNameLabel;
    QLabel m_modelNameLabel;
    QLabel m_fwVersionLabel;
    QLabel m_swVersionLabel;
    QLabel m_modeLabel;
    QLabel m_g72224Label;
    QLabel m_psmLabel;
    QLabel m_l2CIDLabel;
    QLabel m_audioStreamingLabel;
    QLabel m_currVolumeLabel;

    bool greyedOut;
};

#endif // REMOTEDEVICE_H
