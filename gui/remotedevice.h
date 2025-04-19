#ifndef REMOTEDEVICE_H
#define REMOTEDEVICE_H

#include <QObject>
#include <QByteArray>
#include <QString>
#include <qqmlintegration.h>
#include "asha_comms.hpp"

class RemoteDevice : public QObject
{
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(bool connected READ connected WRITE setConnected NOTIFY connectedChanged FINAL)
    Q_PROPERTY(unsigned int connID READ connID WRITE setConnID NOTIFY connIDChanged FINAL)
    Q_PROPERTY(unsigned int hciConnHandle READ hciConnHandle WRITE setHciConnHandle NOTIFY hciConnHandleChanged FINAL)
    Q_PROPERTY(QByteArray addr READ addr WRITE setAddr NOTIFY addrChanged FINAL)

    Q_PROPERTY(bool pairedAndBonded READ pairedAndBonded WRITE setPairedAndBonded NOTIFY pairedAndBondedChanged FINAL)

    Q_PROPERTY(QString deviceName READ deviceName WRITE setDeviceName NOTIFY deviceNameChanged FINAL)
    Q_PROPERTY(QString mfgName READ mfgName WRITE setMfgName NOTIFY mfgNameChanged FINAL)
    Q_PROPERTY(QString modelName READ modelName WRITE setModelName NOTIFY modelNameChanged FINAL)
    Q_PROPERTY(QString fwVersion READ fwVersion WRITE setFwVersion NOTIFY fwVersionChanged FINAL)
    Q_PROPERTY(Side side READ side WRITE setSide NOTIFY sideChanged FINAL)
    Q_PROPERTY(Mode mode READ mode WRITE setMode NOTIFY modeChanged FINAL)
    Q_PROPERTY(bool g16kHZ READ g16kHZ WRITE setG16kHZ NOTIFY g16kHZChanged FINAL)
    Q_PROPERTY(bool g24kHZ READ g24kHZ WRITE setG24kHZ NOTIFY g24kHZChanged FINAL)
    Q_PROPERTY(unsigned int psm READ psm WRITE setPsm NOTIFY psmChanged FINAL)
    Q_PROPERTY(unsigned int l2capCID READ l2capCID WRITE setL2capCID NOTIFY l2capCIDChanged FINAL)

    Q_PROPERTY(bool audioStreaming READ audioStreaming WRITE setAudioStreaming NOTIFY audioStreamingChanged FINAL)
    Q_PROPERTY(int currVolume READ currVolume WRITE setCurrVolume NOTIFY currVolumeChanged FINAL)

public:
    enum Side {Left = static_cast<int>(asha::comm::CSide::Left), Right, SideUnset};
    enum Mode {Mono = static_cast<int>(asha::comm::CMode::Mono), Binaural, ModeUnset};

    Q_ENUM(Side);
    Q_ENUM(Mode);

    struct CachedProps {
        QString deviceName;
        QString mfgName;
        QString modelName;
        QString fwVersion;
        Side side;
        Mode mode;
        bool g16kHZ;
        bool g24kHZ;
    };

    explicit RemoteDevice(QObject *parent = nullptr);
    explicit RemoteDevice(asha::comm::RemoteInfo const& remote, QObject *parent = nullptr);

    bool connected() const;
    void setConnected(bool newConnected);

    unsigned int hciConnHandle() const;
    void setHciConnHandle(unsigned int newHciConnHandle);

    QByteArray addr() const;
    void setAddr(const QByteArray &newAddr);

    bool pairedAndBonded() const;
    void setPairedAndBonded(bool newPairedAndBonded);

    QString deviceName() const;
    void setDeviceName(const QString &newDeviceName);

    QString mfgName() const;
    void setMfgName(const QString &newMfgName);

    QString modelName() const;
    void setModelName(const QString &newModelName);

    QString fwVersion() const;
    void setFwVersion(const QString &newFwVersion);

    Side side() const;
    void setSide(const Side &newSide);

    Mode mode() const;
    void setMode(const Mode &newMode);

    bool g16kHZ() const;
    void setG16kHZ(bool newG16kHZ);

    bool g24kHZ() const;
    void setG24kHZ(bool newG24kHZ);

    unsigned int psm() const;
    void setPsm(unsigned int newPsm);

    unsigned int l2capCID() const;
    void setL2capCID(unsigned int newL2capCID);

    bool audioStreaming() const;
    void setAudioStreaming(bool newAudioStreaming);

    int currVolume() const;
    void setCurrVolume(int newCurrVolume);

    unsigned int connID() const;
    void setConnID(unsigned int newConnID);

    void setROPInfo(const char* rop);

    CachedProps cachedProps();
    void setCachedProps(CachedProps const& props);

signals:
    void connectedChanged();
    void hciConnHandleChanged();

    void addrChanged();

    void pairedAndBondedChanged();

    void deviceNameChanged();

    void mfgNameChanged();

    void modelNameChanged();

    void fwVersionChanged();

    void sideChanged();

    void modeChanged();

    void g16kHZChanged();

    void g24kHZChanged();

    void psmChanged();

    void l2capCIDChanged();

    void audioStreamingChanged();

    void currVolumeChanged();

    void connIDChanged();

private:
    bool m_connected = false;
    unsigned int m_hciConnHandle = 0U;
    QByteArray m_addr = {};
    bool m_pairedAndBonded = false;
    QString m_deviceName = {};
    QString m_mfgName = {};
    QString m_modelName = {};
    QString m_fwVersion = {};
    Side m_side = Side::SideUnset;
    Mode m_mode = {Mode::ModeUnset};
    bool m_g16kHZ = false;
    bool m_g24kHZ = false;
    unsigned int m_psm = 0;
    unsigned int m_l2capCID = 0;
    bool m_audioStreaming = false;
    int m_currVolume = -128;
    unsigned int m_connID = 0;
};

#endif // REMOTEDEVICE_H
