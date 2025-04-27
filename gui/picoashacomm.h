#ifndef PICOASHACOMM_H
#define PICOASHACOMM_H

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QByteArray>
#include <QFile>
#include <QSerialPort>
#include <QString>
#include <QStringListModel>
#include <QTimer>
#include <QUrl>

#include <asha_comms.hpp>

#include "remotedevicemodel.h"

class PicoAshaComm : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool serialConnected READ serialConnected WRITE setSerialConnected NOTIFY serialConnectedChanged FINAL)
    Q_PROPERTY(QString paFirmwareVers READ paFirmwareVers WRITE setPaFirmwareVers NOTIFY paFirmwareVersChanged FINAL)
    Q_PROPERTY(QString remoteError READ remoteError WRITE setRemoteError NOTIFY remoteErrorChanged FINAL)
    Q_PROPERTY(bool hciLoggingEnabled READ hciLoggingEnabled WRITE setHciLoggingEnabled NOTIFY hciLoggingEnabledChanged FINAL)
    Q_PROPERTY(QUrl hciLoggingPath READ hciLoggingPath WRITE setHciLoggingPath NOTIFY hciLoggingPathChanged FINAL)
    Q_PROPERTY(QString errMsg READ errMsg WRITE setErrMsg NOTIFY errMsgChanged FINAL)

    QML_ELEMENT
    QML_SINGLETON

public:
    explicit PicoAshaComm(QObject *parent = nullptr);
    virtual ~PicoAshaComm();

    bool serialConnected() const;
    void setSerialConnected(bool newSerialConnected);

    QString paFirmwareVers() const;
    void setPaFirmwareVers(const QString &newPaFirmwareVers);

    Q_INVOKABLE RemoteDeviceModel* remoteModelList();

    Q_INVOKABLE QStringListModel* logModelList();

    QString remoteError() const;
    void setRemoteError(const QString &newRemoteError);

    QUrl hciLoggingPath() const;
    void setHciLoggingPath(const QUrl &newHciLoggingPath);

    bool hciLoggingEnabled() const;
    void setHciLoggingEnabled(bool newHciLoggingEnabled);

    QString errMsg() const;
    void setErrMsg(const QString &newErrMsg);

private:
    void handleDecodedData(QByteArray const& decoded);
    void handleEventPacket(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt);
    bool checkError(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt, QString const& state);

    QString logHeader(asha::comm::HeaderPacket const header);
    bool appendLog(QString const& logLine);

    void closeSerial();

    void handleHciLogChanged();
    void sendCommandPacket(asha::comm::CmdPacket const& cmd_pkt);
    void writeHciPacket(const char* data, size_t len);

    QSerialPort m_serial;
    QByteArray m_currPacket;

    QTimer connect_timer;

    QFile m_hciLogFile;

    bool m_serialConnected;

    QString m_paFirmwareVers;

    RemoteDeviceModel m_remoteModelList;
    QStringListModel m_logModelList;

    QString m_remoteError;

    QUrl m_hciLoggingPath;

    bool m_hciLoggingEnabled;

    QString m_errMsg;

signals:
    void serialConnectedChanged();

    void paFirmwareVersChanged();

    void remoteModelListChanged();

    void remoteErrorChanged();

    void hciLoggingPathChanged();

    void hciLoggingEnabledChanged();

    void errMsgChanged();

public slots:
    void onConnectTimer();
    void onSerialError(QSerialPort::SerialPortError error);
    void onSerialReadyRead();
};

#endif // PICOASHACOMM_H
