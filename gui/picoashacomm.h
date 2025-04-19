#ifndef PICOASHACOMM_H
#define PICOASHACOMM_H

#include <QtQmlIntegration/qqmlintegration.h>

#include <QObject>
#include <QByteArray>
#include <QSerialPort>
#include <QString>
#include <QStringListModel>
#include <QTimer>

#include <asha_comms.hpp>

#include "remotedevicemodel.h"

class PicoAshaComm : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool serialConnected READ serialConnected WRITE setSerialConnected NOTIFY serialConnectedChanged FINAL)
    Q_PROPERTY(QString paFirmwareVers READ paFirmwareVers WRITE setPaFirmwareVers NOTIFY paFirmwareVersChanged FINAL)
    Q_PROPERTY(QString remoteError READ remoteError WRITE setRemoteError NOTIFY remoteErrorChanged FINAL)

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

private:
    void handleDecodedData(QByteArray const& decoded);
    void handleEventPacket(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt);
    bool checkError(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt, QString const& state);

    QString logHeader(asha::comm::HeaderPacket const header);
    bool appendLog(QString const& logLine);

    void closeSerial();

    QSerialPort m_serial;
    QByteArray m_currPacket;

    QTimer connect_timer;

    bool m_serialConnected;

    QString m_paFirmwareVers;

    RemoteDeviceModel m_remoteModelList;
    QStringListModel m_logModelList;

    QString m_remoteError;

signals:
    void serialConnectedChanged();

    void paFirmwareVersChanged();

    void remoteModelListChanged();

    void remoteErrorChanged();

public slots:
    void onConnectTimer();
    void onSerialError(QSerialPort::SerialPortError error);
    void onSerialReadyRead();
};

#endif // PICOASHACOMM_H
