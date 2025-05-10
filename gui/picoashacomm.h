#ifndef PICOASHACOMM_H
#define PICOASHACOMM_H

#include <QObject>
#include <QByteArray>
#include <QFile>
#include <QSerialPort>
#include <QString>
#include <QStringListModel>
#include <QTimer>
#include <QUrl>

#include <asha_comms.hpp>
#include "picoashamainwindow.h"

class PicoAshaComm : public QObject
{
    Q_OBJECT

public:
    explicit PicoAshaComm(QObject *parent = nullptr);
    virtual ~PicoAshaComm();

    void showUI() const;

    QString paFirmwareVers() const;
    void setPaFirmwareVers(const QString &newPaFirmwareVers);

    QString remoteError() const;
    void setRemoteError(const QString &newRemoteError);

    QString errMsg() const;
    void setErrMsg(const QString &newErrMsg);

private:
    void handleDecodedData(QByteArray const& decoded);
    void handleEventPacket(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt);
    bool checkError(asha::comm::HeaderPacket const header, asha::comm::EventPacket const& pkt, QString const& state);

    QString logHeader(asha::comm::HeaderPacket const header);
    bool appendLog(QString const& logLine);

    void closeSerial();

    bool sendCommandPacket(asha::comm::CmdPacket const& cmd_pkt);

    void writeHciPacket(const char* data, size_t len);

    QSerialPort m_serial;
    QByteArray m_currPacket;

    QTimer connect_timer;

    PicoAshaMainWindow* m_ui;

    QFile m_hciLogFile;

    bool m_serialConnected;

    QString m_paFirmwareVers;

    QString m_remoteError;

    QString m_hciLoggingPath;

    bool m_hciLoggingEnabled;

    QString m_errMsg;

signals:

    void paFirmwareVersChanged();

    void remoteErrorChanged();

    void hciLoggingPathChanged();

    void hciLoggingEnabledChanged();

    void errMsgChanged();

public slots:
    void onConnectTimer();
    void onSerialError(QSerialPort::SerialPortError error);
    void onSerialReadyRead();
    void onHciLogPathChanged(QString const& path);
    void onHciLogActionBtnClicked();
    void onCmdRestartBtnClicked();
    void onCmdConnAllowedBtnClicked(bool allowed);
};

#endif // PICOASHACOMM_H
