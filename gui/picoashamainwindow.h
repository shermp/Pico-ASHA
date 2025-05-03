#ifndef PICOASHAMAINWINDOW_H
#define PICOASHAMAINWINDOW_H

#include <QMainWindow>
#include <QFrame>
#include <QPlainTextEdit>
#include <QLabel>

#include "remotedevice.h"
#include "asha_comms.hpp"

class PicoAshaMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PicoAshaMainWindow(QWidget *parent = nullptr);
    ~PicoAshaMainWindow();

    QFrame* remoteFrame() const;
    void appendLog(QString const& logLine);

    RemoteDevice* addRemote(uint16_t connID);
    RemoteDevice* addRemote(asha::comm::RemoteInfo const& remote);
    void removeRemote(uint16_t connID);
    void removeRemotes();

    void setRemoteOrder();

    void setSerialConnectedStatus(QString const& statusStr);

    RemoteDevice* getRemote(uint16_t connID);

private:
    QWidget* m_mainWidget;
    QFrame* m_remoteFrame;
    QPlainTextEdit* m_logWidget;
    QLabel* m_serialConnectedStatus;

    RemoteDevice* addRemote(RemoteDevice* remote);
};

#endif // PICOASHAMAINWINDOW_H
