#ifndef PICOASHAMAINWINDOW_H
#define PICOASHAMAINWINDOW_H

#include <QMainWindow>
#include <QFrame>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QList>

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

    RemoteDevice* addRemote(uint16_t connID, const asha::comm::RemoteInfo* remote = nullptr);
    RemoteDevice* addRemote(asha::comm::RemoteInfo const& remote);
    void removeRemote(uint16_t connID);
    void removeRemotes();

    void setRemoteOrder();

    RemoteDevice* getRemote(uint16_t connID);

    void setHciActionBtnStart(bool enabled);
    void setHciActionBtnStop(bool enabled);

public slots:
    void onSerialConnected(bool connected, const QString &statusStr = "");

signals:
    void hciLogPathChanged(QString const& path);
    void hciLogActionBtnClicked();
    void cmdRestartBtnClicked();

private:
    QWidget* m_mainWidget;
    QFrame* m_remoteFrame;
    QPlainTextEdit* m_logWidget;
    QLabel* m_serialConnectedStatus;
    QList<RemoteDevice*> m_remotes;

    QPushButton* m_cmdRestartBtn;

    QPushButton* m_hciActionBtn;
    QPushButton* m_hciPathBtn;
    QLabel* m_hciPathLbl;
};

#endif // PICOASHAMAINWINDOW_H
