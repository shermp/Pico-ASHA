#ifndef PICOASHAMAINWINDOW_H
#define PICOASHAMAINWINDOW_H

#include <QMainWindow>
#include <QFrame>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QList>
#include <QComboBox>
#include <QSpinBox>

#include "remotedevice.h"
#include "pairdialog.h"
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

    void onSerialConnected(bool connected);

    void setCmdBtnsEnabled(bool enabled);
    void setUSBWidgetsEnabled(bool enabled);

    void setPicoAshaVerStr(QString const& version);
    void setConnectionsAllowed(bool allowed);
    void setAudioStreamingEnabled(bool enabled);
    void setUSBInfo(asha::comm::USBInfo const& usb_info);
    void setUSBSettingsBtnState();

    void setHciActionBtnStart(bool enabled);
    void setHciActionBtnStop(bool enabled);

    void onAdPacketReceived(asha::comm::AdvertisingPacket const& ad_pkt);

public slots:
    void onPairDialogAcceptedRejected();

signals:
    void hciLogPathChanged(QString const& path);
    void hciLogActionBtnClicked();
    void cmdRestartBtnClicked();
    void cmdConnAllowedBtnClicked(bool allowed);
    void cmdStreamingEnabledBtnClicked(bool enabled);
    void cmdRemoveBondBtnClicked();
    void usbSettingsBtnClicked(asha::comm::USBInfo const& usb_info);
    void pairWithAddress(QByteArray const& addr, uint8_t addr_type);

private:
    QWidget* m_mainWidget;
    QFrame* m_remoteFrame;
    QPlainTextEdit* m_logWidget;
    QLabel* m_serialConnectedStatus;
    QList<RemoteDevice*> m_remotes;

    QPushButton* m_cmdRestartBtn;
    QPushButton* m_cmdConnAllowedBtn;
    QPushButton* m_cmdStreamingEnabledBtn;
    QPushButton* m_cmdRemoveBondBtn;

    QComboBox*   m_USBUacVersCombo;
    QSpinBox*    m_USBVolMinSpin;
    QSpinBox*    m_USBVolMaxSpin;
    QPushButton* m_USBSettingsBtn;

    QPushButton* m_hciActionBtn;
    QPushButton* m_hciPathBtn;
    QLabel* m_hciPathLbl;

    PairDialog* m_currPairDlg;

    bool m_serialConnected;
    bool m_connectionsAllowed;
    bool m_streamingEnabled;

    asha::comm::USBInfo m_usbInfo;
    asha::comm::USBInfo fromUsbWidgets();
};

#endif // PICOASHAMAINWINDOW_H
