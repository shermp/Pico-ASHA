#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QFrame>
#include <QMessageBox>
#include <QStatusBar>
#include <QVariant>

#include "picoashamainwindow.h"
#include "remotedevice.h"

static QString widgetNameFromConnID(uint16_t connID)
{
    return QString("remote_connid_%1").arg(connID);
}

PicoAshaMainWindow::PicoAshaMainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_currPairDlg = nullptr;
    m_mainWidget = new QWidget;
    auto mainVBox = new QVBoxLayout;

    m_remoteFrame = new QFrame;
    auto remoteHBox = new QHBoxLayout;
    remoteHBox->addStretch(1);

    RemoteDevice* rem1 = new RemoteDevice;
    RemoteDevice* rem2 = new RemoteDevice;

    m_remotes.append(rem1);
    m_remotes.append(rem2);

    for (auto r : std::as_const(m_remotes)) {
        remoteHBox->addWidget(r);
    }

    remoteHBox->addStretch(1);
    m_remoteFrame->setLayout(remoteHBox);
    mainVBox->addWidget(m_remoteFrame);

    auto cmdGroup = new QGroupBox("Send Commands");
    auto cmdLayout = new QHBoxLayout;
    cmdLayout->addStretch();

    m_cmdStreamingEnabledBtn = new QPushButton;
    m_cmdStreamingEnabledBtn->setToolTip("Stop or start ASHA streaming.\n"
                                         "This can be useful to restart audio streaming if you "
                                         "encounter stereo sync issues.");
    cmdLayout->addWidget(m_cmdStreamingEnabledBtn);

    m_cmdConnAllowedBtn = new QPushButton;
    m_cmdConnAllowedBtn->setToolTip("Allow or deny hearing aids from connecting to Pico-ASHA.\n"
                                    "This can be useful if you want to stream from another "
                                    "device such as a mobile phone.");
    cmdLayout->addWidget(m_cmdConnAllowedBtn);

    m_cmdRestartBtn = new QPushButton("Restart");
    m_cmdRestartBtn->setToolTip("Restarts Pico-ASHA.\n"
                                "It can be useful if your previously connected "
                                "hearing aids stop connecting.");
    cmdLayout->addWidget(m_cmdRestartBtn);

    m_cmdRemoveBondBtn = new QPushButton("Unpair");
    m_cmdRemoveBondBtn->setToolTip("Unpair connected hearing aids.\n"
                                   "As Pico-ASHA only supports one set of hearing aids at a time,\n"
                                   "this allows pairing a different set of hearing aids.");
    cmdLayout->addWidget(m_cmdRemoveBondBtn);

    m_cmdUacVersCb = new QComboBox();
    m_cmdUacVersCb->addItem("UAC1", QVariant(1u));
    m_cmdUacVersCb->addItem("UAC2", QVariant(2u));
    m_cmdUacVersCb->setToolTip("Change USB Audio Class version.\n"
                               "Most people should stick with UAC2, however UAC1 may be compatible "
                               "With older operating systems such as Windows XP - 8.\n"
                               "This setting will persist, allowing you to set it on one device, "
                               "then plug it into an older device that only supports UAC1.");
    cmdLayout->addWidget(m_cmdUacVersCb);

    cmdLayout->addStretch();
    cmdGroup->setLayout(cmdLayout);
    mainVBox->addWidget(cmdGroup);

    QObject::connect(m_cmdRestartBtn, &QPushButton::clicked, this, &PicoAshaMainWindow::cmdRestartBtnClicked);
    QObject::connect(m_cmdConnAllowedBtn, &QPushButton::clicked, this, [=, this](bool clicked) {
        emit cmdConnAllowedBtnClicked(!m_connectionsAllowed);
    });
    QObject::connect(m_cmdStreamingEnabledBtn, &QPushButton::clicked, this, [=, this](bool clicked) {
        emit cmdStreamingEnabledBtnClicked(!m_streamingEnabled);
    });
    QObject::connect(m_cmdRemoveBondBtn, &QPushButton::clicked, this, [=, this](bool clicked) {
        auto ans = QMessageBox::question(this, this->windowTitle(), "Are you sure you want to unpair connected hearing aids?");
        if (ans == QMessageBox::Yes) {
            emit cmdRemoveBondBtnClicked();
        }
    });
    QObject::connect(m_cmdUacVersCb, &QComboBox::activated, this, [=, this](int index) {
        auto uac_ver_item = m_cmdUacVersCb->itemData(index);
        uint16_t uac_ver = static_cast<uint16_t>(uac_ver_item.toUInt());
        if (uac_ver != m_UacVersion) {
            emit cmdUacVersChanged(uac_ver);
        }
    });
    setConnectionsAllowed(false);
    setAudioStreamingEnabled(false);
    setCmdBtnsEnabled(false);

    auto hciGroup = new QGroupBox("HCI Logging");
    auto hciLayout = new QHBoxLayout;
    m_hciActionBtn = new QPushButton;
    m_hciActionBtn->setToolTip("Start or stop HCI logging.\n"
                               "This can be useful to help diagnose pairing and connection issues,\n"
                               "especially for hearing aids which have not previously been tested.\n"
                               "Not required for general use.");
    setHciActionBtnStart(false);
    m_hciPathBtn = new QPushButton("Browse...");
    m_hciPathLbl = new QLabel("Set HCI logging path");
    hciLayout->addWidget(m_hciActionBtn, 0);
    hciLayout->addWidget(m_hciPathBtn, 0);
    hciLayout->addWidget(m_hciPathLbl, 1);
    hciGroup->setLayout(hciLayout);
    mainVBox->addWidget(hciGroup);

    QObject::connect(m_hciPathBtn, &QPushButton::clicked, this, [=, this](bool clicked) {
        auto filename = QFileDialog::getSaveFileName(this, "Create HCI Log File", "", "btsnoop (*.log)");
        if (!filename.isEmpty()) {
            m_hciPathLbl->setText(filename);
            emit hciLogPathChanged(filename);
        }
    });

    QObject::connect(m_hciActionBtn, &QPushButton::clicked, this, [=, this](bool clicked) {
        emit hciLogActionBtnClicked();
    });

    m_logWidget = new QPlainTextEdit;
    m_logWidget->setLineWrapMode(QPlainTextEdit::NoWrap);
    m_logWidget->setReadOnly(true);
    m_logWidget->setCenterOnScroll(true);
    mainVBox->addWidget(m_logWidget);

    m_mainWidget->setLayout(mainVBox);
    setCentralWidget(m_mainWidget);

    m_serialConnectedStatus = new QLabel();
    onSerialConnected(false);

    QStatusBar* mainStatusBar = new QStatusBar;
    mainStatusBar->addPermanentWidget(m_serialConnectedStatus);
    setStatusBar(mainStatusBar);
}

PicoAshaMainWindow::~PicoAshaMainWindow()
{
}

QFrame *PicoAshaMainWindow::remoteFrame() const
{
    return m_remoteFrame;
}

void PicoAshaMainWindow::appendLog(const QString &logLine)
{
    m_logWidget->appendPlainText(logLine);
}

RemoteDevice* PicoAshaMainWindow::addRemote(uint16_t connID, const asha::comm::RemoteInfo* remote)
{
    using namespace asha::comm;
    QString remName = widgetNameFromConnID(connID);
    if (getRemote(connID) != nullptr) {
        qDebug("Remote device already exists");
        return nullptr;
    }
    auto rem0 = m_remotes.at(0);
    auto rem1 = m_remotes.at(1);
    RemoteDevice* r = nullptr;

    auto side = RemoteDevice::Side::SideUnset;
    if (remote != nullptr) {
        if (remote->side == CSide::Left) {
            side = RemoteDevice::Side::Left;
        } else if (remote->side == CSide::Right) {
            side = RemoteDevice::Side::Right;
        }
    }

    if (side == RemoteDevice::Side::Left) {
        r = rem0;
    } else if (side == RemoteDevice::Side::Right) {
        r = rem1;
    } else if (rem0->isDefaultValues() && rem1->isDefaultValues()) {
        r = rem0;
    } else {
        r = rem0->isDefaultValues() ? rem0 : rem1;
    }
    r->setObjectName(remName);
    r->setGreyedOut(false);
    if (remote != nullptr) {
        r->setRemoteInfo(*remote);
    } else {
        r->setConnID(connID);
    }
    return r;
}

RemoteDevice* PicoAshaMainWindow::addRemote(const asha::comm::RemoteInfo &remote)
{
    return addRemote(remote.conn_id, &remote);
}

void PicoAshaMainWindow::removeRemote(uint16_t connID)
{
    auto rem = getRemote(connID);
    if (rem) {
        rem->setObjectName("");
        rem->setDefaultValues();
        rem->setGreyedOut(true);
    }
}

void PicoAshaMainWindow::removeRemotes()
{
    for (auto rem : std::as_const(m_remotes)) {
        rem->setObjectName("");
        rem->setDefaultValues();
        rem->setGreyedOut(true);
    }
}

void PicoAshaMainWindow::setRemoteOrder()
{
    QHBoxLayout* l = qobject_cast<QHBoxLayout*>(m_remoteFrame->layout());
    auto rem0 = m_remotes.at(0);
    auto rem1 = m_remotes.at(1);
    auto p0 = rem0->cachedProps();
    auto p1 = rem1->cachedProps();

    if (p0.side == RemoteDevice::Side::Right) {
        auto li = l->takeAt(1);
        l->insertItem(2, li);
        m_remotes.swapItemsAt(0,1);
    } else if (p1.side == RemoteDevice::Side::Left) {
        auto li = l->takeAt(2);
        l->insertItem(1, li);
        m_remotes.swapItemsAt(1,0);
    }
}

RemoteDevice *PicoAshaMainWindow::getRemote(uint16_t connID)
{
    auto name = widgetNameFromConnID(connID);
    for (auto r : std::as_const(m_remotes)) {
        if (r->objectName() == name) {
            return r;
        }
    }
    return nullptr;
}

void PicoAshaMainWindow::setConnectionsAllowed(bool allowed)
{
    m_connectionsAllowed = allowed;
    if (m_connectionsAllowed) {
        m_cmdConnAllowedBtn->setText("Disable Connections");
    } else {
        m_cmdConnAllowedBtn->setText("Enable Connections");
    }
}

void PicoAshaMainWindow::setAudioStreamingEnabled(bool enabled)
{
    m_streamingEnabled = enabled;
    if (m_streamingEnabled) {
        m_cmdStreamingEnabledBtn->setText("Stop Audio");
    } else {
        m_cmdStreamingEnabledBtn->setText("Start Audio");
    }
}

void PicoAshaMainWindow::setUacVersion(uint16_t uac_ver)
{
    // Note index 0 = UAC1, index 1 = UAC2
    if (uac_ver == 1U || uac_ver == 2U) {
        m_UacVersion = uac_ver;
        m_cmdUacVersCb->setCurrentIndex(uac_ver - 1);
    }
}

void PicoAshaMainWindow::setHciActionBtnStart(bool enabled)
{
    m_hciActionBtn->setText("HCI Start");
    m_hciActionBtn->setEnabled(enabled);
}

void PicoAshaMainWindow::setHciActionBtnStop(bool enabled)
{
    m_hciActionBtn->setText("HCI Stop");
    m_hciActionBtn->setEnabled(enabled);
}

void PicoAshaMainWindow::onAdPacketReceived(const asha::comm::AdvertisingPacket &ad_pkt)
{
    if (!m_currPairDlg) {
        m_currPairDlg = new PairDialog(this);
        QObject::connect(m_currPairDlg, &PairDialog::addressSelected, this, &PicoAshaMainWindow::pairWithAddress);
        QObject::connect(m_currPairDlg, &PairDialog::accepted, this, &PicoAshaMainWindow::onPairDialogAcceptedRejected);
        m_currPairDlg->open();
    }
    m_currPairDlg->setAdPacket(ad_pkt);
}

void PicoAshaMainWindow::onPairDialogAcceptedRejected()
{
    m_currPairDlg->deleteLater();
    m_currPairDlg = nullptr;
}

void PicoAshaMainWindow::onSerialConnected(bool connected)
{
    m_serialConnected = connected;
    if (m_serialConnected) {
        m_serialConnectedStatus->setText("Connected");
        m_serialConnectedStatus->setStyleSheet("QLabel { color : green; }");
    } else {
        m_serialConnectedStatus->setText("Not Connected");
        m_serialConnectedStatus->setStyleSheet("QLabel { color : red; }");
    }
}

void PicoAshaMainWindow::setCmdBtnsEnabled(bool enabled)
{
    m_cmdRestartBtn->setEnabled(enabled);
    m_cmdConnAllowedBtn->setEnabled(enabled);
    m_cmdStreamingEnabledBtn->setEnabled(enabled);
    m_cmdRemoveBondBtn->setEnabled(enabled);
    m_cmdUacVersCb->setEnabled(enabled);
}

void PicoAshaMainWindow::setPicoAshaVerStr(const QString &version)
{
    m_serialConnectedStatus->setText(QString("Connected :: %1").arg(version));
}

