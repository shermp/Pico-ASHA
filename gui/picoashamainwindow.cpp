#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QFrame>
#include <QStatusBar>

#include "picoashamainwindow.h"
#include "remotedevice.h"

static QString widgetNameFromConnID(uint16_t connID)
{
    return QString("remote_connid_%1").arg(connID);
}

PicoAshaMainWindow::PicoAshaMainWindow(QWidget *parent)
    : QMainWindow(parent)
{
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
    m_cmdRestartBtn = new QPushButton("Restart Pico-ASHA");
    cmdLayout->addWidget(m_cmdRestartBtn);
    cmdLayout->addStretch();
    cmdGroup->setLayout(cmdLayout);
    mainVBox->addWidget(cmdGroup);

    QObject::connect(m_cmdRestartBtn, &QPushButton::clicked, this, &PicoAshaMainWindow::cmdRestartBtnClicked);

    auto hciGroup = new QGroupBox("HCI Logging");
    auto hciLayout = new QHBoxLayout;
    m_hciActionBtn = new QPushButton;
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

void PicoAshaMainWindow::onSerialConnected(bool connected, const QString &statusStr)
{
    if (connected) {
        m_serialConnectedStatus->setText(statusStr);
        m_serialConnectedStatus->setStyleSheet("QLabel { color : green; }");
        m_cmdRestartBtn->setEnabled(true);
    } else {
        m_serialConnectedStatus->setText("Not Connected");
        m_serialConnectedStatus->setStyleSheet("QLabel { color : red; }");
        m_cmdRestartBtn->setEnabled(false);
    }
}
