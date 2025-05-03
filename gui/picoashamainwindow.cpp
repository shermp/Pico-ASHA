#include <QVBoxLayout>
#include <QHBoxLayout>
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
    remoteHBox->addStretch(1);
    m_remoteFrame->setLayout(remoteHBox);
    mainVBox->addWidget(m_remoteFrame, 4);

    m_logWidget = new QPlainTextEdit;
    m_logWidget->setLineWrapMode(QPlainTextEdit::NoWrap);
    m_logWidget->setReadOnly(true);
    m_logWidget->setCenterOnScroll(true);
    mainVBox->addWidget(m_logWidget, 1);

    m_mainWidget->setLayout(mainVBox);
    setCentralWidget(m_mainWidget);

    m_serialConnectedStatus = new QLabel();
    setSerialConnectedStatus("Not Connected");

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

RemoteDevice* PicoAshaMainWindow::addRemote(uint16_t connID)
{
    QString remName = widgetNameFromConnID(connID);
    if (m_remoteFrame->findChild<RemoteDevice*>(remName)) {
        qDebug("Remote device already exists");
        return nullptr;
    }
    RemoteDevice* rem = new RemoteDevice(m_remoteFrame);
    rem->setObjectName(remName);
    rem->setConnID(connID);
    return addRemote(rem);
}

RemoteDevice* PicoAshaMainWindow::addRemote(const asha::comm::RemoteInfo &remote)
{
    QString remName = widgetNameFromConnID(remote.conn_id);
    if (m_remoteFrame->findChild<RemoteDevice*>(remName)) {
        qDebug() << "Remote device already exists";
        return nullptr;
    }
    RemoteDevice* rem = new RemoteDevice(remote, m_remoteFrame);
    rem->setObjectName(remName);
    return addRemote(rem);
}

void PicoAshaMainWindow::removeRemote(uint16_t connID)
{
    QString remName = widgetNameFromConnID(connID);
    auto rem = m_remoteFrame->findChild<RemoteDevice*>(remName);
    if (rem) {
        rem->deleteLater();
    }
}

void PicoAshaMainWindow::removeRemotes()
{
    auto rem = m_remoteFrame->findChildren<RemoteDevice*>();
    for (int i = 0; i < rem.size(); ++i) {
        rem[i]->deleteLater();
    }
}

void PicoAshaMainWindow::setRemoteOrder()
{
    QHBoxLayout* layout = qobject_cast<QHBoxLayout*>(m_remoteFrame->layout());
    if (layout->count() == 4) {
        auto r1 = qobject_cast<RemoteDevice*>(layout->itemAt(1)->widget());
        auto r2 = qobject_cast<RemoteDevice*>(layout->itemAt(2)->widget());

        auto p1 = r1->cachedProps();
        auto p2 = r2->cachedProps();

        if (p1.side == RemoteDevice::Side::Right) {
            auto item = layout->takeAt(1);
            layout->insertItem(2, item);
        } else if (p2.side == RemoteDevice::Side::Left) {
            auto item = layout->takeAt(2);
            layout->insertItem(1, item);
        }
    }
}

void PicoAshaMainWindow::setSerialConnectedStatus(const QString &statusStr)
{
    m_serialConnectedStatus->setText(statusStr);
    if (m_serialConnectedStatus->text() == "Not Connected") {
        m_serialConnectedStatus->setStyleSheet("QLabel { color : red; }");
    } else {
        m_serialConnectedStatus->setStyleSheet("QLabel { color : green; }");
    }
}

RemoteDevice *PicoAshaMainWindow::getRemote(uint16_t connID)
{
    auto name = widgetNameFromConnID(connID);
    return m_remoteFrame->findChild<RemoteDevice*>(name);
}

RemoteDevice* PicoAshaMainWindow::addRemote(RemoteDevice *remote)
{
    QHBoxLayout* layout = qobject_cast<QHBoxLayout*>(m_remoteFrame->layout());

    int index = 1;
    if (layout->count() == 2) {
        index = 1;
    } else if (layout->count() == 3) {
        RemoteDevice* existing = qobject_cast<RemoteDevice*>(layout->itemAt(1)->widget());
        auto props = existing->cachedProps();
        if (props.side == RemoteDevice::Side::Left) {
            index = 2;
        } else if (props.side == RemoteDevice::Side::Right) {
            index = 1;
        } else {
            // Have to potentially swap later
            index = 1;
        }
    } else {
        qDebug() << "Unexpected widget count in layout";
        delete remote;
        return nullptr;
    }
    layout->insertWidget(index, remote, 2);
    return remote;
}
