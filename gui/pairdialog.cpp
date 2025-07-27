#include <QDialogButtonBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QListWidgetItem>

#include "pairdialog.h"

PairDialog::PairDialog(QWidget *parent) : QDialog(parent)
{
    m_adListWidget = new QListWidget(this);
    m_adListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    QDialogButtonBox* bb = new QDialogButtonBox(this);
    bb->addButton("Pair", QDialogButtonBox::AcceptRole);
    QObject::connect(bb, &QDialogButtonBox::accepted, this, [=,this]() {
        auto selections = m_adListWidget->selectedItems();
        if (!selections.empty()) {
            auto selected = selections.at(0);
            auto addrHex = selected->text().sliced(0, 17);
            if (m_adMap.contains(addrHex)) {
                auto a = m_adMap.value(addrHex);
                emit addressSelected(a.addr, a.addr_type);
            }
        }
        this->accept();
    });

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(m_adListWidget);
    layout->addWidget(bb);
    this->setLayout(layout);
}

void PairDialog::setAdPacket(const asha::comm::AdvertisingPacket &ad_pkt)
{
    QByteArray addr = QByteArray((const char*)ad_pkt.addr, sizeof(ad_pkt.addr));
    QString addrStr = addr.toHex(':');
    if (m_adMap.contains(addrStr)) {
        QString name = ad_pkt.name;
        bool updateList = false;
        auto a = m_adMap.value(addrStr);
        // if (a.rssi != ad_pkt.rssi) {
        //     updateList = true;
        //     a.rssi = ad_pkt.rssi;
        // }
        if (ad_pkt.is_ha && !a.is_ha) {
            updateList = true;
            a.is_ha = true;
        }
        if (a.name.isEmpty() && !name.isEmpty()) {
            updateList = true;
            a.name = ad_pkt.name;
        }
        if (updateList) {
            auto lw = m_adListWidget->findItems(addrStr, Qt::MatchStartsWith);
            if (!lw.empty()) {
                auto w = lw.at(0);
                w->setText(QString("%1 (%2)").arg(addrStr).arg(a.name));
            }
        }
    } else {
        AdDetails a;
        a.addr = addr;
        a.addr_type = ad_pkt.addr_type;
        a.rssi = ad_pkt.rssi;
        a.is_ha = ad_pkt.is_ha;
        a.name = ad_pkt.name;
        if (a.is_ha) {
            m_adMap[addrStr] = a;
            new QListWidgetItem(QString("%1 (%2)").arg(addrStr).arg(a.name), m_adListWidget);
        }
    }
}
