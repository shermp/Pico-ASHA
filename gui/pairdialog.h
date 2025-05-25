#ifndef PAIRDIALOG_H
#define PAIRDIALOG_H

#include <QByteArray>
#include <QDialog>
#include <QListWidget>
#include <QMap>
#include <QString>

#include <asha_comms.hpp>

struct AdDetails {
    QByteArray addr;
    uint8_t addr_type;
    uint8_t rssi;
    bool is_ha;
    QString name;
};

class PairDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PairDialog(QWidget *parent = nullptr);

    void setAdPacket(asha::comm::AdvertisingPacket const& ad_pkt);

signals:
    void addressSelected(QByteArray const& addr, uint8_t addr_type);
private:
    QMap<QString, AdDetails> m_adMap;
    QListWidget* m_adListWidget;
};

#endif // PAIRDIALOG_H
