#ifndef REMOTEDEVICEMODEL_H
#define REMOTEDEVICEMODEL_H

#include <QAbstractListModel>
#include <QObject>
#include <QHash>
#include <QVector>

#include "remotedevice.h"

class RemoteDeviceModel : public QAbstractListModel
{
    Q_OBJECT
public:
    RemoteDeviceModel(QObject *parent = nullptr);

    Q_INVOKABLE int rowCount(const QModelIndex&) const override;
    Q_INVOKABLE QVariant data(const QModelIndex& index, int role) const override;

    bool insert(RemoteDevice* remote);
    void remove(unsigned int conn_id);
    void reset();

    bool hasConnID(unsigned int conn_id);
    RemoteDevice* getByConnID(unsigned int conn_id) const;
    unsigned int minConnID() const;

    QHash<int, QByteArray> roleNames() const override;
private:
    QVector<RemoteDevice*> m_remoteDevices;
};

#endif // REMOTEDEVICEMODEL_H
