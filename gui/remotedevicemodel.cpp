#include <algorithm>
#include "remotedevicemodel.h"

RemoteDeviceModel::RemoteDeviceModel(QObject *parent) : QAbstractListModel(parent) {}

int RemoteDeviceModel::rowCount([[maybe_unused]] const QModelIndex &) const
{
    return m_remoteDevices.size();
}

QVariant RemoteDeviceModel::data(const QModelIndex &index, [[maybe_unused]] int role) const
{
    RemoteDevice* remote = m_remoteDevices.at(index.row());
    return QVariant::fromValue(remote);
}

bool RemoteDeviceModel::insert(RemoteDevice *remote)
{
    auto rem_list_sz = m_remoteDevices.size();
    if (m_remoteDevices.contains(remote) || rem_list_sz >= 2) {
        return false;
    }
    int index = 0;
    if (rem_list_sz == 1) {
        const auto r = m_remoteDevices.at(0);
        if (remote->side() > r->side()) {
            index = 1;
        }
    }
    beginInsertRows(QModelIndex(), index, index);
    m_remoteDevices.insert(index, remote);
    endInsertRows();
    return true;
}

void RemoteDeviceModel::remove(unsigned int conn_id)
{
    int index = -1;
    for (int i = 0; i < m_remoteDevices.size(); ++i) {
        const auto r = m_remoteDevices.at(i);
        if (r->connID() == conn_id) {
            index = i;
            break;
        }
    }
    if (index >= 0) {
        beginRemoveRows(QModelIndex(), index, index);
        auto remote = m_remoteDevices.takeAt(index);
        delete remote;
        endRemoveRows();
    }
}

void RemoteDeviceModel::reset()
{
    beginResetModel();
    qDeleteAll(m_remoteDevices.begin(), m_remoteDevices.end());
    m_remoteDevices.clear();
    endResetModel();
}

bool RemoteDeviceModel::hasConnID(unsigned int conn_id)
{
    return std::find_if(m_remoteDevices.begin(),m_remoteDevices.end(),
                        [conn_id](const RemoteDevice* dev){return dev->connID() == conn_id;}) != m_remoteDevices.end();
}

RemoteDevice *RemoteDeviceModel::getByConnID(unsigned int conn_id) const
{
    for (const auto r : m_remoteDevices) {
        if (r->connID() == conn_id) return r;
    }
    return nullptr;
}

unsigned int RemoteDeviceModel::minConnID() const
{
    auto sz = m_remoteDevices.size();
    if (sz == 1) {
        return m_remoteDevices.at(0)->connID();
    } else if (sz == 2) {
        return std::min(m_remoteDevices.at(0)->connID(), m_remoteDevices.at(1)->connID());
    }
    return asha::comm::unset_conn_id;
}

QHash<int, QByteArray> RemoteDeviceModel::roleNames() const
{
    static QHash<int, QByteArray> roles;
    roles[Qt::UserRole + 1] = "remote";
    return roles;
}
