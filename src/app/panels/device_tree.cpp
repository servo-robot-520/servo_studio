#include "app/panels/device_tree.hpp"

#include <QHeaderView>

namespace app {

    namespace {
        constexpr int kBusPortRole = Qt::UserRole;
        constexpr int kServoIdRole = Qt::UserRole + 1;
    }

    DeviceTree::DeviceTree(QWidget* parent) :
        QTreeWidget(parent) {
        setObjectName("devices");
        setColumnCount(4);
        setHeaderLabels({tr("设备"), tr("型号"), tr("固件"), tr("状态")});
        setRootIsDecorated(true);
        header()->setSectionResizeMode(0, QHeaderView::Stretch);
        for (int column = 1; column < 4; ++column) {
            header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
        }
    }

    void DeviceTree::addBus(const QString& port) {
        if (buses_.contains(port)) {
            return;
        }
        auto* item = new QTreeWidgetItem(this);
        item->setText(0, port);
        item->setData(0, kBusPortRole, port);
        item->setText(3, tr("未连接"));
        item->setForeground(3, QColor("#8B98A5"));
        buses_[port] = item;
    }

    void DeviceTree::removeBus(const QString& port) {
        const auto it = buses_.find(port);
        if (it == buses_.end()) {
            return;
        }
        delete it->second;
        buses_.erase(it);
    }

    QTreeWidgetItem* DeviceTree::busItem(const QString& port) const {
        const auto it = buses_.find(port);
        return it == buses_.end() ? nullptr : it->second;
    }

    void DeviceTree::setBusConnected(const QString& port, const bool connected) const {
        if (auto* item = busItem(port)) {
            item->setText(3, connected ? tr("已连接") : tr("未连接"));
            item->setForeground(3, QColor(connected ? "#7FBF7A" : "#8B98A5"));
        }
    }

    void DeviceTree::setServos(const QString& port, const QVector<servo_core::ServoInfo>& infos) const {
        auto* bus = busItem(port);
        if (bus == nullptr) {
            return;
        }
        while (bus->childCount() > 0) {
            delete bus->takeChild(0);
        }
        for (const auto& info : infos) {
            auto* servo = new QTreeWidgetItem(bus);
            servo->setText(0, QString("id %1").arg(info.id));
            servo->setData(0, kServoIdRole, info.id);
            servo->setText(1, QString::fromStdString(info.model));
            servo->setText(2, QString("%1.%2").arg(info.firmware_major).arg(info.firmware_minor));
            servo->setText(3, tr("在线"));
            servo->setForeground(3, QColor("#7FBF7A"));
        }
        bus->setExpanded(true);
    }

    QVector<servo_core::ServoId> DeviceTree::servoIds(const QString& port) const {
        QVector<servo_core::ServoId> ids;
        if (const auto* bus = busItem(port)) {
            for (int row = 0; row < bus->childCount(); ++row) {
                ids.push_back(static_cast<servo_core::ServoId>(
                    bus->child(row)->data(0, kServoIdRole).toUInt()));
            }
        }
        return ids;
    }

    bool DeviceTree::selectedServo(QString* port, servo_core::ServoId* id) const {
        const auto items = selectedItems();
        if (items.isEmpty()) {
            return false;
        }
        const QTreeWidgetItem* item = items.first();
        const QTreeWidgetItem* bus = item->parent();
        if (bus == nullptr) {
            return false; // 选中的是总线节点。
        }
        if (port != nullptr) {
            *port = bus->data(0, kBusPortRole).toString();
        }
        if (id != nullptr) {
            *id = static_cast<servo_core::ServoId>(item->data(0, kServoIdRole).toUInt());
        }
        return true;
    }

    void DeviceTree::selectServo(const QString& port, const servo_core::ServoId id) {
        const QTreeWidgetItem* bus = busItem(port);
        if (bus == nullptr) {
            return;
        }
        for (int i = 0; i < bus->childCount(); ++i) {
            QTreeWidgetItem* child = bus->child(i);
            if (static_cast<servo_core::ServoId>(child->data(0, kServoIdRole).toUInt()) == id) {
                setCurrentItem(child);
                scrollToItem(child);
                return;
            }
        }
    }

} // namespace app
