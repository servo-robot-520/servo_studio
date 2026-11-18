#pragma once

#include <QTreeWidget>
#include <map>

#include "servo_core/types.hpp"

namespace app {

    /// @brief 设备树：总线节点 + 在线舵机节点（id/型号/固件/状态）。
    class DeviceTree : public QTreeWidget {
        Q_OBJECT

    public:
        explicit DeviceTree(QWidget* parent = nullptr);

        /// 添加总线节点（幂等）。
        void addBus(const QString& port);
        /// 移除总线及其舵机节点。
        void removeBus(const QString& port);
        /// 更新总线连接状态。
        void setBusConnected(const QString& port, bool connected);
        /// 设置某总线的在线舵机列表（覆盖式更新）。
        void setServos(const QString& port, const QVector<servo_core::ServoInfo>& infos);
        /// 某总线当前登记的舵机 id。
        [[nodiscard]] QVector<servo_core::ServoId> servoIds(const QString& port) const;

    private:
        [[nodiscard]] QTreeWidgetItem* busItem(const QString& port) const;
        std::map<QString, QTreeWidgetItem*> buses_;
    };

} // namespace app
