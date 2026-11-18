#pragma once

#include <QTableWidget>
#include <QVector>

#include "servo_core/types.hpp"

namespace app {

    /// @brief 多舵机实时状态表：按 (总线, id) 定位行，轮询批次到达时原位刷新。
    class StatusTable : public QTableWidget {
        Q_OBJECT

    public:
        explicit StatusTable(QWidget* parent = nullptr);

        /// 用一批轮询状态刷新表格（原位更新；新行按 总线/id 升序插入）。
        void updateStates(const QString& port, const QVector<servo_core::ServoState>& states);
        /// 移除某总线的全部行。
        void removeBus(const QString& port);
        /// 清空全部行。
        void clearAll();

    private:
        /// 查找 (port, id) 所在行；不存在返回 -1。
        [[nodiscard]] int findRow(const QString& port, servo_core::ServoId id) const;
        /// 按 总线/id 升序插入新行，返回行号。
        int insertRowFor(const QString& port, const servo_core::ServoState& state);
    };

} // namespace app
