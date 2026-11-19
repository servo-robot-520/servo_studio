#pragma once

#include <QDialog>
#include <QVector>
#include <vector>

#include "servo_core/types.hpp"

class QEvent;
class QLabel;
class QPushButton;
class QTableWidget;

namespace app {

    /// @brief 单条批量目标（按总线分组下发）。
    struct BatchGoal {
        QString port;
        servo_core::ServoId id{};
        int ticks{};
        int timeMs{};
        int speed{};
    };

    /// @brief 批量操作弹窗：多舵机目标编排。
    ///
    /// 行 = 所有已连接总线的舵机；目标/速度/时间点击编辑（滚轮守卫
    /// 覆盖），目标(°)随输入换算；\"参与\"勾选且目标非空的行才下发。
    /// 支持同步下发（SYNC WRITE 异值同时）与异步预存→触发
    ///（REG_WRITE/ACTION，预存有应答可确认，触发统一生效）。
    class BatchDialog : public QDialog {
        Q_OBJECT

    public:
        /// 弹窗中一行的初始化信息。
        struct BatchServo {
            QString port;
            servo_core::ServoId id{};
            servo_core::ServoConfig config;
        };

        explicit BatchDialog(QWidget* parent = nullptr);

        /// 重建行（同 id 保留已编辑值）。
        void setServos(const QVector<BatchServo>& servos);
        /// 后端是否支持预存式下发（控制预存/触发按钮）。
        void setStagedSupported(bool supported);

    signals:
        /// 同步下发（各行自己的目标值）。
        void syncRequested(const QVector<BatchGoal>& goals);
        /// 异步预存（逐台确认）。
        void stageRequested(const QVector<BatchGoal>& goals);
        /// 触发所有已预存目标。
        void triggerRequested();
        /// 请求用当前连接重建行。
        void refreshRequested();

    public slots:
        /// 预存成功 count 条参与行，更新预存计数。
        void markStaged(int count);
        /// 触发完成：清空预存标记。
        void markTriggered();

    private:
        struct Row {
            QString port;
            servo_core::ServoId id{};
            servo_core::ServoConfig config;
            bool has_goal{false};
            int goal{};
            int timeMs{0};
            int speed{1000};
            bool include{false};
            bool staged{false};
        };

        [[nodiscard]] QVector<BatchGoal> collectGoals() const;
        void rebuildRows(const QVector<BatchServo>& servos);
        void beginEdit(int row, int column);
        void commitEditor();
        void updateRowDisplay(int row);
        void updateTriggerLabel();
        bool eventFilter(QObject* watched, QEvent* event) override;

        QTableWidget* table_;
        QPushButton* refresh_button_;
        QPushButton* sync_button_;
        QPushButton* stage_button_;
        QPushButton* trigger_button_;
        std::vector<Row> rows_;
        int editing_row_{-1};
        int editing_column_{-1};
        bool staged_supported_{false};
    };

} // namespace app

Q_DECLARE_METATYPE(app::BatchGoal)
Q_DECLARE_METATYPE(QVector<app::BatchGoal>)
