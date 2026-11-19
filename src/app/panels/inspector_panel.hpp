#pragma once

#include <QWidget>

#include "servo_core/types.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTimer;

namespace app {

    /// @brief 目标下发范围。
    enum class GoalTarget {
        /// 当前舵机。
        kCurrent,
        /// 状态表选中的舵机（可多选、可跨总线）。
        kSelection,
        /// 当前协议的全部舵机（所有同协议串口总线）。
        kAll,
    };

    /// @brief 检查器：选中舵机的命令区（目标位置/速度/时间/扭矩/点动/回中）
    ///        与换算配置；参数编辑在寄存器工作台弹窗（ParamsDialog）中进行。
    class InspectorPanel : public QWidget {
        Q_OBJECT

    public:
        explicit InspectorPanel(QWidget* parent = nullptr);

        /// 设置监控对象并按配置初始化换算；同一对象重复调用为空操作。
        /// @param port
        /// @param id
        /// @param config
        /// @param maxRangeDeg 型号允许的最大机械行程，约束行程输入。
        void setSubject(const QString& port, servo_core::ServoId id,
                        const servo_core::ServoConfig& config, double maxRangeDeg = 360.0);
        /// 更新当前对象的换算配置。
        void updateConfig(const servo_core::ServoConfig& config);
        /// 清空监控对象。
        void clear();

    signals:
        /// 下发目标位置（tick），范围由 GoalTarget 决定；acc 仅对带加速度
        /// 寄存器的系列生效。
        void goalRequested(const QString& port, int ticks, int timeMs, int speed, int acc,
                           GoalTarget target);
        /// 扭矩设置：0 自由 / 1 锁紧 / 2 阻尼（寄存器 40）。
        void torqueRequested(const QString& port, servo_core::ServoId id, int mode);
        /// 应用换算配置（机械行程/命令钳位）。
        void servoConfigRequested(const QString& port, servo_core::ServoId id, double rangeDeg,
                                  double minTicks, double maxTicks);
        /// 打开寄存器工作台弹窗。
        void registerRequested();
        /// 打开批量操作弹窗。
        void batchRequested();

    private slots:
        void onGoalValueChanged() const;

    private:
        [[nodiscard]] bool hasSubject() const;
        /// 启动自动动作（sweep=往复，否则步进）；互斥，运行中先停。
        void startAutoMotion(bool sweep);
        /// 停止自动动作并恢复按钮。
        void stopAutoMotion();
        /// 自动动作与手动目标控件互斥：运行期间禁用手动目标控件。
        void setManualGoalControlsEnabled(bool enabled) const;

        QString subject_port_;
        servo_core::ServoId subject_id_{};
        servo_core::ServoConfig subject_config_;
        double max_range_deg_{360.0};
        bool has_subject_{false};

        QSpinBox* ticks_spin_;
        QLabel* deg_label_;
        QSlider* ticks_slider_;
        QSpinBox* speed_spin_;
        QSpinBox* time_spin_;
        QSpinBox* acc_spin_;
        QComboBox* torque_combo_;
        QDoubleSpinBox* range_spin_;
        QSpinBox* min_ticks_spin_;
        QSpinBox* max_ticks_spin_;
        QPushButton* apply_config_button_;
        QPushButton* goal_button_;
        QPushButton* goal_all_button_;
        QPushButton* goal_protocol_button_;
        QPushButton* jog_minus_button_;
        QPushButton* jog_plus_button_;
        QPushButton* home_button_;
        QPushButton* batch_button_;
        QPushButton* register_button_;

        // 自动调试：往复/步进自动动作发生器（辅助人工整定）。
        QSpinBox* auto_start_spin_;
        QSpinBox* auto_end_spin_;
        QSpinBox* auto_step_spin_;
        QSpinBox* auto_interval_spin_;
        QPushButton* sweep_button_;
        QPushButton* setp_button_;
        QTimer* auto_timer_;
        bool sweep_running_{false};
        bool setp_running_{false};
        bool auto_increase_{true};
        int auto_goal_{0};
    };

} // namespace app
