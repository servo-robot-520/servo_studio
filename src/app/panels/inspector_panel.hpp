#pragma once

#include <QWidget>
#include <QVector>
#include <vector>

#include "app/session_bridge.hpp"
#include "servo_core/types.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTableWidget;

namespace app {

    /// @brief 检查器：选中舵机的命令区（目标位置/速度/时间/扭矩/点动）
    ///        与参数编辑器（按参数描述表渲染，ID/波特率走专属流程）。
    class InspectorPanel : public QWidget {
        Q_OBJECT

    public:
        explicit InspectorPanel(QWidget* parent = nullptr);

        /// 设置监控对象并按配置初始化换算；同一对象重复调用为空操作。
        /// 主题信息（串口/id/型号）由监控区统一展示，此处不再重复。
        /// @param maxRangeDeg 型号允许的最大机械行程，约束行程输入。
        void setSubject(const QString& port, servo_core::ServoId id,
                        const servo_core::ServoConfig& config, double maxRangeDeg = 360.0);
        /// 更新当前对象的换算配置（不重置参数表）。
        void updateConfig(const servo_core::ServoConfig& config);
        /// 清空监控对象。
        void clear();

    signals:
        /// 下发目标位置（tick）；toSelection 为真时作用于状态表全部选中行。
        void goalRequested(const QString& port, int ticks, int timeMs, int speed,
                           bool toSelection);
        /// 扭矩开关。
        void torqueRequested(const QString& port, servo_core::ServoId id, bool enabled);
        /// 写入单个参数（id/baud_rate 由主窗口拦截走专属流程）。
        /// valueText 为编辑控件当前显示值（枚举参数为实际取值）。
        void paramWriteRequested(const QString& port, servo_core::ServoId id, const QString& key,
                                 int value, const QString& valueText);
        /// 将参数写入总线全部舵机。
        void paramWriteAllRequested(const QString& port, const QString& key, int value);
        /// 备份整表参数到文件。
        void backupRequested(const QString& port, servo_core::ServoId id);
        /// 从文件恢复参数。
        void restoreRequested(const QString& port, servo_core::ServoId id);
        /// 重新读取参数。
        void reloadParametersRequested(const QString& port, servo_core::ServoId id);
        /// 应用换算配置（机械行程/命令钳位）。
        void servoConfigRequested(const QString& port, servo_core::ServoId id, double rangeDeg,
                                  double minTicks, double maxTicks);

    public slots:
        /// 填充参数表（重建行）。
        void setParameters(const QString& port, int id, const QVector<ParamValue>& values);

    private slots:
        void onGoalValueChanged();

    private:
        [[nodiscard]] bool hasSubject() const;
        void rebuildParameterRows(const QVector<ParamValue>& values);
        /// 进入某行的值编辑（点击触发）。
        void beginEdit(int row);
        /// 提交当前编辑（换行/点击别处/写入前调用）。
        void commitEditor();

        bool eventFilter(QObject* watched, QEvent* event) override;

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
        QCheckBox* torque_check_;
        QDoubleSpinBox* range_spin_;
        QSpinBox* min_ticks_spin_;
        QSpinBox* max_ticks_spin_;
        QPushButton* apply_config_button_;
        QPushButton* goal_button_;
        QPushButton* goal_all_button_;
        QPushButton* jog_minus_button_;
        QPushButton* jog_plus_button_;
        QPushButton* home_button_;
        QTableWidget* param_table_;
        QPushButton* reload_button_;
        QPushButton* write_all_button_;
        QPushButton* backup_button_;
        QPushButton* restore_button_;
        /// 参数行的暂存值（点击值列进入编辑后写回）。
        std::vector<ParamValue> param_rows_;
        int editing_row_{-1};
    };

} // namespace app
