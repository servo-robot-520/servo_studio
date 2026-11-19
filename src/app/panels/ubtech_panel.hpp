#pragma once

#include <QWidget>

#include "app/session_bridge.hpp"
#include "servo_core/types.hpp"

class QGroupBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;

namespace app {

    /// @brief UBTECH 专用操作面板：只呈现该协议真正支持的能力——
    ///        转动（角度 0–240° / 运动时间 / 锁定时间）、强制中止（失电）、
    ///        角度回读（⚠失电副作用）、角度偏移校准、修改 ID、固件版本。
    ///
    ///        协议无速度/扭矩/温度等遥测，也无自动调试与批量编排，
    ///        因此不提供对应组件；参数编辑走通用 ParamsDialog。
    class UbtechPanel : public QWidget {
        Q_OBJECT

    public:
        explicit UbtechPanel(QWidget* parent = nullptr);

        /// 设置监控对象；同一对象重复调用为空操作。
        void setSubject(const QString& port, servo_core::ServoId id);
        /// 清空监控对象与读数。
        void clear();

        /// 显示角度回读结果（仅当属于当前对象时生效）。
        void updateState(const QString& port, const servo_core::ServoState& state);
        /// 显示参数刷新结果中的偏移量与固件版本（仅当前对象生效）。
        void setParameters(const QString& port, int id, const QVector<ParamValue>& values);

    signals:
        /// 下发转动目标：angleDeg 0–240（度），moveMs/lockMs 为 0 表示全速/不锁定。
        void moveRequested(const QString& port, servo_core::ServoId id, int angleDeg, int moveMs,
                           int lockMs);
        /// 强制中止：立即停转并失电。
        void stopRequested(const QString& port, servo_core::ServoId id);
        /// 角度回读（回读后舵机失电，仅手动触发）。
        void readAngleRequested(const QString& port, servo_core::ServoId id);
        /// 参数写入（走通用确认/重扫流程；key 为 "id"/"angle_offset"）。
        void paramWriteRequested(const QString& port, servo_core::ServoId id, const QString& key,
                                 int value, const QString& valueText);
        /// 重新读取参数（刷新偏移量与固件版本显示）。
        void paramReloadRequested(const QString& port, servo_core::ServoId id);
        /// 打开寄存器工作台弹窗。
        void registerRequested();

    private:
        [[nodiscard]] bool hasSubject() const;
        void setManualControlsEnabled(bool enabled) const;

        QString subject_port_;
        servo_core::ServoId subject_id_{};
        bool has_subject_{false};

        QLabel* title_label_;
        QSpinBox* angle_spin_;
        QSlider* angle_slider_;
        QSpinBox* move_time_spin_;
        QSpinBox* lock_time_spin_;
        QPushButton* goal_button_;
        QPushButton* home_button_;
        QPushButton* stop_button_;
        QPushButton* read_angle_button_;
        QLabel* angle_readout_;
        QSpinBox* offset_spin_;
        QPushButton* offset_read_button_;
        QPushButton* offset_write_button_;
        QLabel* offset_hint_;
        QSpinBox* id_spin_;
        QPushButton* id_write_button_;
        QPushButton* version_read_button_;
        QLabel* version_label_;
        QPushButton* register_button_;
    };

} // namespace app
