#include "app/panels/inspector_panel.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace app {

    InspectorPanel::InspectorPanel(QWidget* parent) :
        QWidget(parent) {
        // 允许缩窄，把横向空间让给监控区。
        setMinimumWidth(240);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(10);

        // ---------- 命令区 ----------
        auto* command_group = new QGroupBox(tr("命令"), this);
        auto* form = new QFormLayout(command_group);

        ticks_spin_ = new QSpinBox(command_group);
        ticks_spin_->setRange(0, 1023);
        ticks_spin_->setToolTip(tr("目标位置（tick），511 为中位；回车直接下发"));
        deg_label_ = new QLabel("= 0.00°", command_group);
        deg_label_->setObjectName("muted");
        auto* position_row = new QHBoxLayout;
        position_row->addWidget(ticks_spin_);
        position_row->addWidget(deg_label_);
        position_row->addStretch(1);
        form->addRow(tr("目标位置(tick)"), position_row);

        // 滑杆与点动独立成行，点动位于目标位置与滑杆之下。
        ticks_slider_ = new QSlider(Qt::Horizontal, command_group);
        ticks_slider_->setRange(0, 1023);
        ticks_slider_->setToolTip(tr("拖动调整目标位置，松开鼠标自动下发"));
        form->addRow(ticks_slider_);

        jog_minus_button_ = new QPushButton(tr("−1°"), command_group);
        jog_plus_button_ = new QPushButton(tr("+1°"), command_group);
        jog_minus_button_->setToolTip(tr("目标位置换算为度后减 1°"));
        jog_plus_button_->setToolTip(tr("目标位置换算为度后加 1°"));
        auto* jog_row = new QHBoxLayout;
        jog_row->addWidget(jog_minus_button_);
        jog_row->addWidget(jog_plus_button_);
        jog_row->addStretch(1);
        form->addRow(jog_row);

        speed_spin_ = new QSpinBox(command_group);
        speed_spin_->setRange(0, 1000);
        speed_spin_->setToolTip(tr("运行速度（寄存器 46），单位 tick/s；为 0 时按运行时间运动"));
        speed_spin_->setValue(1000);
        form->addRow(tr("速度(tick/s)"), speed_spin_);

        time_spin_ = new QSpinBox(command_group);
        time_spin_->setRange(0, 9999);
        time_spin_->setToolTip(tr("运行时间（寄存器 44），单位 ms，速度为 0 时生效；\n"
                                  "电机模式下该寄存器为 PWM 占空比"));
        form->addRow(tr("运行时间(ms)"), time_spin_);

        torque_check_ = new QCheckBox(tr("扭矩输出"), command_group);
        torque_check_->setToolTip(tr("扭矩开关（寄存器 40）：勾选=通电锁轴保持位置；\n"
                                     "取消=断开力矩，输出轴自由。阻尼模式（写 2）见寄存器工作台"));
        connect(torque_check_, &QCheckBox::toggled, this, [this](const bool enabled) {
            if (hasSubject()) {
                emit torqueRequested(subject_port_, subject_id_, enabled);
            }
        });
        form->addRow(torque_check_);

        // 动作按钮逐行排布，窄面板下不拥挤。
        goal_button_ = new QPushButton(tr("下发目标"), command_group);
        goal_button_->setObjectName("primary");
        goal_all_button_ = new QPushButton(tr("下发到选中舵机"), command_group);
        goal_protocol_button_ = new QPushButton(tr("下发到所有舵机"), command_group);
        home_button_ = new QPushButton(tr("回中"), command_group);
        goal_button_->setToolTip(tr("向当前舵机下发目标位置、速度与运行时间"));
        goal_all_button_->setToolTip(tr("同步下发到状态表中选中的舵机（可多选、可跨总线）；\n"
                                        "未选中时作用于当前舵机"));
        goal_protocol_button_->setToolTip(tr("同步下发到当前协议的全部舵机\n"
                                             "（所有已连接的同协议串口总线，一条串口仅支持一种协议）"));
        home_button_->setToolTip(tr("目标设为编码器中位并下发"));
        form->addRow(goal_button_);
        form->addRow(goal_all_button_);
        form->addRow(goal_protocol_button_);
        form->addRow(home_button_);

        range_spin_ = new QDoubleSpinBox(command_group);
        range_spin_->setRange(0.1, 360.0);
        range_spin_->setToolTip(tr("机械行程（度）：编码器全量程对应的角度，用于 tick↔度换算"));
        range_spin_->setDecimals(1);
        range_spin_->setValue(300.0);
        form->addRow(tr("行程(°)"), range_spin_);

        min_ticks_spin_ = new QSpinBox(command_group);
        max_ticks_spin_ = new QSpinBox(command_group);
        min_ticks_spin_->setToolTip(tr("命令位置钳位下限（tick）"));
        max_ticks_spin_->setToolTip(tr("命令位置钳位上限（tick）"));
        max_ticks_spin_->setMaximum(65535);
        max_ticks_spin_->setValue(1023);
        auto* clamp_row = new QHBoxLayout;
        clamp_row->addWidget(min_ticks_spin_);
        clamp_row->addWidget(new QLabel("～", command_group));
        clamp_row->addWidget(max_ticks_spin_);
        clamp_row->addStretch(1);
        form->addRow(tr("钳位(tick)"), clamp_row);

        apply_config_button_ = new QPushButton(tr("应用"), command_group);
        apply_config_button_->setToolTip(tr("应用行程与钳位（影响换算显示与命令钳位，不写舵机寄存器）"));
        form->addRow(apply_config_button_);
        layout->addWidget(command_group);

        // ---------- 参数入口 ----------
        auto* param_group = new QGroupBox(tr("参数"), this);
        auto* param_layout = new QVBoxLayout(param_group);
        register_button_ = new QPushButton(tr("寄存器…"), param_group);
        register_button_->setToolTip(tr("打开全量寄存器工作台（地址/中英文/区域读写）"));
        register_button_->setObjectName("primary");
        auto* hint = new QLabel(tr("全量寄存器读写（地址/中英文/区域）"), param_group);
        hint->setObjectName("muted");
        param_layout->addWidget(register_button_);
        param_layout->addWidget(hint);
        layout->addWidget(param_group);

        layout->addStretch(1);

        // ---------- 信号 ----------
        connect(ticks_spin_, &QSpinBox::valueChanged, this, &InspectorPanel::onGoalValueChanged);
        connect(ticks_slider_, &QSlider::valueChanged, ticks_spin_, &QSpinBox::setValue);
        connect(ticks_spin_, &QSpinBox::valueChanged, ticks_slider_, &QSlider::setValue);
        // 滑块松开鼠标自动下发目标。
        connect(ticks_slider_, &QSlider::sliderReleased, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), GoalTarget::kCurrent);
            }
        });
        // 目标位置回车直接下发（仅回车触发，失焦不触发）。
        connect(ticks_spin_->findChild<QLineEdit*>(), &QLineEdit::returnPressed, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), GoalTarget::kCurrent);
            }
        });
        // 回中：目标设为编码器中位并直接下发。
        connect(home_button_, &QPushButton::clicked, this, [this] {
            if (!hasSubject()) {
                return;
            }
            ticks_spin_->setValue(static_cast<int>(subject_config_.center_ticks));
            emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                               speed_spin_->value(), GoalTarget::kCurrent);
        });

        connect(goal_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), GoalTarget::kCurrent);
            }
        });
        connect(goal_all_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), GoalTarget::kSelection);
            }
        });
        connect(goal_protocol_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), GoalTarget::kAll);
            }
        });
        auto jog = [this](const double delta_deg) {
            if (!hasSubject()) {
                return;
            }
            const double current_deg = servo_core::ticks_to_degrees(subject_config_,
                                                                    ticks_spin_->value());
            ticks_spin_->setValue(
                servo_core::degrees_to_ticks(subject_config_, current_deg + delta_deg));
        };
        connect(jog_minus_button_, &QPushButton::clicked, this, [jog] { jog(-1.0); });
        connect(jog_plus_button_, &QPushButton::clicked, this, [jog] { jog(1.0); });
        connect(apply_config_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit servoConfigRequested(subject_port_, subject_id_, range_spin_->value(),
                                          min_ticks_spin_->value(), max_ticks_spin_->value());
            }
        });
        connect(register_button_, &QPushButton::clicked, this,
                [this] { emit registerRequested(); });
    }

    bool InspectorPanel::hasSubject() const {
        return has_subject_;
    }

    void InspectorPanel::updateConfig(const servo_core::ServoConfig& config) {
        subject_config_ = config;
        ticks_spin_->setRange(static_cast<int>(config.min_ticks),
                              static_cast<int>(config.max_ticks));
        ticks_slider_->setRange(static_cast<int>(config.min_ticks),
                                static_cast<int>(config.max_ticks));
        range_spin_->setValue(config.range_deg);
        min_ticks_spin_->setValue(static_cast<int>(config.min_ticks));
        max_ticks_spin_->setValue(static_cast<int>(config.max_ticks));
        // 中位与行程上限随舵机编码器几何而异（SCS 511/300°、STS 2048/…），提示动态计算。
        home_button_->setToolTip(
            tr("目标设为编码器中位（%1）并下发").arg(static_cast<int>(config.center_ticks)));
        range_spin_->setToolTip(
            tr("机械行程（度）：编码器全量程对应的角度，用于 tick↔度换算；\n上限 %1")
                .arg(max_range_deg_));
        onGoalValueChanged();
    }

    void InspectorPanel::setSubject(const QString& port, const servo_core::ServoId id,
                                    const servo_core::ServoConfig& config,
                                    const double maxRangeDeg) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        subject_config_ = config;
        max_range_deg_ = maxRangeDeg;
        has_subject_ = true;

        range_spin_->setRange(0.1, max_range_deg_);
        updateConfig(config);
        ticks_spin_->setValue(static_cast<int>(config.center_ticks));
    }

    void InspectorPanel::clear() {
        has_subject_ = false;
    }

    void InspectorPanel::onGoalValueChanged() {
        if (!has_subject_) {
            deg_label_->setText("= 0.00°");
            return;
        }
        deg_label_->setText(QStringLiteral("= %1°").arg(
            servo_core::ticks_to_degrees(subject_config_, ticks_spin_->value()), 0, 'f', 2));
    }

} // namespace app
