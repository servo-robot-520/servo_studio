#include "app/panels/inspector_panel.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

namespace app {

    InspectorPanel::InspectorPanel(QWidget* parent) :
        QWidget(parent) {
        // 控件保持完整宽度：不设横向滚动，宽度不足时由窗口最小尺寸兜底。
        setMinimumWidth(320);
        // 内容放进滚动区：面板控件多，不限定窗口最小高度，
        // 小窗口下可滚动而不是把整个窗口顶大。
        auto* outer = new QVBoxLayout(this);
        outer->setContentsMargins(0, 0, 0, 0);
        auto* scroll = new QScrollArea(this);
        scroll->setObjectName("panelScroll");
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* content = new QWidget(scroll);
        content->setObjectName("panelContent");
        auto* layout = new QVBoxLayout(content);
        layout->setContentsMargins(0, 0, 4, 0);
        layout->setSpacing(10);

        // ---------- 命令区 ----------
        auto* command_group = new QGroupBox(tr("舵机操作"), this);
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

        // 加速度设置：随目标下发（寄存器 41，SMS/STS 系列才有）。
        // 当前 SCS 系列无此寄存器，先接入命令通道但不显示。
        acc_spin_ = new QSpinBox(command_group);
        acc_spin_->setRange(0, 254);
        acc_spin_->setToolTip(tr("加速度（寄存器 41，SMS/STS 系列）；SCS 系列忽略"));
        auto* acc_row = new QWidget(command_group);
        auto* acc_layout = new QHBoxLayout(acc_row);
        acc_layout->setContentsMargins(0, 0, 0, 0);
        acc_layout->addWidget(new QLabel(tr("加速度"), acc_row));
        acc_layout->addWidget(acc_spin_);
        acc_layout->addStretch(1);
        form->addRow(acc_row);
        acc_row->setVisible(false); // SCS 系列暂不显示

        torque_combo_ = new QComboBox(command_group);
        torque_combo_->addItem(tr("自由"), 0);
        torque_combo_->addItem(tr("锁紧"), 1);
        torque_combo_->addItem(tr("阻尼"), 2);
        torque_combo_->setToolTip(tr("扭矩设置（寄存器 40）：锁紧=通电保持位置；\n"
                                     "自由=断开力矩；阻尼=输出阻尼（需固件支持）"));
        connect(torque_combo_, &QComboBox::currentIndexChanged, this, [this](const int index) {
            if (hasSubject()) {
                emit torqueRequested(subject_port_, subject_id_, torque_combo_->itemData(index).toInt());
            }
        });
        form->addRow(tr("扭矩设置"), torque_combo_);

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

        // ---------- 自动调试 ----------
        // 自动动作发生器：持续下发目标辅助人工整定（调 P/D 看阶跃响应、
        // 老化、回差检查）。复用目标下发通道，切换对象即自动停止。
        auto* auto_group = new QGroupBox(tr("自动调试"), this);
        auto* auto_form = new QFormLayout(auto_group);

        auto_start_spin_ = new QSpinBox(auto_group);
        auto_start_spin_->setRange(0, 1023);
        auto_start_spin_->setToolTip(tr("自动动作的起始位置（tick）"));
        auto_start_spin_->setValue(400);
        auto_form->addRow(tr("起点(tick)"), auto_start_spin_);

        auto_end_spin_ = new QSpinBox(auto_group);
        auto_end_spin_->setRange(0, 1023);
        auto_end_spin_->setToolTip(tr("自动动作的终点位置（tick）"));
        auto_end_spin_->setValue(600);
        auto_form->addRow(tr("终点(tick)"), auto_end_spin_);

        auto_step_spin_ = new QSpinBox(auto_group);
        auto_step_spin_->setRange(1, 512);
        auto_step_spin_->setValue(10);
        auto_step_spin_->setToolTip(tr("步进模式每周期的位移（tick），用于回差/精度检查"));
        auto_form->addRow(tr("步长(tick)"), auto_step_spin_);

        auto_interval_spin_ = new QSpinBox(auto_group);
        auto_interval_spin_->setRange(20, 10000);
        auto_interval_spin_->setValue(500);
        auto_interval_spin_->setToolTip(tr("自动下发周期（毫秒）"));
        auto_form->addRow(tr("间隔(ms)"), auto_interval_spin_);

        sweep_button_ = new QPushButton(tr("往复"), auto_group);
        sweep_button_->setToolTip(tr("目标在起点与终点间来回切换（方波）\n"
                                     "用于参数整定（看阶跃响应）与老化检查\n"
                                     "运行期间与手动目标控件互斥"));
        setp_button_ = new QPushButton(tr("步进"), auto_group);
        setp_button_->setToolTip(tr("按步长阶梯爬升到终点后折返\n"
                                    "用于回差、定位精度与有效行程检查\n"
                                    "运行期间与手动目标控件互斥"));
        auto* mode_row = new QHBoxLayout;
        mode_row->addWidget(sweep_button_);
        mode_row->addWidget(setp_button_);
        mode_row->addStretch(1);
        auto_form->addRow(mode_row);
        layout->addWidget(auto_group);

        // ---------- 编程入口 ----------
        auto* param_group = new QGroupBox(tr("编程"), this);
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
        scroll->setWidget(content);
        outer->addWidget(scroll);

        // ---------- 信号 ----------
        connect(ticks_spin_, &QSpinBox::valueChanged, this, &InspectorPanel::onGoalValueChanged);
        connect(ticks_slider_, &QSlider::valueChanged, ticks_spin_, &QSpinBox::setValue);
        connect(ticks_spin_, &QSpinBox::valueChanged, ticks_slider_, &QSlider::setValue);
        // 滑块松开鼠标自动下发目标。
        connect(ticks_slider_, &QSlider::sliderReleased, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), acc_spin_->value(),
                                   GoalTarget::kCurrent);
            }
        });
        // 目标位置回车直接下发（仅回车触发，失焦不触发）。
        connect(ticks_spin_->findChild<QLineEdit*>(), &QLineEdit::returnPressed, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), acc_spin_->value(),
                                   GoalTarget::kCurrent);
            }
        });
        // 回中：目标设为编码器中位并直接下发。
        connect(home_button_, &QPushButton::clicked, this, [this] {
            if (!hasSubject()) {
                return;
            }
            ticks_spin_->setValue(static_cast<int>(subject_config_.center_ticks));
            emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                               speed_spin_->value(), acc_spin_->value(), GoalTarget::kCurrent);
        });

        connect(goal_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), acc_spin_->value(),
                                   GoalTarget::kCurrent);
            }
        });
        connect(goal_all_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), acc_spin_->value(),
                                   GoalTarget::kSelection);
            }
        });
        connect(goal_protocol_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), acc_spin_->value(), GoalTarget::kAll);
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

        auto_timer_ = new QTimer(this);
        connect(auto_timer_, &QTimer::timeout, this, [this] {
            if (!hasSubject()) {
                stopAutoMotion();
                return;
            }
            if (sweep_running_) {
                const int start = auto_start_spin_->value();
                const int end = auto_end_spin_->value();
                auto_goal_ = (auto_goal_ == start) ? end : start;
            }
            else if (setp_running_) {
                const int start = auto_start_spin_->value();
                const int end = auto_end_spin_->value();
                const int step = auto_step_spin_->value();
                auto_goal_ += auto_increase_ ? step : -step;
                if (auto_goal_ >= end) {
                    auto_goal_ = end;
                    auto_increase_ = false;
                }
                else if (auto_goal_ <= start) {
                    auto_goal_ = start;
                    auto_increase_ = true;
                }
            }
            else {
                auto_timer_->stop();
                return;
            }
            ticks_spin_->setValue(auto_goal_);
            emit goalRequested(subject_port_, auto_goal_, time_spin_->value(),
                               speed_spin_->value(), acc_spin_->value(),
                               GoalTarget::kCurrent);
        });
        connect(sweep_button_, &QPushButton::clicked, this, [this] {
            if (sweep_running_) {
                stopAutoMotion();
            }
            else {
                startAutoMotion(true);
            }
        });
        connect(setp_button_, &QPushButton::clicked, this, [this] {
            if (setp_running_) {
                stopAutoMotion();
            }
            else {
                startAutoMotion(false);
            }
        });
    }

    void InspectorPanel::startAutoMotion(const bool sweep) {
        if (!hasSubject()) {
            return;
        }
        stopAutoMotion();
        sweep_running_ = sweep;
        setp_running_ = !sweep;
        auto_increase_ = true;
        auto_goal_ = auto_start_spin_->value();
        auto* running_button = sweep ? sweep_button_ : setp_button_;
        auto* idle_button = sweep ? setp_button_ : sweep_button_;
        running_button->setText(tr("停止"));
        idle_button->setEnabled(false);
        setManualGoalControlsEnabled(false);
        ticks_spin_->setValue(auto_goal_);
        // 先下发起点，再按周期动作。
        emit goalRequested(subject_port_, auto_goal_, time_spin_->value(),
                           speed_spin_->value(), acc_spin_->value(), GoalTarget::kCurrent);
        auto_timer_->start(auto_interval_spin_->value());
    }

    void InspectorPanel::stopAutoMotion() {
        auto_timer_->stop();
        sweep_running_ = false;
        setp_running_ = false;
        sweep_button_->setText(tr("往复"));
        setp_button_->setText(tr("步进"));
        sweep_button_->setEnabled(true);
        setp_button_->setEnabled(true);
        setManualGoalControlsEnabled(true);
    }

    void InspectorPanel::setManualGoalControlsEnabled(const bool enabled) {
        // 自动动作每周期覆盖目标，与手动目标控件互斥。
        ticks_spin_->setEnabled(enabled);
        ticks_slider_->setEnabled(enabled);
        jog_minus_button_->setEnabled(enabled);
        jog_plus_button_->setEnabled(enabled);
        goal_button_->setEnabled(enabled);
        goal_all_button_->setEnabled(enabled);
        goal_protocol_button_->setEnabled(enabled);
        home_button_->setEnabled(enabled);
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
        auto_start_spin_->setRange(static_cast<int>(config.min_ticks),
                                   static_cast<int>(config.max_ticks));
        auto_end_spin_->setRange(static_cast<int>(config.min_ticks),
                                 static_cast<int>(config.max_ticks));
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
        stopAutoMotion();
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
        stopAutoMotion();
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
