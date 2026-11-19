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
        setMinimumWidth(320);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(10);

        // ---------- 命令区 ----------
        auto* command_group = new QGroupBox(tr("命令"), this);
        auto* form = new QFormLayout(command_group);

        ticks_spin_ = new QSpinBox(command_group);
        ticks_spin_->setRange(0, 1023);
        ticks_slider_ = new QSlider(Qt::Horizontal, command_group);
        ticks_slider_->setRange(0, 1023);
        deg_label_ = new QLabel("= 0.00°", command_group);
        deg_label_->setObjectName("muted");
        auto* position_row = new QHBoxLayout;
        position_row->addWidget(ticks_spin_);
        position_row->addWidget(deg_label_);
        position_row->addWidget(ticks_slider_, 1);
        form->addRow(tr("目标位置(tick)"), position_row);

        speed_spin_ = new QSpinBox(command_group);
        speed_spin_->setRange(0, 1000);
        speed_spin_->setValue(1000);
        form->addRow(tr("速度(tick/s)"), speed_spin_);

        time_spin_ = new QSpinBox(command_group);
        time_spin_->setRange(0, 9999);
        form->addRow(tr("运行时间(ms)"), time_spin_);

        auto* goal_row = new QHBoxLayout;
        goal_button_ = new QPushButton(tr("下发目标"), command_group);
        goal_button_->setObjectName("primary");
        goal_all_button_ = new QPushButton(tr("下发到选中行"), command_group);
        home_button_ = new QPushButton(tr("回中"), command_group);
        jog_minus_button_ = new QPushButton(tr("−1°"), command_group);
        jog_plus_button_ = new QPushButton(tr("+1°"), command_group);
        goal_row->addWidget(goal_button_);
        goal_row->addWidget(goal_all_button_);
        goal_row->addWidget(home_button_);
        goal_row->addStretch(1);
        goal_row->addWidget(jog_minus_button_);
        goal_row->addWidget(jog_plus_button_);
        form->addRow(goal_row);

        torque_check_ = new QCheckBox(tr("扭矩输出"), command_group);
        connect(torque_check_, &QCheckBox::toggled, this, [this](const bool enabled) {
            if (hasSubject()) {
                emit torqueRequested(subject_port_, subject_id_, enabled);
            }
        });
        form->addRow(torque_check_);

        range_spin_ = new QDoubleSpinBox(command_group);
        range_spin_->setRange(0.1, 360.0);
        range_spin_->setDecimals(1);
        range_spin_->setValue(300.0);
        min_ticks_spin_ = new QSpinBox(command_group);
        max_ticks_spin_ = new QSpinBox(command_group);
        max_ticks_spin_->setMaximum(65535);
        max_ticks_spin_->setValue(1023);
        apply_config_button_ = new QPushButton(tr("应用"), command_group);
        auto* config_row = new QHBoxLayout;
        config_row->addWidget(range_spin_);
        config_row->addWidget(new QLabel(tr("钳位"), command_group));
        config_row->addWidget(min_ticks_spin_);
        config_row->addWidget(max_ticks_spin_);
        config_row->addWidget(apply_config_button_);
        config_row->addStretch(1);
        form->addRow(tr("行程(°)"), config_row);
        layout->addWidget(command_group);

        // ---------- 参数入口 ----------
        auto* param_group = new QGroupBox(tr("参数"), this);
        auto* param_layout = new QVBoxLayout(param_group);
        register_button_ = new QPushButton(tr("寄存器…"), param_group);
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
                                   speed_spin_->value(), false);
            }
        });
        // 目标位置回车直接下发（仅回车触发，失焦不触发）。
        connect(ticks_spin_->findChild<QLineEdit*>(), &QLineEdit::returnPressed, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), false);
            }
        });
        // 回中：目标设为编码器中位并直接下发。
        connect(home_button_, &QPushButton::clicked, this, [this] {
            if (!hasSubject()) {
                return;
            }
            ticks_spin_->setValue(static_cast<int>(subject_config_.center_ticks));
            emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                               speed_spin_->value(), false);
        });

        connect(goal_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), false);
            }
        });
        connect(goal_all_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit goalRequested(subject_port_, ticks_spin_->value(), time_spin_->value(),
                                   speed_spin_->value(), true);
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
