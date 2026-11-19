#include "app/panels/ubtech_panel.hpp"

#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace app {
    namespace {
        /// 协议物理中位（0–240° 绝对角的中间值）。
        constexpr int kCenterAngleDeg = 120;
    } // namespace

    UbtechPanel::UbtechPanel(QWidget* parent) :
        QWidget(parent) {
        auto* scroll = new QScrollArea(this);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        auto* body = new QWidget(scroll);
        auto* layout = new QVBoxLayout(body);
        layout->setContentsMargins(0, 0, 4, 0);
        layout->setSpacing(10);
        scroll->setWidget(body);

        auto* outer = new QVBoxLayout(this);
        outer->setContentsMargins(0, 0, 0, 0);
        outer->addWidget(scroll);

        title_label_ = new QLabel(tr("UBTECH 舵机（未选中）"), body);
        title_label_->setObjectName("title");
        layout->addWidget(title_label_);
        auto* hint = new QLabel(tr("非官方协议：仅支持转动/偏移/ID 等少量命令，无连续遥测"), body);
        hint->setWordWrap(true);
        layout->addWidget(hint);

        // ---------- 运动 ----------
        auto* motion_group = new QGroupBox(tr("运动"), body);
        auto* motion_layout = new QVBoxLayout(motion_group);
        motion_layout->setSpacing(8);

        auto* angle_row = new QHBoxLayout;
        angle_row->addWidget(new QLabel(tr("目标角度 (°)"), motion_group));
        angle_spin_ = new QSpinBox(motion_group);
        angle_spin_->setRange(0, 240);
        angle_spin_->setValue(kCenterAngleDeg);
        angle_spin_->setToolTip(tr("协议目标角度 0–240°（绝对角）；给定值大于 240 也只转到 240°"));
        angle_row->addWidget(angle_spin_);
        angle_slider_ = new QSlider(Qt::Horizontal, motion_group);
        angle_slider_->setRange(0, 240);
        angle_slider_->setValue(kCenterAngleDeg);
        angle_row->addWidget(angle_slider_, 1);
        motion_layout->addLayout(angle_row);

        auto* move_time_row = new QHBoxLayout;
        move_time_row->addWidget(new QLabel(tr("运动时间 (ms)"), motion_group));
        move_time_spin_ = new QSpinBox(motion_group);
        move_time_spin_->setRange(0, 5100);
        move_time_spin_->setSingleStep(20);
        move_time_spin_->setValue(1000);
        move_time_spin_->setToolTip(tr("运动快慢，协议粒度 20ms（0–255×20ms）；0 = 全速转动"));
        move_time_row->addWidget(move_time_spin_);
        move_time_row->addStretch(1);
        motion_layout->addLayout(move_time_row);

        auto* lock_time_row = new QHBoxLayout;
        lock_time_row->addWidget(new QLabel(tr("锁定时间 (ms)"), motion_group));
        lock_time_spin_ = new QSpinBox(motion_group);
        lock_time_spin_->setRange(0, 65400);
        lock_time_spin_->setSingleStep(20);
        lock_time_spin_->setValue(0);
        lock_time_spin_->setToolTip(
            tr("锁定时间，协议粒度 20ms（0–3270×20ms）；从开始转动计时，锁定期内舵机忽略新目标；\n"
               "0 = 不锁定"));
        lock_time_row->addWidget(lock_time_spin_);
        lock_time_row->addStretch(1);
        motion_layout->addLayout(lock_time_row);

        auto* goal_row = new QHBoxLayout;
        goal_button_ = new QPushButton(tr("下发目标"), motion_group);
        home_button_ = new QPushButton(tr("转中位 120°"), motion_group);
        goal_row->addWidget(goal_button_);
        goal_row->addWidget(home_button_);
        goal_row->addStretch(1);
        motion_layout->addLayout(goal_row);

        stop_button_ = new QPushButton(tr("强制中止（失电）"), motion_group);
        stop_button_->setToolTip(tr("协议 0x01 FF：立即停转并失电，仅靠齿轮阻尼维持位置"));
        motion_layout->addWidget(stop_button_);

        auto* read_row = new QHBoxLayout;
        read_angle_button_ = new QPushButton(tr("读取角度"), motion_group);
        read_angle_button_->setToolTip(
            tr("⚠ 协议副作用：返回角度数据后舵机立即失电（仅靠齿轮阻尼维持位置），请手动单次使用"));
        read_row->addWidget(read_angle_button_);
        angle_readout_ = new QLabel(tr("实际：--"), motion_group);
        read_row->addWidget(angle_readout_, 1);
        motion_layout->addLayout(read_row);

        layout->addWidget(motion_group);

        // ---------- 校准 ----------
        auto* trim_group = new QGroupBox(tr("角度偏移"), body);
        auto* trim_layout = new QVBoxLayout(trim_group);
        trim_layout->setSpacing(8);

        auto* offset_row = new QHBoxLayout;
        offset_row->addWidget(new QLabel(tr("偏移 (1/3°)"), trim_group));
        offset_spin_ = new QSpinBox(trim_group);
        offset_spin_->setRange(-90, 90);
        offset_spin_->setToolTip(tr("±90 ≈ ±30°；面向舵盘正=顺时针、负=逆时针；掉电清零"));
        offset_row->addWidget(offset_spin_);
        offset_read_button_ = new QPushButton(tr("读取"), trim_group);
        offset_write_button_ = new QPushButton(tr("写入"), trim_group);
        offset_row->addWidget(offset_read_button_);
        offset_row->addWidget(offset_write_button_);
        offset_row->addStretch(1);
        trim_layout->addLayout(offset_row);

        offset_hint_ = new QLabel(tr("当前偏移：未读取"), trim_group);
        trim_layout->addWidget(offset_hint_);

        layout->addWidget(trim_group);

        // ---------- 身份 ----------
        auto* identity_group = new QGroupBox(tr("身份"), body);
        auto* identity_layout = new QVBoxLayout(identity_group);
        identity_layout->setSpacing(8);

        auto* id_row = new QHBoxLayout;
        id_row->addWidget(new QLabel(tr("舵机 ID"), identity_group));
        id_spin_ = new QSpinBox(identity_group);
        id_spin_->setRange(1, 240);
        id_spin_->setToolTip(tr("协议 ID 1–240（0 为广播）；修改立即生效，协议无 ID 查询命令"));
        id_row->addWidget(id_spin_);
        id_write_button_ = new QPushButton(tr("修改 ID…"), identity_group);
        id_row->addWidget(id_write_button_);
        id_row->addStretch(1);
        identity_layout->addLayout(id_row);

        auto* version_row = new QHBoxLayout;
        version_read_button_ = new QPushButton(tr("读取固件版本"), identity_group);
        version_row->addWidget(version_read_button_);
        version_label_ = new QLabel(tr("版本：--"), identity_group);
        version_row->addWidget(version_label_, 1);
        identity_layout->addLayout(version_row);

        layout->addWidget(identity_group);

        // ---------- 参数 ----------
        auto* param_group = new QGroupBox(tr("参数"), body);
        auto* param_layout = new QVBoxLayout(param_group);
        register_button_ = new QPushButton(tr("寄存器工作台…"), param_group);
        param_layout->addWidget(register_button_);
        layout->addWidget(param_group);

        layout->addStretch(1);

        // 目标角度滑杆与数字框双向同步。
        connect(angle_spin_, &QSpinBox::valueChanged, angle_slider_, &QSlider::setValue);
        connect(angle_slider_, &QSlider::valueChanged, angle_spin_, &QSpinBox::setValue);

        connect(goal_button_, &QPushButton::clicked, this, [this] {
            if (!hasSubject()) {
                return;
            }
            emit moveRequested(subject_port_, subject_id_, angle_spin_->value(),
                               move_time_spin_->value(), lock_time_spin_->value());
        });
        connect(home_button_, &QPushButton::clicked, this, [this] {
            if (!hasSubject()) {
                return;
            }
            angle_spin_->setValue(kCenterAngleDeg);
            emit moveRequested(subject_port_, subject_id_, kCenterAngleDeg,
                               move_time_spin_->value(), lock_time_spin_->value());
        });
        connect(stop_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit stopRequested(subject_port_, subject_id_);
            }
        });
        connect(read_angle_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit readAngleRequested(subject_port_, subject_id_);
            }
        });
        connect(offset_read_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit paramReloadRequested(subject_port_, subject_id_);
            }
        });
        connect(offset_write_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit paramWriteRequested(subject_port_, subject_id_,
                                         QStringLiteral("angle_offset"), offset_spin_->value(),
                                         QString::number(offset_spin_->value()));
            }
        });
        connect(id_write_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit paramWriteRequested(subject_port_, subject_id_, QStringLiteral("id"),
                                         id_spin_->value(), QString::number(id_spin_->value()));
            }
        });
        connect(version_read_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit paramReloadRequested(subject_port_, subject_id_);
            }
        });
        connect(register_button_, &QPushButton::clicked, this,
                [this] { emit registerRequested(); });

        setManualControlsEnabled(false);
    }

    bool UbtechPanel::hasSubject() const {
        return has_subject_;
    }

    void UbtechPanel::setManualControlsEnabled(const bool enabled) const {
        angle_spin_->setEnabled(enabled);
        angle_slider_->setEnabled(enabled);
        move_time_spin_->setEnabled(enabled);
        lock_time_spin_->setEnabled(enabled);
        goal_button_->setEnabled(enabled);
        home_button_->setEnabled(enabled);
        stop_button_->setEnabled(enabled);
        read_angle_button_->setEnabled(enabled);
        offset_spin_->setEnabled(enabled);
        offset_read_button_->setEnabled(enabled);
        offset_write_button_->setEnabled(enabled);
        id_spin_->setEnabled(enabled);
        id_write_button_->setEnabled(enabled);
        version_read_button_->setEnabled(enabled);
        register_button_->setEnabled(enabled);
    }

    void UbtechPanel::setSubject(const QString& port, const servo_core::ServoId id) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        has_subject_ = true;
        title_label_->setText(tr("UBTECH 舵机：%1 · id %2").arg(port).arg(id));
        angle_readout_->setText(tr("实际：--"));
        offset_hint_->setText(tr("当前偏移：未读取"));
        version_label_->setText(tr("版本：--"));
        id_spin_->setValue(id);
        setManualControlsEnabled(true);
    }

    void UbtechPanel::clear() {
        has_subject_ = false;
        title_label_->setText(tr("UBTECH 舵机（未选中）"));
        angle_readout_->setText(tr("实际：--"));
        offset_hint_->setText(tr("当前偏移：未读取"));
        version_label_->setText(tr("版本：--"));
        setManualControlsEnabled(false);
    }

    void UbtechPanel::updateState(const QString& port, const servo_core::ServoState& state) {
        if (!has_subject_ || port != subject_port_ || state.id != subject_id_) {
            return;
        }
        angle_readout_->setText(
            tr("实际：%1°（%2）")
                .arg(state.position_deg)
                .arg(state.moving ? tr("运动中/未到位") : tr("已到位")));
    }

    void UbtechPanel::setParameters(const QString& port, const int id,
                                    const QVector<ParamValue>& values) {
        if (!has_subject_ || port != subject_port_ || id != subject_id_) {
            return;
        }
        QVector<int> version;
        for (const auto& value : values) {
            if (value.key == QStringLiteral("angle_offset") && value.valid) {
                offset_spin_->setValue(value.value);
                offset_hint_->setText(tr("当前偏移：%1（≈ %2°）")
                                          .arg(value.value)
                                          .arg(value.value / 3.0, 0, 'f', 1));
            }
            else if (value.key.startsWith(QStringLiteral("firmware_v"))) {
                version.push_back(value.valid ? value.value : -1);
            }
        }
        if (version.size() == 4) {
            version_label_->setText(tr("版本：%1.%2.%3.%4")
                                        .arg(version[0])
                                        .arg(version[1])
                                        .arg(version[2])
                                        .arg(version[3]));
        }
    }

} // namespace app
