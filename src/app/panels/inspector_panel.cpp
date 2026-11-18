#include "app/panels/inspector_panel.hpp"

#include <QAbstractButton>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QEvent>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

namespace app {

    namespace {
        enum ParamColumn {
            kColumnKey = 0,
            kColumnValue,
            kColumnArea,
            kColumnAction,
            kParamColumnCount,
        };
    }

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

        // ---------- 参数区 ----------
        auto* param_group = new QGroupBox(tr("参数"), this);
        auto* param_layout = new QVBoxLayout(param_group);
        auto* param_bar = new QHBoxLayout;
        reload_button_ = new QPushButton(tr("重新读取"), param_group);
        write_all_button_ = new QPushButton(tr("写入到全部舵机"), param_group);
        backup_button_ = new QPushButton(tr("备份参数"), param_group);
        restore_button_ = new QPushButton(tr("恢复参数"), param_group);
        param_bar->addWidget(reload_button_);
        param_bar->addWidget(write_all_button_);
        param_bar->addStretch(1);
        param_bar->addWidget(backup_button_);
        param_bar->addWidget(restore_button_);
        param_layout->addLayout(param_bar);

        param_table_ = new QTableWidget(param_group);
        param_table_->installEventFilter(this);
        param_table_->setColumnCount(kParamColumnCount);
        param_table_->setHorizontalHeaderLabels({tr("参数"), tr("值"), tr("区域"), tr("操作")});
        param_table_->verticalHeader()->setVisible(false);
        param_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        param_table_->horizontalHeader()->setMinimumSectionSize(80);
        param_table_->horizontalHeader()->setSectionResizeMode(kColumnKey, QHeaderView::Stretch);
        param_table_->horizontalHeader()->setSectionResizeMode(kColumnValue,
                                                               QHeaderView::Fixed);
        param_table_->horizontalHeader()->resizeSection(kColumnValue, 110);
        param_table_->horizontalHeader()->setSectionResizeMode(kColumnArea,
                                                               QHeaderView::ResizeToContents);
        param_table_->horizontalHeader()->setSectionResizeMode(kColumnAction,
                                                               QHeaderView::ResizeToContents);
        param_layout->addWidget(param_table_);
        layout->addWidget(param_group, 1);

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

        // 参数值点击进入编辑；换行/点击别处即提交，避免悬停滚轮误触。
        connect(param_table_, &QTableWidget::cellClicked, this,
                [this](const int row, const int column) {
                    if (column == kColumnValue) {
                        beginEdit(row);
                    }
                    else {
                        commitEditor();
                    }
                });
        connect(param_table_, &QTableWidget::currentCellChanged, this,
                [this] { commitEditor(); });

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
        connect(reload_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit reloadParametersRequested(subject_port_, subject_id_);
            }
        });
        connect(write_all_button_, &QPushButton::clicked, this, [this] {
            const int row = param_table_->currentRow();
            if (!hasSubject() || row < 0) {
                return;
            }
            commitEditor();
            if (row >= static_cast<int>(param_rows_.size())) {
                return;
            }
            const ParamValue& parameter = param_rows_[row];
            emit paramWriteAllRequested(subject_port_, parameter.key, parameter.value);
        });
        connect(backup_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit backupRequested(subject_port_, subject_id_);
            }
        });
        connect(restore_button_, &QPushButton::clicked, this, [this] {
            if (hasSubject()) {
                emit restoreRequested(subject_port_, subject_id_);
            }
        });
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
                                    const servo_core::ServoConfig& config) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        subject_config_ = config;
        has_subject_ = true;

        updateConfig(config);
        ticks_spin_->setValue(static_cast<int>(config.center_ticks));
        param_table_->setRowCount(0);
    }

    void InspectorPanel::clear() {
        has_subject_ = false;
        param_table_->setRowCount(0);
    }

    void InspectorPanel::onGoalValueChanged() {
        if (!has_subject_) {
            deg_label_->setText("= 0.00°");
            return;
        }
        deg_label_->setText(QStringLiteral("= %1°").arg(
            servo_core::ticks_to_degrees(subject_config_, ticks_spin_->value()), 0, 'f', 2));
    }

    void InspectorPanel::setParameters(const QString& port, const int id,
                                       const QVector<ParamValue>& values) {
        if (!has_subject_ || port != subject_port_ || id != subject_id_) {
            return;
        }
        rebuildParameterRows(values);
    }

    QString valueTextFor(const ParamValue& parameter) {
        if (!parameter.valid) {
            return "--";
        }
        if (!parameter.choices.isEmpty() && parameter.value >= 0 &&
            parameter.value < parameter.choices.size()) {
            return QString::number(parameter.choices[parameter.value]);
        }
        return QString::number(parameter.value);
    }

    void InspectorPanel::rebuildParameterRows(const QVector<ParamValue>& values) {
        commitEditor();
        param_rows_.assign(values.begin(), values.end());
        param_table_->setRowCount(values.size());
        for (int row = 0; row < values.size(); ++row) {
            const ParamValue& parameter = param_rows_[row];

            auto* key_item = new QTableWidgetItem(
                parameter.unit.isEmpty() ? parameter.key
                                         : QStringLiteral("%1 (%2)").arg(parameter.key,
                                                                         parameter.unit));
            key_item->setFlags(key_item->flags() & ~Qt::ItemIsEditable);
            key_item->setToolTip(parameter.label);

            auto* value_item = new QTableWidgetItem(valueTextFor(parameter));
            value_item->setFlags(value_item->flags() & ~Qt::ItemIsEditable);

            param_table_->setItem(row, kColumnKey, key_item);
            param_table_->setItem(row, kColumnValue, value_item);
            param_table_->setItem(row, kColumnArea,
                                  new QTableWidgetItem(parameter.eeprom ? tr("EEPROM") : tr("RAM")));

            auto* write_button = new QPushButton(tr("写入"));
            param_table_->setCellWidget(row, kColumnAction, write_button);
            connect(write_button, &QPushButton::clicked, this, [this, row] {
                if (!hasSubject()) {
                    return;
                }
                commitEditor();
                if (row >= static_cast<int>(param_rows_.size())) {
                    return;
                }
                const ParamValue& parameter = param_rows_[row];
                emit paramWriteRequested(subject_port_, subject_id_, parameter.key,
                                         parameter.value, valueTextFor(parameter));
            });
        }
    }

    void InspectorPanel::beginEdit(const int row) {
        if (editing_row_ == row) {
            return;
        }
        commitEditor();
        if (row < 0 || row >= static_cast<int>(param_rows_.size())) {
            return;
        }
        const ParamValue& parameter = param_rows_[row];
        QWidget* editor = nullptr;
        if (!parameter.choices.isEmpty()) {
            // 语义：参数值 = choices 的下标；条目文本为该取值的实际含义。
            auto* combo = new QComboBox;
            for (int index = 0; index < parameter.choices.size(); ++index) {
                combo->addItem(QString::number(parameter.choices[index]), index);
            }
            if (parameter.value >= 0 && parameter.value < parameter.choices.size()) {
                combo->setCurrentIndex(parameter.value);
            }
            editor = combo;
        }
        else {
            auto* spin = new QSpinBox;
            spin->setRange(parameter.min, parameter.max);
            spin->setValue(parameter.value);
            editor = spin;
        }
        editor->installEventFilter(this);
        param_table_->setCellWidget(row, kColumnValue, editor);
        editing_row_ = row;
        editor->setFocus();
    }

    void InspectorPanel::commitEditor() {
        if (editing_row_ < 0) {
            return;
        }
        const int row = editing_row_;
        editing_row_ = -1;
        QWidget* editor = param_table_->cellWidget(row, kColumnValue);
        if (editor == nullptr || row >= static_cast<int>(param_rows_.size())) {
            return;
        }
        ParamValue& parameter = param_rows_[row];
        if (const auto* combo = qobject_cast<QComboBox*>(editor)) {
            parameter.value = combo->currentData().toInt();
        }
        else if (const auto* spin = qobject_cast<QSpinBox*>(editor)) {
            parameter.value = spin->value();
        }
        parameter.valid = true;
        param_table_->removeCellWidget(row, kColumnValue);
        editor->deleteLater();
        param_table_->item(row, kColumnValue)->setText(valueTextFor(parameter));
    }

    bool InspectorPanel::eventFilter(QObject* watched, QEvent* event) {
        // 编辑控件未获焦点时忽略滚轮，杜绝悬停滚动误改值。
        if (event->type() == QEvent::Wheel && watched->isWidgetType() &&
            !static_cast<QWidget*>(watched)->hasFocus()) {
            return true;
        }
        // 编辑控件失焦即提交；焦点真正离开表格族时清除选中高亮，
        // 让行恢复与未编辑状态一致的样式。焦点移到表格内部（编辑控件）
        // 不算离开，避免编辑刚打开就被收掉。
        if (event->type() == QEvent::FocusOut) {
            QWidget* next = QApplication::focusWidget();
            const bool left_table = next == nullptr || !param_table_->isAncestorOf(next);
            if (watched == param_table_) {
                if (left_table) {
                    commitEditor();
                    param_table_->clearSelection();
                }
            }
            else if (watched->isWidgetType() &&
                     param_table_->isAncestorOf(static_cast<QWidget*>(watched))) {
                commitEditor();
                if (left_table) {
                    param_table_->clearSelection();
                }
            }
        }
        return QWidget::eventFilter(watched, event);
    }

} // namespace app
