#include "app/panels/inspector_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
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

        subject_label_ = new QLabel(tr("未选中舵机"), this);
        subject_label_->setObjectName("title");
        layout->addWidget(subject_label_);

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
        jog_minus_button_ = new QPushButton(tr("−1°"), command_group);
        jog_plus_button_ = new QPushButton(tr("+1°"), command_group);
        goal_row->addWidget(goal_button_);
        goal_row->addWidget(goal_all_button_);
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
            if (hasSubject() && row >= 0) {
                emit paramWriteAllRequested(subject_port_,
                                            param_table_->item(row, kColumnKey)->text(),
                                            param_table_->item(row, kColumnValue)->data(Qt::UserRole).toInt());
            }
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
                                    const QString& model, const servo_core::ServoConfig& config) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        subject_model_ = model;
        subject_config_ = config;
        has_subject_ = true;

        subject_label_->setText(tr("%1 · id %2 · %3").arg(port).arg(id).arg(model));
        updateConfig(config);
        ticks_spin_->setValue(static_cast<int>(config.center_ticks));
        param_table_->setRowCount(0);
    }

    void InspectorPanel::clear() {
        has_subject_ = false;
        subject_label_->setText(tr("未选中舵机"));
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

    void InspectorPanel::rebuildParameterRows(const QVector<ParamValue>& values) {
        param_table_->setRowCount(values.size());
        for (int row = 0; row < values.size(); ++row) {
            const ParamValue& parameter = values[row];

            auto* key_item = new QTableWidgetItem(
                parameter.unit.isEmpty() ? parameter.key
                                         : QStringLiteral("%1 (%2)").arg(parameter.key,
                                                                         parameter.unit));
            key_item->setFlags(key_item->flags() & ~Qt::ItemIsEditable);
            param_table_->setItem(row, kColumnKey, key_item);
            key_item->setToolTip(parameter.label);

            QWidget* editor = nullptr;
            if (!parameter.choices.isEmpty()) {
                // 语义：参数值 = choices 的下标；条目文本为该取值的实际含义。
                auto* combo = new QComboBox;
                for (int index = 0; index < parameter.choices.size(); ++index) {
                    combo->addItem(QString::number(parameter.choices[index]), index);
                }
                if (parameter.valid && parameter.value >= 0 &&
                    parameter.value < parameter.choices.size()) {
                    combo->setCurrentIndex(parameter.value);
                }
                editor = combo;
            }
            else {
                auto* spin = new QSpinBox;
                spin->setRange(parameter.min, parameter.max);
                if (parameter.valid) {
                    spin->setValue(parameter.value);
                }
                else {
                    spin->setSpecialValueText("--");
                }
                editor = spin;
            }
            param_table_->setCellWidget(row, kColumnValue, editor);
            // 提交时以 UserRole 携带当前值。
            auto* value_item = new QTableWidgetItem;
            value_item->setData(Qt::UserRole, parameter.valid ? parameter.value : parameter.min);
            param_table_->setItem(row, kColumnValue, value_item);
            const auto store_value = [value_item](const int value) {
                value_item->setData(Qt::UserRole, value);
            };
            if (auto* combo = qobject_cast<QComboBox*>(editor)) {
                connect(combo, &QComboBox::currentIndexChanged, combo, [combo, store_value] {
                    store_value(combo->currentData().toInt());
                });
            }
            else if (auto* spin = qobject_cast<QSpinBox*>(editor)) {
                connect(spin, &QSpinBox::valueChanged, spin, [store_value](const int value) {
                    store_value(value);
                });
            }

            param_table_->setItem(row, kColumnArea,
                                  new QTableWidgetItem(parameter.eeprom ? tr("EEPROM") : tr("RAM")));

            auto* write_button = new QPushButton(tr("写入"));
            param_table_->setCellWidget(row, kColumnAction, write_button);
            connect(write_button, &QPushButton::clicked, this, [this, row] {
                if (!hasSubject()) {
                    return;
                }
                QString value_text;
                if (const auto* combo =
                        qobject_cast<QComboBox*>(param_table_->cellWidget(row, kColumnValue))) {
                    value_text = combo->currentText();
                }
                else if (const auto* spin =
                             qobject_cast<QSpinBox*>(param_table_->cellWidget(row, kColumnValue))) {
                    value_text = spin->text();
                }
                emit paramWriteRequested(subject_port_, subject_id_,
                                         param_table_->item(row, kColumnKey)->text(),
                                         param_table_->item(row, kColumnValue)
                                             ->data(Qt::UserRole)
                                             .toInt(),
                                         value_text);
            });
        }
    }

} // namespace app
