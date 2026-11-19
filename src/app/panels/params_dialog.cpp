#include "app/panels/params_dialog.hpp"

#include <QApplication>
#include <QComboBox>
#include <QEvent>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

namespace app {

    namespace {
        enum Column {
            kColumnAddress = 0,
            kColumnKey,
            kColumnZh,
            kColumnValue,
            kColumnArea,
            kColumnAction,
            kColumnCount,
        };

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
    } // namespace

    ParamsDialog::ParamsDialog(QWidget* parent) :
        QDialog(parent) {
        setWindowTitle(tr("寄存器"));
        resize(880, 640);

        auto* layout = new QVBoxLayout(this);
        layout->setSpacing(8);

        subject_label_ = new QLabel(tr("未选中舵机"), this);
        subject_label_->setObjectName("title");
        layout->addWidget(subject_label_);

        auto* bar = new QHBoxLayout;
        reload_button_ = new QPushButton(tr("重新读取"), this);
        write_all_button_ = new QPushButton(tr("写入到全部舵机"), this);
        backup_button_ = new QPushButton(tr("备份参数"), this);
        restore_button_ = new QPushButton(tr("恢复参数"), this);
        reload_button_->setToolTip(tr("重新读取全部寄存器的当前值"));
        write_all_button_->setToolTip(tr("将选中行参数写入总线上全部舵机（需确认）"));
        backup_button_->setToolTip(tr("将整表参数备份为 JSON 文件（含型号与 ID）"));
        restore_button_->setToolTip(tr("从备份文件恢复参数（自动跳过 ID 与波特率）"));
        bar->addWidget(reload_button_);
        bar->addWidget(write_all_button_);
        bar->addStretch(1);
        bar->addWidget(backup_button_);
        bar->addWidget(restore_button_);
        layout->addLayout(bar);

        table_ = new QTableWidget(this);
        table_->setToolTip(tr("点击值列编辑可修行；只读行仅展示。写入点操作列按钮并确认"));
        table_->installEventFilter(this);
        table_->setColumnCount(kColumnCount);
        table_->setHorizontalHeaderLabels(
            {tr("地址"), tr("英文名"), tr("中文名"), tr("值"), tr("区域"), tr("操作")});
        table_->verticalHeader()->setVisible(false);
        table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_->setAlternatingRowColors(true);
        auto* header = table_->horizontalHeader();
        header->setSectionResizeMode(kColumnAddress, QHeaderView::ResizeToContents);
        header->setSectionResizeMode(kColumnKey, QHeaderView::Stretch);
        header->setSectionResizeMode(kColumnZh, QHeaderView::ResizeToContents);
        header->setSectionResizeMode(kColumnValue, QHeaderView::Fixed);
        header->resizeSection(kColumnValue, 110);
        header->setSectionResizeMode(kColumnArea, QHeaderView::Fixed);
        header->resizeSection(kColumnArea, 110);
        header->setSectionResizeMode(kColumnAction, QHeaderView::Interactive);
        header->resizeSection(kColumnAction, 110);
        // 加大行高，写入按钮不再顶满单元格。
        table_->verticalHeader()->setDefaultSectionSize(38);
        layout->addWidget(table_, 1);

        // 点击值列进入编辑；换行/点击别处即提交，避免悬停滚轮误触。
        connect(table_, &QTableWidget::cellClicked, this, [this](const int row, const int column) {
            if (column == kColumnValue) {
                beginEdit(row);
            }
            else {
                commitEditor();
            }
        });
        connect(table_, &QTableWidget::currentCellChanged, this, [this] { commitEditor(); });

        connect(reload_button_, &QPushButton::clicked, this, [this] {
            if (has_subject_) {
                emit reloadParametersRequested(port_, id_);
            }
        });
        connect(write_all_button_, &QPushButton::clicked, this, [this] {
            const int row = table_->currentRow();
            if (!has_subject_ || row < 0) {
                return;
            }
            commitEditor();
            if (row >= static_cast<int>(rows_.size())) {
                return;
            }
            emit paramWriteAllRequested(port_, rows_[row].key, rows_[row].value);
        });
        connect(backup_button_, &QPushButton::clicked, this, [this] {
            if (has_subject_) {
                emit backupRequested(port_, id_);
            }
        });
        connect(restore_button_, &QPushButton::clicked, this, [this] {
            if (has_subject_) {
                emit restoreRequested(port_, id_);
            }
        });
    }

    void ParamsDialog::setSubject(const QString& port, const servo_core::ServoId id) {
        if (has_subject_ && port_ == port && id_ == id) {
            return;
        }
        port_ = port;
        id_ = id;
        has_subject_ = true;
        subject_label_->setText(tr("寄存器：%1 · id %2").arg(port).arg(id));
        setWindowTitle(tr("寄存器 · %1 · id %2").arg(port).arg(id));
        commitEditor();
        rows_.clear();
        table_->setRowCount(0);
    }

    void ParamsDialog::setParameters(const QString& port, const int id,
                                     const QVector<ParamValue>& values) {
        if (!has_subject_ || port != port_ || id != id_) {
            return;
        }
        rebuildRows(values);
    }

    void ParamsDialog::rebuildRows(const QVector<ParamValue>& values) {
        commitEditor();
        rows_.assign(values.begin(), values.end());
        table_->setRowCount(values.size());
        for (int row = 0; row < values.size(); ++row) {
            const ParamValue& parameter = rows_[row];

            auto make_item = [](const QString& text) {
                auto* item = new QTableWidgetItem(text);
                item->setFlags(item->flags() & ~Qt::ItemIsEditable);
                return item;
            };
            table_->setItem(row, kColumnAddress, make_item(QString::number(parameter.address)));
            table_->setItem(row, kColumnKey, make_item(parameter.key));
            auto* zh_item = make_item(parameter.zh);
            zh_item->setToolTip(parameter.label);
            table_->setItem(row, kColumnZh, zh_item);
            table_->setItem(row, kColumnValue, make_item(valueTextFor(parameter)));
            table_->setItem(row, kColumnArea, make_item(parameter.area));

            if (parameter.read_only) {
                auto* read_only_item = make_item(tr("只读"));
                read_only_item->setTextAlignment(Qt::AlignCenter);
                table_->setItem(row, kColumnAction, read_only_item);
                continue;
            }
            auto* write_button = new QPushButton(tr("写入"));
            table_->setCellWidget(row, kColumnAction, write_button);
            connect(write_button, &QPushButton::clicked, this, [this, row] {
                if (!has_subject_) {
                    return;
                }
                commitEditor();
                if (row >= static_cast<int>(rows_.size())) {
                    return;
                }
                emit paramWriteRequested(port_, id_, rows_[row].key, rows_[row].value,
                                         valueTextFor(rows_[row]));
            });
        }
    }

    void ParamsDialog::beginEdit(const int row) {
        if (editing_row_ == row) {
            return;
        }
        commitEditor();
        if (row < 0 || row >= static_cast<int>(rows_.size()) || rows_[row].read_only) {
            return;
        }
        const ParamValue& parameter = rows_[row];
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
        table_->setCellWidget(row, kColumnValue, editor);
        editing_row_ = row;
        editor->setFocus();
    }

    void ParamsDialog::commitEditor() {
        if (editing_row_ < 0) {
            return;
        }
        const int row = editing_row_;
        editing_row_ = -1;
        QWidget* editor = table_->cellWidget(row, kColumnValue);
        if (editor == nullptr || row >= static_cast<int>(rows_.size())) {
            return;
        }
        ParamValue& parameter = rows_[row];
        if (const auto* combo = qobject_cast<QComboBox*>(editor)) {
            parameter.value = combo->currentData().toInt();
        }
        else if (const auto* spin = qobject_cast<QSpinBox*>(editor)) {
            parameter.value = spin->value();
        }
        parameter.valid = true;
        table_->removeCellWidget(row, kColumnValue);
        editor->deleteLater();
        table_->item(row, kColumnValue)->setText(valueTextFor(parameter));
    }

    bool ParamsDialog::eventFilter(QObject* watched, QEvent* event) {
        // 编辑控件未获焦点时忽略滚轮，杜绝悬停滚动误改值。
        if (event->type() == QEvent::Wheel && watched->isWidgetType() &&
            !static_cast<QWidget*>(watched)->hasFocus()) {
            return true;
        }
        // 编辑控件失焦即提交；焦点真正离开表格族时清除选中高亮。
        if (event->type() == QEvent::FocusOut) {
            QWidget* next = QApplication::focusWidget();
            const bool left_table = next == nullptr || !table_->isAncestorOf(next);
            if (watched == table_) {
                if (left_table) {
                    commitEditor();
                    table_->clearSelection();
                }
            }
            else if (watched->isWidgetType() &&
                     table_->isAncestorOf(static_cast<QWidget*>(watched))) {
                commitEditor();
                if (left_table) {
                    table_->clearSelection();
                }
            }
        }
        return QDialog::eventFilter(watched, event);
    }

} // namespace app
