#include "app/panels/batch_dialog.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QEvent>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

namespace app {

    namespace {
        enum Column {
            kColumnPort = 0,
            kColumnId,
            kColumnGoal,
            kColumnDeg,
            kColumnSpeed,
            kColumnTime,
            kColumnInclude,
            kColumnCount,
        };
    } // namespace

    BatchDialog::BatchDialog(QWidget* parent) :
        QDialog(parent) {
        setWindowTitle(tr("批量操作"));
        resize(720, 560);

        auto* layout = new QVBoxLayout(this);
        layout->setSpacing(8);
        // 弹窗默认宽度 = 表格宽度：水平边距归零，按钮行自带内边距。
        layout->setContentsMargins(0, 0, 0, 6);

        auto* bar = new QHBoxLayout;
        refresh_button_ = new QPushButton(tr("刷新"), this);
        refresh_button_->setToolTip(tr("按当前连接的总线重建行（保留已编辑值）"));
        sync_button_ = new QPushButton(tr("同步下发选中"), this);
        sync_button_->setToolTip(tr("各行自己的目标值组包 SYNC WRITE，一帧/总线同时生效"));
        stage_button_ = new QPushButton(tr("异步预存选中"), this);
        stage_button_->setToolTip(tr("逐台 REG_WRITE 预存（有应答可确认），舵机不动作"));
        trigger_button_ = new QPushButton(tr("触发"), this);
        trigger_button_->setToolTip(tr("ACTION 触发所有已预存目标生效（未预存的不动）"));
        bar->setContentsMargins(8, 8, 8, 2);
        bar->addWidget(refresh_button_);
        bar->addWidget(sync_button_);
        bar->addWidget(stage_button_);
        bar->addWidget(trigger_button_);
        bar->addStretch(1);
        layout->addLayout(bar);

        table_ = new QTableWidget(this);
        table_->installEventFilter(this);
        table_->setColumnCount(kColumnCount);
        table_->setHorizontalHeaderLabels(
            {tr("总线"), tr("ID"), tr("目标(tick)"), tr("目标(°)"), tr("速度"), tr("时间"),
             tr("参与")});
        table_->verticalHeader()->setVisible(false);
        table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_->setAlternatingRowColors(true);
        auto* header = table_->horizontalHeader();
        header->setSectionResizeMode(kColumnPort, QHeaderView::ResizeToContents);
        header->setSectionResizeMode(kColumnId, QHeaderView::ResizeToContents);
        header->setSectionResizeMode(kColumnGoal, QHeaderView::Fixed);
        header->resizeSection(kColumnGoal, 110);
        header->setSectionResizeMode(kColumnDeg, QHeaderView::ResizeToContents);
        header->setSectionResizeMode(kColumnSpeed, QHeaderView::Fixed);
        header->resizeSection(kColumnSpeed, 90);
        header->setSectionResizeMode(kColumnTime, QHeaderView::Fixed);
        header->resizeSection(kColumnTime, 90);
        header->setSectionResizeMode(kColumnInclude, QHeaderView::Fixed);
        header->resizeSection(kColumnInclude, 64);
        table_->setMinimumHeight(320);
        layout->addWidget(table_, 1);

        // 点击目标/速度/时间列进入编辑；换行/点别处提交。
        connect(table_, &QTableWidget::cellClicked, this, [this](const int row, const int column) {
            if (column == kColumnGoal || column == kColumnSpeed || column == kColumnTime) {
                beginEdit(row, column);
            }
            else {
                commitEditor();
            }
        });
        connect(table_, &QTableWidget::currentCellChanged, this, [this] { commitEditor(); });

        connect(refresh_button_, &QPushButton::clicked, this, [this] { emit refreshRequested(); });
        connect(sync_button_, &QPushButton::clicked, this,
                [this] { emit syncRequested(collectGoals()); });
        connect(stage_button_, &QPushButton::clicked, this,
                [this] { emit stageRequested(collectGoals()); });
        connect(trigger_button_, &QPushButton::clicked, this, [this] { emit triggerRequested(); });

        setStagedSupported(false);
    }

    void BatchDialog::setStagedSupported(const bool supported) {
        staged_supported_ = supported;
        stage_button_->setEnabled(supported);
        trigger_button_->setEnabled(supported);
    }

    void BatchDialog::setServos(const QVector<BatchServo>& servos) {
        commitEditor();
        // 保留同 id 行的已编辑值。
        std::vector<Row> previous;
        previous.swap(rows_);
        for (const auto& [port, id, config] : servos) {
            Row row;
            row.port = port;
            row.id = id;
            row.config = config;
            for (const auto& old : previous) {
                if (old.port == port && old.id == id) {
                    row = old;
                    row.config = config;
                    break;
                }
            }
            rows_.push_back(row);
        }
        rebuildRows(servos);
    }

    void BatchDialog::rebuildRows(const QVector<BatchServo>& /*servos*/) {
        table_->setRowCount(static_cast<int>(rows_.size()));
        for (int row = 0; row < static_cast<int>(rows_.size()); ++row) {
            const Row& data = rows_[static_cast<size_t>(row)];
            auto make_item = [](const QString& text) {
                auto* item = new QTableWidgetItem(text);
                item->setFlags(item->flags() & ~Qt::ItemIsEditable);
                return item;
            };
            table_->setItem(row, kColumnPort, make_item(data.port));
            table_->setItem(row, kColumnId, make_item(QString::number(data.id)));
            updateRowDisplay(row);

            auto* include = new QCheckBox;
            include->setChecked(data.include);
            connect(include, &QCheckBox::toggled, this, [this, row](const bool checked) {
                rows_[static_cast<size_t>(row)].include = checked;
                updateTriggerLabel();
            });
            // 居中的复选框容器。
            auto* holder = new QWidget;
            auto* holder_layout = new QHBoxLayout(holder);
            holder_layout->setContentsMargins(0, 0, 0, 0);
            holder_layout->addWidget(include);
            holder_layout->setAlignment(Qt::AlignCenter);
            table_->setCellWidget(row, kColumnInclude, holder);
        }
        updateTriggerLabel();
        resizeToTable();
    }

    void BatchDialog::resizeToTable() {
        // 弹窗默认宽度 = 表格宽度：内容列自适应后按各列总宽收紧窗口。
        for (const int column : {kColumnPort, kColumnId, kColumnDeg}) {
            table_->resizeColumnToContents(column);
        }
        const int table_width = table_->horizontalHeader()->length() + table_->frameWidth() * 2;
        resize(table_width, height());
    }

    void BatchDialog::updateRowDisplay(const int row) const {
        const Row& data = rows_[static_cast<size_t>(row)];
        auto set_text = [this, row](const int column, const QString& text) {
            if (auto* item = table_->item(row, column)) {
                item->setText(text);
            }
            else {
                auto* created = new QTableWidgetItem(text);
                created->setFlags(created->flags() & ~Qt::ItemIsEditable);
                table_->setItem(row, column, created);
            }
        };
        set_text(kColumnGoal,
                 data.has_goal ? QString::number(data.goal) : QStringLiteral("--"));
        set_text(kColumnDeg,
                 data.has_goal
                     ? QString::number(
                           servo_core::ticks_to_degrees(data.config, data.goal), 'f', 2)
                     : QStringLiteral("--"));
        set_text(kColumnSpeed, QString::number(data.speed));
        set_text(kColumnTime, QString::number(data.timeMs));
    }

    void BatchDialog::beginEdit(const int row, const int column) {
        if (editing_row_ == row && editing_column_ == column) {
            return;
        }
        commitEditor();
        if (row < 0 || row >= static_cast<int>(rows_.size())) {
            return;
        }
        const Row& data = rows_[static_cast<size_t>(row)];
        const int current =
            column == kColumnGoal ? (data.has_goal ? data.goal : 0)
                                  : (column == kColumnSpeed ? data.speed : data.timeMs);
        auto* spin = new QSpinBox;
        if (column == kColumnGoal) {
            spin->setRange(static_cast<int>(data.config.min_ticks),
                           static_cast<int>(data.config.max_ticks));
            spin->setToolTip(tr("目标位置（tick），回车或点击别处提交"));
        }
        else if (column == kColumnSpeed) {
            spin->setRange(0, 1000);
        }
        else {
            spin->setRange(0, 9999);
        }
        spin->setValue(current);
        spin->installEventFilter(this);
        table_->setCellWidget(row, column, spin);
        editing_row_ = row;
        editing_column_ = column;
        spin->setFocus();
    }

    void BatchDialog::commitEditor() {
        if (editing_row_ < 0) {
            return;
        }
        const int row = editing_row_;
        const int column = editing_column_;
        editing_row_ = -1;
        editing_column_ = -1;
        QWidget* editor = table_->cellWidget(row, column);
        if (editor == nullptr || row >= static_cast<int>(rows_.size())) {
            return;
        }
        Row& data = rows_[static_cast<size_t>(row)];
        const auto* spin = qobject_cast<QSpinBox*>(editor);
        if (spin == nullptr) {
            return;
        }
        if (column == kColumnGoal) {
            data.goal = spin->value();
            data.has_goal = true;
            data.include = true; // 编辑目标即视为参与
        }
        else if (column == kColumnSpeed) {
            data.speed = spin->value();
        }
        else {
            data.timeMs = spin->value();
        }
        data.staged = false;
        table_->removeCellWidget(row, column);
        editor->deleteLater();
        updateRowDisplay(row);
        // 同步\"参与\"复选框（编辑目标自动勾选）。
        if (const auto* holder = table_->cellWidget(row, kColumnInclude)) {
            if (auto* box = holder->findChild<QCheckBox*>()) {
                box->setChecked(data.include);
            }
        }
        updateTriggerLabel();
    }

    void BatchDialog::updateTriggerLabel() const {
        int participating = 0;
        int staged = 0;
        for (const auto& row : rows_) {
            if (row.include && row.has_goal) {
                ++participating;
                if (row.staged) {
                    ++staged;
                }
            }
        }
        trigger_button_->setText(tr("触发 (%1/%2 已预存)").arg(staged).arg(participating));
    }

    QVector<BatchGoal> BatchDialog::collectGoals() const {
        QVector<BatchGoal> goals;
        for (const auto& row : rows_) {
            if (!row.include || !row.has_goal) {
                continue;
            }
            goals.push_back({.port = row.port, .id = row.id, .ticks = row.goal, .timeMs = row.timeMs, .speed = row.speed});
        }
        return goals;
    }

    void BatchDialog::markStaged(const int count) {
        int marked = 0;
        for (auto& row : rows_) {
            if (row.include && row.has_goal && marked < count) {
                row.staged = true;
                ++marked;
            }
        }
        updateTriggerLabel();
    }

    void BatchDialog::markTriggered() {
        for (auto& row : rows_) {
            row.staged = false;
        }
        updateTriggerLabel();
    }

    bool BatchDialog::eventFilter(QObject* watched, QEvent* event) {
        if (event->type() == QEvent::Wheel && watched->isWidgetType() &&
            !dynamic_cast<QWidget*>(watched)->hasFocus()) {
            return true;
        }
        if (event->type() == QEvent::FocusOut) {
            const QWidget* next = QApplication::focusWidget();
            const bool left_table = next == nullptr || !table_->isAncestorOf(next);
            if (watched == table_) {
                if (left_table) {
                    commitEditor();
                    table_->clearSelection();
                }
            }
            else if (watched->isWidgetType() &&
                     table_->isAncestorOf(dynamic_cast<QWidget*>(watched))) {
                commitEditor();
                if (left_table) {
                    table_->clearSelection();
                }
            }
        }
        return QDialog::eventFilter(watched, event);
    }

} // namespace app
