#include "app/panels/log_panel.hpp"

#include <QComboBox>
#include <QDate>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextCursor>
#include <QTextEdit>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>

namespace app {

    LogPanel::LogPanel(QWidget* parent) :
        QWidget(parent) {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(6);

        auto* bar = new QHBoxLayout;
        bar->addWidget(new QLabel(tr("日志"), this));
        filter_ = new QComboBox(this);
        filter_->setToolTip(tr("按级别过滤日志（帧级收发为 trace）"));
        filter_->addItems({tr("全部"), "info", "warn", "error", "trace"});
        connect(filter_, &QComboBox::currentIndexChanged, this, &LogPanel::applyFilter);
        bar->addWidget(filter_);
        bar->addStretch(1);
        auto* export_button = new QPushButton(tr("导出"), this);
        auto* clear_button = new QPushButton(tr("清空"), this);
        export_button->setToolTip(tr("导出日志到文本文件"));
        clear_button->setToolTip(tr("清空当前日志"));
        connect(export_button, &QPushButton::clicked, this, &LogPanel::exportEntries);
        connect(clear_button, &QPushButton::clicked, this, &LogPanel::clearEntries);
        bar->addWidget(export_button);
        bar->addWidget(clear_button);
        layout->addLayout(bar);

        view_ = new QTextEdit(this);
        view_->setReadOnly(true);
        view_->setObjectName("telemetry");
        view_->setFontFamily("monospace");
        // 超出上限的旧行由文档自动丢弃。
        view_->document()->setMaximumBlockCount(kMaxEntries);
        layout->addWidget(view_);

        flush_timer_ = new QTimer(this);
        flush_timer_->setSingleShot(true);
        flush_timer_->setInterval(kFlushIntervalMs);
        connect(flush_timer_, &QTimer::timeout, this, &LogPanel::flushPending);
    }

    QString LogPanel::hexBytes(const QByteArray& data) {
        QString text;
        text.reserve(data.size() * 3);
        for (const unsigned char byte : data) {
            if (!text.isEmpty()) {
                text += ' ';
            }
            text += QStringLiteral("%1").arg(byte, 2, 16, QLatin1Char('0')).toUpper();
        }
        return text;
    }

    void LogPanel::appendLog(const QString& level, const QString& message) {
        appendEntry(level, message);
    }

    void LogPanel::appendFrame(const QString& port, const QString& direction,
                               const QByteArray& frame) {
        appendEntry("trace", QStringLiteral("%1 %2 %3").arg(port, direction, hexBytes(frame)));
    }

    void LogPanel::appendEntry(const QString& level, QString text) {
        entries_.push_back({level, QStringLiteral("[%1] [%2] %3")
                                       .arg(QTime::currentTime().toString("HH:mm:ss.zzz"), level,
                                            std::move(text))});
        if (entries_.size() > kMaxEntries) {
            entries_.erase(entries_.begin());
            if (flushed_ > 0) {
                --flushed_;
            }
        }
        if (!flush_timer_->isActive()) {
            flush_timer_->start();
        }
    }

    void LogPanel::flushPending() {
        const QString filter = filter_->currentText();
        QString text;
        for (size_t index = flushed_; index < entries_.size(); ++index) {
            const auto& entry = entries_[index];
            if (filter != tr("全部") && entry.level != filter) {
                continue;
            }
            text += entry.text;
            text += '\n';
        }
        if (!text.isEmpty()) {
            // 追加而不是重建；滚动到底部。
            view_->moveCursor(QTextCursor::End);
            view_->insertPlainText(text);
            view_->moveCursor(QTextCursor::End);
        }
        flushed_ = entries_.size();
    }

    void LogPanel::applyFilter() {
        // 过滤条件变化时全量重建。
        flushed_ = 0;
        view_->clear();
        flushPending();
    }

    void LogPanel::exportEntries() {
        const QString path = QFileDialog::getSaveFileName(
            this, tr("导出日志"),
            QStringLiteral("servo_studio_%1.log").arg(QDate::currentDate().toString("yyyyMMdd")),
            tr("日志文件 (*.log *.txt)"));
        if (path.isEmpty()) {
            return;
        }
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            appendEntry("error", "Failed to open '" + path + "' for writing");
            return;
        }
        for (const auto& entry : entries_) {
            file.write(entry.text.toUtf8());
            file.write("\n");
        }
        appendEntry("info", "Log exported to " + path);
    }

    void LogPanel::clearEntries() {
        entries_.clear();
        flushed_ = 0;
        view_->clear();
    }

} // namespace app
