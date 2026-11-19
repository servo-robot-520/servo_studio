#pragma once

#include <QWidget>
#include <map>
#include <vector>

class QComboBox;
class QTextEdit;
class QTimer;

namespace app {

    /// @brief 日志面板：事件日志 + 帧级 hex 跟踪，级别过滤、导出。
    ///
    /// 帧跟踪流量大（50 Hz 轮询 × 每事务收发两帧），因此条目按 100 ms
    /// 批量刷入文本视图，避免逐条重建拖垮 GUI 线程。
    class LogPanel : public QWidget {
        Q_OBJECT

    public:
        explicit LogPanel(QWidget* parent = nullptr);

    public slots:
        /// 追加事件日志（级别：info/warn/error）。
        void appendLog(const QString& level, const QString& message);
        /// 追加帧级跟踪（方向 tx/rx + 原始字节）。
        void appendFrame(const QString& port, const QString& direction, const QByteArray& frame);

    private slots:
        void applyFilter();
        void flushPending();
        void exportEntries();
        void clearEntries();

    private:
        struct Entry {
            QString level;
            QString text;
        };

        void appendEntry(const QString& level, const QString& text);
        /// 超限驱逐：trace 不缓存优先淘汰；其余级别保证保留最近 kCachedPerLevel 条。
        void evictOverflow();
        [[nodiscard]] static QString hexBytes(const QByteArray& data);

        QComboBox* filter_;
        QTextEdit* view_;
        QTimer* flush_timer_;
        std::vector<Entry> entries_;
        std::map<QString, size_t> level_counts_;
        size_t flushed_{0};

        static constexpr int kMaxEntries = 2000;
        /// 驱逐时各级（trace 除外）保证保留的最近条数。
        static constexpr size_t kCachedPerLevel = 20;
        static constexpr int kFlushIntervalMs = 100;
    };

} // namespace app
