#pragma once

#include <QDialog>
#include <vector>

#include "app/session_bridge.hpp"

class QEvent;
class QLabel;
class QPushButton;
class QTableWidget;

namespace app {

    /// @brief 寄存器工作台（非模态弹窗）：全量寄存器表，
    ///        列为 地址 | 英文名 | 中文名 | 值 | 区域 | 操作。
    ///
    /// 只读行仅显示值；可修行点击值列进入编辑（与检查器一致的
    /// 点击编辑/滚轮防误触/失焦恢复交互），操作列为[写入]。
    class ParamsDialog : public QDialog {
        Q_OBJECT

    public:
        explicit ParamsDialog(QWidget* parent = nullptr);

        /// 设置监控对象（切换时清空表格）。
        void setSubject(const QString& port, servo_core::ServoId id);

    signals:
        /// 写入单个参数（id/baud_rate 由主窗口拦截走专属流程）。
        void paramWriteRequested(const QString& port, servo_core::ServoId id, const QString& key,
                                 int value, const QString& valueText);
        /// 将参数写入总线全部舵机。
        void paramWriteAllRequested(const QString& port, const QString& key, int value);
        /// 重新读取全部寄存器。
        void reloadParametersRequested(const QString& port, servo_core::ServoId id);
        /// 备份整表参数到文件。
        void backupRequested(const QString& port, servo_core::ServoId id);
        /// 从文件恢复参数。
        void restoreRequested(const QString& port, servo_core::ServoId id);

    public slots:
        /// 填充/刷新表格。
        void setParameters(const QString& port, int id, const QVector<ParamValue>& values);

    private:
        void rebuildRows(const QVector<ParamValue>& values);
        void beginEdit(int row);
        void commitEditor();
        bool eventFilter(QObject* watched, QEvent* event) override;

        QTableWidget* table_;
        QLabel* subject_label_;
        QPushButton* reload_button_;
        QPushButton* write_all_button_;
        QPushButton* backup_button_;
        QPushButton* restore_button_;
        std::vector<ParamValue> rows_;
        int editing_row_{-1};
        QString port_;
        servo_core::ServoId id_{};
        bool has_subject_{false};
    };

} // namespace app
