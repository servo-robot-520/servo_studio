#pragma once

#include <QWidget>

class QComboBox;
class QSpinBox;
class QPushButton;

namespace app {

    /// @brief 连接面板：串口/波特率/超时选择，连接、扫描与全局松扭矩。
    class ConnectionPanel : public QWidget {
        Q_OBJECT

    public:
        explicit ConnectionPanel(QWidget* parent = nullptr);

        /// 顶栏：协议 / 串口 / 刷新 / 波特率 / 打开。
        [[nodiscard]] QWidget* topBar() const;
        /// 侧栏（设备列表上方）：超时 / 轮询 / 扫描 / 全部松扭矩。
        [[nodiscard]] QWidget* sideBar() const;

        /// 当前表单中的协议后端（驱动名）。
        [[nodiscard]] QString backend() const;
        /// 当前表单中的串口设备。
        [[nodiscard]] QString port() const;
        /// 当前表单中的波特率。
        [[nodiscard]] int baudRate() const;
        /// 当前表单中的应答超时（毫秒）。
        [[nodiscard]] int timeoutMs() const;
        /// 当前表单中的轮询频率（Hz）。
        [[nodiscard]] int pollRate() const;

    signals:
        /// 请求连接总线（协议后端 + 串口）。
        void connectRequested(const QString& backend, const QString& port, int baudRate,
                              int timeoutMs);
        /// 协议后端变化（用于界面按协议能力即时切换）。
        void backendChanged(const QString& backend);
        /// 请求断开当前表单中的总线。
        void disconnectRequested(const QString& port);
        /// 请求扫描当前表单中的总线。
        void scanRequested(const QString& port);
        /// 请求全部松扭矩（急停）。
        void stopAllRequested(const QString& port);
        /// 轮询频率变化（Hz）。
        void pollRateChanged(int hz);

    public slots:
        /// 更新按钮使能：已连接时"连接"变为"断开"。
        void setConnected(const QString& port, bool connected) const;
        /// 扫描进行中时禁用扫描按钮，防止并发扫描。
        void setScanning(bool scanning);

    private slots:
        void refreshPorts() const;

    private:
        QWidget* top_bar_;
        QWidget* side_bar_;
        QComboBox* backend_combo_;
        QComboBox* port_combo_;
        QComboBox* baud_combo_;
        QSpinBox* timeout_spin_;
        QSpinBox* poll_spin_;
        QPushButton* connect_button_;
        QPushButton* scan_button_;
        QPushButton* stop_button_;
        bool scanning_{false};
    };

} // namespace app
