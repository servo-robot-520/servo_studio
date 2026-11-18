#include "app/session_bridge.hpp"

namespace app {

    SessionBridge::SessionBridge(QObject* parent) :
        QObject(parent) {
    }

    SessionBridge::~SessionBridge() {
        // 停止并 join 工作线程：之后不会再有回调触达本对象。
        if (session_) {
            session_->stop();
        }
    }

    void SessionBridge::attach(std::shared_ptr<servo_core::BusSession> session) {
        session_ = std::move(session);
        port_ = QString::fromStdString(session_->config().port);
        session_->set_log_handler([this](const std::string& level, const std::string& message) {
            emit logReceived(QString::fromStdString(level), QString::fromStdString(message));
        });
        session_->set_states_handler([this](const std::vector<servo_core::ServoState>& states) {
            emit statesReceived(port_,
                                QVector<servo_core::ServoState>(states.begin(), states.end()));
        });
    }

    std::shared_ptr<servo_core::BusSession> SessionBridge::session() const {
        return session_;
    }

    bool SessionBridge::attached() const {
        return session_ != nullptr;
    }

    void SessionBridge::scan(const int first_id, const int last_id) {
        if (!session_) {
            return;
        }
        session_->scan(first_id, last_id,
                       [this](const servo_core::Expected<std::vector<servo_core::ServoInfo>>& infos) {
                           if (!infos) {
                               emit logReceived("error", QString::fromStdString(infos.error()));
                               emit scanFinished(port_, {});
                               return;
                           }
                           emit scanFinished(
                               port_, QVector<servo_core::ServoInfo>(infos.value().begin(),
                                                                     infos.value().end()));
                       });
    }

    void SessionBridge::setPolling(const std::vector<servo_core::ServoId>& ids, const int hz) {
        if (session_) {
            session_->set_polling(ids, hz);
        }
    }

    void SessionBridge::submit(std::function<servo_core::Result(servo_core::IServoDriver&)> command) {
        if (!session_) {
            return;
        }
        session_->submit(std::move(command),
                         [this](const servo_core::Result& result) {
                             emit commandFinished(static_cast<bool>(result),
                                                  QString::fromStdString(result.error()));
                         });
    }

    void SessionBridge::stop() {
        if (session_) {
            session_->stop();
            emit runningChanged(false);
        }
    }

} // namespace app
