#include "app/session_bridge.hpp"

#include <QFile>
#include <algorithm>

#include "app/params_backup.hpp"

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

    void SessionBridge::setPolling(const std::vector<servo_core::ServoId>& ids, const int hz) const {
        if (session_) {
            session_->set_polling(ids, hz);
        }
    }

    void SessionBridge::submit(std::function<servo_core::Result(servo_core::IServoDriver&)> command,
                               std::function<void(bool, const QString&)> done) {
        if (!session_) {
            return;
        }
        session_->submit(std::move(command),
                         [this, done = std::move(done)](const servo_core::Result& result) {
                             emit commandFinished(static_cast<bool>(result),
                                                  QString::fromStdString(result.error()));
                             if (done) {
                                 done(static_cast<bool>(result),
                                      QString::fromStdString(result.error()));
                             }
                         });
    }

    void SessionBridge::readParameters(const QString& port, const servo_core::ServoId id) {
        if (!session_) {
            return;
        }
        auto values = std::make_shared<QVector<ParamValue>>();
        session_->submit(
            [id, values](servo_core::IServoDriver& driver) {
                for (const auto& descriptor : driver.parameters()) {
                    ParamValue entry;
                    entry.key = QString::fromStdString(descriptor.key);
                    entry.label = QString::fromStdString(descriptor.label);
                    entry.zh = QString::fromStdString(descriptor.zh);
                    entry.unit = QString::fromStdString(descriptor.unit);
                    entry.address = descriptor.address;
                    entry.min = descriptor.min;
                    entry.max = descriptor.max;
                    entry.read_only = descriptor.access == servo_core::ParamAccess::kReadOnly;
                    entry.eeprom = descriptor.access == servo_core::ParamAccess::kEeprom;
                    switch (descriptor.area) {
                    case servo_core::ParamArea::kEeprom:
                        entry.area = "EPROM";
                        break;
                    case servo_core::ParamArea::kSram:
                        entry.area = "SRAM";
                        break;
                    case servo_core::ParamArea::kDefault:
                        entry.area = "DEFAULT";
                        break;
                    }
                    entry.choices = QVector<int>(descriptor.choices.begin(), descriptor.choices.end());
                    if (const auto value = driver.read_param(id, descriptor.key)) {
                        entry.value = value.value();
                        entry.valid = true;
                    }
                    values->push_back(entry);
                }
                return servo_core::Result::success();
            },
            [this, port, id, values](const servo_core::Result&) {
                emit parametersReceived(port, id, *values);
            });
    }

    void SessionBridge::readServoState(const QString& port, const servo_core::ServoId id) {
        if (!session_) {
            return;
        }
        auto state = std::make_shared<servo_core::ServoState>();
        submit(
            [id, state](servo_core::IServoDriver& driver) {
                const auto result = driver.read_state(id);
                if (!result) {
                    return servo_core::Result::failure(result.error());
                }
                *state = result.value();
                return servo_core::Result::success();
            },
            [this, port, state](const bool success, const QString& error) {
                if (success) {
                    emit statesReceived(port, QVector<servo_core::ServoState>{*state});
                }
                else {
                    // 错误经 commandFinished 统一上报，此处仅避免误报成功。
                    (void)error;
                }
            });
    }

    void SessionBridge::queryStagedSupport(const QString& port) {
        if (!session_) {
            return;
        }
        auto supported = std::make_shared<bool>(false);
        submit(
            [supported](const servo_core::IServoDriver& driver) {
                *supported = driver.supports_staged_write();
                return servo_core::Result::success();
            },
            [this, port, supported](bool, const QString&) {
                emit stagedSupportKnown(port, *supported);
            });
    }

    void SessionBridge::traceFrame(const char* direction, const std::vector<uint8_t>& frame) {
        emit frameTraced(port_, QString::fromLatin1(direction),
                         QByteArray(reinterpret_cast<const char*>(frame.data()),
                                    static_cast<int>(frame.size())));
    }

    void SessionBridge::backupParameters(const QString& port, const servo_core::ServoId id,
                                         const QString& path) {
        if (!session_) {
            return;
        }
        submit(
            [id, path](servo_core::IServoDriver& driver) {
                ParamsBackup backup;
                backup.id = id;
                if (const auto info = driver.identify(id)) {
                    backup.model = QString::fromStdString(info.value().model);
                }
                for (const auto& descriptor : driver.parameters()) {
                    if (const auto value = driver.read_param(id, descriptor.key)) {
                        backup.params[QString::fromStdString(descriptor.key)] = value.value();
                    }
                }
                QFile file(path);
                if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    return servo_core::Result::failure(
                        "Failed to open '" + path.toStdString() + "' for writing");
                }
                file.write(serializeBackup(backup));
                return servo_core::Result::success();
            },
            [this, port, id, path](const bool success, const QString& error) {
                emit backupFinished(path, success, error);
                if (success) {
                    emit logReceived("info",
                                     QStringLiteral("Parameters of servo %1 backed up to %2")
                                     .arg(id)
                                     .arg(path));
                }
            });
    }

    void SessionBridge::restoreParameters(const QString& port, const servo_core::ServoId id,
                                          const QMap<QString, int>& params) {
        if (!session_) {
            return;
        }
        auto written = std::make_shared<int>(0);
        submit(
            [id, params, written](servo_core::IServoDriver& driver) {
                // 只写当前描述表已知的键；旧备份中的废弃键跳过而不是中断恢复。
                std::vector<std::string> known;
                for (const auto& descriptor : driver.parameters()) {
                    known.push_back(descriptor.key);
                }
                for (auto it = params.constBegin(); it != params.constEnd(); ++it) {
                    const std::string key = it.key().toStdString();
                    if (std::ranges::find(known, key) == known.end()) {
                        continue;
                    }
                    if (const auto result = driver.write_param(id, key, it.value()); !result) {
                        return result;
                    }
                    ++*written;
                }
                return servo_core::Result::success();
            },
            [this, port, id, written](const bool success, const QString& error) {
                emit restoreFinished(port, id, *written, success, error);
            });
    }

    void SessionBridge::stop() {
        if (session_) {
            session_->stop();
            emit runningChanged(false);
        }
    }

} // namespace app
