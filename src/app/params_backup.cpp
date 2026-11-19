#include "app/params_backup.hpp"

#include <QJsonObject>
#include <QJsonParseError>

namespace app {

    namespace {
        constexpr int kFormatVersion = 1;
        constexpr auto kFormatName = "servo_studio.params";
    }

    QByteArray serializeBackup(const ParamsBackup& backup) {
        QJsonObject root;
        root["format"] = kFormatName;
        root["version"] = kFormatVersion;
        root["model"] = backup.model;
        root["id"] = backup.id;
        QJsonObject params;
        for (auto it = backup.params.constBegin(); it != backup.params.constEnd(); ++it) {
            params[it.key()] = it.value();
        }
        root["params"] = params;
        return QJsonDocument(root).toJson(QJsonDocument::Indented);
    }

    bool parseBackup(const QByteArray& data, ParamsBackup* backup, QString* error) {
        QJsonParseError parse_error{};
        const QJsonDocument document = QJsonDocument::fromJson(data, &parse_error);
        if (parse_error.error != QJsonParseError::NoError) {
            if (error != nullptr) {
                *error = "Invalid JSON: " + parse_error.errorString();
            }
            return false;
        }
        if (!document.isObject()) {
            if (error != nullptr) {
                *error = "Backup root must be an object";
            }
            return false;
        }
        const QJsonObject root = document.object();
        if (root["format"].toString() != kFormatName) {
            if (error != nullptr) {
                *error = "Not a servo_studio parameter backup";
            }
            return false;
        }
        if (!root["params"].isObject()) {
            if (error != nullptr) {
                *error = "Backup has no 'params' object";
            }
            return false;
        }
        ParamsBackup result;
        result.model = root["model"].toString();
        result.id = root["id"].toInt();
        const QJsonObject params = root["params"].toObject();
        for (auto it = params.constBegin(); it != params.constEnd(); ++it) {
            if (!it.value().isDouble()) {
                if (error != nullptr) {
                    *error = "Parameter '" + it.key() + "' is not a number";
                }
                return false;
            }
            result.params[it.key()] = it.value().toInt();
        }
        *backup = std::move(result);
        return true;
    }

    QMap<QString, int> paramsForRestore(const ParamsBackup& backup) {
        QMap<QString, int> result = backup.params;
        // id 与波特率写错会失联，由专属流程单独处理，恢复时跳过。
        result.remove("id");
        result.remove("baud_rate");
        return result;
    }

} // namespace app
