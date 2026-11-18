#pragma once

#include <QByteArray>
#include <QMap>
#include <QString>

namespace app {

    /// @brief 舵机参数备份：型号/ID 与整表参数。
    struct ParamsBackup {
        /// 型号名称（记录用）。
        QString model;
        /// 舵机 id（记录用）。
        int id{};
        /// 参数键 → 值（整表）。
        QMap<QString, int> params;
    };

    /// @brief 序列化备份为 JSON 文本。
    QByteArray serializeBackup(const ParamsBackup& backup);

    /// @brief 解析备份文件内容。
    /// @return 格式非法时返回 false 并给出错误信息。
    bool parseBackup(const QByteArray& data, ParamsBackup* backup, QString* error);

    /// @brief 恢复时实际写入的参数集合：剔除 id 与波特率
    ///        （写错会失联，由专属流程单独处理）。
    QMap<QString, int> paramsForRestore(const ParamsBackup& backup);

} // namespace app
