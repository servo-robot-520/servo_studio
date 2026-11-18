#pragma once

#include <QVector>

#include "servo_core/types.hpp"

// 中立类型在 Qt 信号中跨线程 queued 投递所需的元类型声明。
// 所有以这些类型为信号参数的头文件都必须包含本文件，
// 保证声明先于 moc 生成代码的实例化。
Q_DECLARE_METATYPE(QVector<servo_core::ServoState>)
Q_DECLARE_METATYPE(QVector<servo_core::ServoInfo>)
