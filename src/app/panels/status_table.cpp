#include "app/panels/status_table.hpp"

#include <QHeaderView>

namespace app {

    namespace {
        enum Column {
            kColumnPort = 0,
            kColumnId,
            kColumnPositionTicks,
            kColumnPositionDeg,
            kColumnVelocity,
            kColumnLoad,
            kColumnVoltage,
            kColumnTemperature,
            kColumnCurrent,
            kColumnMoving,
            kColumnCount,
        };
    }

    StatusTable::StatusTable(QWidget* parent) :
        QTableWidget(parent) {
        setObjectName("telemetry");
        setColumnCount(kColumnCount);
        setHorizontalHeaderLabels({tr("总线"), tr("ID"), tr("位置(tick)"), tr("位置(°)"),
                                   tr("速度(°/s)"), tr("负载"), tr("电压(V)"), tr("温度(℃)"),
                                   tr("电流(A)"), tr("移动")});
        horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        horizontalHeader()->setSectionResizeMode(kColumnPort, QHeaderView::ResizeToContents);
        horizontalHeader()->setSectionResizeMode(kColumnId, QHeaderView::ResizeToContents);
        verticalHeader()->setVisible(false);
        setEditTriggers(QAbstractItemView::NoEditTriggers);
        setSelectionBehavior(QAbstractItemView::SelectRows);
        setAlternatingRowColors(true);
    }

    int StatusTable::findRow(const QString& port, const servo_core::ServoId id) const {
        for (int row = 0; row < rowCount(); ++row) {
            if (item(row, kColumnPort)->text() == port &&
                item(row, kColumnId)->text().toUInt() == id) {
                return row;
            }
        }
        return -1;
    }

    int StatusTable::insertRowFor(const QString& port, const servo_core::ServoState& state) {
        int row = 0;
        for (; row < rowCount(); ++row) {
            const QString row_port = item(row, kColumnPort)->text();
            const int row_id = item(row, kColumnId)->text().toInt();
            if (row_port > port || (row_port == port && row_id > state.id)) {
                break;
            }
        }
        insertRow(row);
        for (int column = 0; column < kColumnCount; ++column) {
            setItem(row, column, new QTableWidgetItem());
        }
        item(row, kColumnPort)->setText(port);
        item(row, kColumnId)->setText(QString::number(state.id));
        return row;
    }

    void StatusTable::updateStates(const QString& port,
                                   const QVector<servo_core::ServoState>& states) {
        for (const auto& state : states) {
            int row = findRow(port, state.id);
            if (row < 0) {
                row = insertRowFor(port, state);
            }
            item(row, kColumnPositionTicks)->setText(QString::number(state.position_ticks, 'f', 1));
            item(row, kColumnPositionDeg)->setText(QString::number(state.position_deg, 'f', 2));
            item(row, kColumnVelocity)->setText(QString::number(state.velocity_deg_s, 'f', 1));
            item(row, kColumnLoad)->setText(QString::number(state.load, 'f', 3));
            item(row, kColumnVoltage)->setText(QString::number(state.voltage, 'f', 2));
            item(row, kColumnTemperature)->setText(QString::number(state.temperature, 'f', 0));
            item(row, kColumnCurrent)->setText(QString::number(state.current, 'f', 3));
            item(row, kColumnMoving)->setText(state.moving ? tr("是") : tr("否"));
        }
    }

    void StatusTable::removeBus(const QString& port) {
        for (int row = rowCount() - 1; row >= 0; --row) {
            if (item(row, kColumnPort)->text() == port) {
                removeRow(row);
            }
        }
    }

    void StatusTable::clearAll() {
        setRowCount(0);
    }

    bool StatusTable::selectedServo(QString* port, servo_core::ServoId* id) const {
        const auto ranges = selectedRanges();
        if (ranges.isEmpty()) {
            return false;
        }
        const int row = ranges.first().topRow();
        if (port != nullptr) {
            *port = item(row, kColumnPort)->text();
        }
        if (id != nullptr) {
            *id = static_cast<servo_core::ServoId>(item(row, kColumnId)->text().toUInt());
        }
        return true;
    }

    QVector<QPair<QString, servo_core::ServoId>> StatusTable::selectedServos() const {
        QVector<QPair<QString, servo_core::ServoId>> result;
        for (const auto& range : selectedRanges()) {
            for (int row = range.topRow(); row <= range.bottomRow(); ++row) {
                result.push_back({item(row, kColumnPort)->text(),
                                  static_cast<servo_core::ServoId>(
                                      item(row, kColumnId)->text().toUInt())});
            }
        }
        return result;
    }

} // namespace app
