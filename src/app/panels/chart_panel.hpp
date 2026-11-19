#pragma once

#include <QColor>
#include <QElapsedTimer>
#include <QPointF>
#include <QString>
#include <QWidget>
#include <deque>
#include <vector>

#include "app/qt_types.hpp"
#include "servo_core/types.hpp"

class QCheckBox;
class QLabel;
class QChartView;
class QChart;
class QLineSeries;
class QValueAxis;
class QGridLayout;

namespace app {

    /// @brief 选中舵机的 6 通道滚动曲线（小图组）：
    ///        位置/速度/负载/电压/温度/电流。
    ///
    /// 各通道单位不同，因此每通道一张小图、独立 Y 轴，共享时间窗
    /// （默认 30 s）；每张小图可开关。未选中舵机时显示提示。
    class ChartPanel : public QWidget {
        Q_OBJECT

    public:
        explicit ChartPanel(QWidget* parent = nullptr);

        /// 切换监控对象（同一对象重复调用为空操作）。
        void setSubject(const QString& port, servo_core::ServoId id, const QString& model);
        /// 清空监控对象与历史。
        void clear();

    public slots:
        /// 追加一批轮询状态；仅处理当前监控对象。
        void appendStates(const QString& port, const QVector<servo_core::ServoState>& states);

    private:
        struct Channel {
            QString title;
            QString unit;
            QColor color;
            double (*value)(const servo_core::ServoState&);
            QCheckBox* toggle{};
            QChart* chart{};
            QLineSeries* series{};
            QValueAxis* axis_x{};
            QValueAxis* axis_y{};
            QChartView* view{};
            QLabel* header{};
            QWidget* container{};
            std::deque<QPointF> points;
        };

        void resetSeries();
        /// 按当前勾选状态重排小图：每行三个为固定组，取消的格位由
        /// 右侧相邻吸收（行尾缺口并入左侧）；仅剩一个时独占整个监控区。
        void relayout();

        std::vector<Channel> channels_;
        QGridLayout* grid_;
        QLabel* subject_label_;
        QString subject_port_;
        servo_core::ServoId subject_id_{};
        bool has_subject_{false};
        QElapsedTimer clock_;

        static constexpr double kWindowSeconds = 30.0;
    };

} // namespace app
