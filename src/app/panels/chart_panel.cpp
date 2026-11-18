#include "app/panels/chart_panel.hpp"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

namespace app {

    namespace {
        // 曲线色与状态表语义一致，每通道固定一色。
        struct ChannelSpec {
            const char* title;
            const char* unit;
            const char* color;
            const char* label_format;
            double min_span;
            double (*value)(const servo_core::ServoState&);
        };

        constexpr ChannelSpec kChannels[]{
            {"位置", "deg", "#56B4C4", "%.1f", 2.0,
             [](const servo_core::ServoState& s) { return s.position_deg; }},
            {"速度", "deg/s", "#E8B44A", "%.1f", 4.0,
             [](const servo_core::ServoState& s) { return s.velocity_deg_s; }},
            {"负载", "", "#A98BD4", "%.2f", 0.2,
             [](const servo_core::ServoState& s) { return s.load; }},
            {"电压", "V", "#6C9BD2", "%.3f", 0.2,
             [](const servo_core::ServoState& s) { return s.voltage; }},
            {"温度", "C", "#C97BB5", "%.1f", 2.0,
             [](const servo_core::ServoState& s) { return s.temperature; }},
            {"电流", "A", "#7FBF7A", "%.3f", 0.02,
             [](const servo_core::ServoState& s) { return s.current; }},
        };

        const QColor kAxisColor("#8B98A5");
        const QColor kGridColor("#2A323B");
        const QColor kPanelColor("#1D222A");
    } // namespace

    ChartPanel::ChartPanel(QWidget* parent) :
        QWidget(parent) {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        subject_label_ = new QLabel(tr("未选中舵机（在左侧设备树或状态表中选择）"), this);
        subject_label_->setObjectName("muted");
        layout->addWidget(subject_label_);

        auto* grid = new QGridLayout;
        grid->setSpacing(8);
        for (size_t index = 0; index < std::size(kChannels); ++index) {
            const auto& spec = kChannels[index];
            Channel channel;
            channel.title = QString::fromUtf8(spec.title);
            channel.unit = QString::fromUtf8(spec.unit);
            channel.color = QColor(spec.color);
            channel.value = spec.value;

            channel.series = new QLineSeries;
            QPen pen(channel.color);
            pen.setWidth(2);
            channel.series->setPen(pen);

            channel.chart = new QChart;
            channel.chart->addSeries(channel.series);
            channel.chart->setBackgroundBrush(kPanelColor);
            channel.chart->setBackgroundRoundness(6);
            channel.chart->setPlotAreaBackgroundBrush(kPanelColor);
            channel.chart->setPlotAreaBackgroundVisible(true);
            channel.chart->legend()->hide();

            channel.axis_x = new QValueAxis;
            channel.axis_x->setRange(-kWindowSeconds, 0);
            channel.axis_x->setLabelsVisible(false);
            channel.axis_x->setGridLinePen(QPen(kGridColor));
            channel.axis_x->setLinePen(QPen(kAxisColor));
            channel.axis_y = new QValueAxis;
            channel.axis_y->setLabelFormat(spec.label_format);
            channel.axis_y->setTickCount(4);
            channel.axis_y->setLabelsColor(kAxisColor);
            channel.axis_y->setGridLinePen(QPen(kGridColor));
            channel.axis_y->setLinePen(QPen(kAxisColor));
            channel.chart->addAxis(channel.axis_x, Qt::AlignBottom);
            channel.chart->addAxis(channel.axis_y, Qt::AlignLeft);
            channel.series->attachAxis(channel.axis_x);
            channel.series->attachAxis(channel.axis_y);

            channel.view = new QChartView(channel.chart);
            channel.view->setMinimumHeight(140);

            channel.toggle = new QCheckBox(channel.title);
            channel.toggle->setChecked(true);
            connect(channel.toggle, &QCheckBox::toggled, this,
                    [view = channel.view](const bool checked) { view->setVisible(checked); });

            const int row = static_cast<int>(index) / 3;
            const int column = static_cast<int>(index) % 3;
            auto* cell = new QVBoxLayout;
            cell->setSpacing(4);
            cell->addWidget(channel.toggle, 0, Qt::AlignLeft);
            cell->addWidget(channel.view);
            grid->addLayout(cell, row, column);
            channels_.push_back(std::move(channel));
        }
        layout->addLayout(grid, 1);
        clock_.start();
    }

    void ChartPanel::setSubject(const QString& port, const servo_core::ServoId id) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        has_subject_ = true;
        subject_label_->setText(tr("监控：%1 · id %2").arg(port).arg(id));
        resetSeries();
    }

    void ChartPanel::clear() {
        has_subject_ = false;
        subject_label_->setText(tr("未选中舵机（在左侧设备树或状态表中选择）"));
        resetSeries();
    }

    void ChartPanel::resetSeries() {
        clock_.restart();
        for (auto& channel : channels_) {
            channel.points.clear();
            channel.series->clear();
            channel.axis_x->setRange(-kWindowSeconds, 0);
        }
    }

    void ChartPanel::appendStates(const QString& port,
                                  const QVector<servo_core::ServoState>& states) {
        if (!has_subject_ || port != subject_port_) {
            return;
        }
        const servo_core::ServoState* subject = nullptr;
        for (const auto& state : states) {
            if (state.id == subject_id_) {
                subject = &state;
                break;
            }
        }
        if (subject == nullptr) {
            return;
        }

        const double now = clock_.elapsed() / 1000.0;
        const double cutoff = now - kWindowSeconds;
        for (size_t index = 0; index < channels_.size(); ++index) {
            auto& channel = channels_[index];
            const double y = channel.value(*subject);
            channel.points.push_back(QPointF(now, y));
            channel.series->append(now, y);

            // 裁剪时间窗外的旧点。
            size_t stale = 0;
            while (stale < channel.points.size() && channel.points[stale].x() < cutoff) {
                ++stale;
            }
            if (stale > 0) {
                channel.points.erase(channel.points.begin(),
                                     channel.points.begin() + static_cast<long>(stale));
                channel.series->removePoints(0, static_cast<int>(stale));
            }

            // 滑动窗口内 Y 轴自适应：窗口范围不足最小量程时居中展开，
            // 保证静态信号下标签可读。
            double min = y;
            double max = y;
            for (const auto& point : channel.points) {
                min = std::min(min, point.y());
                max = std::max(max, point.y());
            }
            const double span = std::max(max - min, kChannels[index].min_span);
            const double center = (min + max) / 2.0;
            channel.axis_y->setRange(center - span * 0.6, center + span * 0.6);
            channel.axis_x->setRange(cutoff, now);
        }
    }

} // namespace app
