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

        subject_label_ = new QLabel(tr("监控：未选中舵机（在左侧设备树或状态表中选择）"), this);
        subject_label_->setObjectName("title");
        layout->addWidget(subject_label_);

        // 通道开关行：常驻显示，取消/恢复随时可点。
        auto* toggle_row = new QHBoxLayout;
        toggle_row->setSpacing(12);
        layout->addLayout(toggle_row);

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

            // 通道开关常驻于顶部开关行，不随小图隐藏，
            // 保证取消后随时可以重新勾选。
            channel.toggle = new QCheckBox(channel.title);
            channel.toggle->setChecked(true);
            channel.toggle->setStyleSheet(
                QStringLiteral("color: %1;").arg(channel.color.name()));
            connect(channel.toggle, &QCheckBox::toggled, this, [this] { relayout(); });
            toggle_row->addWidget(channel.toggle);
            channels_.push_back(std::move(channel));
        }
        toggle_row->addStretch(1);
        grid_ = new QGridLayout;
        grid_->setSpacing(8);
        for (int column = 0; column < 6; ++column) {
            grid_->setColumnStretch(column, 1);
        }
        layout->addLayout(grid_, 1);
        relayout();
        clock_.start();
    }

    void ChartPanel::relayout() {
        // 移除全部格位（保留容器控件）。
        while (grid_->count() > 0) {
            QLayoutItem* item = grid_->takeAt(0);
            if (item != nullptr && item->widget() != nullptr) {
                item->widget()->hide();
            }
            delete item;
        }
        for (int row = 0; row < 2; ++row) {
            grid_->setRowStretch(row, 0);
        }

        std::vector<Channel*> visible;
        for (auto& channel : channels_) {
            if (channel.toggle->isChecked()) {
                visible.push_back(&channel);
            }
        }
        if (visible.empty()) {
            return;
        }

        // 只剩一个通道时独占整个监控区。
        if (visible.size() == 1) {
            grid_->addWidget(visible.front()->view, 0, 0, 2, 6);
            grid_->setRowStretch(0, 1);
            grid_->setRowStretch(1, 1);
            visible.front()->view->show();
            return;
        }

        // 每行三个（0..2 / 3..5）为固定组，行间不搬移：
        // 取消的格位由右侧相邻吸收，行尾缺口并入左侧相邻。
        for (int row = 0; row < 2; ++row) {
            std::vector<int> cells; // 本行可见通道的格位 0..2
            for (int cell = 0; cell < 3; ++cell) {
                if (channels_[static_cast<size_t>(row * 3 + cell)].toggle->isChecked()) {
                    cells.push_back(cell);
                }
            }
            if (cells.empty()) {
                continue;
            }
            int previous = -1;
            for (size_t index = 0; index < cells.size(); ++index) {
                const int cell = cells[index];
                // 可见格位 + 其左侧缺口（左侧缺口由本格右侧相邻吸收）。
                int span = cell - previous;
                if (index + 1 == cells.size()) {
                    span += 2 - cell; // 行尾缺口并入最后一个可见格位。
                }
                grid_->addWidget(channels_[static_cast<size_t>(row * 3 + cell)].view,
                                 row, (previous + 1) * 2, 1, span * 2);
                channels_[static_cast<size_t>(row * 3 + cell)].view->show();
                previous = cell;
            }
            grid_->setRowStretch(row, 1);
        }
    }

    void ChartPanel::setSubject(const QString& port, const servo_core::ServoId id,
                                const QString& model) {
        if (has_subject_ && subject_port_ == port && subject_id_ == id) {
            return;
        }
        subject_port_ = port;
        subject_id_ = id;
        has_subject_ = true;
        // 主题信息（串口/id/型号）只在监控区展示一次。
        subject_label_->setText(tr("监控：%1 · id %2 · %3").arg(port).arg(id).arg(model));
        resetSeries();
    }

    void ChartPanel::clear() {
        has_subject_ = false;
        subject_label_->setText(tr("监控：未选中舵机（在左侧设备树或状态表中选择）"));
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
