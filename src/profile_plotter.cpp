#include "profile_plotter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <QColor>
#include <QGridLayout>
#include <QPen>
#include <QString>
#include <QVector>
#include <qcustomplot.h>

namespace ground_vehicle_motion_tester
{
  namespace
  {
    /** @brief Select one numeric PreviewSample field for a velocity or acceleration graph. */
    using SignalValue = double PreviewSample::*;

    /**
     * @brief Fit the requested view with equal physical scales on both axes.
     * @param plot Trajectory widget with its current drawable width and height.
     * @param x_range Requested horizontal view before adapting it to the window shape.
     * @param y_range Requested vertical view before adapting it to the window shape.
     */
    void equal_axis_scales(QCustomPlot& plot, const QCPRange& x_range, const QCPRange& y_range)
    {
      const auto width{static_cast<double>(plot.axisRect()->width())};
      const auto height{static_cast<double>(plot.axisRect()->height())};
      if(width <= 0.0 || height <= 0.0)
      {
        return;
      }
      const auto scale{std::max(x_range.size() / width, y_range.size() / height)};
      plot.xAxis->setRange(x_range.center(), width * scale, Qt::AlignCenter);
      plot.yAxis->setRange(y_range.center(), height * scale, Qt::AlignCenter);
    }

    /**
     * @brief Create one temporal plot with the same time range as the other five signals.
     * @param parent Figure owning the graph widget.
     * @param samples Computed values at regular points and all acceleration changes.
     * @param value Member identifying which numeric signal to draw.
     * @param name Widget identifier for the displayed signal.
     * @param title Human-readable graph title.
     * @param unit Unit shown on the vertical axis.
     * @param acceleration Whether to show discontinuities as vertical steps.
     * @param color Line color shared by the velocity/acceleration pair of one axis.
     * @return A graph widget owned by parent.
     */
    QCustomPlot* signal_plot(QWidget& parent,
                             const std::vector<PreviewSample>& samples,
                             SignalValue value,
                             const QString& name,
                             const QString& title,
                             const QString& unit,
                             bool acceleration,
                             const QColor& color)
    {
      auto* plot{new QCustomPlot{&parent}};
      plot->setObjectName(name);
      plot->plotLayout()->insertRow(0);
      plot->plotLayout()->addElement(0, 0, new QCPTextElement{plot, title});
      QVector<double> times{};
      QVector<double> values{};
      auto previous{samples.front().*value};
      for(const auto& sample: samples)
      {
        if(acceleration && previous != sample.*value)
        {
          // Repeating the event time draws a vertical jump instead of a fictitious ramp.
          times.push_back(sample.time);
          values.push_back(previous);
        }
        times.push_back(sample.time);
        values.push_back(sample.*value);
        previous = sample.*value;
      }
      plot->addGraph();
      plot->graph()->setData(times, values, true);
      plot->graph()->setPen(QPen{color});
      plot->xAxis->setLabel("Time [s]");
      plot->yAxis->setLabel(unit);
      plot->rescaleAxes();
      plot->xAxis->setRange(0.0, samples.back().time);
      plot->yAxis->scaleRange(1.15);
      plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
      return plot;
    }

    /**
     * @brief Draw a time-ordered XY curve and platform orientation at every displayed point.
     * @param parent Figure owning the trajectory graph.
     * @param samples Ideal positions and yaw angles produced by the odometry library.
     * @return A graph widget owned by parent, with equal physical scales on X and Y.
     */
    QCustomPlot* trajectory_plot(QWidget& parent, const std::vector<PreviewSample>& samples)
    {
      auto* plot{new QCustomPlot{&parent}};
      plot->setObjectName("trajectory");
      plot->plotLayout()->insertRow(0);
      plot->plotLayout()->addElement(0, 0, new QCPTextElement{plot, "Ideal trajectory and platform orientation"});
      QVector<double> times{};
      QVector<double> x{};
      QVector<double> y{};
      for(const auto& sample: samples)
      {
        times.push_back(sample.time);
        x.push_back(sample.x);
        y.push_back(sample.y);
      }
      // A parametric curve preserves execution order even if X decreases or the path loops.
      auto* curve{new QCPCurve{plot->xAxis, plot->yAxis}};
      curve->setData(times, x, y, true);
      curve->setPen(QPen{QColor{32, 112, 160}});
      curve->setScatterStyle(QCPScatterStyle{QCPScatterStyle::ssCircle, 3.0});
      const auto x_bounds{std::minmax_element(x.cbegin(), x.cend())};
      const auto y_bounds{std::minmax_element(y.cbegin(), y.cend())};
      const auto path_span{std::max(*x_bounds.second - *x_bounds.first, *y_bounds.second - *y_bounds.first)};
      const auto span{path_span > 0.0 ? path_span : 1.0};
      const auto arrow_length{0.03 * span};
      for(const auto& sample: samples)
      {
        auto* arrow{new QCPItemLine{plot}};
        arrow->start->setCoords(sample.x, sample.y);
        arrow->end->setCoords(sample.x + (arrow_length * std::cos(sample.theta)),
                              sample.y + (arrow_length * std::sin(sample.theta)));
        arrow->setPen(QPen{QColor{180, 70, 35}});
        arrow->setHead(QCPLineEnding{QCPLineEnding::esSpikeArrow});
      }
      plot->xAxis->setLabel("X [m]");
      plot->yAxis->setLabel("Y [m]");
      plot->xAxis->setRange(*x_bounds.first - (0.15 * span), *x_bounds.second + (0.15 * span));
      plot->yAxis->setRange(*y_bounds.first - (0.15 * span), *y_bounds.second + (0.15 * span));
      const auto ranges{std::pair{plot->xAxis->range(), plot->yAxis->range()}};
      // Keep the requested view separate from the ranges expanded to fit the window.
      // A resize reuses that view; zoom or drag supplies a new one. Repeated redraws therefore
      // cannot enlarge the requested view by treating a previous expansion as new input.
      auto update_view{[plot, requested{ranges}, rendered{ranges}]() mutable {
        const auto current{std::pair{plot->xAxis->range(), plot->yAxis->range()}};
        if(current != rendered)
        {
          requested = current;
        }
        equal_axis_scales(*plot, requested.first, requested.second);
        rendered = {plot->xAxis->range(), plot->yAxis->range()};
      }};
      QObject::connect(plot, &QCustomPlot::beforeReplot, plot, std::move(update_view));
      plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
      return plot;
    }
  }  // namespace

  PlotterFigures::PlotterFigures(const std::vector<PreviewSample>& samples, const std::string& profile_file)
  {
    if(samples.empty() || samples.size() > static_cast<std::size_t>(std::numeric_limits<int>::max() / 2))
    {
      throw std::invalid_argument{"plotter needs a nonempty preview that fits Qt containers"};
    }
    signals_window_.setObjectName("profile_signals_window");
    trajectory_window_.setObjectName("profile_trajectory_window");
    const auto file{QString::fromStdString(profile_file)};
    signals_window_.setWindowTitle("Velocity and acceleration — " + file);
    trajectory_window_.setWindowTitle("Ideal trajectory — " + file);
    signals_window_.resize(1100, 850);
    trajectory_window_.resize(850, 750);
    auto* grid{new QGridLayout{&signals_window_}};
    const QColor linear_x{32, 112, 160};
    const QColor linear_y{45, 140, 80};
    const QColor angular_z{180, 70, 35};
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::vx,
                                "velocity_x",
                                "X velocity",
                                "m/s",
                                false,
                                linear_x),
                    0,
                    0);
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::ax,
                                "acceleration_x",
                                "X acceleration",
                                "m/s²",
                                true,
                                linear_x),
                    0,
                    1);
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::vy,
                                "velocity_y",
                                "Y velocity",
                                "m/s",
                                false,
                                linear_y),
                    1,
                    0);
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::ay,
                                "acceleration_y",
                                "Y acceleration",
                                "m/s²",
                                true,
                                linear_y),
                    1,
                    1);
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::wz,
                                "velocity_z",
                                "Z angular velocity",
                                "rad/s",
                                false,
                                angular_z),
                    2,
                    0);
    grid->addWidget(signal_plot(signals_window_,
                                samples,
                                &PreviewSample::awz,
                                "acceleration_z",
                                "Z angular acceleration",
                                "rad/s²",
                                true,
                                angular_z),
                    2,
                    1);
    auto* trajectory_layout{new QGridLayout{&trajectory_window_}};
    trajectory_layout->addWidget(trajectory_plot(trajectory_window_, samples), 0, 0);
  }

  void PlotterFigures::show()
  {
    signals_window_.show();
    trajectory_window_.show();
  }
}  // namespace ground_vehicle_motion_tester
