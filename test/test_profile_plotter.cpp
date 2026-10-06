#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QGridLayout>
#include <QPixmap>
#include <QString>
#include <QWidget>
#include <qcustomplot.h>

#include "ground_vehicle_motion_tester/profile_preview.hpp"
#include "ground_vehicle_motion_tester/profile_yaml.hpp"
#include "profile_plotter.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  /** @brief Create one Qt application shared by the offscreen widget tests. */
  QApplication& application()
  {
    static int argc{1};
    static char name[]{"profile_plotter_test"};
    static char* argv[]{name, nullptr};
    static QApplication instance{argc, argv};
    return instance;
  }

  /**
   * @brief Locate a figure through the same top-level widget interface used by Qt itself.
   * @param name Stable figure identifier.
   * @return The matching window, or nullptr when the figure was not created.
   */
  QWidget* window(const QString& name)
  {
    for(auto* widget: QApplication::topLevelWidgets())
    {
      if(widget->objectName() == name)
      {
        return widget;
      }
    }
    return nullptr;
  }

  /** @brief Provide a path that reverses X while its heading changes independently. */
  std::vector<gvmt::PreviewSample> turning_samples()
  {
    const auto pi{std::acos(-1.0)};
    return {{0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, pi / 2.0},
            {2.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, pi}};
  }
}  // namespace

/** @test The first figure places each velocity left and its acceleration right on its axis row. */
TEST(ProfilePlotter, CreatesTheRequestedThreeByTwoSignalFigure)
{
  static_cast<void>(application());
  const gvmt::PlotterFigures figures{turning_samples(), "fixture.yaml"};
  auto* signals_figure{window("profile_signals_window")};
  ASSERT_NE(signals_figure, nullptr);
  auto* grid{dynamic_cast<QGridLayout*>(signals_figure->layout())};
  ASSERT_NE(grid, nullptr);
  const std::array<std::array<const char*, 2>, 3> names{
    {{"velocity_x", "acceleration_x"}, {"velocity_y", "acceleration_y"}, {"velocity_z", "acceleration_z"}}};
  for(int row{0}; row < 3; ++row)
  {
    for(int column{0}; column < 2; ++column)
    {
      ASSERT_NE(grid->itemAtPosition(row, column), nullptr);
      auto* plot{dynamic_cast<QCustomPlot*>(grid->itemAtPosition(row, column)->widget())};
      ASSERT_NE(plot, nullptr);
      EXPECT_EQ(plot->objectName(), QString{names[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)]});
      EXPECT_DOUBLE_EQ(plot->xAxis->range().lower, 0.0);
      EXPECT_DOUBLE_EQ(plot->xAxis->range().upper, 2.0);
    }
  }
}

/** @test Every curve point has an equal-length vector aligned with its stored platform yaw. */
TEST(ProfilePlotter, PreservesPathOrderAndDrawsOneOrientationArrowPerPoint)
{
  static_cast<void>(application());
  const auto samples{turning_samples()};
  const gvmt::PlotterFigures figures{samples, "fixture.yaml"};
  auto* trajectory_window{window("profile_trajectory_window")};
  ASSERT_NE(trajectory_window, nullptr);
  auto* plot{trajectory_window->findChild<QCustomPlot*>("trajectory")};
  ASSERT_NE(plot, nullptr);
  auto* curve{dynamic_cast<QCPCurve*>(plot->plottable(0))};
  ASSERT_NE(curve, nullptr);
  ASSERT_EQ(curve->data()->size(), static_cast<int>(samples.size()));
  ASSERT_EQ(plot->itemCount(), static_cast<int>(samples.size()));
  auto point{curve->data()->constBegin()};
  auto length{0.0};
  for(std::size_t index{0}; index < samples.size(); ++index, ++point)
  {
    EXPECT_DOUBLE_EQ(point->key, samples[index].x);
    EXPECT_DOUBLE_EQ(point->value, samples[index].y);
    auto* arrow{dynamic_cast<QCPItemLine*>(plot->item(static_cast<int>(index)))};
    ASSERT_NE(arrow, nullptr);
    EXPECT_DOUBLE_EQ(arrow->start->coords().x(), samples[index].x);
    EXPECT_DOUBLE_EQ(arrow->start->coords().y(), samples[index].y);
    const auto vector{arrow->end->coords() - arrow->start->coords()};
    const auto current_length{std::hypot(vector.x(), vector.y())};
    if(index == 0)
    {
      length = current_length;
    }
    ASSERT_GT(current_length, 0.0);
    EXPECT_NEAR(current_length, length, 1e-12);
    EXPECT_NEAR(vector.x() / length, std::cos(samples[index].theta), 1e-12);
    EXPECT_NEAR(vector.y() / length, std::sin(samples[index].theta), 1e-12);
  }
}

/**
 * @test Acceleration changes remain vertical discontinuities instead of diagonal interpolations.
 */
TEST(ProfilePlotter, DrawsAccelerationChangesAsSteps)
{
  static_cast<void>(application());
  const gvmt::PlotterFigures figures{turning_samples(), "fixture.yaml"};
  auto* signals_figure{window("profile_signals_window")};
  ASSERT_NE(signals_figure, nullptr);
  auto* plot{signals_figure->findChild<QCustomPlot*>("acceleration_x")};
  ASSERT_NE(plot, nullptr);
  const auto data{plot->graph()->data()};
  bool found_step{false};
  for(auto previous{data->constBegin()}, current{previous + 1}; current != data->constEnd(); ++previous, ++current)
  {
    if(previous->key == 1.0 && current->key == 1.0 && previous->value == 1.0 && current->value == 0.0)
    {
      found_step = true;
    }
  }
  EXPECT_TRUE(found_step);
}

/** @test Repeated resizing preserves the view, including a user-selected zoom and center. */
TEST(ProfilePlotter, KeepsTrajectoryViewStableAcrossResizeCycles)
{
  auto& app{application()};
  const gvmt::ProfilePreview preview{gvmt::load_profile_file(PROFILE_EXAMPLE_PATH)};
  gvmt::PlotterFigures figures{preview.samples(), "resize test"};
  figures.show();
  app.processEvents();
  auto* figure{window("profile_trajectory_window")};
  ASSERT_NE(figure, nullptr);
  auto* plot{figure->findChild<QCustomPlot*>("trajectory")};
  ASSERT_NE(plot, nullptr);
  for(auto pass{0}; pass < 3; ++pass)
  {
    plot->replot();
  }
  const auto original_size{figure->size()};
  const auto full_x{plot->xAxis->range()};
  const auto full_y{plot->yAxis->range()};
  for(auto view{0}; view < 2; ++view)
  {
    const auto selected_x{plot->xAxis->range()};
    const auto selected_y{plot->yAxis->range()};
    for(auto cycle{0}; cycle < 3; ++cycle)
    {
      figure->resize(1400, 450);
      app.processEvents();
      for(auto pass{0}; pass < 3; ++pass)
      {
        plot->replot();
      }
      figure->resize(original_size);
      app.processEvents();
      for(auto pass{0}; pass < 3; ++pass)
      {
        plot->replot();
      }
      EXPECT_NEAR(plot->xAxis->range().size(), selected_x.size(), 1e-10);
      EXPECT_NEAR(plot->yAxis->range().size(), selected_y.size(), 1e-10);
      EXPECT_NEAR(plot->xAxis->range().center(), selected_x.center(), 1e-12);
      EXPECT_NEAR(plot->yAxis->range().center(), selected_y.center(), 1e-12);
      EXPECT_NEAR(plot->xAxis->range().size() / plot->axisRect()->width(),
                  plot->yAxis->range().size() / plot->axisRect()->height(),
                  1e-12);
    }
    if(view == 0)
    {
      plot->xAxis->scaleRange(0.5);
      plot->yAxis->scaleRange(0.5);
      plot->xAxis->moveRange(0.4);
      plot->yAxis->moveRange(-0.6);
      for(auto pass{0}; pass < 3; ++pass)
      {
        plot->replot();
      }
      EXPECT_LT(plot->xAxis->range().size(), full_x.size() * 0.65);
      EXPECT_LT(plot->yAxis->range().size(), full_y.size() * 0.65);
      EXPECT_NEAR(plot->xAxis->range().center(), full_x.center() + 0.4, 1e-12);
      EXPECT_NEAR(plot->yAxis->range().center(), full_y.center() - 0.6, 1e-12);
    }
  }
}

/** @test The original rounded square closes at rest after four 90-degree turns. */
TEST(ProfilePlotter, RotatingRoundedSquareClosesAfterOneFullTurn)
{
  const gvmt::ProfilePreview preview{gvmt::load_profile_file(PROFILE_ROTATING_SQUARE_PATH)};
  const auto& start{preview.samples().front()};
  const auto& finish{preview.samples().back()};
  EXPECT_NEAR(finish.x, start.x, 1e-4);
  EXPECT_NEAR(finish.y, start.y, 1e-4);
  EXPECT_NEAR(finish.theta - start.theta, 2.0 * std::acos(-1.0), 1e-9);
  EXPECT_NEAR(finish.time, 15.0 + 2.0 * std::acos(-1.0), 1e-12);
  EXPECT_DOUBLE_EQ(finish.vx, 0.0);
  EXPECT_DOUBLE_EQ(finish.vy, 0.0);
  EXPECT_DOUBLE_EQ(finish.wz, 0.0);
}

/** @test The omnidirectional square closes without rotating and renders with equal XY scales. */
TEST(ProfilePlotter, RendersConstantOrientationRoundedSquareWithEqualXYScales)
{
  auto& app{application()};
  const gvmt::ProfilePreview preview{gvmt::load_profile_file(PROFILE_EXAMPLE_PATH)};
  const auto& start{preview.samples().front()};
  const auto& finish{preview.samples().back()};
  EXPECT_NEAR(finish.x, start.x, 1e-9);
  EXPECT_NEAR(finish.y, start.y, 1e-9);
  EXPECT_DOUBLE_EQ(finish.theta, start.theta);
  EXPECT_DOUBLE_EQ(finish.time, 27.0);
  EXPECT_DOUBLE_EQ(finish.vx, 0.0);
  EXPECT_DOUBLE_EQ(finish.vy, 0.0);
  EXPECT_DOUBLE_EQ(finish.wz, 0.0);
  gvmt::PlotterFigures figures{preview.samples(), "example_rounded_corner_square_constant_orientation.yaml"};
  figures.show();
  app.processEvents();
  auto* signals_figure{window("profile_signals_window")};
  auto* trajectory_window{window("profile_trajectory_window")};
  ASSERT_NE(signals_figure, nullptr);
  ASSERT_NE(trajectory_window, nullptr);
  auto* plot{trajectory_window->findChild<QCustomPlot*>("trajectory")};
  ASSERT_NE(plot, nullptr);
  plot->replot();
  const auto x_scale{plot->xAxis->range().size() / plot->axisRect()->width()};
  const auto y_scale{plot->yAxis->range().size() / plot->axisRect()->height()};
  EXPECT_NEAR(x_scale, y_scale, 1e-12);
  auto positive_y{false};
  auto negative_y{false};
  for(const auto& sample: preview.samples())
  {
    EXPECT_DOUBLE_EQ(sample.theta, start.theta);
    EXPECT_DOUBLE_EQ(sample.wz, 0.0);
    positive_y = positive_y || sample.vy > 0.0;
    negative_y = negative_y || sample.vy < 0.0;
    EXPECT_TRUE(plot->xAxis->range().contains(sample.x));
    EXPECT_TRUE(plot->yAxis->range().contains(sample.y));
  }
  EXPECT_TRUE(positive_y);
  EXPECT_TRUE(negative_y);
  const QString artifact_dir{PLOTTER_TEST_ARTIFACT_DIR};
  EXPECT_TRUE(signals_figure->grab().save(artifact_dir + "/profile_signals.png"));
  EXPECT_TRUE(trajectory_window->grab().save(artifact_dir + "/profile_trajectory.png"));
}
