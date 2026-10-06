#pragma once

#include <string>
#include <vector>

#include <QWidget>

#include "ground_vehicle_motion_tester/profile_preview.hpp"

namespace ground_vehicle_motion_tester
{
  /**
   * @brief Display previously computed samples in two independent Qt figures.
   *
   * This presentation component receives numeric data. The ProfilePreview constructor has
   * already validated the profiles and computed their ideal odometry before these widgets exist.
   */
  class PlotterFigures
  {
    public:
      /**
       * @brief Create the 3-by-2 signal figure and the XY trajectory with one arrow per point.
       * @param samples Validated preview samples in increasing time order.
       * @param profile_file File name used to identify the figures.
       * @throws std::invalid_argument The preview is empty or too large for Qt containers.
       */
      PlotterFigures(const std::vector<PreviewSample>& samples, const std::string& profile_file);

      /** @brief Show both figures; QApplication handles interaction until they are closed. */
      void show();

    private:
      QWidget signals_window_{};
      QWidget trajectory_window_{};
  };
}  // namespace ground_vehicle_motion_tester
