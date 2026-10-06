#pragma once

#include <vector>

#include "ground_vehicle_motion_tester/profile.hpp"

namespace ground_vehicle_motion_tester
{
  /// @brief Default separation of plotted points, in seconds; motion events are also included.
  inline constexpr auto kPreviewSamplePeriod{0.1};

  /** @brief Velocities, accelerations, and ideal planar pose at one preview time. */
  struct PreviewSample
  {
      /// @brief Time elapsed since the beginning of the complete sequence, in seconds.
      double time{0.0};
      /// @brief Commanded longitudinal velocity in the body frame, in m/s.
      double vx{0.0};
      /// @brief Commanded lateral velocity in the body frame, in m/s.
      double vy{0.0};
      /// @brief Commanded yaw velocity, in rad/s.
      double wz{0.0};
      /// @brief Instantaneous longitudinal profile acceleration, in m/s².
      double ax{0.0};
      /// @brief Instantaneous lateral profile acceleration, in m/s².
      double ay{0.0};
      /// @brief Instantaneous yaw profile acceleration, in rad/s².
      double awz{0.0};
      /// @brief Ideal X position relative to the initial pose, in metres.
      double x{0.0};
      /// @brief Ideal Y position relative to the initial pose, in metres.
      double y{0.0};
      /// @brief Integrated platform orientation relative to its initial heading, in radians.
      double theta{0.0};
  };

  /**
   * @brief Validate a sequence and compute its complete preview without Qt or ROS.
   *
   * Plot points include regular samples, profile boundaries, and every ramp completion.
   * The odometry library integrates from pose (0, 0, 0) with steps no larger than 10 ms.
   * This finer integration is independent of the plotted point separation.
   */
  class ProfilePreview
  {
    public:
      /**
       * @brief Prepare all plot data before a graphical consumer creates its figures.
       * @param sequence Profile data to validate and evaluate in execution order.
       * @param sample_period Positive separation of regular plotted points, in seconds.
       * @throws std::invalid_argument Invalid profiles or unrepresentable preview times.
       * @throws std::runtime_error The odometry library rejected a numerical integration step.
       */
      explicit ProfilePreview(const ProfileSequence& sequence, double sample_period = kPreviewSamplePeriod);

      /** @brief Return immutable samples in time order, including the final profile endpoint. */
      const std::vector<PreviewSample>& samples() const noexcept;

    private:
      std::vector<PreviewSample> samples_{};
  };
}  // namespace ground_vehicle_motion_tester
