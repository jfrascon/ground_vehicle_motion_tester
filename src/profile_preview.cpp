#include "ground_vehicle_motion_tester/profile_preview.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>

#include "ground_vehicle_motion_tester/profile_program.hpp"
#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace ground_vehicle_motion_tester
{
  namespace
  {
    constexpr std::int64_t kIntegrationStepNs{10'000'000};

    /**
     * @brief Select the profile for an arbitrary preview time and use the shared evaluator.
     * @param program Validated, precomputed profile intervals and ramps.
     * @param stamp Relative nanosecond timestamp.
     * @return Commanded velocities and accelerations at the requested point.
     */
    MotionState preview_state(const ProfileProgram& program, std::int64_t stamp)
    {
      const auto& profiles{program.profiles()};
      const auto found{std::upper_bound(profiles.begin(),
                                        profiles.end(),
                                        stamp,
                                        [](std::int64_t value, const PreparedProfile& profile) {
                                          return value < profile.end_ns;
                                        })};
      const auto index{found == profiles.end() ? profiles.size() - 1 :
                                                 static_cast<std::size_t>(std::distance(profiles.begin(), found))};
      return program.evaluate(index, stamp);
    }
  }  // namespace

  ProfilePreview::ProfilePreview(const ProfileSequence& sequence, double sample_period)
  {
    // Preparation validates and computes every ramp once, before any pose integration.
    const ProfileProgram program{sequence};
    if(!std::isfinite(sample_period) || sample_period <= 0.0)
    {
      throw std::invalid_argument{"preview sample period must be finite and positive"};
    }
    const auto sample_step{profile_time_to_nanoseconds(sample_period)};
    if(sample_step == 0)
    {
      throw std::invalid_argument{"preview sample period is shorter than clock resolution"};
    }

    std::map<std::int64_t, double> times{{0, 0.0}};
    for(const auto& profile: program.profiles())
    {
      times[profile.start_ns] = profile.start_time;
      times[profile.end_ns] = profile.end_time;
      for(const auto& transition: {profile.x, profile.y, profile.z})
      {
        if(transition.initial != transition.final)
        {
          times[transition.end_ns] = transition.end_time;
        }
      }
    }
    const auto last_stamp{program.duration_ns()};
    // Integer keys merge decimal grid samples and coincident configured motion events.
    for(std::int64_t stamp{sample_step}; stamp < last_stamp;)
    {
      times.emplace(stamp, static_cast<double>(stamp) * kSecondsPerNanosecond);
      if(last_stamp - stamp <= sample_step)
      {
        break;
      }
      stamp += sample_step;
    }

    ground_vehicle_twist_odometry::GroundVehicleTwistOdometry odometry{};
    std::int64_t current_stamp{0};
    for(const auto& point: times)
    {
      // Integrate additional points without increasing the number of orientation arrows.
      do
      {
        current_stamp += std::min(kIntegrationStepNs, point.first - current_stamp);
        const auto state{preview_state(program, current_stamp)};
        const auto result{odometry.update({{state.vx, state.vy, state.wz}, current_stamp})};
        using Status = ground_vehicle_twist_odometry::UpdateStatus;
        if(result.status != Status::Initialized && result.status != Status::Integrated &&
           result.status != Status::SameTimestamp)
        {
          throw std::runtime_error{"odometry could not integrate the profile at t=" +
                                   std::to_string(static_cast<double>(current_stamp) * kSecondsPerNanosecond)};
        }
      } while(current_stamp < point.first);
      const auto state{preview_state(program, point.first)};
      const auto pose{odometry.pose()};
      samples_.push_back(
        {point.second, state.vx, state.vy, state.wz, state.ax, state.ay, state.awz, pose.x, pose.y, pose.yaw});
    }
  }

  const std::vector<PreviewSample>& ProfilePreview::samples() const noexcept
  {
    return samples_;
  }
}  // namespace ground_vehicle_motion_tester
