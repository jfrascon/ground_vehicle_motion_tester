#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "ground_vehicle_motion_tester/profile_validator.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  /**
   * @brief Build a two-profile fixture with different axis transition times and signed ramps.
   * @return A valid sequence whose last linear velocities remain nonzero.
   */
  gvmt::ProfileSequence valid_sequence()
  {
    gvmt::ProfileSequence result{};
    result.limits.x = {2.0, 2.0, 0.0, 0.0};
    result.limits.y = {2.0, 2.0, 0.0, 0.0};
    result.limits.z = {0.0, 0.0, 2.0, 2.0};
    gvmt::Profile first{};
    first.duration = 3.0;
    first.x = {0.5, 0.0, 1.0, 0.0, 0.0, 0.0};
    first.y = {-0.5, 0.0, -0.5, 0.0, 0.0, 0.0};
    first.z = {0.0, 0.0, 0.0, 0.25, 0.0, 0.5};
    gvmt::Profile second{};
    second.duration = 3.0;
    second.x = {-0.5, 1.0, -0.5, 0.0, 0.0, 0.0};
    second.y = {0.0, -0.5, -0.5, 0.0, 0.0, 0.0};
    second.z = {0.0, 0.0, 0.0, -0.25, 0.5, 0.0};
    result.profiles = {first, second};
    return result;
  }

  /**
   * @brief Verify rejection identifies the exact field responsible for an invalid sequence.
   * @param sequence Fixture modified to violate one rule.
   * @param field YAML path expected in the thrown diagnostic.
   */
  void expect_error(const gvmt::ProfileSequence& sequence, const std::string& field)
  {
    try
    {
      gvmt::validate_profile_sequence(sequence);
      FAIL() << "Expected invalid field: " << field;
    }
    catch(const std::invalid_argument& error)
    {
      EXPECT_NE(std::string{error.what()}.find(field + ":"), std::string::npos) << error.what();
    }
  }
}  // namespace

/** @test Independent axis ramps and constant segments share one profile duration. */
TEST(ProfileValidator, AcceptsIndependentRampTimesAndConstantMotion)
{
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(valid_sequence()));
}

/** @test A sequence may start and end in motion within its configured bounds. */
TEST(ProfileValidator, AcceptsNonzeroInitialAndFinalVelocities)
{
  auto sequence{valid_sequence()};
  sequence.profiles.erase(sequence.profiles.begin());
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
}

/** @test Exactly matching an absolute bound remains valid. */
TEST(ProfileValidator, AcceptsLimitsAtTheirInclusiveBoundaries)
{
  auto sequence{valid_sequence()};
  sequence.limits.x.a_max_abs = 0.5;
  sequence.limits.x.v_max_abs = 1.0;
  sequence.limits.y.a_max_abs = 0.5;
  sequence.limits.y.v_max_abs = 0.5;
  sequence.limits.z.alpha_max_abs = 0.25;
  sequence.limits.z.w_max_abs = 0.5;
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
}

/** @test Zero bounds permit an unused axis when all its commands are zero. */
TEST(ProfileValidator, AcceptsAnAxisRestrictedToZero)
{
  auto sequence{valid_sequence()};
  sequence.limits.y = {};
  for(auto& profile: sequence.profiles)
  {
    profile.y = {};
  }
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
}

/** @test An empty sequence cannot describe an executable motion test. */
TEST(ProfileValidator, RejectsEmptySequence)
{
  auto sequence{valid_sequence()};
  sequence.profiles.clear();
  expect_error(sequence, "profiles");
}

/** @test Every profile needs a positive interval for its motion. */
TEST(ProfileValidator, RejectsNonpositiveDuration)
{
  for(const auto value: {0.0, -1.0})
  {
    auto sequence{valid_sequence()};
    sequence.profiles[0].duration = value;
    expect_error(sequence, "profiles[0].T");
  }
}

/** @test All input fields reject NaN and infinities before arithmetic. */
TEST(ProfileValidator, RejectsEveryNonfiniteProfileField)
{
  for(const auto value: {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()})
  {
    auto sequence{valid_sequence()};
    sequence.profiles[0].duration = value;
    expect_error(sequence, "profiles[0].T");
    for(const auto axis: {"x", "y", "z"})
    {
      for(const auto field: {"a", "v_init", "v_end", "alpha", "w_init", "w_end"})
      {
        sequence = valid_sequence();
        auto& profile{sequence.profiles[0]};
        auto& component{std::string{axis} == "x" ? profile.x : std::string{axis} == "y" ? profile.y : profile.z};
        if(std::string{field} == "a")
        {
          component.a = value;
        }
        else if(std::string{field} == "v_init")
        {
          component.v_init = value;
        }
        else if(std::string{field} == "v_end")
        {
          component.v_end = value;
        }
        else if(std::string{field} == "alpha")
        {
          component.alpha = value;
        }
        else if(std::string{field} == "w_init")
        {
          component.w_init = value;
        }
        else
        {
          component.w_end = value;
        }
        expect_error(sequence, std::string{"profiles[0]."} + axis + "." + field);
      }
    }
  }
}

/** @test Absolute bounds must be finite and nonnegative. */
TEST(ProfileValidator, RejectsNonfiniteAndNegativeActiveLimits)
{
  using Member = double gvmt::AxisLimits::*;
  for(const auto value: {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    for(const auto axis: {"x", "y", "z"})
    {
      const bool angular{std::string{axis} == "z"};
      for(const bool acceleration: {false, true})
      {
        auto sequence{valid_sequence()};
        auto& limits{std::string{axis} == "x" ? sequence.limits.x : angular ? sequence.limits.z : sequence.limits.y};
        const Member member{angular ? (acceleration ? &gvmt::AxisLimits::alpha_max_abs : &gvmt::AxisLimits::w_max_abs) :
                                      (acceleration ? &gvmt::AxisLimits::a_max_abs : &gvmt::AxisLimits::v_max_abs)};
        limits.*member = value;
        const auto field{angular ? (acceleration ? "alpha_max_abs" : "w_max_abs") :
                                   (acceleration ? "a_max_abs" : "v_max_abs")};
        expect_error(sequence, std::string{"limits."} + axis + "." + field);
      }
    }
  }
}

/** @test Zero-only fields stay strict even for tiny nonzero values. */
TEST(ProfileValidator, RejectsNonzeroInactiveLimitsAndComponents)
{
  auto sequence{valid_sequence()};
  sequence.limits.x.w_max_abs = 1e-12;
  expect_error(sequence, "limits.x.w_max_abs");
  sequence = valid_sequence();
  sequence.limits.y.alpha_max_abs = 1.0;
  expect_error(sequence, "limits.y.alpha_max_abs");
  sequence = valid_sequence();
  sequence.limits.z.a_max_abs = 1.0;
  expect_error(sequence, "limits.z.a_max_abs");
  sequence = valid_sequence();
  sequence.profiles[0].x.alpha = 1.0;
  expect_error(sequence, "profiles[0].x.alpha");
  sequence = valid_sequence();
  sequence.profiles[0].y.w_end = 1.0;
  expect_error(sequence, "profiles[0].y.w_end");
  sequence = valid_sequence();
  sequence.profiles[0].z.v_init = 1.0;
  expect_error(sequence, "profiles[0].z.v_init");
}

/** @test Signed commands obey absolute bounds. */
TEST(ProfileValidator, RejectsAccelerationAndBothVelocityEndpointsOutsideLimits)
{
  auto sequence{valid_sequence()};
  sequence.profiles[0].x.a = 2.1;
  expect_error(sequence, "profiles[0].x.a");
  sequence = valid_sequence();
  sequence.profiles[0].y.v_init = -2.1;
  expect_error(sequence, "profiles[0].y.v_init");
  sequence = valid_sequence();
  sequence.profiles[0].x.v_end = 2.1;
  expect_error(sequence, "profiles[0].x.v_end");
  sequence = valid_sequence();
  sequence.profiles[0].z.alpha = 2.1;
  expect_error(sequence, "profiles[0].z.alpha");
  sequence = valid_sequence();
  sequence.profiles[0].z.w_init = -2.1;
  expect_error(sequence, "profiles[0].z.w_init");
  sequence = valid_sequence();
  sequence.profiles[0].z.w_end = 2.1;
  expect_error(sequence, "profiles[0].z.w_end");
}

/** @test A changing velocity cannot reach its target without acceleration. */
TEST(ProfileValidator, RejectsZeroAccelerationForVelocityChange)
{
  auto sequence{valid_sequence()};
  sequence.profiles[0].x.a = 0.0;
  expect_error(sequence, "profiles[0].x.a");
  sequence = valid_sequence();
  sequence.profiles[0].z.alpha = 0.0;
  expect_error(sequence, "profiles[0].z.alpha");
}

/** @test Constant motion cannot request a nonzero acceleration. */
TEST(ProfileValidator, RejectsNonzeroAccelerationForConstantVelocity)
{
  auto sequence{valid_sequence()};
  sequence.profiles[1].y.a = 0.1;
  expect_error(sequence, "profiles[1].y.a");
  sequence = valid_sequence();
  sequence.profiles[0].z.w_end = sequence.profiles[0].z.w_init;
  expect_error(sequence, "profiles[0].z.alpha");
}

/** @test Acceleration must point toward the requested final velocity. */
TEST(ProfileValidator, RejectsWrongAccelerationSign)
{
  auto sequence{valid_sequence()};
  sequence.profiles[0].x.a = -0.5;
  expect_error(sequence, "profiles[0].x.a");
  sequence = valid_sequence();
  sequence.profiles[0].y.a = 0.5;
  expect_error(sequence, "profiles[0].y.a");
  sequence = valid_sequence();
  sequence.profiles[0].z.alpha = -0.25;
  expect_error(sequence, "profiles[0].z.alpha");
}

/** @test A profile must reach every target before its duration expires. */
TEST(ProfileValidator, RejectsTransitionLongerThanProfile)
{
  auto sequence{valid_sequence()};
  sequence.profiles[0].x.a = 0.25;
  expect_error(sequence, "profiles[0].x.a");
  sequence = valid_sequence();
  sequence.profiles[0].z.alpha = 0.1;
  expect_error(sequence, "profiles[0].z.alpha");
}

/** @test Rounding at T_acc = T is allowed without accepting a longer ramp. */
TEST(ProfileValidator, AcceptsRoundingAtTheTransitionTimeBoundary)
{
  auto sequence{valid_sequence()};
  sequence.profiles.resize(1);
  sequence.profiles[0].x.a = 0.1;
  sequence.profiles[0].x.v_end = 0.3;
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
  sequence.profiles[0].x.a = 0.1 - 1e-12;
  expect_error(sequence, "profiles[0].x.a");
}

/** @test Every active component joins the previous profile continuously. */
TEST(ProfileValidator, RejectsDiscontinuityInEachActiveComponent)
{
  auto sequence{valid_sequence()};
  sequence.profiles[1].x.v_init = 0.9;
  expect_error(sequence, "profiles[1].x.v_init");
  sequence = valid_sequence();
  sequence.profiles[1].y.v_init = 0.0;
  sequence.profiles[1].y.v_end = 0.0;
  expect_error(sequence, "profiles[1].y.v_init");
  sequence = valid_sequence();
  sequence.profiles[1].z.w_init = 0.4;
  expect_error(sequence, "profiles[1].z.w_init");
}

/** @test Only a small absolute velocity rounding difference is accepted. */
TEST(ProfileValidator, UsesAnAbsoluteContinuityTolerance)
{
  auto sequence{valid_sequence()};
  sequence.profiles[1].x.v_init -= gvmt::kVelocityContinuityTolerance / 2.0;
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
  sequence.profiles[1].x.v_init -= 2.0 * gvmt::kVelocityContinuityTolerance;
  expect_error(sequence, "profiles[1].x.v_init");
}

/** @test Finite extreme values use wider transition arithmetic. */
TEST(ProfileValidator, HandlesLargeFiniteVelocityDifferencesWithoutDoubleOverflow)
{
  auto sequence{valid_sequence()};
  sequence.profiles.resize(1);
  const auto maximum{std::numeric_limits<double>::max()};
  sequence.limits.x = {maximum, maximum, 0.0, 0.0};
  sequence.profiles[0].x = {maximum, -maximum, maximum, 0.0, 0.0, 0.0};
  EXPECT_NO_THROW(gvmt::validate_profile_sequence(sequence));
  sequence.profiles[0].duration = 1.0;
  expect_error(sequence, "profiles[0].x.a");
}
