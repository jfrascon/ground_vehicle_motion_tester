#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "ground_vehicle_motion_tester/profile_yaml.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  const char* const kValidYaml{R"(
limits:
  x: {a_max_abs: 2, v_max_abs: 2}
  y: {a_max_abs: 2, v_max_abs: 2}
  z: {alpha_max_abs: 2, w_max_abs: 2}
profiles:
  - T: 3
    x: {a: 0.5, v_init: 0, v_end: 1}
    y: {a: -0.5, v_init: 0, v_end: -0.5}
    z: {alpha: 0.25, w_init: 0, w_end: 0.5}
)"};

  /**
   * @brief Replace one fragment of the valid fixture to isolate a YAML contract violation.
   * @param original Known text identifying one fixture field or mapping.
   * @param replacement Invalid or alternative YAML to insert in its place.
   * @return The modified YAML fixture.
   */
  std::string changed_yaml(const std::string& original, const std::string& replacement)
  {
    std::string result{kValidYaml};
    const auto position{result.find(original)};
    if(position == std::string::npos)
    {
      throw std::logic_error{"Test fixture fragment not found: " + original};
    }
    result.replace(position, original.size(), replacement);
    return result;
  }

  /**
   * @brief Verify that invalid YAML is rejected before any profile sequence is returned.
   * @param yaml Input that violates a schema or motion rule.
   * @param field Field path expected in the diagnostic.
   */
  void expect_error(const std::string& yaml, const std::string& field)
  {
    try
    {
      static_cast<void>(gvmt::load_profile_yaml(yaml));
      FAIL() << "Expected invalid YAML at: " << field;
    }
    catch(const std::invalid_argument& error)
    {
      EXPECT_NE(std::string{error.what()}.find(field + ":"), std::string::npos) << error.what();
    }
  }

  /** @brief One isolated schema mistake and the field that must explain its rejection. */
  struct InvalidYamlCase
  {
      const char* original;
      const char* replacement;
      const char* field;
  };

  /** @brief Run the same field-path assertion for each isolated YAML schema mistake. */
  class InvalidYaml: public ::testing::TestWithParam<InvalidYamlCase>
  {};
}  // namespace

/** @test Plain integer and floating values load into the independent data model. */
TEST(ProfileYaml, LoadsValidNumericFieldsAndDefaultsInactiveComponentsToZero)
{
  const auto sequence{gvmt::load_profile_yaml(kValidYaml)};
  ASSERT_EQ(sequence.profiles.size(), 1U);
  EXPECT_DOUBLE_EQ(sequence.limits.x.a_max_abs, 2.0);
  EXPECT_DOUBLE_EQ(sequence.profiles[0].duration, 3.0);
  EXPECT_DOUBLE_EQ(sequence.profiles[0].y.v_end, -0.5);
  EXPECT_DOUBLE_EQ(sequence.profiles[0].z.w_end, 0.5);
  EXPECT_DOUBLE_EQ(sequence.limits.x.w_max_abs, 0.0);
  EXPECT_DOUBLE_EQ(sequence.profiles[0].z.v_init, 0.0);
}

/** @test Explicit zero-only fields obey the same contract as omitted optional components. */
TEST(ProfileYaml, AcceptsExplicitZeroInactiveFields)
{
  const auto yaml{std::string{R"(
limits:
  x: {a_max_abs: 2, v_max_abs: 2, alpha_max_abs: 0, w_max_abs: 0}
  y: {a_max_abs: 2, v_max_abs: 2, alpha_max_abs: 0, w_max_abs: 0}
  z: {alpha_max_abs: 2, w_max_abs: 2, a_max_abs: 0, v_max_abs: 0}
profiles:
  - T: 3
    x: {a: 0.5, v_init: 0, v_end: 1, alpha: 0, w_init: 0, w_end: 0}
    y: {a: -0.5, v_init: 0, v_end: -0.5, alpha: 0, w_init: 0, w_end: 0}
    z: {alpha: 0.25, w_init: 0, w_end: 0.5, a: 0, v_init: 0, v_end: 0}
)"}};
  EXPECT_NO_THROW(gvmt::load_profile_yaml(yaml));
}

/** @test Missing, unknown, duplicate, and incorrectly typed fields identify their own paths. */
TEST_P(InvalidYaml, RejectsInvalidFieldWithItsPath)
{
  const auto& param{GetParam()};
  expect_error(changed_yaml(param.original, param.replacement), param.field);
}

INSTANTIATE_TEST_SUITE_P(
  Schema,
  InvalidYaml,
  ::testing::Values(
    InvalidYamlCase{"limits:", "limits:\n  q: {}", "limits.q"},
    InvalidYamlCase{"profiles:", "extra: 1\nprofiles:", "document.extra"},
    InvalidYamlCase{"profiles:", "profiles: []\nprofiles:", "document.profiles"},
    InvalidYamlCase{"limits:", "bounds:", "document.bounds"},
    InvalidYamlCase{"profiles:\n  - T: 3", "profiles:\n  - duration: 3", "profiles[0].duration"},
    InvalidYamlCase{"  - T: 3", "  - x: {}", "profiles[0].x"},
    InvalidYamlCase{"  - T: 3", "  - T: 3\n    T: 3", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: '3'", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: !!str 3", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: true", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: null", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: [3]", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: .nan", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: .inf", "profiles[0].T"},
    InvalidYamlCase{"  - T: 3", "  - T: 0", "profiles[0].T"},
    InvalidYamlCase{"x: {a_max_abs: 2, v_max_abs: 2}", "x: {}", "limits.x.a_max_abs"},
    InvalidYamlCase{"x: {a_max_abs: 2, v_max_abs: 2}", "x: 1", "limits.x"},
    InvalidYamlCase{"x: {a_max_abs: 2, v_max_abs: 2}",
                    "x: {a_max_abs: 2, v_max_abs: 2, v_max_abs: 1}",
                    "limits.x.v_max_abs"},
    InvalidYamlCase{"x: {a_max_abs: 2, v_max_abs: 2}", "x: {a_max_abs: -2, v_max_abs: 2}", "limits.x.a_max_abs"},
    InvalidYamlCase{"x: {a_max_abs: 2, v_max_abs: 2}",
                    "x: {a_max_abs: 2, v_max_abs: 2, w_max_abs: 1}",
                    "limits.x.w_max_abs"},
    InvalidYamlCase{"y: {a_max_abs: 2, v_max_abs: 2}",
                    "y: {a_max_abs: 2, v_max_abs: 2, alpha_max_abs: 1}",
                    "limits.y.alpha_max_abs"},
    InvalidYamlCase{"z: {alpha_max_abs: 2, w_max_abs: 2}",
                    "z: {alpha_max_abs: 2, w_max_abs: 2, v_max_abs: 1}",
                    "limits.z.v_max_abs"},
    InvalidYamlCase{"x: {a: 0.5, v_init: 0, v_end: 1}", "x: {v_init: 0, v_end: 1}", "profiles[0].x.a"},
    InvalidYamlCase{"x: {a: 0.5, v_init: 0, v_end: 1}",
                    "x: {a: 0.5, v_init: 0, v_end: 1, typo: 0}",
                    "profiles[0].x.typo"},
    InvalidYamlCase{"x: {a: 0.5, v_init: 0, v_end: 1}", "x: {a: 0.5, a: 0.5, v_init: 0, v_end: 1}", "profiles[0].x.a"},
    InvalidYamlCase{"x: {a: 0.5, v_init: 0, v_end: 1}",
                    "x: {a: 0.5, v_init: 0, v_end: 1, alpha: 1}",
                    "profiles[0].x.alpha"},
    InvalidYamlCase{"y: {a: -0.5, v_init: 0, v_end: -0.5}",
                    "y: {a: -0.5, v_init: 0, v_end: -0.5, w_init: 1}",
                    "profiles[0].y.w_init"},
    InvalidYamlCase{"z: {alpha: 0.25, w_init: 0, w_end: 0.5}",
                    "z: {alpha: 0.25, w_init: 0, w_end: 0.5, a: 1}",
                    "profiles[0].z.a"},
    InvalidYamlCase{"z: {alpha: 0.25, w_init: 0, w_end: 0.5}",
                    "z: {alpha: 0, w_init: 0, w_end: 0.5}",
                    "profiles[0].z.alpha"},
    InvalidYamlCase{"z: {alpha: 0.25, w_init: 0, w_end: 0.5}",
                    "z: {alpha: 0.1, w_init: 0, w_end: 0.5}",
                    "profiles[0].z.alpha"}));

/** @test Empty input, wrong collection shapes, and extra documents cannot hide invalid data. */
TEST(ProfileYaml, RejectsInvalidDocumentAndCollectionShapes)
{
  expect_error("", "document");
  expect_error("[]", "document");
  expect_error("limits: {}\nprofiles: []", "limits.x");
  expect_error("limits: []\nprofiles: []", "limits");
  std::string limits_only{kValidYaml};
  limits_only.erase(limits_only.find("profiles:"));
  expect_error(limits_only + "profiles: {}", "profiles");
  expect_error(limits_only + "profiles: [1]", "profiles[0]");
  expect_error(limits_only + "profiles: [{}]", "profiles[0].T");
  expect_error(limits_only, "document.profiles");
  expect_error("profiles: []", "document.limits");
  expect_error(std::string{kValidYaml} + "\n---\n{}", "document");
  expect_error("limits: [", "document");
}

/** @test An empty sequence is invalid even when all limit mappings are correct. */
TEST(ProfileYaml, RejectsEmptyProfileList)
{
  std::string yaml{kValidYaml};
  yaml.erase(yaml.find("profiles:"));
  yaml += "profiles: []\n";
  expect_error(yaml, "profiles");
}

/** @test All profiles pass the shared validator before the loader returns the sequence. */
TEST(ProfileYaml, RejectsDiscontinuityInALaterProfile)
{
  const std::string yaml{std::string{kValidYaml} + R"(
  - T: 3
    x: {a: 0, v_init: 0, v_end: 0}
    y: {a: 0, v_init: -0.5, v_end: -0.5}
    z: {alpha: 0, w_init: 0.5, w_end: 0.5}
)"};
  expect_error(yaml, "profiles[1].x.v_init");
}

/** @test The rounded square example loads with continuous velocities and a final stop. */
TEST(ProfileYaml, LoadsTheShippedExampleFile)
{
  const auto sequence{gvmt::load_profile_file(PROFILE_EXAMPLE_PATH)};
  ASSERT_EQ(sequence.profiles.size(), 14U);
  EXPECT_DOUBLE_EQ(sequence.profiles.front().x.v_init, 0.0);
  EXPECT_DOUBLE_EQ(sequence.profiles.back().x.v_end, 0.0);
  EXPECT_DOUBLE_EQ(sequence.profiles.back().y.v_end, 0.0);
  EXPECT_DOUBLE_EQ(sequence.profiles.back().z.w_end, 0.0);
}

/** @test File access errors include the supplied path so a constructor failure is actionable. */
TEST(ProfileYaml, ReportsMissingFile)
{
  const std::string path{std::string{PROFILE_EXAMPLE_PATH} + ".does_not_exist"};
  try
  {
    static_cast<void>(gvmt::load_profile_file(path));
    FAIL() << "Expected missing-file error";
  }
  catch(const std::invalid_argument& error)
  {
    EXPECT_NE(std::string{error.what()}.find(path + ":"), std::string::npos);
  }
}

/** @test A directory cannot leak a stream exception outside the documented loader contract. */
TEST(ProfileYaml, ReportsUnreadableFileWithTheSameExceptionType)
{
  const std::string path{std::filesystem::path(PROFILE_EXAMPLE_PATH).parent_path().string()};
  try
  {
    static_cast<void>(gvmt::load_profile_file(path));
    FAIL() << "Expected directory-read error";
  }
  catch(const std::invalid_argument& error)
  {
    EXPECT_NE(std::string{error.what()}.find(path + ":"), std::string::npos);
  }
}
