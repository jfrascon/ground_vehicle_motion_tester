#include "ground_vehicle_motion_tester/profile_yaml.hpp"

#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <set>
#include <string>

#include <yaml-cpp/yaml.h>

#include "ground_vehicle_motion_tester/profile_validator.hpp"

namespace ground_vehicle_motion_tester
{
  namespace
  {
    /**
     * @brief Enforce a mapping's required keys and reject unknown or duplicate keys.
     *
     * Inspect entries before accessing them by name: duplicate YAML keys could otherwise
     * silently select one value and hide a configuration mistake.
     *
     * @param node Mapping to inspect.
     * @param path Mapping path used in errors.
     * @param required Fields that must exist even when their value is zero.
     * @param optional Fields allowed to be omitted.
     * @throws std::invalid_argument Wrong shape, duplicate/unknown key, or missing field.
     */
    void check_mapping(const YAML::Node& node,
                       const std::string& path,
                       std::initializer_list<std::string> required,
                       std::initializer_list<std::string> optional = {})
    {
      if(!node.IsMap())
      {
        throw std::invalid_argument{path + ": must be a mapping"};
      }
      std::set<std::string> allowed{required};
      allowed.insert(optional);
      std::set<std::string> seen{};
      for(const auto& entry: node)
      {
        if(!entry.first.IsScalar())
        {
          throw std::invalid_argument{path + ": field names must be scalar strings"};
        }
        const auto key{entry.first.as<std::string>()};
        const auto field_path{path + "." + key};
        if(!seen.insert(key).second)
        {
          throw std::invalid_argument{field_path + ": duplicate field"};
        }
        if(allowed.count(key) == 0)
        {
          throw std::invalid_argument{field_path + ": unknown field"};
        }
      }
      for(const auto& key: required)
      {
        if(seen.count(key) == 0)
        {
          throw std::invalid_argument{path + "." + key + ": required field is missing"};
        }
      }
    }

    /**
     * @brief Convert an existing numeric scalar without accepting quoted numeric strings.
     *
     * Schema validation establishes that the field exists. Finiteness is checked later by
     * the shared validator so programmatic inputs and YAML inputs obey the same numeric rules.
     *
     * @param node Mapping containing the field.
     * @param key Existing field name.
     * @param path Parent mapping path used in the error.
     * @return The numeric value as a double.
     * @throws std::invalid_argument If the field has a nonnumeric type or conversion fails.
     */
    double read_number(const YAML::Node& node, const std::string& key, const std::string& path)
    {
      const auto value{node[key]};
      const auto tag{value.Tag()};
      // Quoted numbers and explicitly tagged strings are configuration type errors.
      if(!value.IsScalar() || (tag != "?" && tag != "tag:yaml.org,2002:int" && tag != "tag:yaml.org,2002:float"))
      {
        throw std::invalid_argument{path + "." + key + ": must be a number"};
      }
      try
      {
        return value.as<double>();
      }
      catch(const YAML::Exception&)
      {
        throw std::invalid_argument{path + "." + key + ": must be a number"};
      }
    }

    /**
     * @brief Read an optional component, using zero only when the key is absent.
     * @param node Mapping already checked against its axis schema.
     * @param key Numeric field name.
     * @param path Parent mapping path used in the error.
     * @return Supplied numeric value or zero for an omitted key.
     * @throws std::invalid_argument A present field cannot be converted to a number.
     */
    double optional_number(const YAML::Node& node, const std::string& key, const std::string& path)
    {
      return node[key].IsDefined() ? read_number(node, key, path) : 0.0;
    }

    /**
     * @brief Load one axis's bounds after verifying its required and optional field names.
     * @param node Mapping containing axis bounds.
     * @param path Axis path under limits.
     * @param angular True for Z rotation; false for X or Y translation.
     * @return All bounds, with omitted inactive components set to zero.
     * @throws std::invalid_argument An axis schema or numeric field is invalid.
     */
    AxisLimits read_limits(const YAML::Node& node, const std::string& path, bool angular)
    {
      if(angular)
      {
        check_mapping(node, path, {"alpha_max_abs", "w_max_abs"}, {"a_max_abs", "v_max_abs"});
      }
      else
      {
        check_mapping(node, path, {"a_max_abs", "v_max_abs"}, {"alpha_max_abs", "w_max_abs"});
      }
      AxisLimits result{};
      result.a_max_abs = optional_number(node, "a_max_abs", path);
      result.v_max_abs = optional_number(node, "v_max_abs", path);
      result.alpha_max_abs = optional_number(node, "alpha_max_abs", path);
      result.w_max_abs = optional_number(node, "w_max_abs", path);
      return result;
    }

    /**
     * @brief Load one axis's transition with the schema appropriate to planar motion.
     * @param node Mapping containing the requested transition.
     * @param path Profile and axis path used in errors.
     * @param angular True for Z rotation; false for X or Y translation.
     * @return All components, with omitted inactive components set to zero.
     * @throws std::invalid_argument An axis schema or numeric field is invalid.
     */
    AxisProfile read_axis(const YAML::Node& node, const std::string& path, bool angular)
    {
      if(angular)
      {
        check_mapping(node, path, {"alpha", "w_init", "w_end"}, {"a", "v_init", "v_end"});
      }
      else
      {
        check_mapping(node, path, {"a", "v_init", "v_end"}, {"alpha", "w_init", "w_end"});
      }
      AxisProfile result{};
      result.a = optional_number(node, "a", path);
      result.v_init = optional_number(node, "v_init", path);
      result.v_end = optional_number(node, "v_end", path);
      result.alpha = optional_number(node, "alpha", path);
      result.w_init = optional_number(node, "w_init", path);
      result.w_end = optional_number(node, "w_end", path);
      return result;
    }
  }  // namespace

  ProfileSequence load_profile_yaml(const std::string& yaml)
  {
    try
    {
      // Loading all documents lets us reject an accidental second configuration instead of
      // silently ignoring it. Profiles still belong to exactly one limits block.
      const auto documents{YAML::LoadAll(yaml)};
      if(documents.size() != 1)
      {
        throw std::invalid_argument{"document: must contain exactly one YAML document"};
      }
      const auto& root{documents.front()};
      check_mapping(root, "document", {"limits", "profiles"});
      const auto limits{root["limits"]};
      check_mapping(limits, "limits", {"x", "y", "z"});
      ProfileSequence result{};
      result.limits.x = read_limits(limits["x"], "limits.x", false);
      result.limits.y = read_limits(limits["y"], "limits.y", false);
      result.limits.z = read_limits(limits["z"], "limits.z", true);

      const auto profiles{root["profiles"]};
      if(!profiles.IsSequence())
      {
        throw std::invalid_argument{"profiles: must be a sequence"};
      }
      for(std::size_t index{0}; index < profiles.size(); ++index)
      {
        const auto node{profiles[index]};
        const auto path{"profiles[" + std::to_string(index) + "]"};
        check_mapping(node, path, {"T", "x", "y", "z"});
        Profile profile{};
        profile.duration = read_number(node, "T", path);
        profile.x = read_axis(node["x"], path + ".x", false);
        profile.y = read_axis(node["y"], path + ".y", false);
        profile.z = read_axis(node["z"], path + ".z", true);
        result.profiles.push_back(profile);
      }
      // Parsing checks structure and types; the independent validator owns all motion rules.
      // Return only after the complete sequence passes so no consumer sees a partial result.
      validate_profile_sequence(result);
      return result;
    }
    catch(const YAML::Exception& error)
    {
      throw std::invalid_argument{std::string{"document: invalid YAML: "} + error.what()};
    }
  }

  ProfileSequence load_profile_file(const std::string& path)
  {
    std::ifstream stream{path};
    if(!stream)
    {
      throw std::invalid_argument{path + ": cannot open profile file"};
    }
    std::string yaml{};
    try
    {
      // An opened path can still be unreadable, for example when it points to a directory.
      // Keep these stream failures in the same exception contract as validation failures.
      yaml.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
    }
    catch(const std::ios_base::failure&)
    {
      throw std::invalid_argument{path + ": cannot read profile file"};
    }
    if(stream.bad())
    {
      throw std::invalid_argument{path + ": cannot read profile file"};
    }
    try
    {
      return load_profile_yaml(yaml);
    }
    catch(const std::invalid_argument& error)
    {
      throw std::invalid_argument{path + ": " + error.what()};
    }
  }
}  // namespace ground_vehicle_motion_tester
