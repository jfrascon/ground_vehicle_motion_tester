#include <exception>
#include <iostream>
#include <string>

#include <QApplication>

#include "ground_vehicle_motion_tester/profile_preview.hpp"
#include "ground_vehicle_motion_tester/profile_yaml.hpp"
#include "profile_plotter.hpp"

/**
 * @brief Validate a YAML profile, optionally save its figures, and display them unless disabled.
 * @param argc Argument count, including the executable name.
 * @param argv Profile YAML path and optional PNG output paths, --no-show, or --help.
 * @return Zero on success, one for profile or export errors, or two for incorrect CLI usage.
 */
int main(int argc, char** argv)
{
  const auto print_usage{[]() {
    std::cout << "Usage: ground_vehicle_motion_plotter PROFILE.yaml [OPTIONS]\n"
              << "  --save-vel-profile PATH.png  Save velocities and accelerations.\n"
              << "  --save-trajectory PATH.png   Save the nominal XY trajectory.\n"
              << "  --no-show                    Export without opening windows.\n"
              << "  -h, --help                   Show this help and exit.\n"
              << "Without --no-show, both windows remain open after any requested exports.\n"
              << "--no-show requires at least one save option. Output directories must exist.\n";
  }};
  const auto usage_error{[&print_usage](const std::string& reason) {
    std::cerr << "ground_vehicle_motion_plotter: " << reason << '\n';
    print_usage();
    return 2;
  }};
  std::string profile_file{};
  std::string velocity_file{};
  std::string trajectory_file{};
  bool no_show{false};
  for(int index{1}; index < argc; ++index)
  {
    const std::string argument{argv[index]};
    if(argument == "--help" || argument == "-h")
    {
      print_usage();
      return 0;
    }
    if(argument == "--no-show")
    {
      no_show = true;
      continue;
    }
    if(argument == "--save-vel-profile" || argument == "--save-trajectory")
    {
      if(index + 1 >= argc || std::string{argv[index + 1]}.empty() || argv[index + 1][0] == '-')
      {
        return usage_error(argument + " requires a PNG output path");
      }
      auto& output{argument == "--save-vel-profile" ? velocity_file : trajectory_file};
      if(!output.empty())
      {
        return usage_error(argument + " may only be specified once");
      }
      output = argv[++index];
      continue;
    }
    if(argument.empty() || argument.front() == '-' || !profile_file.empty())
    {
      return usage_error("unexpected argument: " + argument);
    }
    profile_file = argument;
  }
  if(profile_file.empty())
  {
    return usage_error("one profile YAML path is required");
  }
  if(no_show && velocity_file.empty() && trajectory_file.empty())
  {
    return usage_error("--no-show requires at least one save option");
  }
  if(!velocity_file.empty() && velocity_file == trajectory_file)
  {
    return usage_error("the two figures require different output paths");
  }
  try
  {
    // Invalid files and numerical preview failures are reported before Qt needs a display.
    const ground_vehicle_motion_tester::ProfilePreview preview{
      ground_vehicle_motion_tester::load_profile_file(profile_file)};
    if(no_show)
    {
      // Qt still renders the widgets, using its offscreen backend instead of a display server.
      qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    QApplication application{argc, argv};
    ground_vehicle_motion_tester::PlotterFigures figures{preview.samples(), profile_file};
    if(!no_show)
    {
      figures.show();
    }
    if(!velocity_file.empty())
    {
      figures.save_velocity_profile(velocity_file);
    }
    if(!trajectory_file.empty())
    {
      figures.save_trajectory(trajectory_file);
    }
    if(no_show)
    {
      return 0;
    }
    return QApplication::exec();
  }
  catch(const std::exception& error)
  {
    std::cerr << "ground_vehicle_motion_plotter: " << error.what() << '\n';
    return 1;
  }
}
