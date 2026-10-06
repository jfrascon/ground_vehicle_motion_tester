#include <exception>
#include <iostream>
#include <string>

#include <QApplication>

#include "ground_vehicle_motion_tester/profile_preview.hpp"
#include "ground_vehicle_motion_tester/profile_yaml.hpp"
#include "profile_plotter.hpp"

/**
 * @brief Validate a YAML profile from the terminal and display its two preview figures.
 * @param argc Argument count, including the executable name.
 * @param argv Arguments containing one profile YAML path, or --help.
 * @return Zero after normal closure, one for invalid input, or two for incorrect CLI usage.
 */
int main(int argc, char** argv)
{
  if(argc != 2 || std::string{argv[1]} == "--help" || std::string{argv[1]} == "-h")
  {
    std::cout << "Usage: ground_vehicle_motion_plotter PROFILE.yaml\n"
              << "Validate the profile, then open its signals and ideal trajectory figures.\n";
    return argc == 2 ? 0 : 2;
  }
  try
  {
    const std::string profile_file{argv[1]};
    // Invalid files and numerical preview failures are reported before Qt needs a display.
    const ground_vehicle_motion_tester::ProfilePreview preview{
      ground_vehicle_motion_tester::load_profile_file(profile_file)};
    QApplication application{argc, argv};
    ground_vehicle_motion_tester::PlotterFigures figures{preview.samples(), profile_file};
    figures.show();
    return QApplication::exec();
  }
  catch(const std::exception& error)
  {
    std::cerr << "ground_vehicle_motion_plotter: " << error.what() << '\n';
    return 1;
  }
}
