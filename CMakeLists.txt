cmake_minimum_required(VERSION 3.5)
project(ground_vehicle_motion_tester)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ground_vehicle_twist_odometry REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets PrintSupport)
find_package(QCustomPlot REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Keep validation, prepared mathematics, and the execution cursor independent of ROS and YAML.
add_library(
  profile_core STATIC src/profile_validator.cpp src/profile_program.cpp src/profile_execution.cpp)
add_library(profile_yaml STATIC src/profile_yaml.cpp)
add_library(profile_preview STATIC src/profile_preview.cpp)

foreach(target IN ITEMS profile_core profile_yaml profile_preview)
  set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(
    ${target}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_options(${target} PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
endforeach()

# YAML consumers also link the shared validator, so both entry points enforce the same rules.
target_link_libraries(profile_yaml PUBLIC profile_core yaml-cpp)
target_link_libraries(
  profile_preview PUBLIC profile_core ground_vehicle_twist_odometry::twist_odometry)

add_library(profile_plotter_gui STATIC src/profile_plotter.cpp)
target_link_libraries(
  profile_plotter_gui PUBLIC profile_preview Qt5::Widgets Qt5::PrintSupport QCustomPlot::QCustomPlot)
target_compile_options(profile_plotter_gui PRIVATE -Wall -Wconversion -Wextra -Wpedantic)

add_executable(ground_vehicle_motion_plotter src/profile_plotter_main.cpp)
target_link_libraries(ground_vehicle_motion_plotter PRIVATE profile_plotter_gui profile_yaml)
target_compile_options(ground_vehicle_motion_plotter PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
install(TARGETS ground_vehicle_motion_plotter RUNTIME DESTINATION bin)

add_library(profile_executor_ros STATIC src/profile_executor_ros.cpp)
target_link_libraries(profile_executor_ros profile_core profile_yaml)
target_compile_options(profile_executor_ros PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
ament_target_dependencies(profile_executor_ros geometry_msgs rclcpp rcl_interfaces)

add_executable(ground_vehicle_motion_executor_node src/profile_executor_main.cpp)
target_link_libraries(ground_vehicle_motion_executor_node profile_executor_ros)
target_compile_options(ground_vehicle_motion_executor_node PRIVATE -Wall -Wconversion -Wextra -Wpedantic)
install(TARGETS ground_vehicle_motion_executor_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS profile_core profile_yaml profile_preview
  EXPORT export_ground_vehicle_motion_tester
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
# Launch imports can create bytecode caches beside the source; distribute maintained resources.
install(
  DIRECTORY config doc launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md LICENSE deps.repos DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_ground_vehicle_motion_tester HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp ground_vehicle_twist_odometry)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcl REQUIRED)

  # Exclude linters that require package-specific configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_cppcheck)

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)

  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format" include src test)

  find_package(ament_cmake_cpplint REQUIRED)

  ament_cpplint(
    MAX_LINE_LENGTH
    120
    FILTERS
    -whitespace/newline
    -whitespace/forcolon
    -build/include_order
    -runtime/indentation_namespace
    -legal/copyright)

  find_package(ament_cmake_flake8 REQUIRED)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  find_package(ament_cmake_cppcheck REQUIRED)

  # Check production headers and sources. Test sources have their own registered checks.
  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # Allow the installed Cppcheck version through the ROS 2 Jazzy wrapper.
  set_tests_properties(cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_add_gtest(test_profile_validator test/test_profile_validator.cpp)
  target_link_libraries(test_profile_validator profile_core)

  ament_add_gtest(test_profile_yaml test/test_profile_yaml.cpp)
  target_link_libraries(test_profile_yaml profile_yaml)
  target_compile_definitions(
    test_profile_yaml PRIVATE PROFILE_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/example_rounded_corner_square.yaml")

  ament_add_gtest(test_profile_preview test/test_profile_preview.cpp)
  target_link_libraries(test_profile_preview profile_preview)

  ament_add_gtest(test_profile_plotter test/test_profile_plotter.cpp)
  target_include_directories(test_profile_plotter PRIVATE src)
  target_link_libraries(test_profile_plotter profile_plotter_gui profile_yaml)
  target_compile_definitions(
    test_profile_plotter
    PRIVATE PLOTTER_TEST_ARTIFACT_DIR="${CMAKE_CURRENT_BINARY_DIR}"
    PROFILE_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/example_rounded_corner_square_constant_orientation.yaml"
    PROFILE_ROTATING_SQUARE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/example_rounded_corner_square.yaml")
  set_tests_properties(test_profile_plotter PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

  ament_add_gtest(test_profile_execution test/test_profile_execution.cpp)
  target_link_libraries(test_profile_execution profile_core profile_yaml)
  target_compile_definitions(
    test_profile_execution PRIVATE PROFILE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  ament_add_gtest(test_profile_executor_ros test/test_profile_executor_ros.cpp TIMEOUT 20)
  target_include_directories(test_profile_executor_ros PRIVATE src)
  target_link_libraries(test_profile_executor_ros profile_executor_ros)
  ament_target_dependencies(test_profile_executor_ros rclcpp rcl geometry_msgs)
  target_compile_definitions(
    test_profile_executor_ros
    PRIVATE EXECUTOR_TEST_PROFILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/executor_profile.yaml")
  set_tests_properties(
    test_profile_executor_ros PROPERTIES ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")

  ament_add_pytest_test(test_executor_launch test/test_executor_launch.py)
endif()

ament_package()
