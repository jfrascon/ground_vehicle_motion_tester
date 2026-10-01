# ground_vehicle_motion_tester

## About the package

`ground_vehicle_motion_tester` is a ROS 2 package for defining ground vehicle motion tests
through velocity profiles stored in YAML. It provides C++17 libraries to load and validate them
and a standalone desktop plotter that receives the YAML path as a terminal argument.

## Purpose

Use it to prepare tests such as forward/reverse motion, lateral movement, rotation, or combinations
of them by editing a configuration file. Each test describes how velocities change over time.

The validator detects incorrect fields, exceeded velocity or acceleration bounds, transitions
that cannot finish within the requested duration, and discontinuities between movements.
Your application can check the complete test before using it to command or preview robot motion.
The plotter shows the commanded velocities and accelerations together with the ideal trajectory
and the platform orientation at every plotted point.

## Usage

### Install dependencies

Place this package and `ground_vehicle_twist_odometry` under your workspace's `src/` directory.
The odometry checkout must provide its reusable C++ library. That library is currently present
as local changes in this workspace; upstream commit `2a910c9` does not yet export it.
Include the additional source dependencies listed in that package's `deps.repos`, such as
`ros2_launch_helpers`, before resolving system dependencies.

From the workspace root, install system dependencies through `rosdep`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`package.xml` declares yaml-cpp, Qt 5, QCustomPlot, and the odometry package. `rosdep` installs
the system libraries and skips dependencies supplied as source packages in the workspace.

### 1. Build the package

Run from your ROS 2 workspace directory:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --packages-up-to ground_vehicle_motion_tester
source install/setup.bash
```

### 2. Define your test

Start from [config/example_profile.yaml](config/example_profile.yaml).
Set your platform's absolute velocity and acceleration bounds in `limits`, then edit the ordered
movements in `profiles`:

- `T`: the movement duration in seconds, shared by all axes.
- `x` and `y`: linear acceleration `a` and velocities `v_init` and `v_end`.
- `z`: angular acceleration `alpha` and velocities `w_init` and `w_end`.

Use m/s and m/s² for linear motion, and rad/s and rad/s² for angular motion.
Choose an acceleration that reaches the target within `T`; the target velocity is then held
for the rest of the movement. For constant motion, set equal initial and final velocities and
zero acceleration. Match each final velocity to the next movement's initial velocity.

See the [complete YAML format](doc/design.md#profile-file-contract) for the required fields.

### 3. Preview the test from the terminal

After building and sourcing `install/setup.bash`, run:

```bash
ground_vehicle_motion_plotter src/0_deps/ground_vehicle_motion_tester/config/example_profile.yaml
```

Replace the example path with your profile YAML. The program checks the complete file before
opening two windows:

- Velocities and accelerations: three rows for X, Y, and yaw; velocity on the left and acceleration
  on the right.
- Ideal trajectory: positions in the XY plane with an orientation arrow at every plotted point.

Drag a graph to pan and use the mouse wheel to zoom. Close both windows to exit.
The program runs as a Qt desktop application. It requires a graphical session to display the figures.

```bash
ground_vehicle_motion_plotter --help
```

### 4. Use the validator in your application

Add `ground_vehicle_motion_tester` as a dependency of your package and link your existing CMake
target to the YAML loader:

```cmake
find_package(ground_vehicle_motion_tester REQUIRED)
target_link_libraries(my_consumer
  ground_vehicle_motion_tester::profile_yaml)
```

Include `ground_vehicle_motion_tester/profile_yaml.hpp` and call:

```cpp
const auto profiles{ground_vehicle_motion_tester::load_profile_file(profile_file)};
```

`profile_file` is your YAML file path. Relative paths are resolved from the application's
working directory. The function returns only after the whole file passes validation.
Use the returned sequence to carry out your test or produce its preview.

Catch `std::invalid_argument` to report configuration or file-reading errors. Messages identify
the file and the invalid field, for example:

```text
profiles[0].x.a: transition time (final - initial) / acceleration exceeds T
```

Profile indices in diagnostics start at zero. Check the indicated acceleration, velocity
endpoints, or duration before trying again.

### 5. Check the package

From the workspace directory, after building:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --merge-install --packages-select ground_vehicle_motion_tester
colcon test-result --verbose --test-result-base build/ground_vehicle_motion_tester
```

See [doc/design.md](doc/design.md) for the architecture, validation contract, and constructor
integration example.
