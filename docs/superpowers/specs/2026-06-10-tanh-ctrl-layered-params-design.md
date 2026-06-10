# tanh_ctrl Layered Parameters Design

## Goal

Refactor `tanh_ctrl` into a clearer control-computation layer and ROS2 interface layer, reduce the number of C++ source/header files, and add runtime tuning for controller gains and filters through a Python GUI.

## Current Structure

The package currently separates small helper units into many files:

- `common.*` for math helpers and trajectory attitude reconstruction.
- `tanh_blocks.*` for tanh feedback and low-pass filters.
- `px4_bridge.*` for PX4 setpoint message construction.
- `tanh_controller.*` for controller state and loop math.
- `tanh_node.*` for ROS2 subscriptions, publications, mission state, and parameter loading.
- `tanh_node_main.cpp` for process entry.

This is functionally clear but too fragmented for the current project size. The refactor should reduce file count while preserving the boundary between control computation and ROS2 integration.

## Target Structure

The C++ implementation will use three source files:

- `src/tanh_ctrl/src/main.cpp`
  - Owns `rclcpp::init`, node construction, spin, and shutdown.
- `src/tanh_ctrl/src/controller.cpp`
  - Owns tanh control math, low-pass filters, attitude reconstruction, flatness feedforward, and controller runtime state.
  - Contains code currently spread across `common.cpp`, `tanh_blocks.cpp`, and `tanh_controller.cpp`.
- `src/tanh_ctrl/src/node.cpp`
  - Owns the ROS2 node, PX4 subscriptions/publications, mission state, parameter declaration/loading, and PX4 setpoint message construction.
  - Contains code currently spread across `tanh_node.cpp` and `px4_bridge.cpp`.

The public C++ headers will be reduced to the layer boundary:

- `include/tanh_ctrl/tanh_controller.hpp`
  - Defines controller-facing data types such as vehicle state, trajectory reference, gains, attitude reference, control output, and `TanhController`.
  - Exposes only the API needed by the ROS2 layer.
- `include/tanh_ctrl/tanh_node.hpp`
  - Defines `TanhNode` and ROS2-layer state.

Helper headers such as `common.hpp`, `tools.hpp`, `px4_bridge.hpp`, `tanh_blocks.hpp`, and `types.hpp` will be removed after their declarations are folded into the two layer headers or made private to source files.

## Layering Rules

The control layer must not include ROS2 or PX4 message headers. It should depend only on Eigen, Sophus, and standard C++.

The ROS2 layer converts PX4 and flat trajectory messages into controller-native types, calls the controller, and converts controller output back into PX4 thrust/torque setpoints.

This keeps the controller usable as a plain C++ component while allowing the ROS2 node to handle timing, topics, parameters, and mission state.

## Runtime Parameter Loading

The node will keep startup parameter declaration in one place, then add a 10Hz timer that reloads controller tuning parameters:

- Position gains: `M_P`, `K_P`, `M_V`, `K_V`, `K_Acceleration`, observer `P_V`, observer `L_V`.
- Attitude gains: `M_Angle`, `K_Angle`, `M_AngularVelocity`, `K_AngularVelocity`, `K_AngularAcceleration`, observer `P_AngularVelocity`, observer `L_AngularVelocity`.
- Filters: `filters.velocity_disturbance_cutoff_hz`, `filters.angular_velocity_disturbance_cutoff_hz`.
- Model values needed by controller output scaling: mass, gravity, inertia, max collective thrust, diagonal wheelbase, and moment-to-thrust ratio. Body torque limits are computed internally from these mixer geometry parameters.

Mission parameters and topic names remain startup configuration. They should not be part of the fast tuning loop unless a future requirement explicitly asks for that.

The timer must avoid disturbing 800Hz control continuity. Reloading gains should not reset controller runtime observer state. Low-pass filter state should only reset when its cutoff value changes.

## Python Parameter GUI

Add a Python executable in `tanh_ctrl` using `tkinter` and `rclpy`.

The GUI will:

- Connect to the `/tanh_ctrl` parameter services.
- Read and display the same tuning parameters that the 10Hz C++ timer reloads.
- Allow numeric edits and send changed values with `set_parameters`.
- Provide refresh and apply actions.

The GUI should avoid adding Qt or other heavy dependencies. It should be installed through the `tanh_ctrl` package so it can be launched with `ros2 run tanh_ctrl tanh_ctrl_param_gui`.

The integrated launch file `mujoco_sim.launch.py` will add an optional `use_param_gui` argument, defaulting to `false`, and start the GUI only when requested.

## Verification

The implementation will be verified with:

- `colcon build --symlink-install --packages-select tanh_ctrl`
- `ros2 launch tanh_ctrl mujoco_sim.launch.py --show-args`
- `timeout 5s ros2 launch tanh_ctrl mujoco_sim.launch.py use_viewer:=false use_param_gui:=false`

If GUI verification is possible in the current environment, the parameter service interaction will also be checked by launching the node and using ROS2 parameter APIs. Any temporary test artifacts created during verification must be removed before the final commit.
