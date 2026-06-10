# tanh_ctrl Layered Parameters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor `tanh_ctrl` into a compact control-computation layer plus ROS2 layer, add 10Hz dynamic gain/filter reload, and provide a Python GUI for parameter tuning.

**Architecture:** `controller.cpp/.hpp` owns plain C++ control math and data structures without ROS2/PX4 dependencies. `node.cpp/.hpp` owns ROS2 subscriptions, publications, mission state, parameter IO, and PX4 message conversion. `main.cpp` only starts the ROS2 node.

**Tech Stack:** ROS2 Humble, C++17, Eigen, Sophus, px4_msgs, flat_trajectory_msgs, Python 3, rclpy, tkinter, clang-format.

---

### Task 1: Collapse C++ Files Into Two Layers

**Files:**
- Modify: `src/tanh_ctrl/CMakeLists.txt`
- Modify: `src/tanh_ctrl/include/tanh_ctrl/tanh_controller.hpp`
- Modify: `src/tanh_ctrl/include/tanh_ctrl/tanh_node.hpp`
- Rename: `src/tanh_ctrl/src/tanh_controller.cpp` to `src/tanh_ctrl/src/controller.cpp`
- Rename: `src/tanh_ctrl/src/tanh_node.cpp` to `src/tanh_ctrl/src/node.cpp`
- Rename: `src/tanh_ctrl/src/tanh_node_main.cpp` to `src/tanh_ctrl/src/main.cpp`
- Delete: `src/tanh_ctrl/include/tanh_ctrl/common.hpp`
- Delete: `src/tanh_ctrl/include/tanh_ctrl/tools.hpp`
- Delete: `src/tanh_ctrl/include/tanh_ctrl/px4_bridge.hpp`
- Delete: `src/tanh_ctrl/include/tanh_ctrl/tanh_blocks.hpp`
- Delete: `src/tanh_ctrl/include/tanh_ctrl/types.hpp`
- Delete: `src/tanh_ctrl/src/common.cpp`
- Delete: `src/tanh_ctrl/src/px4_bridge.cpp`
- Delete: `src/tanh_ctrl/src/tanh_blocks.cpp`

- [ ] **Step 1: Move controller-facing types into `tanh_controller.hpp`**

  Put `VehicleState`, `TrajectoryRef`, `PositionGains`, `AttitudeGains`, `AttitudeReference`, `ControlOutput`, private `Vec3LowPass`, and `TanhController` in `tanh_ctrl` namespace. Do not include ROS2 or PX4 headers in this file.

- [ ] **Step 2: Move math helper declarations out of public headers**

  Keep helper functions such as `planarAxisVec`, `computeLoopDtFromSample`, `computeDesiredAttitude`, `tanh_feedback`, and low-pass helpers private in `controller.cpp` or `node.cpp` unless they are part of the controller API.

- [ ] **Step 3: Merge implementation files**

  Merge `common.cpp` and `tanh_blocks.cpp` into `controller.cpp`. Merge `px4_bridge.cpp` and `tools.hpp` helpers into `node.cpp`.

- [ ] **Step 4: Add readable block comments**

  In `controller.cpp` and `node.cpp`, use section blocks in this style:

  ```cpp
  /************ math tools ***************/
  // code
  /***************************************/
  ```

  Use concise section names such as `math tools`, `controller lifecycle`, `parameter loading`, `px4 messages`, `callbacks`, and `mission state`.

- [ ] **Step 5: Update CMake target**

  Replace the internal helper libraries with one executable:

  ```cmake
  add_executable(tanh_ctrl_node
    src/main.cpp
    src/node.cpp
    src/controller.cpp
  )
  ```

  Link `Eigen3::Eigen` and `Sophus::Sophus`, and keep `ament_target_dependencies` for `rclcpp`, `px4_msgs`, `std_msgs`, and `flat_trajectory_msgs`.

### Task 2: Add 10Hz Runtime Parameter Reload

**Files:**
- Modify: `src/tanh_ctrl/include/tanh_ctrl/tanh_controller.hpp`
- Modify: `src/tanh_ctrl/include/tanh_ctrl/tanh_node.hpp`
- Modify: `src/tanh_ctrl/src/controller.cpp`
- Modify: `src/tanh_ctrl/src/node.cpp`

- [ ] **Step 1: Split startup and runtime parameter loading**

  Keep topic and mission parameters in a startup-only loader. Add a runtime loader for controller gains, filter cutoff values, mass, gravity, inertia, max collective thrust, diagonal wheelbase, and moment-to-thrust ratio. Compute body torque limits internally from those mixer geometry parameters.

- [ ] **Step 2: Add a 10Hz timer**

  Add a `rclcpp::TimerBase::SharedPtr parameter_reload_timer_` member and create it with `100ms` period after parameters are declared.

- [ ] **Step 3: Prevent filter reset on unchanged cutoff**

  In `TanhController::setVelocityDisturbanceLowPassHz` and `setAngularVelocityDisturbanceLowPassHz`, reset the low-pass state only when the cutoff changes meaningfully.

- [ ] **Step 4: Keep 800Hz loop continuity**

  The 10Hz loader must not call `controller_.reset()`. It should only set gains/model/filter parameters.

### Task 3: Add Python Parameter GUI

**Files:**
- Create: `src/tanh_ctrl/scripts/tanh_ctrl_param_gui.py`
- Modify: `src/tanh_ctrl/CMakeLists.txt`
- Modify: `src/tanh_ctrl/package.xml`

- [ ] **Step 1: Build tkinter/rclpy GUI**

  The GUI creates an `rclpy` node, connects to `/tanh_ctrl/get_parameters` and `/tanh_ctrl/set_parameters`, displays editable numeric entries for runtime tuning parameters, and provides `Refresh` and `Apply` buttons.

- [ ] **Step 2: Install the script**

  Add:

  ```cmake
  install(PROGRAMS
    scripts/tanh_ctrl_param_gui.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME tanh_ctrl_param_gui
  )
  ```

- [ ] **Step 3: Add runtime dependencies**

  Add `rclpy` as an `exec_depend` in `package.xml`. No Qt dependency is added.

### Task 4: Launch Integration

**Files:**
- Modify: `src/tanh_ctrl/launch/mujoco_sim.launch.py`

- [ ] **Step 1: Add `use_param_gui` launch argument**

  Default it to `false`.

- [ ] **Step 2: Conditionally launch GUI**

  Add a `launch_ros.actions.Node` for package `tanh_ctrl`, executable `tanh_ctrl_param_gui`, guarded by `IfCondition(LaunchConfiguration("use_param_gui"))`.

### Task 5: Format, Verify, and Commit

**Files:**
- Modify only files needed by Tasks 1-4.

- [ ] **Step 1: Run clang-format after code edits**

  Run:

  ```bash
  clang-format -i \
    src/tanh_ctrl/include/tanh_ctrl/tanh_controller.hpp \
    src/tanh_ctrl/include/tanh_ctrl/tanh_node.hpp \
    src/tanh_ctrl/src/controller.cpp \
    src/tanh_ctrl/src/node.cpp \
    src/tanh_ctrl/src/main.cpp
  ```

- [ ] **Step 2: Build**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select tanh_ctrl
  ```

- [ ] **Step 3: Check launch arguments**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch tanh_ctrl mujoco_sim.launch.py --show-args
  ```

  Confirm `use_param_gui` appears.

- [ ] **Step 4: Smoke test launch without GUI**

  Run:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  timeout 5s ros2 launch tanh_ctrl mujoco_sim.launch.py use_viewer:=false use_param_gui:=false
  ```

- [ ] **Step 5: Clean generated caches**

  Run:

  ```bash
  find src -type d -name __pycache__ -prune -exec rm -rf {} +
  ```

- [ ] **Step 6: Commit only necessary files**

  Run:

  ```bash
  git status --short
  git add src/tanh_ctrl docs/superpowers/plans/2026-06-10-tanh-ctrl-layered-params.md
  git commit -m "refactor: layer tanh_ctrl and add parameter gui"
  ```
