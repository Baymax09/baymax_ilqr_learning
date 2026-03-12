# AGENTS.md - iLQR Project Development Guide

## Project Overview

This is a ROS2 workspace with an iLQR (Iterative Linear Quadratic Regulator) solver for autonomous vehicle trajectory optimization. Main package: `ilqr_learning`.

## Build Commands

### Build the Workspace

```bash
colcon build
```

### Build a Specific Package

```bash
colcon build --packages-select ilqr_learning
```

### Clean Build

```bash
rm -rf build/ install/ log/
colcon build
```

### Run the iLQR Node

```bash
source install/setup.bash
ros2 run ilqr_learning ilqr_node

# Or use the provided script
./run_ilqr.sh
```

### Run with Custom Parameters

```bash
ros2 run ilqr_learning ilqr_node --ros-args \
    -p dt:=0.05 -p wheelbase:=2.5 -p horizon:=200 \
    -p x0:="[0.0, 0.0, 0.0, 0.0]" -p target:="[10.0, 10.0, 0.0, 0.0]"
```

## Lint and Test Commands

### Run Linters

```bash
colcon test --packages-select ilqr_learning
colcon test-result --verbose
```

### Run CPPLint

```bash
cpplint.py ilqr_learning/src/*.cpp ilqr_learning/include/ilqr_learning/*.h
```

### Run Clang Format

```bash
# Check formatting
clang-format -n ilqr_learning/src/*.cpp ilqr_learning/include/ilqr_learning/*.h

# Format files (in-place)
clang-format -i ilqr_learning/src/*.cpp ilqr_learning/include/ilqr_learning/*.h
```

### Run Tests

This project uses `ament_lint_auto` for lint testing (no unit tests). Run tests:

```bash
colcon test --packages-select ilqr_learning --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/ilqr_learning
```

## Code Style Guidelines

Follow Google C++ Style Guide + .clang-format configuration.

### File Structure

```
ilqr_learning/
├── CMakeLists.txt
├── package.xml
├── include/ilqr_learning/
│   ├── types.h, dynamics.h, cost.h, ilqr_solver.h
└── src/
    ├── dynamics.cpp, cost.cpp, ilqr_solver.cpp, ilqr_node.cpp
```

### Naming Conventions

- **Classes**: PascalCase (`Dynamics`, `ILQRSolver`, `Cost`)
- **Member Variables**: snake_case with trailing underscore (`dt_`, `L_`, `Q_`)
- **Constants**: k prefix + PascalCase (`kStateDim`, `kControlDim`, `kMaxIter`)
- **Functions**: PascalCase (`Step`, `Linearize`, `Solve`)
- **Files**: snake_case

### Include Order

1. Corresponding header (for .cpp)
2. C++ standard library (`<algorithm>`, `<iostream>`, etc.)
3. Third-party (`<Eigen/Dense>`)
4. Project headers (`"types.h"`)

### Header Guards & Access

Use `#pragma once`. Order access specifiers: `public`, `protected`, `private`.

### Error Handling

- ROS code: `RCLCPP_INFO`, `RCLCPP_ERROR`
- Utility code: `std::cerr`
- Validate early, return on errors

### Eigen Usage

Pass by const reference (`const Eigen::VectorXd&`). Use `.transpose()`, `.norm()`, `.diagonal()`, `.setZero()`, `.Identity()`.

### Namespace Usage

Use anonymous namespaces for file-local helpers:
```cpp
namespace {
double Clamp(double value, double min_val, double max_val) {
  return std::max(min_val, std::min(value, max_val));
}
}  // namespace
```

### ROS2 Guidelines

- Base on `rclcpp::Node`
- Use `declare_parameter`
- Initialize: `std::make_shared<MyNode>()`

## Project Architecture

### Core Components

- **Dynamics**: Bicycle model kinematics
- **Cost**: Quadratic cost (Q, R, Qf matrices)
- **ILQRSolver**: iLQR algorithm
- **ilqr_node**: ROS2 wrapper

### Dimensions

- State (kStateDim): 4 - [px, py, theta, v]
- Control (kControlDim): 2 - [acceleration, steering_angle]

### Key Parameters

- `dt`: 0.05s (default)
- `wheelbase`: 2.5m (default)
- `horizon`: 200 steps (default)

## Debugging

- Check `ilqr_results/ilqr_log.txt`
- Use `visualize_ilqr.py` to plot trajectories
- Adjust regularization in `ilqr_solver.cpp` for convergence issues
