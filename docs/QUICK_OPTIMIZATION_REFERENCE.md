# Quick Optimization Reference

## Immediate Improvements You Can Apply Now

### 1. Replace Magic Numbers (Top 20)

**Find these patterns and replace:**
```cpp
// OLD → NEW
0.001 → SimConfig::DEFAULT_TIME_STEP
5.0 → SimConfig::DEFAULT_DURATION
1.5 → SimConfig::APPROACH_DURATION
3.0 → SimConfig::MIN_CIRCLE_DURATION
0.06 → SimConfig::CIRCLE_RADIUS
41 → SimConfig::LINEAR_WAYPOINTS
81 → SimConfig::CIRCULAR_WAYPOINTS
12 → SimConfig::IK_MAX_ATTEMPTS
0.0001 → SimConfig::IK_POSITION_TOLERANCE
5.0 * M_PI / 180.0 → SimConfig::IK_ORIENTATION_TOLERANCE
0.001 → SimConfig::IK_SUCCESS_THRESHOLD
0.001 → SimConfig::COLLISION_PENETRATION_THRESHOLD
0.01 → SimConfig::COLLISION_WARNING_DISTANCE
0.05 → SimConfig::COLLISION_INFO_DISTANCE
1.0 → SimConfig::COLLISION_ADJACENT_LINK_THRESHOLD
11 → SimConfig::RIGHT_ARM_JOINT_START
17 → SimConfig::RIGHT_ARM_JOINT_END
1200 → SimConfig::WINDOW_WIDTH
900 → SimConfig::WINDOW_HEIGHT
```

### 2. Add Section Dividers

Insert at key locations:
```cpp
// ============================================================================
// SECTION NAME
// ============================================================================
```

Locations:
- After includes
- Before constants
- Before MuJoCoSimulator class
- Before DrakeSimulator class
- Before callback functions
- Before main()

### 3. Improve Function Documentation

**Before:**
```cpp
void render(const std::vector<float> &traj_points = std::vector<float>());
```

**After:**
```cpp
/**
 * @brief Render MuJoCo scene with trajectory overlay
 * @param traj_points Vector of (x,y,z) points for trajectory visualization
 *                    Format: [x1,y1,z1, x2,y2,z2, ...]
 * @note Disables depth test to draw trajectory on top
 * @note Must be called from OpenGL context thread
 */
void render(const std::vector<float>& traj_points = std::vector<float>());
```

### 4. Extract Common Patterns

#### Pattern A: Print Joint Angles
```cpp
// EXTRACT TO FUNCTION
inline void print_right_arm_joints(std::ostream& os, const VectorXd& q) {
    os << " q_desired[11:17]: [";
    for (int j = 11; j <= 17; ++j) {
        os << q(j);
        if (j < 17) os << " ";
    }
    os << "]";
}
```

#### Pattern B: Check IK Success
```cpp
// EXTRACT TO FUNCTION
inline bool verify_ik_solution(const Eigen::Vector3d& actual,
                               const Eigen::Vector3d& desired,
                               double threshold = SimConfig::IK_SUCCESS_THRESHOLD) {
    double error = (actual - desired).norm();
    return error < threshold;
}
```

#### Pattern C: Format Time Display
```cpp
// EXTRACT TO FUNCTION
inline std::string format_time(double seconds) {
    int mins = static_cast<int>(seconds) / 60;
    double secs = seconds - mins * 60;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%d:%06.3f", mins, secs);
    return std::string(buffer);
}
```

### 5. Improve Variable Names

**Before → After:**
```cpp
col_result → collision_result
ee_pos → ee_position
ee_start_waist → ee_position_waist
q_circle_start → joint_config_circle_start
approach_duration → approach_phase_duration
circle_duration → drawing_phase_duration
num_waypoints → waypoint_count
max_joint_change → max_joint_angle_change
```

### 6. Simplify Complex Conditionals

**Before:**
```cpp
if (i % 20 == 0) {
    // Only print every 20th waypoint to reduce spam
    std::cout << "  [INFO] Waypoint " << i << ": "
              << col_result.warning_message << std::endl;
}
```

**After:**
```cpp
const bool should_log = (i % 20 == 0);
if (should_log && !warning_msg.empty()) {
    log_info("Waypoint %d: %s", i, warning_msg.c_str());
}
```

### 7. Use Helper Functions for Repetitive Code

**Drake EE Position Display Pattern:**
```cpp
inline void print_ee_position(const std::string& label,
                             const Eigen::Vector3d& pos) {
    std::cout << label << ": " << pos.transpose() << " m" << std::endl;
}

// Usage:
print_ee_position("Drake EE start", ee_start);
print_ee_position("MuJoCo EE start", mujoco_ee_start);
print_ee_position("Difference", mujoco_ee_start - ee_start);
```

### 8. Add Const Correctness

```cpp
// Before:
void SomeFunction(VectorXd& q);

// After:
void SomeFunction(const VectorXd& q);          // Input only
void SomeFunction(VectorXd& q_output);         // Output
void SomeFunction(VectorXd& q_in_out);         // Both
```

### 9. Use Structured Bindings (C++17)

```cpp
// Before:
auto T_ee_start = drake_sim.ComputeEEPose(q_start);
Eigen::Vector3d ee_start = T_ee_start.translation();

// After:
auto [rotation, ee_start] = drake_sim.ComputeEEPose(q_start);
```

### 10. Add TODO Comments for Future Work

```cpp
// TODO: Add trajectory replay functionality
// TODO: Implement adaptive IK solver with caching
// TODO: Add support for multiple trajectory types
// FIXME: IPOPT warnings need investigation
// HACK: Workaround for Drake collision detection limitation
```

## Quick Wins Summary

| Improvement | Effort | Impact | Time |
|------------|--------|---------|------|
| Extract constants | Low | High | 30 min |
| Add section dividers | Low | Medium | 15 min |
| Improve documentation | Medium | High | 1 hour |
| Extract helper functions | Medium | High | 2 hours |
| Improve variable names | Low | Medium | 30 min |
| Add const correctness | Low | Low | 30 min |
| **Total** | | | **~5 hours** |

## Files to Create

1. **constants.hpp** - All configuration constants
2. **types.hpp** - Common type definitions
3. **utils.hpp/cpp** - Utility functions
4. **logging.hpp/cpp** - Logging system

## Build System Updates

Add to CMakeLists.txt:
```cmake
# Enable warnings
target_compile_options(demo_drake_mujoco_cosim PRIVATE
    -Wall -Wextra -Wpedantic)

# Enable sanitizers (debug mode)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(demo_drake_mujoco_cosim PRIVATE
        -fsanitize=address -fno-omit-frame-pointer)
    target_link_options(demo_drake_mujoco_cosim PRIVATE
        -fsanitize=address)
endif()
```

## Testing Quick Start

```bash
# Run with sanitizers
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
./demo_drake_mujoco_cosim circular 5.0 0.001

# Check for memory leaks
valgrind --leak-check=full ./demo_drake_mujoco_cosim circular 5.0 0.001
```

---

**Last Updated**: 2025-12-29
**Priority**: HIGH - Start with Priority 1 items
