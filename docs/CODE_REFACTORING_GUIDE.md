# Demo Code Refactoring Guide

## Overview
This document provides a comprehensive guide for refactoring `demo_drake_mujoco_cosim.cpp` to improve code quality, maintainability, and performance.

## Current State Analysis

### File Statistics
- **Lines of Code**: 2,202
- **Classes**: 2 (MuJoCoSimulator, DrakeSimulator)
- **Functions**: 20+ (including callbacks and helpers)
- **Key Issues**:
  - Monolithic structure (all code in one file)
  - Magic numbers scattered throughout
  - Inconsistent naming conventions
  - Insufficient error handling
  - Mixed concerns (physics, visualization, planning)

## Refactoring Strategy

### Phase 1: Immediate Improvements (Low Hanging Fruit)

#### 1.1 Extract Constants
**Current**: Magic numbers scattered throughout code
```cpp
// BAD: Magic numbers
double radius = 0.06;
int num_waypoints = 81;
double dt = duration / (num_waypoints - 1);
```

**Proposed**: Named constants in namespace
```cpp
// GOOD: Named constants
namespace TrajectoryConfig {
    constexpr double CIRCLE_RADIUS = 0.06;      // 6cm in meters
    constexpr int CIRCULAR_WAYPOINTS = 81;       // Number of waypoints
    constexpr int LINEAR_WAYPOINTS = 41;
    constexpr double SAMPLE_INTERVAL = 0.01;     // 10ms
}
```

#### 1.2 Improve File Organization
Add clear section markers:
```cpp
// ============================================================================
// SECTION 1: Includes
// ============================================================================
// [Standard library, third-party, project includes]

// ============================================================================
// SECTION 2: Constants and Configuration
// ============================================================================
namespace Config { ... }

// ============================================================================
// SECTION 3: Type Definitions
// ============================================================================
struct SimulationState { ... };

// ============================================================================
// SECTION 4: MuJoCo Simulator Class
// ============================================================================
class MuJoCoSimulator { ... };

// ============================================================================
// SECTION 5: Drake Simulator Class
// ============================================================================
class DrakeSimulator { ... };

// ============================================================================
// SECTION 6: Callback Functions
// ============================================================================
void glfw_callbacks() { ... }

// ============================================================================
// SECTION 7: Main Function
// ============================================================================
int main() { ... }
```

#### 1.3 Enhance Documentation
**Current**: Basic Doxygen comments
**Proposed**: Comprehensive documentation with:
- File purpose and usage examples
- Algorithm descriptions
- Parameter constraints
- Return value meanings
- Thread-safety notes
- Performance considerations

```cpp
/**
 * @file demo_drake_mujoco_cosim.cpp
 * @brief Drake-MuJoCo Co-Simulation for Circular Trajectory Planning
 *
 * @section features Key Features
 * - Fixed-base robot configuration (welded to world frame)
 * - Two-phase trajectory planning: approach → circle drawing
 * - Real-time collision detection with adjacent link filtering
 * - Advanced IK with multiple random initializations
 * - Interactive 3D visualization with OpenGL overlay
 *
 * @section usage Usage
 * ./demo_drake_mujoco_cosim [circular|line] [duration] [timestep]
 *
 * @section implementation Implementation Details
 * - Phase 1: Linear approach trajectory (1.5s)
 * - Phase 2: Circular trajectory in YZ plane (3.5s)
 * - Collision detection enabled throughout
 * - IK tolerance: 1mm position, 5° orientation
 *
 * @author Robot Grasp Team
 * @version 2.0
 * @date 2025-12-29
 */
```

### Phase 2: Code Structure Improvements

#### 2.1 Split into Multiple Files
**Recommended structure**:
```
demo/
├── include/
│   ├── mujoco_simulator.hpp          # MuJoCoSimulator class
│   ├── drake_simulator.hpp           # DrakeSimulator class
│   ├── collision_detector.hpp        # Collision detection utilities
│   ├── ik_solver.hpp                 # IK solver interfaces
│   └── trajectory_planner.hpp        # Trajectory planning algorithms
├── src/
│   ├── mujoco_simulator.cpp
│   ├── drake_simulator.cpp
│   ├── collision_detector.cpp
│   ├── ik_solver.cpp
│   ├── trajectory_planner.cpp
│   └── main.cpp                       # Main program logic
└── CMakeLists.txt
```

#### 2.2 Extract Collision Detection
Create dedicated collision detection module:

```cpp
// collision_detector.hpp
class CollisionDetector {
public:
    struct Result {
        bool has_collision;
        double min_distance;
        std::string message;
        bool IsSafe() const;
    };

    static Result CheckDetailed(
        const drake::multibody::MultibodyPlant<double>& plant,
        const drake::systems::Context<double>& context,
        const VectorXd& q);

    static bool IsAdjacentBodies(
        const drake::multibody::MultibodyPlant<double>& plant,
        drake::geometry::GeometryId id1,
        drake::geometry::GeometryId id2);
};
```

#### 2.3 Extract IK Solver
Create dedicated IK solver module:

```cpp
// ik_solver.hpp
class IKSolver {
public:
    struct Options {
        int max_attempts = 12;
        double position_tolerance = 0.0001;  // 0.1mm
        double orientation_tolerance = 5.0 * M_PI / 180.0;  // 5 degrees
        double success_threshold = 0.001;    // 1mm
    };

    static std::optional<VectorXd> SolveGlobalIK(
        const drake::multibody::MultibodyPlant<double>& plant,
        const drake::multibody::Frame<double>& ee_frame,
        const drake::multibody::Frame<double>& reference_frame,
        const drake::math::RigidTransformd& desired_pose,
        const VectorXd& q_guess,
        const Options& options = Options());
};
```

### Phase 3: Code Quality Improvements

#### 3.1 Add RAII Wrappers
```cpp
// RAII wrapper for GLFW
class GLFWInitializer {
public:
    GLFWInitializer() { glfwInit(); }
    ~GLFWInitializer() { glfwTerminate(); }
};

// RAII wrapper for MuJoCo
class MuJoCoModel {
public:
    explicit MuJoCoModel(const std::string& xml_path);
    ~MuJoCoModel();
    mjModel* get() { return model_; }
    mjData* create_data();

private:
    mjModel* model_ = nullptr;
};
```

#### 3.2 Improve Error Handling
```cpp
// Custom exception types
class SimulationException : public std::runtime_error {
public:
    explicit SimulationException(const std::string& msg)
        : std::runtime_error(msg) {}
};

class IKException : public SimulationException {
public:
    explicit IKException(const std::string& msg, int attempt)
        : SimulationException(msg + " (attempt " + std::to_string(attempt) + ")") {}
};

// Usage in code
void SomeFunction() {
    if (error_condition) {
        throw SimulationException("Failed to load model: " + error_msg);
    }
}
```

#### 3.3 Add Logging System
```cpp
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Logger {
public:
    template<typename... Args>
    static void log(LogLevel level, const char* fmt, Args... args) {
        if (level >= min_level_) {
            printf("[%s] ", levelToString(level));
            printf(fmt, args...);
            printf("\n");
        }
    }

private:
    static LogLevel min_level_;
    static const char* levelToString(LogLevel level);
};

// Usage
Logger::log(LogLevel::INFO, "IK succeeded with error: %.3f mm", error_mm);
```

### Phase 4: Performance Optimizations

#### 4.1 Reduce IK Solver Attempts
**Current**: Always 12 attempts
**Optimized**: Adaptive attempts based on success rate

```cpp
int AdaptiveMaxAttempts(int consecutive_failures) {
    if (consecutive_failures < 3) return 3;      // Fast path
    if (consecutive_failures < 10) return 6;     // Medium
    return 12;                                    // Thorough search
}
```

#### 4.2 Cache Forward Kinematics Results
```cpp
class FKBucket {
public:
    void clear() { cache_.clear(); }

    Eigen::Vector3d get_or_compute(const VectorXd& q) {
        auto key = hash_vector(q);
        if (cache_.find(key) != cache_.end()) {
            return cache_[key];
        }
        auto result = compute_fk(q);
        cache_[key] = result;
        return result;
    }

private:
    std::unordered_map<size_t, Eigen::Vector3d> cache_;
};
```

#### 4.3 Parallel Trajectory Planning
```cpp
// Use OpenMP to parallelize waypoint IK solving
#pragma omp parallel for schedule(dynamic)
for (int i = 0; i < num_waypoints; ++i) {
    auto result = SolveIK(waypoints[i]);
    joint_samples[i] = result;
}
```

### Phase 5: Testing Improvements

#### 5.1 Add Unit Tests
```cpp
// test/demo_test.cpp
#include <gtest/gtest.h>

TEST(DrakeSimulator, ForwardKinematics) {
    DrakeSimulator sim(URDF_PATH);
    VectorXd q = VectorXd::Zero(20);
    auto pose = sim.ComputeEEPose(q);

    EXPECT_NEAR(pose.translation().norm(), 0.3, 0.1);  // Approximate reach
}

TEST(CollisionDetector, AdjacentLinks) {
    // Test adjacent link filtering
    EXPECT_TRUE(CollisionDetector::IsAdjacentBodies(plant, id1, id2));
}
```

#### 5.2 Add Integration Tests
```cpp
TEST(Integration, FullTrajectoryPlanning) {
    // Test complete pipeline
    auto trajectory = PlanFullTrajectory();
    EXPECT_GT(trajectory.get_number_of_segments(), 0);
}
```

## Recommended Refactoring Order

### Priority 1: Do Immediately (1-2 hours)
1. ✅ Extract all magic numbers to constants namespace
2. ✅ Add file header with comprehensive documentation
3. ✅ Add section divider comments
4. ✅ Improve function documentation (Doxygen format)

### Priority 2: Short Term (1 week)
1. Extract collision detection to separate module
2. Extract IK solver to separate module
3. Add proper error handling with exceptions
4. Add logging system
5. Create separate header/implementation files

### Priority 3: Medium Term (2-4 weeks)
1. Complete file separation (as shown in Phase 2.1)
2. Add RAII wrappers for resource management
3. Implement performance optimizations
4. Add comprehensive unit tests
5. Add integration tests

### Priority 4: Long Term (1-2 months)
1. Consider architecture redesign (pattern: Model-View-Controller)
2. Add configuration file support (YAML/JSON)
3. Implement trajectory replay system
4. Add profiling and benchmarking
5. Consider moving to ROS 2 architecture

## Code Style Guidelines

### Naming Conventions
```cpp
// Namespaces: lower_case
namespace trajectory_config { ... }

// Classes: PascalCase
class DrakeSimulator { ... };

// Methods: PascalCase
void ComputeEEPose() { ... }

// Variables: snake_case
double circle_radius;
int num_waypoints;

// Constants: UPPER_SNAKE_CASE
namespace Config {
    constexpr double MAX_ITERATIONS = 100;
}

// Private members: trailing_underscore
class Foo {
private:
    int value_;
    std::string name_;
};
```

### Formatting Guidelines
- Use 4 spaces for indentation (no tabs)
- Line length: 100 characters (soft limit), 120 (hard limit)
- Opening brace: same line for functions/classes, new line for control flow
```cpp
void Function() {  // Same line
    if (condition) {  // Same line
        // code
    }
}
```

## Metrics for Success

### Code Quality Metrics
- **Cyclomatic Complexity**: < 10 per function
- **Function Length**: < 50 lines (ideal), < 100 (acceptable)
- **File Length**: < 500 lines (ideal)
- **Test Coverage**: > 80%

### Performance Metrics
- **Trajectory Planning Time**: < 5 seconds for 81 waypoints
- **IK Success Rate**: > 95%
- **Memory Usage**: < 500 MB
- **Startup Time**: < 2 seconds

## Migration Checklist

- [ ] Create new file structure
- [ ] Move MuJoCoSimulator to separate files
- [ ] Move DrakeSimulator to separate files
- [ ] Extract collision detection module
- [ ] Extract IK solver module
- [ ] Extract trajectory planner module
- [ ] Update CMakeLists.txt
- [ ] Add unit tests
- [ ] Update documentation
- [ ] Performance testing
- [ ] Code review
- [ ] Update user manual

## Resources

### Books
- "Clean Code" by Robert C. Martin
- "Effective C++" by Scott Meyers
- "Game Engine Architecture" by Jason Gregory

### Tools
- **Clang-Tidy**: Static analysis
- **Clang-Format**: Code formatting
- **CppCheck**: Additional static analysis
- **Google Test**: Unit testing framework
- **Benchmark**: Performance testing

### C++ Standards
- C++ Core Guidelines: https://isocpp.github.io/CppCoreGuidelines/
- MISRA C++ (for safety-critical applications)

---

**Document Version**: 1.0
**Last Updated**: 2025-12-29
**Author**: Robot Grasp Team
**Status**: Draft - Ready for Review
