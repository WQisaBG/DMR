/**
 * @file demo_drake_mujoco_cosim.cpp
 * @brief Drake and MuJoCo Co-Simulation with Circular Trajectory Planning
 *
 * @details
 * This program demonstrates advanced robot trajectory planning using Drake's
 * kinematics capabilities and MuJoCo's physics simulation with real-time visualization.
 *
 * Key Features:
 * - Dual-simulation architecture (Drake for planning, MuJoCo for visualization)
 * - Fixed-base robot configuration (base welded to world frame)
 * - Cartesian space circular trajectory planning in YZ plane
 * - Two-phase trajectory: approach + circle drawing
 * - Real-time collision detection with adjacent link filtering
 * - Advanced inverse kinematics with multiple initialization strategies
 * - Interactive 3D visualization with trajectory overlay
 *
 * @author Robot Grasp Team
 * @date 2025-12-29
 * @version 2.0
 */

// ============================================================================
// SECTION 1: Standard Library Includes
// ============================================================================
#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>
#include <optional>
#include <sstream>
#include <string>

// ============================================================================
// SECTION 2: Third-Party Library Includes
// ============================================================================

// MuJoCo Physics Engine
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>

// GLFW Window Management
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// OpenGL Visualization
#include <GL/gl.h>

// ============================================================================
// SECTION 3: Drake Robotics Library Includes
// ============================================================================
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_set.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/query_object.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/global_inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/math/rigid_transform.h>

// ============================================================================
// SECTION 4: Eigen Linear Algebra
// ============================================================================
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

// ============================================================================
// SECTION 5: Constants and Configuration
// ============================================================================

// Simulation Parameters
namespace SimConfig {
    constexpr double DEFAULT_TIME_STEP = 0.001;        // 1ms physics timestep
    constexpr double DEFAULT_DURATION = 5.0;           // Default trajectory duration (s)
    constexpr double APPROACH_DURATION = 1.5;          // Approach phase duration (s)
    constexpr double MIN_CIRCLE_DURATION = 3.0;        // Minimum circle drawing time (s)

    // Visualization Parameters
    constexpr int WINDOW_WIDTH = 1200;
    constexpr int WINDOW_HEIGHT = 900;
    constexpr int FONT_SCALE = 100;
    constexpr int MAX_GEOMETRY = 1000;

    // Trajectory Planning Parameters
    constexpr double CIRCLE_RADIUS = 0.06;            // 6cm circle radius (m)
    constexpr int LINEAR_WAYPOINTS = 41;              // Number of waypoints for linear trajectory
    constexpr int CIRCULAR_WAYPOINTS = 81;             // Number of waypoints for circle trajectory
    constexpr double TRAJECTORY_SAMPLE_INTERVAL = 0.01; // Sample every 10ms

    // IK Solver Parameters
    constexpr int IK_MAX_ATTEMPTS = 12;               // Number of random initializations
    constexpr double IK_POSITION_TOLERANCE = 0.0001; // Position tolerance (m) = 0.1mm
    constexpr double IK_ORIENTATION_TOLERANCE = 5.0 * M_PI / 180.0; // 5 degrees
    constexpr double IK_SUCCESS_THRESHOLD = 0.001;   // 1mm position error threshold

    // Collision Detection Parameters
    constexpr double COLLISION_PENETRATION_THRESHOLD = 0.001;  // 1mm penetration threshold
    constexpr double COLLISION_WARNING_DISTANCE = 0.01;       // 1cm warning distance
    constexpr double COLLISION_INFO_DISTANCE = 0.05;          // 5cm info distance
    constexpr double COLLISION_ADJACENT_LINK_THRESHOLD = 1.0; // Body index difference for adjacent links

    // Joint Limits (right arm joints 11-17)
    constexpr int RIGHT_ARM_JOINT_START = 11;
    constexpr int RIGHT_ARM_JOINT_END = 17;

    // Robot Configuration
    namespace InitialJointAngles {
        constexpr double LEG1 = 0.0;
        constexpr double LEG2 = 0.0;
        constexpr double LEG3 = 0.0;
        constexpr double WAIST = 0.0;

        // Left arm (neutral)
        constexpr double LEFT_ARM1 = 0.0;
        constexpr double LEFT_ARM2 = 0.0;
        constexpr double LEFT_ARM3 = 0.0;
        constexpr double LEFT_ARM4 = 0.0;
        constexpr double LEFT_ARM5 = 0.0;
        constexpr double LEFT_ARM6 = 0.0;
        constexpr double LEFT_ARM7 = 0.0;

        // Right arm (slightly offset from zero to avoid singularities)
        constexpr double RIGHT_ARM1 = 0.05;   // Shoulder pan
        constexpr double RIGHT_ARM2 = 0.0;    // Shoulder lift
        constexpr double RIGHT_ARM3 = 0.05;   // Elbow
        constexpr double RIGHT_ARM4 = 0.2;    // Wrist rotation
        constexpr double RIGHT_ARM5 = 0.1;    // Wrist flex
        constexpr double RIGHT_ARM6 = 0.05;   // Wrist rotation
        constexpr double RIGHT_ARM7 = 0.05;   // Wrist flex

        // Head
        constexpr double HEAD1 = 0.0;
        constexpr double HEAD2 = 0.0;
    }
}

// File paths
namespace FilePath {
    const std::string PROJECT_DIR = "/home/abc/RobotGrasp/DMR";
    const std::string URDF_PATH = PROJECT_DIR + "/model/nezha/urdf/robot_arm.urdf";
    const std::string MUJOCO_SCENE_PATH = PROJECT_DIR + "/model/nezha/scene/scene.xml";
}

// ============================================================================
// SECTION 6: Utility Structures and Classes
// ============================================================================

/**
 * @brief Simulation state container
 */
struct SimulationState
{
    VectorXd q;  // Generalized positions
    VectorXd v;  // Generalized velocities
    double t;    // Simulation time

    SimulationState(int num_positions = 0, int num_velocities = 0)
        : q(num_positions), v(num_velocities), t(0.0)
    {
        q.setZero();
        v.setZero();
    }
};

/**
 * @brief Quintic polynomial trajectory for smooth motion
 *
 * @details Provides smooth trajectory generation with zero velocity
 * and acceleration at boundaries using quintic polynomials.
 */
struct QuinticTrajectory
{
    double a0, a1, a2, a3, a4, a5;

    /**
     * @brief Compute quintic polynomial coefficients
     * @param q0 Initial position
     * @param qf Final position
     * @param t_duration Duration of motion
     * @return QuinticTrajectory with computed coefficients
     */
    static QuinticTrajectory Compute(double q0, double qf, double t_duration);

    /**
     * @brief Evaluate position at time t
     * @param t Time point
     * @return Position
     */
    double pos(double t) const;

    /**
     * @brief Evaluate velocity at time t
     * @param t Time point
     * @return Velocity
     */
    double vel(double t) const;
};

// ============================================================================
// SECTION 7: Forward Declarations
// ============================================================================

// GLFW Callback Functions
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

// ============================================================================
// SECTION 8: MuJoCo Simulator Class
// ============================================================================

/**
 * @class MuJoCoSimulator
 * @brief MuJoCo physics simulator with visualization
 *
 * @details Handles MuJoCo model loading, physics simulation, and real-time
 * rendering with OpenGL visualization. Supports interactive camera control
 * and trajectory overlay.
 */
class MuJoCoSimulator
{
public:
    /**
     * @brief Constructor
     * @param model_path Path to MuJoCo XML model file
     * @param enable_visualization Enable rendering window
     */
    explicit MuJoCoSimulator(const std::string& model_path, bool enable_visualization = true);

    /**
     * @brief Destructor - cleans up resources
     */
    ~MuJoCoSimulator();

    // Core Simulation Methods
    void reset();
    void set_state(const VectorXd& q, const VectorXd& v);
    void get_state(VectorXd& q, VectorXd& v) const;
    void step(double dt);

    // Visualization Methods
    void render(const std::vector<float>& traj_points = std::vector<float>());
    bool should_close() const;

    // Query Methods
    double get_time() const;
    int get_num_dofs() const;
    int get_num_positions() const;

    // End-Effector Position Queries
    Eigen::Vector3d get_ee_position_world() const;  ///< World frame
    Eigen::Vector3d get_ee_position() const;          ///< Waist frame

    // Debug Methods
    void print_state() const;

private:
    void init_visualization();
    void cleanup_visualization();

    // MuJoCo Model and Data
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    // Visualization Components
    GLFWwindow* window_ = nullptr;
    mjvScene scene_;
    mjvCamera camera_;
    mjvOption opt_;
    mjvPerturb pert_;
    mjrContext context_;

    // Interaction State
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double last_mouse_x_ = 0.0;
    double last_mouse_y_ = 0.0;
    bool visualization_enabled_ = false;
    bool should_close_ = false;
    const int font_scale_ = SimConfig::FONT_SCALE;

    // Allow callbacks to access private members
    friend void cursor_position_callback(GLFWwindow*, double, double);
    friend void mouse_button_callback(GLFWwindow*, int, int, int);
    friend void scroll_callback(GLFWwindow*, double, double);
    friend void key_callback(GLFWwindow*, int, int, int);
};

// ============================================================================
// SECTION 9: Drake Simulator Class
// ============================================================================

/**
 * @class DrakeSimulator
 * @brief Drake kinematics and trajectory planning simulator
 *
 * @details Provides robot model loading, forward kinematics, inverse
 * kinematics, trajectory planning, and collision detection capabilities.
 */
class DrakeSimulator
{
public:
    /**
     * @brief Constructor
     * @param urdf_path Path to robot URDF file
     * @param time_step Simulation timestep
     */
    explicit DrakeSimulator(const std::string& urdf_path, double time_step = SimConfig::DEFAULT_TIME_STEP);

    // State Management
    void reset();
    void set_state(const VectorXd& q, const VectorXd& v);
    void get_state(VectorXd& q, VectorXd& v) const;
    void step(double dt);

    // Query Methods
    double get_time() const;
    int get_num_positions() const;
    int get_num_velocities() const;
    void print_state() const;

    // Forward Kinematics
    drake::math::RigidTransformd ComputeEEPose(const VectorXd& q);

    // Inverse Kinematics
    bool SolveIK(const drake::math::RigidTransformd& desired_pose,
                 VectorXd& q_solution,
                 const VectorXd& q_guess,
                 bool debug = false);

    // Trajectory Planning
    drake::trajectories::PiecewisePolynomial<double> PlanCartesianCircle(
        double duration,
        const VectorXd& q_start,
        double radius,
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& normal);

    drake::trajectories::PiecewisePolynomial<double> PlanCartesianLine(
        double duration,
        const VectorXd& q_start,
        const Eigen::Vector3d& goal_position);

    drake::trajectories::PiecewisePolynomial<double> ConcatenateTrajectories(
        const drake::trajectories::PiecewisePolynomial<double>& traj1,
        const drake::trajectories::PiecewisePolynomial<double>& traj2,
        double traj1_duration);

    VectorXd eval_trajectory(const drake::trajectories::PiecewisePolynomial<double>& trajectory, double t);

    // Collision Detection
    struct CollisionResult
    {
        bool has_collision = false;
        double min_distance = std::numeric_limits<double>::infinity();
        std::vector<std::string> colliding_pairs;
        std::string warning_message;

        bool IsSafe() const { return !has_collision && min_distance > 0.0; }
    };

    CollisionResult CheckCollisionDetailed(const VectorXd& q);
    bool CheckCollision(const VectorXd& q);
    double GetMinimumCollisionDistance(const VectorXd& q);
    bool CheckCollisionWithMargin(const VectorXd& q, double safety_margin);
    void PrintCollisionReport(const VectorXd& q);

    drake::multibody::MultibodyPlant<double>* get_plant() { return plant_; }

private:
    // IK Solver Helper Methods
    std::optional<VectorXd> SolveGlobalIK(
        const drake::math::RigidTransformd& desired_pose,
        const VectorXd& q_guess,
        bool debug);

    VectorXd GenerateRandomGuess(const VectorXd& q_guess);

    // Drake Components
    std::unique_ptr<drake::systems::Diagram<double>> diagram_;
    drake::multibody::MultibodyPlant<double>* plant_ = nullptr;
    drake::geometry::SceneGraph<double>* scene_graph_ = nullptr;
    std::unique_ptr<drake::systems::Simulator<double>> simulator_;
};

// ============================================================================
// SECTION 10: Implementation - Quintic Trajectory
// ============================================================================

QuinticTrajectory QuinticTrajectory::Compute(double q0, double qf, double t_duration)
{
    QuinticTrajectory traj;
    double T = t_duration;

    // Quintic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Boundary conditions: zero velocity and acceleration at start and end
    traj.a0 = q0;
    traj.a1 = 0.0;
    traj.a2 = 0.0;
    traj.a3 = 10.0 * (qf - q0) / std::pow(T, 3);
    traj.a4 = -15.0 * (qf - q0) / std::pow(T, 4);
    traj.a5 = 6.0 * (qf - q0) / std::pow(T, 5);

    return traj;
}

double QuinticTrajectory::pos(double t) const
{
    return a0 + a1 * t + a2 * t * t + a3 * std::pow(t, 3) +
           a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
}

double QuinticTrajectory::vel(double t) const
{
    return a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2) +
           4 * a4 * std::pow(t, 3) + 5 * a5 * std::pow(t, 4);
}

// ============================================================================
// SECTION 11: Legacy Function (Deprecated - kept for compatibility)
// ============================================================================

/**
 * @deprecated Use DrakeSimulator::PlanCartesianCircle instead
 * @brief Legacy joint space circular trajectory planning
 */
void plan_circular_trajectory(double t, double duration,
                              const VectorXd& q_center, double radius,
                              VectorXd& q, VectorXd& v)
{
    double tau = std::min(t / duration, 1.0);
    double theta = tau * 2.0 * M_PI;
    double omega = 2.0 * M_PI / duration;

    // Apply circular motion to joints (legacy 6-DOF interface)
    if (q.size() > 0) { q(0) = q_center(0) + radius * std::sin(theta); v(0) = radius * omega * std::cos(theta); }
    if (q.size() > 1) { q(1) = q_center(1) + radius * std::cos(theta); v(1) = -radius * omega * std::sin(theta); }
    if (q.size() > 2) { q(2) = q_center(2) + 0.3 * radius * std::sin(theta + M_PI / 4); v(2) = 0.3 * radius * omega * std::cos(theta + M_PI / 4); }
    if (q.size() > 3) { q(3) = q_center(3) + 0.1 * radius * std::cos(2 * theta); v(3) = -0.2 * radius * omega * std::sin(2 * theta); }
    if (q.size() > 4) { q(4) = q_center(4) + 0.05 * radius * std::sin(theta + M_PI / 2); v(4) = 0.05 * radius * omega * std::cos(theta + M_PI / 2); }
    if (q.size() > 5) { q(5) = q_center(5) + 0.02 * radius * std::cos(3 * theta); v(5) = -0.06 * radius * omega * std::sin(3 * theta); }
}

// ============================================================================
// SECTION 12: MuJoCo Simulator Implementation
// ============================================================================

MuJoCoSimulator::MuJoCoSimulator(const std::string& model_path, bool enable_visualization)
{
    // Load MuJoCo model
    char error[1000];
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!model_) {
        throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error));
    }

    data_ = mj_makeData(model_);

    if (enable_visualization) {
        init_visualization();
    }

    // Print model information
    std::cout << "MuJoCo Model loaded successfully!" << std::endl;
    std::cout << "  - DOFs (nv): " << model_->nv << std::endl;
    std::cout << "  - Positions (nq): " << model_->nq << std::endl;
    std::cout << "  - Bodies: " << model_->nbody << std::endl;
    std::cout << "  - Joints: " << model_->njnt << std::endl;

    // Print joint mapping
    std::cout << "\n  Joint qpos mapping:" << std::endl;
    for (int i = 0; i < model_->njnt; ++i) {
        int qpos_offset = model_->jnt_qposadr[i];
        const char* jnt_name = mj_id2name(model_, mjOBJ_JOINT, i);
        std::cout << "    joint[" << i << "] = " << (jnt_name ? jnt_name : "unknown")
                  << " -> qpos[" << qpos_offset << "]" << std::endl;
    }

    // Print important body IDs
    std::cout << "\n  Important body IDs:" << std::endl;
    std::cout << "    right_arm_link7 ID: " << mj_name2id(model_, mjOBJ_BODY, "right_arm_link7") << std::endl;
    std::cout << "    left_arm_link7 ID: " << mj_name2id(model_, mjOBJ_BODY, "left_arm_link7") << std::endl;
    std::cout << "    ee_site ID: " << mj_name2id(model_, mjOBJ_SITE, "ee_site") << std::endl;
}

MuJoCoSimulator::~MuJoCoSimulator()
{
    cleanup_visualization();
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

void MuJoCoSimulator::init_visualization()
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    window_ = glfwCreateWindow(SimConfig::WINDOW_WIDTH, SimConfig::WINDOW_HEIGHT,
                               "MuJoCo Circular Trajectory Demo", NULL, NULL);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window_, this);
    glfwSetCursorPosCallback(window_, cursor_position_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glfwSetKeyCallback(window_, key_callback);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);
    mjv_makeScene(model_, &scene_, SimConfig::MAX_GEOMETRY);
    mjr_defaultContext(&context_);
    mjr_makeContext(model_, &context_, font_scale_);

    // Set initial camera position
    camera_.distance = 2.0;
    camera_.lookat[0] = 0.5;
    camera_.lookat[1] = 0.0;
    camera_.lookat[2] = 0.5;

    visualization_enabled_ = true;
    std::cout << "  - Visualization: enabled" << std::endl;
}

void MuJoCoSimulator::cleanup_visualization()
{
    if (visualization_enabled_) {
        mjr_freeContext(&context_);
        mjv_freeScene(&scene_);
    }
}

void MuJoCoSimulator::render(const std::vector<float>& traj_points)
{
    if (!visualization_enabled_ || !window_) return;

    mjv_updateScene(model_, data_, &opt_, &pert_, &camera_, mjCAT_ALL, &scene_);

    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjr_render(viewport, &scene_, &context_);

    // Draw trajectory overlay
    if (!traj_points.empty() && traj_points.size() >= 6) {
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);  // Bright red
        glLineWidth(5.0f);

        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < traj_points.size(); i += 3) {
            if (i + 2 < traj_points.size()) {
                glVertex3f(traj_points[i], traj_points[i + 1], traj_points[i + 2]);
            }
        }
        glEnd();

        glPointSize(8.0f);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < traj_points.size(); i += 3) {
            if (i + 2 < traj_points.size()) {
                glVertex3f(traj_points[i], traj_points[i + 1], traj_points[i + 2]);
            }
        }
        glEnd();

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
    }

    glfwSwapBuffers(window_);
    glfwPollEvents();

    if (glfwWindowShouldClose(window_)) {
        should_close_ = true;
    }
}

bool MuJoCoSimulator::should_close() const
{
    return should_close_ || (window_ && glfwWindowShouldClose(window_));
}

void MuJoCoSimulator::reset()
{
    mj_resetData(model_, data_);
    data_->time = 0.0;
}

void MuJoCoSimulator::set_state(const VectorXd& q, const VectorXd& v)
{
    int num_joints = std::min(static_cast<int>(q.size()), 20);
    for (int i = 0; i < num_joints; ++i) {
        data_->qpos[i] = q(i);
    }

    int num_vels = std::min(static_cast<int>(v.size()), 20);
    for (int i = 0; i < num_vels; ++i) {
        data_->qvel[i] = v(i);
    }
}

void MuJoCoSimulator::get_state(VectorXd& q, VectorXd& v) const
{
    q = Eigen::Map<const VectorXd>(data_->qpos, model_->nq);
    v = Eigen::Map<const VectorXd>(data_->qvel, model_->nv);
}

void MuJoCoSimulator::step(double dt)
{
    mj_step(model_, data_);
}

double MuJoCoSimulator::get_time() const { return data_->time; }
int MuJoCoSimulator::get_num_dofs() const { return model_->nv; }
int MuJoCoSimulator::get_num_positions() const { return model_->nq; }

Eigen::Vector3d MuJoCoSimulator::get_ee_position_world() const
{
    int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "ee_site");

    if (ee_site_id >= 0) {
        return Eigen::Vector3d(
            data_->site_xpos[ee_site_id * 3 + 0],
            data_->site_xpos[ee_site_id * 3 + 1],
            data_->site_xpos[ee_site_id * 3 + 2]);
    }

    int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
    if (ee_body_id >= 0) {
        return Eigen::Vector3d(
            data_->xpos[ee_body_id * 3 + 0],
            data_->xpos[ee_body_id * 3 + 1],
            data_->xpos[ee_body_id * 3 + 2]);
    }

    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d MuJoCoSimulator::get_ee_position() const
{
    int waist_body_id = mj_name2id(model_, mjOBJ_BODY, "waist_link");

    Eigen::Vector3d waist_pos(
        data_->xpos[waist_body_id * 3 + 0],
        data_->xpos[waist_body_id * 3 + 1],
        data_->xpos[waist_body_id * 3 + 2]);

    Eigen::Matrix3d waist_rot;
    waist_rot <<
        data_->xmat[waist_body_id * 9 + 0], data_->xmat[waist_body_id * 9 + 1], data_->xmat[waist_body_id * 9 + 2],
        data_->xmat[waist_body_id * 9 + 3], data_->xmat[waist_body_id * 9 + 4], data_->xmat[waist_body_id * 9 + 5],
        data_->xmat[waist_body_id * 9 + 6], data_->xmat[waist_body_id * 9 + 7], data_->xmat[waist_body_id * 9 + 8];

    int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "ee_site");

    if (ee_site_id >= 0) {
        Eigen::Vector3d ee_world_pos(
            data_->site_xpos[ee_site_id * 3 + 0],
            data_->site_xpos[ee_site_id * 3 + 1],
            data_->site_xpos[ee_site_id * 3 + 2]);

        return waist_rot.transpose() * (ee_world_pos - waist_pos);
    }

    return Eigen::Vector3d::Zero();
}

void MuJoCoSimulator::print_state() const
{
    std::cout << "MuJoCo State:" << std::endl;
    std::cout << "  Time: " << data_->time << std::endl;
    std::cout << "  Positions: " << Eigen::Map<const VectorXd>(data_->qpos, model_->nq).transpose().head(6) << "..." << std::endl;
}

// ============================================================================
// SECTION 13: GLFW Callback Implementations
// ============================================================================

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    double dx = xpos - sim->last_mouse_x_;
    double dy = ypos - sim->last_mouse_y_;

    sim->last_mouse_x_ = xpos;
    sim->last_mouse_y_ = ypos;

    if (sim->button_left_) {
        sim->camera_.azimuth += dx * 0.5;
        sim->camera_.elevation += dy * 0.5;
    } else if (sim->button_middle_) {
        sim->camera_.lookat[0] -= dx * 0.01;
        sim->camera_.lookat[1] += dy * 0.01;
    } else if (sim->button_right_) {
        sim->camera_.distance += dy * 0.01;
        if (sim->camera_.distance < 0.1) sim->camera_.distance = 0.1;
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    if (action == GLFW_PRESS) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            sim->button_left_ = true;
            sim->button_middle_ = false;
            sim->button_right_ = false;
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            sim->button_right_ = true;
        }
    } else if (action == GLFW_RELEASE) {
        sim->button_left_ = false;
        sim->button_middle_ = false;
        sim->button_right_ = false;
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (sim) {
        sim->camera_.distance -= yoffset * 0.05;
        if (sim->camera_.distance < 0.1) sim->camera_.distance = 0.1;
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    MuJoCoSimulator* sim = static_cast<MuJoCoSimulator*>(glfwGetWindowUserPointer(window));
    if (!sim) return;

    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

// Note: Due to length constraints, the Drake Simulator implementation and main function
// will continue in the next part. The structure established here provides:
// 1. Clear file organization with numbered sections
// 2. Comprehensive constants in named namespaces
// 3. Well-documented classes with Doxygen comments
// 4. Consistent code style and formatting
// 5. Logical separation of concerns
