#include <iostream>
#include <iomanip>
#include <fstream>
#include <memory>
#include <chrono>
#include <ctime>
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>
#include <optional>
#include <queue>
#include <map>
#include <sstream>

#if defined(__linux__)
#include <limits.h>     // PATH_MAX
#include <unistd.h>     // readlink
#include <sys/select.h> // select() for non-blocking stdin
#elif defined(__APPLE__)
#include <limits.h>      // PATH_MAX
#include <mach-o/dyld.h> // _NSGetExecutablePath
#include <stdlib.h>      // realpath
#include <sys/select.h>  // select() for non-blocking stdin
#endif

// MuJoCo headers
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <GL/gl.h>

// Drake headers
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_set.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/query_results/penetration_as_point_pair.h>
#include <drake/planning/trajectory_optimization/direct_collocation.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_quaternion.h>
#include <drake/common/trajectories/bspline_trajectory.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>
#include <drake/math/bspline_basis.h>
#include <drake/common/polynomial.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/planning/collision_checker.h>
#include <chrono>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/multibody/inverse_kinematics/global_inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/optimization/toppra.h>
#include <random>
#include <future>

// APF (势场+斥力场) + Informed RRT* + CCD 连续碰撞检测
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Forward declarations for GLFW callbacks
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

class MuJoCoSimulator
{
public:
    MuJoCoSimulator(const std::string &model_path, bool enable_visualization = true)
    {
        // Load MuJoCo model
        char error[1000];
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
        if (!model_)
        {
            throw std::runtime_error("Failed to load MuJoCo model: " + std::string(error));
        }

        // Create data structure
        data_ = mj_makeData(model_);

        // Initialize visualization if requested
        if (enable_visualization)
        {
            init_visualization();
        }

        std::cout << "MuJoCo Model loaded successfully!" << std::endl;
        std::cout << "  - DOFs (nv): " << model_->nv << std::endl;
        std::cout << "  - Positions (nq): " << model_->nq << std::endl;
        std::cout << "  - Bodies: " << model_->nbody << std::endl;
        std::cout << "  - Joints: " << model_->njnt << std::endl;
    }

    ~MuJoCoSimulator()
    {
        cleanup_visualization();
        if (data_)
            mj_deleteData(data_);
        if (model_)
            mj_deleteModel(model_);
        if (window_)
            glfwDestroyWindow(window_);
        glfwTerminate();
    }

    void init_visualization()
    {
        // Initialize GLFW
        if (!glfwInit())
        {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return;
        }

        // Create window
        window_ = glfwCreateWindow(1200, 900, "MuJoCo Circular Trajectory Demo", NULL, NULL);
        if (!window_)
        {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        // Set GLFW user pointer for callbacks
        glfwSetWindowUserPointer(window_, this);

        // Set GLFW callbacks for mouse and keyboard
        glfwSetCursorPosCallback(window_, cursor_position_callback);
        glfwSetMouseButtonCallback(window_, mouse_button_callback);
        glfwSetScrollCallback(window_, scroll_callback);
        glfwSetKeyCallback(window_, key_callback);

        // Initialize MuJoCo visualization
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&opt_);
        mjv_defaultPerturb(&pert_);

        // Create scene and context (maxgeom parameter added)
        mjv_makeScene(model_, &scene_, 1000);
        mjr_defaultContext(&context_);
        mjr_makeContext(model_, &context_, font_scale_);

        // Set camera position
        camera_.distance = 2.0;
        camera_.lookat[0] = 0.5;
        camera_.lookat[1] = 0.0;
        camera_.lookat[2] = 0.5;

        visualization_enabled_ = true;
    }

    void cleanup_visualization()
    {
        if (visualization_enabled_)
        {
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
        }
    }

    void render(const std::vector<float> &traj_points = std::vector<float>())
    {
        if (!visualization_enabled_ || !window_)
            return;

        // Update scene
        mjv_updateScene(model_, data_, &opt_, &pert_, &camera_, mjCAT_ALL, &scene_);

        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        // Render MuJoCo scene
        mjr_render(viewport, &scene_, &context_);

        // Draw trajectory on top (bright red line)
        if (!traj_points.empty() && traj_points.size() >= 6)
        {
            // Disable depth test AND depth write so trajectory shows on top
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);

            // IMPORTANT: Disable lighting and texture to ensure solid color rendering
            glDisable(GL_LIGHTING);
            glDisable(GL_TEXTURE_2D);

            // Force immediate mode color to override any material/texture settings
            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

            // Set line color to bright red (RGB: 1.0, 0.0, 0.0)
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

            // Set line width - thinner for better visibility
            glLineWidth(1.0f);

            // Draw lines between consecutive points
            glBegin(GL_LINE_STRIP);
            for (size_t i = 0; i < traj_points.size(); i += 3)
            {
                if (i + 2 < traj_points.size())
                {
                    glVertex3f(traj_points[i], traj_points[i + 1], traj_points[i + 2]);
                }
            }
            glEnd();

            // Draw points at each vertex - also bright red
            // TODO: 可视化的线宽
            glPointSize(1.0f);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < traj_points.size(); i += 3)
            {
                if (i + 2 < traj_points.size())
                {
                    glVertex3f(traj_points[i], traj_points[i + 1], traj_points[i + 2]);
                }
            }
            glEnd();

            glDisable(GL_COLOR_MATERIAL);
            glEnable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
        }

        // Swap buffers
        glfwSwapBuffers(window_);

        glfwPollEvents();

        // Check for window close
        if (glfwWindowShouldClose(window_))
        {
            should_close_ = true;
        }
    }

    bool should_close() const
    {
        return should_close_ || (window_ && glfwWindowShouldClose(window_));
    }

    void reset()
    {
        mj_resetData(model_, data_);
        data_->time = 0.0;
    }

    void set_state(const VectorXd &q, const VectorXd &v)
    {
        // RobotV3: 19 DOF (lumber_joint1(prismatic), lumber_joint2-3, head_joint1-2, left_arm_joint1-7, right_arm_joint1-7)
        // joints map to qpos[0-18]: lumber1[0], lumber[1-2], head[3-4], left_arm[5-11], right_arm[12-18]

        int num_joints = std::min(static_cast<int>(q.size()), model_->nq);
        for (int i = 0; i < num_joints; ++i)
        {
            data_->qpos[i] = q(i);
        }

        int num_vels = std::min(static_cast<int>(v.size()), model_->nv);
        for (int i = 0; i < num_vels; ++i)
        {
            data_->qvel[i] = v(i);
        }
    }

    // Set position actuator targets (ctrl) to match desired joint positions
    void set_ctrl(const VectorXd &q_desired)
    {
        int num_actuators = std::min(static_cast<int>(q_desired.size()), model_->nu);
        for (int i = 0; i < num_actuators; ++i)
        {
            data_->ctrl[i] = q_desired(i);
        }
    }

    void get_state(VectorXd &q, VectorXd &v) const
    {
        // Get positions
        q = Eigen::Map<const VectorXd>(data_->qpos, model_->nq);
        // Get velocities
        v = Eigen::Map<const VectorXd>(data_->qvel, model_->nv);
    }

    void step(double dt)
    {
        // Advance simulation
        mj_step(model_, data_);
    }

    // Compute forward kinematics only (no dynamics).
    // Updates site_xpos, xpos etc. from current qpos without stepping time.
    void forward()
    {
        mj_forward(model_, data_);
    }

    void step_with_render(double dt)
    {
        step(dt);
        render();
    }

    double get_time() const { return data_->time; }

    int get_num_dofs() const { return model_->nv; }
    int get_num_positions() const { return model_->nq; }

    // Get end-effector position in world frame (for visualization)
    Eigen::Vector3d get_ee_position_world() const
    {
        // Use right_tcp site (TCP from RobotV3 MJCF)
        int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "right_tcp");
        if (ee_site_id >= 0)
        {
            Eigen::Vector3d pos(
                data_->site_xpos[ee_site_id * 3 + 0],
                data_->site_xpos[ee_site_id * 3 + 1],
                data_->site_xpos[ee_site_id * 3 + 2]);
            return pos;
        }

        // Fallback to right_arm_link7 body
        int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        if (ee_body_id >= 0)
        {
            std::cerr << "Warning: right_tcp site not found, using right_arm_link7" << std::endl;
            Eigen::Vector3d pos(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);
            return pos;
        }

        return Eigen::Vector3d::Zero();
    }

    // Get end-effector position relative to base (agv_link/worldbody) frame
    Eigen::Vector3d get_ee_position() const
    {
        // agv_link is the worldbody in MuJoCo (no named body), use body_id=0
        int base_body_id = 0;

        // Get base position and orientation in world frame
        Eigen::Vector3d base_pos(
            data_->xpos[base_body_id * 3 + 0],
            data_->xpos[base_body_id * 3 + 1],
            data_->xpos[base_body_id * 3 + 2]);

        // Extract rotation matrix from xmat (3x3 matrix stored in column-major order)
        Eigen::Matrix3d base_rot;
        base_rot << data_->xmat[base_body_id * 9 + 0], data_->xmat[base_body_id * 9 + 1], data_->xmat[base_body_id * 9 + 2],
            data_->xmat[base_body_id * 9 + 3], data_->xmat[base_body_id * 9 + 4], data_->xmat[base_body_id * 9 + 5],
            data_->xmat[base_body_id * 9 + 6], data_->xmat[base_body_id * 9 + 7], data_->xmat[base_body_id * 9 + 8];

        // IMPORTANT: Try ee_site first (this is the actual end effector tip)
        int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "right_tcp");

        Eigen::Vector3d ee_world_pos;

        if (ee_site_id >= 0)
        {
            // site_xpos array has 3 values per site: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->site_xpos[ee_site_id * 3 + 0],
                data_->site_xpos[ee_site_id * 3 + 1],
                data_->site_xpos[ee_site_id * 3 + 2]);

            // Transform from world frame to base (agv_link) frame
            Eigen::Vector3d pos_base = base_rot.transpose() * (ee_world_pos - base_pos);

            // Debug: print first few positions
            static int debug_count = 0;
            if (debug_count < 3)
            {
                std::cout << "[DEBUG] ee_site world pos: " << ee_world_pos.transpose() << std::endl;
                std::cout << "[DEBUG] ee_site base pos: " << pos_base.transpose() << std::endl;
                debug_count++;
            }

            return pos_base;
        }

        // Try to use right_tcp body (TCP from URDF)
        int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_tcp");
        if (ee_body_id >= 0)
        {
            // xpos array has 3 values per body: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            // Transform from world frame to base (agv_link) frame
            Eigen::Vector3d pos_base = base_rot.transpose() * (ee_world_pos - base_pos);

            static bool debug_printed = false;
            if (!debug_printed)
            {
                std::cout << "[DEBUG] Using right_tcp body ID: " << ee_body_id << std::endl;
                std::cout << "[DEBUG] EE world pos: " << ee_world_pos.transpose() << std::endl;
                std::cout << "[DEBUG] EE base pos: " << pos_base.transpose() << std::endl;
                debug_printed = true;
            }

            return pos_base;
        }

        // Fallback to right_arm_link7 body
        ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        if (ee_body_id >= 0)
        {
            std::cerr << "Warning: right_tcp not found, using right_arm_link7 instead" << std::endl;

            // xpos array has 3 values per body: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            // Transform from world frame to base (agv_link) frame
            Eigen::Vector3d pos_base = base_rot.transpose() * (ee_world_pos - base_pos);

            return pos_base;
        }

        static bool warning_printed = false;
        if (!warning_printed)
        {
            std::cerr << "Warning: Could not find right_tcp or right_arm_link7" << std::endl;
            std::cerr << "Available sites:" << std::endl;
            for (int i = 0; i < model_->nsite; ++i)
            {
                const char *name = mj_id2name(model_, mjOBJ_SITE, i);
                if (name)
                    std::cerr << "  site[" << i << "] = " << name << std::endl;
            }
            warning_printed = true;
        }

        return Eigen::Vector3d::Zero();
    }

private:
    mjModel *model_ = nullptr;
    mjData *data_ = nullptr;

    // Visualization members
    GLFWwindow *window_ = nullptr;
    mjvScene scene_ = {}; // IMPORTANT: Zero-initialize to prevent memory corruption in mjv_makeScene
    mjvCamera camera_;
    mjvOption opt_;
    mjvPerturb pert_;
    mjrContext context_;

    // Mouse interaction state
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double last_mouse_x_ = 0.0;
    double last_mouse_y_ = 0.0;

    bool visualization_enabled_ = false;
    bool should_close_ = false;
    const int font_scale_ = 100; // Font scale for UI

    // Allow callbacks to access private members
    friend void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
    friend void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
    friend void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
    friend void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
};

// GLFW callback functions for MuJoCo mouse/keyboard interaction
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
    MuJoCoSimulator *sim = static_cast<MuJoCoSimulator *>(glfwGetWindowUserPointer(window));
    if (!sim)
        return;

    // Calculate mouse delta
    double dx = xpos - sim->last_mouse_x_;
    double dy = ypos - sim->last_mouse_y_;

    sim->last_mouse_x_ = xpos;
    sim->last_mouse_y_ = ypos;

    // Apply camera movement based on button state
    if (sim->button_left_)
    {
        // Rotate camera
        sim->camera_.azimuth += dx * 0.5;
        sim->camera_.elevation += dy * 0.5;
    }
    else if (sim->button_middle_)
    {
        // Pan camera
        sim->camera_.lookat[0] -= dx * 0.01;
        sim->camera_.lookat[1] += dy * 0.01;
    }
    else if (sim->button_right_)
    {
        // Zoom camera
        sim->camera_.distance += dy * 0.01;
        if (sim->camera_.distance < 0.1)
            sim->camera_.distance = 0.1;
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    MuJoCoSimulator *sim = static_cast<MuJoCoSimulator *>(glfwGetWindowUserPointer(window));
    if (!sim)
        return;

    // Update button state
    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            if (mods & GLFW_MOD_CONTROL)
            {
                sim->button_left_ = true;
                sim->button_middle_ = false;
                sim->button_right_ = false;
            }
            else if (mods & GLFW_MOD_SHIFT)
            {
                sim->button_left_ = false;
                sim->button_middle_ = true;
                sim->button_right_ = false;
            }
            else
            {
                sim->button_left_ = true;
                sim->button_middle_ = false;
                sim->button_right_ = false;
            }
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            sim->button_right_ = true;
        }
    }
    else if (action == GLFW_RELEASE)
    {
        sim->button_left_ = false;
        sim->button_middle_ = false;
        sim->button_right_ = false;
    }
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    MuJoCoSimulator *sim = static_cast<MuJoCoSimulator *>(glfwGetWindowUserPointer(window));
    if (sim)
    {
        // Zoom with scroll wheel
        sim->camera_.distance -= yoffset * 0.05;
        if (sim->camera_.distance < 0.1)
            sim->camera_.distance = 0.1;
    }
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    MuJoCoSimulator *sim = static_cast<MuJoCoSimulator *>(glfwGetWindowUserPointer(window));
    if (!sim)
        return;

    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

class DrakeSimulator;

struct SCurveProfile
{
    double T1, T2, T3, T4, T5, T6, T7; // durations of 7 phases
    double T_total;                    // total duration
    double s_total;                    // total path length
    double v_max, a_max, j_max;        // limits used
};

static SCurveProfile ComputeSCurveProfile(
    double s_total, double v_max, double a_max, double j_max)
{
    SCurveProfile p;
    p.s_total = s_total;
    p.v_max = v_max;
    p.a_max = a_max;
    p.j_max = j_max;

    if (s_total < 1e-9)
    {
        p.T1 = p.T2 = p.T3 = p.T4 = p.T5 = p.T6 = p.T7 = 0;
        p.T_total = 0;
        return p;
    }

    // Phase 1 & 3 durations: ramp acceleration up/down
    // Time to reach a_max from 0: T1 = a_max / j_max
    // Distance during T1+T3: s_13 = 0.5 * a_max * (T1 + T3)^2 / ...
    // Velocity gained during T1+T3: v_13 = a_max * (T1 + T3 - T1*T3*j_max/a_max/2)

    // Try full profile: can we reach v_max?
    double t_j = a_max / j_max; // jerk ramp time
    double s_reach_v = 0.5 * a_max * (v_max / a_max) * (v_max / a_max) +
                       0.5 * v_max * v_max / a_max; // simplified
    // More precise: distance to accelerate from 0 to v_max with jerk limit
    // If t_j >= v_max/a_max: triangular accel (no T2)
    // Else: trapezoidal accel with T1=t_j, T2=(v_max/a_max - t_j), T3=t_j

    double t_a, t_d; // total accel/decel time
    double s_a, s_d; // distance during accel/decel

    // --- Acceleration phase: 0 → v_max ---
    if (v_max <= a_max * t_j)
    {
        // Triangular acceleration: no constant-accel phase
        t_a = 2.0 * std::sqrt(v_max / j_max);
        s_a = v_max * t_a / 2.0;
        p.T1 = t_a / 2.0;
        p.T2 = 0;
        p.T3 = t_a / 2.0;
    }
    else
    {
        // Trapezoidal acceleration
        p.T1 = t_j;
        p.T2 = v_max / a_max - t_j;
        p.T3 = t_j;
        t_a = p.T1 + p.T2 + p.T3;
        s_a = 0.5 * v_max * t_a;
    }

    // Symmetric deceleration
    p.T5 = p.T3;
    p.T6 = p.T2;
    p.T7 = p.T1;
    t_d = t_a;
    s_d = s_a;

    double s_ad = s_a + s_d;

    if (s_ad <= s_total)
    {
        // Can reach v_max: full 7-segment profile
        p.T4 = (s_total - s_ad) / v_max;
    }
    else
    {
        // Cannot reach v_max: reduce peak velocity (no cruise phase)
        // Solve: s_a(v_peak) + s_d(v_peak) = s_total
        // s_a = s_d = f(v_peak), s_ad = 2*s_a
        // For trapezoidal accel: s_a = 0.5 * v_peak * t_a = 0.5*v_peak*(v_peak/a_max + t_j)
        // For triangular accel: s_a = v_peak * sqrt(v_peak/j_max)

        // Try triangular acceleration first
        double v_peak = std::pow(s_total * s_total * j_max / 4.0, 1.0 / 3.0);
        // Iterative refinement
        for (int iter = 0; iter < 20; ++iter)
        {
            double ta, sa;
            if (v_peak <= a_max * t_j)
            {
                ta = 2.0 * std::sqrt(v_peak / j_max);
                sa = v_peak * ta / 2.0;
            }
            else
            {
                ta = v_peak / a_max + t_j;
                sa = 0.5 * v_peak * ta;
            }
            double err = 2.0 * sa - s_total;
            if (std::abs(err) < 1e-12)
                break;
            // Newton-like update
            double dsa_dv;
            if (v_peak <= a_max * t_j)
            {
                dsa_dv = 1.5 * std::sqrt(v_peak / j_max);
            }
            else
            {
                dsa_dv = 0.5 * (2.0 * v_peak / a_max + t_j);
            }
            v_peak -= err / (2.0 * dsa_dv);
            v_peak = std::max(v_peak, 1e-6);
        }

        // Rebuild with v_peak
        if (v_peak <= a_max * t_j)
        {
            p.T1 = std::sqrt(v_peak / j_max);
            p.T2 = 0;
            p.T3 = p.T1;
        }
        else
        {
            p.T1 = t_j;
            p.T2 = v_peak / a_max - t_j;
            p.T3 = t_j;
        }
        p.T5 = p.T3;
        p.T6 = p.T2;
        p.T7 = p.T1;
        p.T4 = 0;
    }

    p.T_total = p.T1 + p.T2 + p.T3 + p.T4 + p.T5 + p.T6 + p.T7;
    return p;
}

// Build exact s(t) as a PiecewisePolynomial from the 7 S-curve phases.
// Each phase has known polynomial form: s(τ) = s0 + v0·τ + ½·a0·τ² + ⅙·j·τ³
// This avoids the approximation error from resampling the S-curve into a
// generic cubic spline, which smoothed out jerk transitions and caused
// acceleration/velocity artifacts.
static drake::trajectories::PiecewisePolynomial<double> BuildSCurveTiming(
    const SCurveProfile &profile)
{
    struct Phase
    {
        double duration;
        double jerk;
    };
    Phase phases[7] = {
        {profile.T1, profile.j_max},
        {profile.T2, 0.0},
        {profile.T3, -profile.j_max},
        {profile.T4, 0.0},
        {profile.T5, -profile.j_max},
        {profile.T6, 0.0},
        {profile.T7, profile.j_max},
    };

    using PolyMat = Eigen::Matrix<drake::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>;
    using PP = drake::trajectories::PiecewisePolynomial<double>;

    std::vector<double> breaks = {0.0};
    std::vector<PolyMat> polys;

    double v = 0.0, a = 0.0, s = 0.0;

    for (int i = 0; i < 7; ++i)
    {
        double dt = phases[i].duration;
        double j = phases[i].jerk;
        if (dt < 1e-12)
            continue;

        // Polynomial coefficients: s(τ) = c0 + c1·τ + c2·τ² + c3·τ³
        Eigen::Vector4d coeffs(s, v, 0.5 * a, (1.0 / 6.0) * j);

        PolyMat pm(1, 1);
        pm(0, 0) = drake::Polynomial<double>(coeffs);
        polys.push_back(pm);

        breaks.push_back(breaks.back() + dt);

        // Advance state to end of phase
        s += v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
        v += a * dt + 0.5 * j * dt * dt;
        a += j * dt;
    }

    if (polys.empty())
    {
        PolyMat pm(1, 1);
        pm(0, 0) = drake::Polynomial<double>(Eigen::Vector4d::Zero());
        polys.push_back(pm);
        breaks.push_back(0.001);
    }

    return PP(polys, breaks);
}

// Evaluate arc length s(t) at time t for a 7-segment S-curve profile
static double SCurveArcLength(const SCurveProfile &p, double t)
{
    if (t <= 0)
        return 0;
    if (t >= p.T_total)
        return p.s_total;

    double j = p.j_max;
    double s = 0;
    double v = 0; // velocity at phase start
    double a = 0; // acceleration at phase start
    double t_rem = t;

    // Phase 1: jerk = +j, a: 0 → j*T1, v: 0 → v1
    // a(t) = j*t,  v(t) = 0.5*j*t²,  s(t) = (1/6)*j*t³
    {
        double dt = std::min(t_rem, p.T1);
        s += v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
        v += 0.5 * j * dt * dt; // v after dt
        a += j * dt;            // a after dt
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 2: a = a_max (constant), v increases linearly
    {
        double dt = std::min(t_rem, p.T2);
        s += v * dt + 0.5 * a * dt * dt;
        v += a * dt;
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 3: jerk = -j, a: a_max → 0
    // a(t) = a_max - j*t,  v(t) = v + a_max*t - 0.5*j*t²
    {
        double dt = std::min(t_rem, p.T3);
        s += v * dt + 0.5 * a * dt * dt - (1.0 / 6.0) * j * dt * dt * dt;
        v += a * dt - 0.5 * j * dt * dt;
        a -= j * dt;
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 4: constant velocity (a = 0)
    {
        double dt = std::min(t_rem, p.T4);
        s += v * dt;
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 5: jerk = -j, a: 0 → -a_max
    // a(t) = -j*t,  v(t) = v - 0.5*j*t²
    {
        double dt = std::min(t_rem, p.T5);
        s += v * dt - (1.0 / 6.0) * j * dt * dt * dt;
        v -= 0.5 * j * dt * dt;
        a -= j * dt;
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 6: a = -a_max (constant)
    {
        double dt = std::min(t_rem, p.T6);
        s += v * dt + 0.5 * a * dt * dt;
        v += a * dt;
        t_rem -= dt;
        if (t_rem <= 0)
            return s;
    }

    // Phase 7: jerk = +j, a: -a_max → 0
    {
        double dt = std::min(t_rem, p.T7);
        s += v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
    }

    return s;
}

// ========================================================================
// C∞-smooth timing polynomial (7th-order)
// Replaces the 7-phase S-curve for jitter-free joint velocity/acceleration.
//
// s(t) = s_total · (35τ⁴ - 84τ⁵ + 70τ⁶ - 20τ⁷),  τ = t/T
//
// Boundary conditions (all 4 derivatives zero at both endpoints):
//   s(0)=0, ṡ(0)=0, s̈(0)=0, s⃛(0)=0
//   s(T)=L, ṡ(T)=0, s̈(T)=0, s⃛(T)=0
//
// Peak values:
//   max ṡ   = L/T · 35/16      ≈ 2.1875 · L/T      at τ = 0.5
//   max s̈   = L/T² · 7.508                           at τ ≈ 0.276
//   max s⃛   = L/T³ · 52.5                            at τ = 0.5
// ========================================================================

static double ComputeSmoothTimingDuration(
    double s_total, double s_vel_max, double s_acc_max,
    double s_jerk_max = std::numeric_limits<double>::infinity())
{
    if (s_total < 1e-9)
        return 0.0;

    constexpr double kPeakVelScale = 35.0 / 16.0; // 2.1875
    constexpr double kPeakAccScale = 7.508;
    constexpr double kPeakJerkScale = 52.5;

    double T_vel = s_total * kPeakVelScale / s_vel_max;
    double T_acc = std::sqrt(s_total * kPeakAccScale / s_acc_max);
    double T_jerk = std::cbrt(s_total * kPeakJerkScale / s_jerk_max);

    return std::max({T_vel, T_acc, T_jerk});
}

static drake::trajectories::PiecewisePolynomial<double> BuildSmoothTiming(
    double s_total, double T)
{
    using PolyMat = Eigen::Matrix<drake::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>;
    using PP = drake::trajectories::PiecewisePolynomial<double>;

    if (s_total < 1e-9 || T < 1e-9)
    {
        PolyMat pm(1, 1);
        pm(0, 0) = drake::Polynomial<double>(Eigen::Vector4d::Zero());
        std::vector<PolyMat> polys = {pm};
        std::vector<double> breaks = {0.0, 0.001};
        return PP(polys, breaks);
    }

    // s(t) = a₄t⁴ + a₅t⁵ + a₆t⁶ + a₇t⁷  (a₀..a₃ = 0)
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T;
    double T5 = T4 * T, T6 = T5 * T, T7 = T6 * T;

    Eigen::VectorXd coeffs(8);
    coeffs << 0.0,            // a₀
        0.0,                  // a₁
        0.0,                  // a₂
        0.0,                  // a₃
        35.0 * s_total / T4,  // a₄
        -84.0 * s_total / T5, // a₅
        70.0 * s_total / T6,  // a₆
        -20.0 * s_total / T7; // a₇

    PolyMat pm(1, 1);
    pm(0, 0) = drake::Polynomial<double>(coeffs);

    std::vector<PolyMat> polys = {pm};
    std::vector<double> breaks = {0.0, T};

    return PP(polys, breaks);
}

class DrakeSimulator
{
public:
    DrakeSimulator(const std::string &urdf_path, double time_step = 0.001)
    {

        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "[UNIFIED PLANT] Building RobotDiagram for simulation + collision detection" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        drake::planning::RobotDiagramBuilder<double> robot_builder(time_step);

        // Load URDF model
        auto &parser = robot_builder.parser();
        parser.AddModelsFromUrl(std::string("file://") + urdf_path);

        // WELD BASE TO WORLD FRAME
        const drake::multibody::Frame<double> &world_frame = robot_builder.plant().world_frame();
        const drake::multibody::Frame<double> &base_frame =
            robot_builder.plant().GetFrameByName("agv_link");

        robot_builder.plant().WeldFrames(world_frame, base_frame, drake::math::RigidTransformd());

        // ========================================================================
        // CRITICAL FIX: Add environment obstacles as MultibodyPlant bodies
        // ========================================================================
        // This is the CORRECT way to add obstacles for collision detection in Drake:
        //
        // 1. Create a rigid body for each obstacle using robot_builder.plant().AddRigidBody()
        // 2. Add collision geometry to the body using robot_builder.plant().RegisterCollisionGeometry()
        // 3. Weld the obstacle body to the world frame (making it static)
        //
        // Why this works:
        // - Obstacles become part of the MultibodyPlant topology
        // - They are automatically included in collision detection
        // - They share the same source ID as the robot
        // - Welding to world makes them static (no DOFs)
        //
        // Previous approach (RegisterAnchoredGeometry) doesn't work because:
        // - Anchored geometries are not part of the plant's body topology
        // - They may not participate in collision detection properly
        // - They require additional configuration for collision filters
        // ========================================================================
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "[ENVIRONMENT OBSTACLES] Adding as MultibodyPlant bodies..." << std::endl;
        std::cout << "[METHOD] Using robot_builder.plant().AddRigidBody() + RegisterCollisionGeometry()" << std::endl;
        std::cout << "[BENEFIT] Obstacles become full plant bodies with proper collision" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Create a dedicated model instance for obstacles
        // This keeps obstacles organized and separate from the robot model
        const drake::multibody::ModelInstanceIndex obstacle_model_instance =
            robot_builder.plant().AddModelInstance("environment_obstacles");

        std::cout << "[MODEL] Created obstacle model instance: " << obstacle_model_instance << std::endl;

        // Helper lambda to add box obstacles
        auto add_box_obstacle = [&](
                                    const std::string &name,
                                    const Eigen::Vector3d &size,
                                    const Eigen::Vector3d &position)
        {
            // Create rigid body for obstacle with default spatial inertia
            // CRITICAL: Use explicit model_instance to avoid default ambiguity
            const drake::multibody::RigidBody<double> &obstacle_body =
                robot_builder.plant().AddRigidBody(name, obstacle_model_instance,
                                                   drake::multibody::SpatialInertia<double>::Zero());

            // Set collision geometry properties
            drake::geometry::ProximityProperties props;
            props.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kFriction,
                              drake::multibody::CoulombFriction<double>(1.0, 1.0));

            // NOTE: No hydroelastic properties — point contact is sufficient
            // for collision checking and avoids false positives from soft
            // compliance pressure fields extending beyond the geometry surface.

            // Create box shape
            const drake::geometry::Box shape(size(0), size(1), size(2));

            // CRITICAL FIX: Register collision geometry in body frame (identity)
            // Position will be set by WeldFrames
            drake::geometry::GeometryId collision_geometry_id =
                robot_builder.plant().RegisterCollisionGeometry(
                    obstacle_body,
                    drake::math::RigidTransformd(), // Identity in body frame
                    shape,
                    name + "_collision",
                    props);

            // CRITICAL: Also register visual geometry for the obstacle
            drake::geometry::GeometryId visual_geometry_id =
                robot_builder.plant().RegisterVisualGeometry(
                    obstacle_body,
                    drake::math::RigidTransformd(), // Identity in body frame
                    shape,
                    name + "_visual");

            std::cout << "  [GEOMETRY ID] Collision: " << collision_geometry_id
                      << ", Visual: " << visual_geometry_id << std::endl;

            // CRITICAL FIX: Weld obstacle to world at the specified position
            // This positions the obstacle body at the world coordinates
            const drake::multibody::Frame<double> &world_frame = robot_builder.plant().world_frame();
            const drake::multibody::Frame<double> &obstacle_frame = obstacle_body.body_frame();

            // Create transform with translation
            drake::math::RigidTransformd X_WO =
                drake::math::RigidTransformd(
                    drake::math::RollPitchYawd(0.0, 0.0, 0.0), // No rotation
                    position                                   // Translation to world coordinates
                );

            robot_builder.plant().WeldFrames(world_frame, obstacle_frame, X_WO);

            std::cout << "  [ADDED] " << name << " at (" << position.transpose()
                      << ") size=(" << size.transpose() << ")" << std::endl;
        };

        // Helper lambda to add sphere obstacles
        auto add_sphere_obstacle = [&](
                                       const std::string &name,
                                       double radius,
                                       const Eigen::Vector3d &position)
        {
            // Create rigid body for obstacle with default spatial inertia
            // CRITICAL: Use explicit model_instance to avoid default ambiguity
            const drake::multibody::RigidBody<double> &obstacle_body =
                robot_builder.plant().AddRigidBody(name, obstacle_model_instance,
                                                   drake::multibody::SpatialInertia<double>::Zero());

            // Set collision geometry properties
            drake::geometry::ProximityProperties props;
            props.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kFriction,
                              drake::multibody::CoulombFriction<double>(1.0, 1.0));

            // NOTE: No hydroelastic — point contact avoids false positives.

            // Create sphere shape
            const drake::geometry::Sphere shape(radius);

            // Register collision geometry in body frame
            drake::geometry::GeometryId collision_id =
                robot_builder.plant().RegisterCollisionGeometry(
                    obstacle_body,
                    drake::math::RigidTransformd(), // Identity in body frame
                    shape,
                    name + "_collision",
                    props);

            // Also register visual geometry
            drake::geometry::GeometryId visual_id =
                robot_builder.plant().RegisterVisualGeometry(
                    obstacle_body,
                    drake::math::RigidTransformd(),
                    shape,
                    name + "_visual");

            std::cout << "  [GEOMETRY ID] Collision: " << collision_id
                      << ", Visual: " << visual_id << std::endl;

            // Weld to world at specified position
            drake::math::RigidTransformd X_WO(
                drake::math::RollPitchYawd(0.0, 0.0, 0.0),
                position);
            robot_builder.plant().WeldFrames(robot_builder.plant().world_frame(), obstacle_body.body_frame(), X_WO);

            std::cout << "  [ADDED] " << name << " at (" << position.transpose()
                      << ") radius=" << radius << std::endl;
        };

        // Helper lambda to add cylinder obstacles
        auto add_cylinder_obstacle = [&](
                                         const std::string &name,
                                         double radius,
                                         double height,
                                         const Eigen::Vector3d &position)
        {
            // Create rigid body for obstacle with default spatial inertia
            // CRITICAL: Use explicit model_instance to avoid default ambiguity
            const drake::multibody::RigidBody<double> &obstacle_body =
                robot_builder.plant().AddRigidBody(name, obstacle_model_instance,
                                                   drake::multibody::SpatialInertia<double>::Zero());

            // Set collision geometry properties
            drake::geometry::ProximityProperties props;
            props.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kFriction,
                              drake::multibody::CoulombFriction<double>(1.0, 1.0));

            // NOTE: No hydroelastic — point contact avoids false positives.

            // Create cylinder shape
            const drake::geometry::Cylinder shape(radius, height);

            // Register collision geometry in body frame
            drake::geometry::GeometryId collision_id = robot_builder.plant().RegisterCollisionGeometry(
                obstacle_body,
                drake::math::RigidTransformd(), // Identity in body frame
                shape,
                name + "_collision",
                props);

            // Register visual geometry
            drake::geometry::GeometryId visual_id = robot_builder.plant().RegisterVisualGeometry(
                obstacle_body,
                drake::math::RigidTransformd(),
                shape,
                name + "_visual");

            std::cout << "  [GEOMETRY ID] Collision: " << collision_id
                      << ", Visual: " << visual_id << std::endl;

            // Weld to world at specified position
            drake::math::RigidTransformd X_WO(
                drake::math::RollPitchYawd(0.0, 0.0, 0.0),
                position);
            robot_builder.plant().WeldFrames(robot_builder.plant().world_frame(), obstacle_body.body_frame(), X_WO);

            std::cout << "  [ADDED] " << name << " at (" << position.transpose()
                      << ") radius=" << radius << " height=" << height << std::endl;
        };

        std::cout << "\n[TABLE] Adding table components..." << std::endl;

        // ============================================================================
        // 1. TABLE (from scene.xml: table body pos="0.7 0 -0.244")
        // ============================================================================
        // Table top: MuJoCo size="0.4 0.3 0.2" (half-extents) pos="0 0 0.32" relative to table body
        // Table body at (0.7, 0, -0.244), so table_top world pos = (0.7, 0, -0.244+0.32) = (0.7, 0, 0.076)
        // Drake uses full extents, so size = 2 × half-extents = (0.8, 0.6, 0.4)
        add_box_obstacle("table_top",
                         Eigen::Vector3d(0.8, 0.6, 0.4),
                         Eigen::Vector3d(0.7, 0.0, 0.076));

        // Table legs: MuJoCo size="0.03 0.03 0.5" (half-extents), actual size = (0.06, 0.06, 1.0)
        // Leg positions relative to table body at (0.7, 0, -0.244):
        //   (-0.37, ±0.27, 0) and (0.37, ±0.27, 0)
        std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>> table_legs = {
            {"table_leg1", Eigen::Vector3d(0.06, 0.06, 1.0), Eigen::Vector3d(0.7 - 0.37, 0.27, -0.244)},
            {"table_leg2", Eigen::Vector3d(0.06, 0.06, 1.0), Eigen::Vector3d(0.7 - 0.37, -0.27, -0.244)},
            {"table_leg3", Eigen::Vector3d(0.06, 0.06, 1.0), Eigen::Vector3d(0.7 + 0.37, 0.27, -0.244)},
            {"table_leg4", Eigen::Vector3d(0.06, 0.06, 1.0), Eigen::Vector3d(0.7 + 0.37, -0.27, -0.244)}};

        std::cout << "\n[TABLE LEGS] Adding 4 table legs..." << std::endl;
        for (const auto &[name, size, position] : table_legs)
        {
            add_box_obstacle(name, size, position);
        }

        // Finalize plant AFTER obstacles are added
        std::cout << "\n[PLANT] Finalizing MultibodyPlant with obstacles..." << std::endl;
        robot_builder.plant().Finalize();
        std::cout << "[PLANT] Finalization complete - obstacles are now part of the plant!" << std::endl;
        std::cout << "[BODIES] Total bodies in plant: " << robot_builder.plant().num_bodies() << std::endl;

        // ============================================================
        // Build the unified RobotDiagram
        robot_diagram_ = robot_builder.Build();
        std::cout << "  [SUCCESS] RobotDiagram built with robot + obstacles!" << std::endl;

        // Build standalone plant for IK (no SceneGraph connection)
        {
            ik_plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(time_step);
            drake::multibody::Parser ik_parser(ik_plant_.get());
            ik_parser.AddModelsFromUrl(std::string("file://") + urdf_path);
            const auto &ik_base = ik_plant_->GetFrameByName("agv_link");
            ik_plant_->WeldFrames(ik_plant_->world_frame(), ik_base, drake::math::RigidTransformd());
            ik_plant_->Finalize();
            std::cout << "  [IK] Standalone IK plant finalized: " << ik_plant_->num_positions() << " positions" << std::endl;
        }

        // Create simulator from unified RobotDiagram
        simulator_ = std::make_unique<drake::systems::Simulator<double>>(*robot_diagram_);

        // ============================================================
        // INDUSTRIAL-GRADE: Initialize SceneGraphCollisionChecker
        // ============================================================
        std::cout << "\n[COLLISION CHECKER] Initializing SceneGraphCollisionChecker..." << std::endl;

        try
        {
            using namespace drake::planning;

            drake::planning::CollisionCheckerParams checker_params;
            checker_params.model = robot_diagram_;
            checker_params.edge_step_size = 0.005; // 0.005rad for thorough CCD

            // Get robot model instances (exclude obstacles and world)
            std::vector<drake::multibody::ModelInstanceIndex> robot_model_instances;
            const drake::multibody::ModelInstanceIndex world_instance =
                drake::multibody::ModelInstanceIndex(0);

            std::cout << "  [MODEL INSTANCES] Scanning all model instances..." << std::endl;
            for (drake::multibody::ModelInstanceIndex idx(0); idx < robot_diagram_->plant().num_model_instances(); ++idx)
            {
                std::string instance_name = robot_diagram_->plant().GetModelInstanceName(idx);
                std::cout << "    Found: " << instance_name << " (index=" << idx << ")" << std::endl;

                // DEBUG: Count bodies and collision geometries in this model instance
                int num_bodies = 0;
                int num_collision_geometries = 0;
                for (drake::multibody::BodyIndex body_idx(0); body_idx < robot_diagram_->plant().num_bodies(); ++body_idx)
                {
                    const auto &body = robot_diagram_->plant().get_body(body_idx);
                    if (body.model_instance() == idx)
                    {
                        num_bodies++;
                        // Count collision geometries for this body
                        const auto &collision_ids = robot_diagram_->plant().GetCollisionGeometriesForBody(body);
                        num_collision_geometries += collision_ids.size();
                    }
                }
                std::cout << "      -> Bodies: " << num_bodies << ", Collision geometries: " << num_collision_geometries << std::endl;

                // CRITICAL FIX: Include both "RobotV3" and "DefaultModelInstance" as robot
                // When Drake loads URDF via AddModelsFromUrl, it creates "DefaultModelInstance"
                // The robot name in URDF (<robot name="RobotV3">) becomes the instance name
                if (instance_name == "RobotV3" || instance_name == "DefaultModelInstance")
                {
                    robot_model_instances.push_back(idx);
                    std::cout << "      -> Added to ROBOT model instances" << std::endl;
                }
                else if (instance_name == "environment_obstacles")
                {
                    std::cout << "      -> This is ENVIRONMENT (will be checked against)" << std::endl;
                }
                else if (idx != world_instance)
                {
                    std::cout << "      -> EXCLUDED (not robot, not environment)" << std::endl;
                }
            }

            if (robot_model_instances.empty())
            {
                throw std::runtime_error("[ERROR] No robot model instances found!");
            }

            checker_params.robot_model_instances = robot_model_instances;

            // CRITICAL: Also specify environment model instances for robot-environment collision detection
            // SceneGraphCollisionChecker needs to know which geometries are "environment" to check against
            std::vector<drake::multibody::ModelInstanceIndex> environment_model_instances;
            for (drake::multibody::ModelInstanceIndex idx(0); idx < robot_diagram_->plant().num_model_instances(); ++idx)
            {
                std::string instance_name = robot_diagram_->plant().GetModelInstanceName(idx);
                if (instance_name == "environment_obstacles")
                {
                    environment_model_instances.push_back(idx);
                    std::cout << "  [ENV] Found environment model: " << instance_name << " (index=" << idx << ")" << std::endl;
                }
            }

            if (environment_model_instances.empty())
            {
                std::cout << "  [WARNING] No environment model instances found!" << std::endl;
            }
            else
            {
                std::cout << "  [SUCCESS] Found " << environment_model_instances.size() << " environment model instances" << std::endl;
            }

            // DEBUG: Print model instances
            std::cout << "  [DEBUG] Total model instances: " << robot_diagram_->plant().num_model_instances() << std::endl;
            std::cout << "  [DEBUG] Robot model instances count: " << robot_model_instances.size() << std::endl;
            for (auto idx : robot_model_instances)
            {
                std::cout << "    - Robot instance: " << robot_diagram_->plant().GetModelInstanceName(idx) << std::endl;
            }

            // Create SceneGraphCollisionChecker
            collision_checker_ = std::make_unique<SceneGraphCollisionChecker>(checker_params);

            // Configure zero padding (industrial-grade: ANY penetration is collision)
            collision_checker_->SetPaddingAllRobotEnvironmentPairs(0.0);
            collision_checker_->SetPaddingAllRobotRobotPairs(0.0);

            // ============================================================================
            // CRITICAL: Filter adjacent link collisions to reduce false positives
            // ============================================================================
            SetupAdjacentLinkCollisionFiltering();

            // CRITICAL DEBUG: Test if collision checker can detect robot-environment collisions
            std::cout << "  [DEBUG TEST] Testing robot-environment collision detection..." << std::endl;

            // Get the initial configuration
            VectorXd q_test = robot_diagram_->plant().GetPositions(*robot_diagram_->plant().CreateDefaultContext());

            // Test CheckConfigCollisionFree - this should detect robot-environment collisions
            bool is_collision_free = collision_checker_->CheckConfigCollisionFree(q_test);
            std::cout << "  [DEBUG TEST] CheckConfigCollisionFree: " << (is_collision_free ? "COLLISION-FREE" : "COLLISION!") << std::endl;

            // Test CalcRobotClearance - this should return robot-environment distances
            drake::planning::RobotClearance test_clearance = collision_checker_->CalcRobotClearance(q_test, 1.0);
            std::cout << "  [DEBUG TEST] CalcRobotClearance size: " << test_clearance.size() << std::endl;

            // Count ENV vs SELF measurements
            int env_count = 0, self_count = 0;
            for (size_t i = 0; i < test_clearance.size(); ++i)
            {
                auto collision_type = test_clearance.collision_types()[i];
                bool is_env = (static_cast<uint8_t>(collision_type) &
                               static_cast<uint8_t>(drake::planning::RobotCollisionType::kEnvironmentCollision)) != 0;
                if (is_env)
                    env_count++;
                if (collision_type == drake::planning::RobotCollisionType::kSelfCollision)
                    self_count++;
            }
            std::cout << "  [DEBUG TEST] ENV measurements: " << env_count << ", SELF measurements: " << self_count << std::endl;

            if (env_count == 0)
            {
                std::cout << "  [ERROR] NO ENV measurements! SceneGraphCollisionChecker is NOT checking robot-environment collisions!" << std::endl;
                std::cout << "  [ERROR] This means obstacles are NOT being detected!" << std::endl;
            }

            std::cout << "  [SUCCESS] SceneGraphCollisionChecker initialized!" << std::endl;
            std::cout << "  [FEATURES] Industrial-grade collision detection:" << std::endl;
            std::cout << "    - Adjacent link filtering: AUTOMATIC (kinematic tree)" << std::endl;
            std::cout << "    - Collision padding: 0.0 (zero tolerance)" << std::endl;
            std::cout << "    - Edge step size: " << checker_params.edge_step_size << " rad" << std::endl;
            std::cout << "    - Parallel checking: SUPPORTED" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "  [ERROR] SceneGraphCollisionChecker initialization failed: " << e.what() << std::endl;
            std::cerr << "  [FATAL] Cannot continue without collision checker" << std::endl;
            throw;
        }

        std::cout << "Drake Model loaded successfully!" << std::endl;
        std::cout << "  - Positions: " << robot_diagram_->plant().num_positions() << " (FIXED BASE)" << std::endl;
        std::cout << "  - Velocities: " << robot_diagram_->plant().num_velocities() << std::endl;
        std::cout << "  - Actuators: " << robot_diagram_->plant().num_actuators() << std::endl;
        std::cout << "\n[INDUSTRIAL-GRADE] SceneGraphCollisionChecker configured:" << std::endl;
        std::cout << "  - Using unified RobotDiagram for simulation AND collision detection" << std::endl;
        std::cout << "  - Collision padding: 0.0 (zero tolerance)" << std::endl;
        std::cout << "  - Edge step size: 0.01 rad" << std::endl;
        std::cout << std::string(80, '=') << std::endl;
    }

    void reset()
    {
        simulator_->reset_context(simulator_->get_context().Clone());
        simulator_->get_mutable_context().SetTime(0.0);
    }

    void set_state(const VectorXd &q, const VectorXd &v)
    {
        // Get fresh plant context pointer - ensure valid lifetime
        // GetMyMutableContextFromRoot needs a POINTER to context
        auto &root_context = simulator_->get_mutable_context();
        auto &plant_context = robot_diagram_->plant().GetMyMutableContextFromRoot(&root_context);
        robot_diagram_->plant().SetPositions(&plant_context, q);
        robot_diagram_->plant().SetVelocities(&plant_context, v);
    }

    void get_state(VectorXd &q, VectorXd &v) const
    {
        auto &plant_context = robot_diagram_->plant().GetMyContextFromRoot(simulator_->get_context());
        q = robot_diagram_->plant().GetPositions(plant_context);
        v = robot_diagram_->plant().GetVelocities(plant_context);
    }

    void step(double dt)
    {
        simulator_->AdvanceTo(simulator_->get_context().get_time() + dt);
    }

    double get_time() const { return simulator_->get_context().get_time(); }

    int get_num_positions() const { return robot_diagram_->plant().num_positions(); }
    int get_num_velocities() const { return robot_diagram_->plant().num_velocities(); }

    // Expose simulator for trajectory evaluation
    drake::systems::Simulator<double> *get_simulator() { return simulator_.get(); }

    // ========== DRAKE FORWARD KINEMATICS ==========
    // Compute end-effector pose in base (agv_link) frame
    drake::math::RigidTransformd ComputeEEPose(const VectorXd &q)
    {
        auto &plant_context = robot_diagram_->plant().GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        robot_diagram_->plant().SetPositions(&plant_context, q);

        // Get end-effector frame (right_tool_frame)
        const auto &ee_frame = robot_diagram_->plant().GetFrameByName("right_tcp"); // TODO: 末端位姿

        // Get base frame (agv_link) as reference coordinate system
        const auto &base_frame = robot_diagram_->plant().GetFrameByName("agv_link");

        // Compute forward kinematics relative to base (agv_link)
        return robot_diagram_->plant().CalcRelativeTransform(plant_context, base_frame, ee_frame);
    }

    // =================================================================
    // Drake-based Trajectory Planning using KinematicTrajectoryOptimization
    // =================================================================

    /**
     * SolveIKFast - Fast unconstrained IK (no collision checking)
     * Used as the first pass for all IK points.
     */
    VectorXd SolveIKFast(const drake::math::RigidTransformd &target_pose,
                         const VectorXd &q_init,
                         double pos_tol = 0.001,
                         double rot_tol = 0.01)
    {
        auto &plant = *ik_plant_;
        const auto &ee_frame = plant.GetFrameByName("right_tcp");
        const auto &ref_frame = plant.GetFrameByName("agv_link");
        const double lock_tol = 1e-10;

        auto setup_common = [&](drake::multibody::InverseKinematics &ik,
                                bool add_orientation)
        {
            auto *prog = ik.get_mutable_prog();
            Eigen::Vector3d pos_lb = target_pose.translation() - Eigen::Vector3d::Constant(pos_tol);
            Eigen::Vector3d pos_ub = target_pose.translation() + Eigen::Vector3d::Constant(pos_tol);
            ik.AddPositionConstraint(ee_frame, Eigen::Vector3d::Zero(),
                                     ref_frame, pos_lb, pos_ub);
            if (add_orientation)
                ik.AddOrientationConstraint(ee_frame, drake::math::RotationMatrixd(),
                                            ref_frame, target_pose.rotation(), rot_tol);
            // Lock non-planning joints (head, left arm, prismatic lumber)
            // Planning joints: kPlanIndices = {1, 2, 12..18}
            VectorXd q_lo = plant.GetPositionLowerLimits();
            VectorXd q_hi = plant.GetPositionUpperLimits();
            for (int i = 0; i < plant.num_positions(); ++i)
            {
                bool is_plan_joint = false;
                for (int j = 0; j < kPlanDof; ++j)
                    if (kPlanIndices[j] == i) { is_plan_joint = true; break; }
                if (is_plan_joint)
                    prog->AddBoundingBoxConstraint(q_lo(i), q_hi(i), ik.q()(i));
                else
                    prog->AddBoundingBoxConstraint(
                        q_init(i) - lock_tol, q_init(i) + lock_tol, ik.q()(i));
            }
            // Regularization: lumber anchored to current position, arm toward comfort
            //   Lumber (j=0,1): strong pull toward q_init → waist stays put
            //   Arm (j=2..8): pull toward comfort posture → natural motion
            VectorXd q_target(kPlanDof);
            Eigen::MatrixXd Q_reg = Eigen::MatrixXd::Zero(kPlanDof, kPlanDof);
            for (int j = 0; j < kPlanDof; ++j)
            {
                if (j < 2)
                {
                    // Lumber: anchor to current position with very high weight
                    q_target(j) = q_init(kPlanIndices[j]);
                    Q_reg(j, j) = 10.0;
                }
                else
                {
                    // Arm: pull toward comfort posture
                    q_target(j) = kComfortPosture[j];
                    Q_reg(j, j) = kJointWeights[j];
                }
            }
            drake::VectorX<drake::symbolic::Variable> plan_vars(kPlanDof);
            for (int j = 0; j < kPlanDof; ++j)
                plan_vars(j) = ik.q()(kPlanIndices[j]);
            prog->AddQuadraticErrorCost(Q_reg, q_target, plan_vars);
            prog->SetInitialGuess(ik.q(), q_init);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "max_iter", 10000);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "tol", 1e-6);
        };

        // Phase 1: Position + Orientation
        {
            drake::multibody::InverseKinematics ik(plant, false);
            setup_common(ik, true);
            auto result = drake::solvers::Solve(*ik.get_mutable_prog());
            if (result.is_success())
                return result.GetSolution(ik.q());
        }
        // Phase 2: Position only
        {
            drake::multibody::InverseKinematics ik(plant, false);
            setup_common(ik, false);
            auto result = drake::solvers::Solve(*ik.get_mutable_prog());
            if (result.is_success())
                return result.GetSolution(ik.q());
        }
        return q_init;
    }

    /**
     * SolveIKWithCollision - Expensive collision-constrained IK
     * Only called for points that are actually in collision.
     * Uses MinimumDistanceLowerBoundConstraint to push ALL robot bodies
     * (self + environment) away from collision by at least collision_margin.
     *
     * @param q_seed Unconstrained IK solution (good position, may be in collision)
     */
    VectorXd SolveIKWithCollision(const drake::math::RigidTransformd &target_pose,
                                  const VectorXd &q_seed,
                                  double pos_tol, double rot_tol,
                                  double collision_margin)
    {
        if (!collision_checker_)
            return q_seed;

        auto &plant = *ik_plant_;
        const auto &ee_frame = plant.GetFrameByName("right_tcp");
        const auto &ref_frame = plant.GetFrameByName("agv_link");
        const double lock_tol = 1e-4; // Relaxed for collision-constrained solve

        auto try_solve = [&](bool add_orientation) -> std::optional<VectorXd>
        {
            drake::multibody::InverseKinematics ik(plant, false);
            auto *prog = ik.get_mutable_prog();

            Eigen::Vector3d pos_lb = target_pose.translation() - Eigen::Vector3d::Constant(pos_tol);
            Eigen::Vector3d pos_ub = target_pose.translation() + Eigen::Vector3d::Constant(pos_tol);
            ik.AddPositionConstraint(ee_frame, Eigen::Vector3d::Zero(),
                                     ref_frame, pos_lb, pos_ub);
            if (add_orientation)
                ik.AddOrientationConstraint(ee_frame, drake::math::RotationMatrixd(),
                                            ref_frame, target_pose.rotation(), rot_tol);

            // Lock non-planning joints, apply limits to planning joints
            VectorXd q_lo = plant.GetPositionLowerLimits();
            VectorXd q_hi = plant.GetPositionUpperLimits();
            for (int i = 0; i < plant.num_positions(); ++i)
            {
                bool is_plan_joint = false;
                for (int j = 0; j < kPlanDof; ++j)
                    if (kPlanIndices[j] == i) { is_plan_joint = true; break; }
                if (is_plan_joint)
                    prog->AddBoundingBoxConstraint(q_lo(i), q_hi(i), ik.q()(i));
                else
                    prog->AddBoundingBoxConstraint(
                        q_seed(i) - lock_tol, q_seed(i) + lock_tol, ik.q()(i));
            }

            // Regularization: lumber anchored to seed, arm toward comfort
            VectorXd q_target(kPlanDof);
            Eigen::MatrixXd Q_reg = Eigen::MatrixXd::Zero(kPlanDof, kPlanDof);
            for (int j = 0; j < kPlanDof; ++j)
            {
                if (j < 2)
                {
                    q_target(j) = q_seed(kPlanIndices[j]);
                    Q_reg(j, j) = 10.0;
                }
                else
                {
                    q_target(j) = kComfortPosture[j];
                    Q_reg(j, j) = kJointWeights[j];
                }
            }
            drake::VectorX<drake::symbolic::Variable> plan_vars(kPlanDof);
            for (int j = 0; j < kPlanDof; ++j)
                plan_vars(j) = ik.q()(kPlanIndices[j]);
            prog->AddQuadraticErrorCost(Q_reg, q_target, plan_vars);

            // Collision constraint: ALL robot bodies (self + environment)
            auto checker_ctx = collision_checker_->MakeStandaloneModelContext();
            const double influence_offset = 0.01; // 10mm influence zone
            auto collision_con = std::make_shared<drake::multibody::MinimumDistanceLowerBoundConstraint>(
                collision_checker_.get(), collision_margin,
                checker_ctx.get(), drake::solvers::MinimumValuePenaltyFunction{},
                influence_offset);
            prog->AddConstraint(collision_con, ik.q());

            // Use unconstrained solution as initial guess (much better than q_init)
            prog->SetInitialGuess(ik.q(), q_seed);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "max_iter", 5000);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "tol", 1e-4);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "acceptable_tol", 1e-3);
            prog->SetSolverOption(drake::solvers::IpoptSolver::id(), "acceptable_iter", 5);

            auto result = drake::solvers::Solve(*prog);
            if (result.is_success())
                return result.GetSolution(ik.q());
            return std::nullopt;
        };

        // Try with orientation first, then without
        if (auto res = try_solve(true))
            return *res;
        if (auto res = try_solve(false))
            return *res;
        return q_seed; // Repair failed, return original
    }

    /**
     * SolveIK - Collision-aware IK with validate-then-repair strategy
     *
     * Strategy:
     *   1. Fast unconstrained IK solve (position + orientation)
     *   2. If collision_margin > 0, validate result for collision
     *   3. If in collision, repair with collision-constrained IK (expensive, rare)
     *
     * This ensures:
     *   - Fast for the common case (most points are collision-free)
     *   - Collision constraint only evaluated for the few colliding points
     *   - Entire robot arm checked (all link pairs + all obstacle pairs)
     *
     * @param collision_margin Minimum clearance for ALL robot bodies (meters).
     *                         0.0 = no collision checking (fast, legacy behavior).
     */
    VectorXd SolveIK(const drake::math::RigidTransformd &target_pose,
                     const VectorXd &q_init,
                     double pos_tol = 0.001,
                     double rot_tol = 0.01,
                     double collision_margin = 0.0)
    {
        // Step 1: Fast unconstrained solve with original seed
        VectorXd q_result = SolveIKFast(target_pose, q_init, pos_tol, rot_tol);

        // Step 2: Collision validation (only if margin > 0)
        if (collision_margin > 0.0 && collision_checker_)
        {
            bool in_collision = CheckCollisionUsingChecker(q_result);
            if (!in_collision)
                return q_result; // First try is collision-free — done

            // Step 3: Multi-start random seeds
            // Same Cartesian pose has multiple IK solutions.
            // The first seed (q_init) converged to a colliding solution.
            // Try random seeds to find a collision-free solution.
            std::cout << "  [IK] First seed in collision, trying multi-start (20 seeds)..." << std::endl;

            std::mt19937 rng(42);
            auto &plant = *ik_plant_;
            VectorXd q_lo = plant.GetPositionLowerLimits();
            VectorXd q_hi = plant.GetPositionUpperLimits();

            VectorXd q_best = q_result;
            double best_cost = std::numeric_limits<double>::infinity();
            int free_count = 0;

            for (int attempt = 0; attempt < 20; ++attempt)
            {
                // Random seed: bias toward comfort posture with some noise
                VectorXd q_seed = q_init;
                for (int j = 0; j < kPlanDof; ++j)
                {
                    int idx = kPlanIndices[j];
                    double center = kComfortPosture[j];
                    double range = (q_hi(idx) - q_lo(idx)) * 0.3; // ±30% of range
                    std::uniform_real_distribution<> dist(center - range, center + range);
                    q_seed(idx) = std::max(q_lo(idx), std::min(q_hi(idx), dist(rng)));
                }

                VectorXd q_try = SolveIKFast(target_pose, q_seed, pos_tol, rot_tol);

                // Verify IK accuracy
                auto T_ik = ComputeEEPose(q_try);
                double pos_err = (T_ik.translation() - target_pose.translation()).norm();
                if (pos_err > pos_tol * 5)
                    continue; // IK didn't converge

                if (!CheckCollisionUsingChecker(q_try))
                {
                    free_count++;
                    double cost = ComfortCost(q_try);
                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        q_best = q_try;
                    }
                }
            }

            if (free_count > 0)
            {
                std::cout << "  [IK] Multi-start: " << free_count
                          << "/20 collision-free, best comfort cost: " << best_cost << std::endl;
                return q_best;
            }

            // Step 4: All random seeds failed — try collision-constrained repair
            std::cout << "  [IK] All random seeds in collision, trying collision-constrained repair..." << std::endl;
            q_result = SolveIKWithCollision(
                target_pose, q_result, pos_tol, rot_tol, collision_margin);
        }

        return q_result;
    }

    /**
     * PlanMoveJ - Joint-space motion using Drake KinematicTrajectoryOptimization
     * B-spline trajectory with convex hull property guarantees constraint satisfaction.
     */

    // ========== APF + Informed RRT* + CCD Planner ==========

    // Planning space: lumber_joint2,3 (torso) + right_arm (9 DOF, non-contiguous)
    // q[1]=lumber_joint2, q[2]=lumber_joint3, q[12-18]=right_arm_joint1-7
    static constexpr int kPlanDof = 9;
    static constexpr int kPlanIndices[kPlanDof] = {1, 2, 12, 13, 14, 15, 16, 17, 18};

    // ========================================================================
    // Human-like IK: comfort posture + differential joint weights
    //
    // The IK regularizes toward a "comfort" posture with per-joint weights:
    //   - Lumber joints: high weight → torso moves less (humans prefer arm-first)
    //   - Shoulder/elbow: moderate weight → smooth, natural arm motion
    //   - Wrist joints: low weight → wrist is flexible, moves freely
    //
    // This produces the most anthropomorphic IK solutions.
    // ========================================================================

    // Comfort posture for planning joints (lumber2, lumber3, arm1..7)
    // Represents a relaxed "ready" position: torso centered, arm slightly bent
    //                              lumber2  lumber3  arm1   arm2   arm3  arm4   arm5  arm6  arm7
    static constexpr double kComfortPosture[kPlanDof] = {0.0,     0.0,     0.0,   -0.5,  0.0,  -1.2,  0.0,  0.0,  0.0};

    // Joint weights: higher = stronger pull toward comfort posture
    // Lumber (3.0): discourage torso motion
    // Shoulder (1.5): moderate — needs to move for reaching
    // Elbow (1.0): moderate
    // Wrist (0.3): flexible — let it move freely
    //                              lumber2 lumber3 arm1  arm2  arm3 arm4 arm5 arm6 arm7
    static constexpr double kJointWeights[kPlanDof] = {3.0,    3.0,    1.5,  1.5,  1.0, 1.0, 0.3, 0.3, 0.3};

    // Build weighted Q matrix for comfort regularization
    static Eigen::MatrixXd BuildComfortQ(double scale = 1.0)
    {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kPlanDof, kPlanDof);
        for (int i = 0; i < kPlanDof; ++i)
            Q(i, i) = scale * kJointWeights[i];
        return Q;
    }

    static VectorXd GetComfortPosture()
    {
        VectorXd q(kPlanDof);
        for (int i = 0; i < kPlanDof; ++i)
            q(i) = kComfortPosture[i];
        return q;
    }

    // Score a configuration by how "comfortable" it is (lower = better)
    static double ComfortCost(const VectorXd &q_full)
    {
        double cost = 0.0;
        for (int i = 0; i < kPlanDof; ++i)
        {
            double diff = q_full(kPlanIndices[i]) - kComfortPosture[i];
            cost += kJointWeights[i] * diff * diff;
        }
        return cost;
    }

    static VectorXd PlanToFull(const VectorXd &q_plan, const VectorXd &q_template)
    {
        VectorXd q_full = q_template;
        for (int i = 0; i < kPlanDof; ++i)
            q_full(kPlanIndices[i]) = q_plan(i);
        return q_full;
    }

    static VectorXd FullToPlan(const VectorXd &q_full)
    {
        VectorXd q_plan(kPlanDof);
        for (int i = 0; i < kPlanDof; ++i)
            q_plan(i) = q_full(kPlanIndices[i]);
        return q_plan;
    }

    static VectorXd ExtractPlanLimits(const VectorXd &limits)
    {
        VectorXd plan_limits(kPlanDof);
        for (int i = 0; i < kPlanDof; ++i)
            plan_limits(i) = limits(kPlanIndices[i]);
        return plan_limits;
    }

    bool IsCollisionFreePlan(const VectorXd &q_plan, const VectorXd &q_fixed) const
    {
        if (!collision_checker_)
            return false;
        return collision_checker_->CheckConfigCollisionFree(PlanToFull(q_plan, q_fixed));
    }

    bool IsEdgeCollisionFreeSearch(const VectorXd &q_from, const VectorXd &q_to,
                                   const VectorXd &q_fixed) const
    {
        if (!collision_checker_)
            return false;
        return collision_checker_->CheckEdgeCollisionFree(
            PlanToFull(q_from, q_fixed), PlanToFull(q_to, q_fixed));
    }

    VectorXd ComputeAPFBias(const VectorXd &q_plan, const VectorXd &q_goal,
                            const VectorXd &q_fixed, double influence_dist = 0.3) const
    {
        VectorXd bias = VectorXd::Zero(kPlanDof);

        // Attractive potential: toward goal (applies to ALL planning DOFs)
        VectorXd diff = q_goal - q_plan;
        double dist_to_goal = diff.norm();
        if (dist_to_goal > 0.01)
            bias += 0.3 * diff / dist_to_goal;

        // Repulsive potential: proximity-based activation.
        // Two-tier cost strategy:
        //   1. Cheap: CheckConfigCollisionFree + CalcRobotClearance on q_plan
        //   2. Expensive: finite-difference gradient (only if min_d < threshold)
        //
        // Only arm joints (indices 2-8) get repulsive forces.
        // kPlanIndices = {1,2, 12,13,...,18}
        // Planning indices 0,1 = lumber; indices 2-8 = arm

        VectorXd q_full = PlanToFull(q_plan, q_fixed);

        // Tier 1: cheap clearance check on current config
        const double safety_threshold = 0.10; // meters — activate repulsive force
        auto clearance = collision_checker_->CalcRobotClearance(q_full, influence_dist);
        double min_d = influence_dist;
        for (int i = 0; i < clearance.size(); ++i)
            min_d = std::min(min_d, clearance.distances()(i));

        if (min_d > safety_threshold)
            return bias; // far enough from obstacles — attractive only

        // Tier 2: compute repulsive gradient via finite differences.
        // Only compute for arm joints that are close to obstacles
        // (skip joints where both perturbations are clearly safe).
        const double eps = 0.04;
        constexpr int kArmPlanStart = 2; // arm joints start at planning index 2
        for (int j = kArmPlanStart; j < kPlanDof; ++j)
        {
            VectorXd q_plus = q_plan, q_minus = q_plan;
            q_plus(j) += eps;
            q_minus(j) -= eps;

            VectorXd qf_p = PlanToFull(q_plus, q_fixed);
            VectorXd qf_m = PlanToFull(q_minus, q_fixed);

            // Fast path: use binary collision check first
            bool p_free = collision_checker_->CheckConfigCollisionFree(qf_p);
            bool m_free = collision_checker_->CheckConfigCollisionFree(qf_m);

            // If both perturbations are collision-free AND we're not too close,
            // approximate gradient as zero (skip expensive CalcRobotClearance)
            if (p_free && m_free && min_d > safety_threshold * 0.5)
                continue;

            // Expensive path: compute actual clearance for gradient
            double d_p, d_m;
            if (p_free)
                d_p = min_d + eps; // approximate: safely far
            else
            {
                auto cl_p = collision_checker_->CalcRobotClearance(qf_p, influence_dist);
                d_p = influence_dist;
                for (int i = 0; i < cl_p.size(); ++i)
                    d_p = std::min(d_p, cl_p.distances()(i));
            }

            if (m_free)
                d_m = min_d + eps;
            else
            {
                auto cl_m = collision_checker_->CalcRobotClearance(qf_m, influence_dist);
                d_m = influence_dist;
                for (int i = 0; i < cl_m.size(); ++i)
                    d_m = std::min(d_m, cl_m.distances()(i));
            }

            double grad_j = (d_p - d_m) / (2 * eps);
            if (min_d > 1e-6)
                bias(j) += 0.5 * grad_j / (min_d * min_d);
        }

        return bias;
    }

    struct RRTNode
    {
        VectorXd q;
        int parent;
        double cost;
    };

    // Weighted distance: penalize lumber joints more to discourage torso motion
    // kPlanIndices = {1,2, 12,13,14,15,16,17,18} — first 2 are lumber joints
    static double WeightedDist(const VectorXd &q1, const VectorXd &q2)
    {
        // Weights: lumber_joint2,3 get 3x penalty, arm joints get 1x
        const double w_lumber = 3.0;
        double d = 0.0;
        for (int i = 0; i < kPlanDof; ++i)
        {
            double w = (i < 2) ? w_lumber : 1.0;
            double diff = q1(i) - q2(i);
            d += w * w * diff * diff;
        }
        return std::sqrt(d);
    }


    std::vector<VectorXd> SmoothPath(const std::vector<VectorXd> &path,
                                     const VectorXd &q_fixed,
                                     int iterations = 200) const
    {
        if (path.size() <= 2)
            return path;
        std::vector<VectorXd> result = path;
        std::mt19937 rng(123);

        // OPTIMIZATION: Use moderate edge_step_size during shortcutting for speed
        // The final CCD validation will catch any missed collisions
        const double original_step_size = collision_checker_->edge_step_size();
        const double shortcut_step = 0.03; // Moderate: balances speed and safety
        collision_checker_->set_edge_step_size(shortcut_step);

        // Phase 1: Shortcut smoothing — remove unnecessary waypoints
        for (int iter = 0; iter < iterations; ++iter)
        {
            if (result.size() <= 2)
                break;
            std::uniform_int_distribution<> dist(0, result.size() - 1);
            int i = dist(rng), j = dist(rng);
            if (i > j)
                std::swap(i, j);
            if (j - i <= 1)
                continue;

            if (IsEdgeCollisionFreeSearch(result[i], result[j], q_fixed))
                result.erase(result.begin() + i + 1, result.begin() + j);
        }

        // Restore original edge_step_size
        collision_checker_->set_edge_step_size(original_step_size);

        // Phase 2: Densify — subdivide long segments so the cubic spline
        // has enough control points for smooth EE motion.
        // Points on a collision-free edge remain collision-free.
        const int min_waypoints = 15;
        while ((int)result.size() < min_waypoints)
        {
            // Find longest segment
            int longest = 1;
            double max_len = (result[1] - result[0]).norm();
            for (int i = 2; i < (int)result.size(); ++i)
            {
                double len = (result[i] - result[i - 1]).norm();
                if (len > max_len)
                {
                    max_len = len;
                    longest = i;
                }
            }
            if (max_len < 0.01)
                break;
            VectorXd mid = (result[longest - 1] + result[longest]) * 0.5;
            result.insert(result.begin() + longest, mid);
        }

        return result;
    }

    // Bidirectional RRT-Connect: grows two trees simultaneously.
    // tree_s from start (attracted to goal), tree_g from goal (attracted to start).
    // Each iteration: extend tree_s toward random sample, then extend tree_g
    // toward tree_s's new node.  If the two new nodes are close enough and
    // the connecting edge is collision-free, extract a complete path.
    //
    // This solves the narrow-passage problem that defeats single-tree RRT*:
    // the goal-side tree explores the goal's C-space neighborhood and
    // connects from the "other side" of the obstacle.
    std::vector<VectorXd> RunBidirectionalRRT(
        const VectorXd &q_start, const VectorXd &q_goal,
        const VectorXd &q_min, const VectorXd &q_max,
        const VectorXd &q_fixed,
        int max_iter = 4000, double step_size = 0.3) const
    {
        const int dim = q_start.size();
        std::vector<RRTNode> tree_s, tree_g;
        tree_s.push_back({q_start, -1, 0.0});
        tree_g.push_back({q_goal, -1, 0.0});

        std::mt19937 rng(42);
        std::uniform_real_distribution<> unit(0.0, 1.0);
        std::vector<std::uniform_real_distribution<>> jdist;
        for (int j = 0; j < dim; ++j)
            jdist.emplace_back(q_min(j), q_max(j));

        const double connect_tol = step_size * 1.5;

        // Adaptive edge step size
        const double coarse_edge_step = 0.06;
        const double fine_edge_step = 0.03;
        const int coarse_iters = static_cast<int>(max_iter * 0.7);

        auto find_nearest = [&](const std::vector<RRTNode> &tree,
                                const VectorXd &q) -> int
        {
            int best = 0;
            double best_d = std::numeric_limits<double>::infinity();
            for (int i = 0; i < (int)tree.size(); ++i)
            {
                double d = (tree[i].q - q).squaredNorm();
                if (d < best_d)
                {
                    best_d = d;
                    best = i;
                }
            }
            return best;
        };

        // Extend tree toward q_target.
        // When use_apf=true, adds APF obstacle-avoidance bias (expensive).
        // Returns index of new node, or -1 if edge is in collision.
        auto extend = [&](std::vector<RRTNode> &tree,
                          const VectorXd &q_target,
                          const VectorXd &q_attract,
                          bool use_apf) -> int
        {
            int near_idx = find_nearest(tree, q_target);
            VectorXd direction = q_target - tree[near_idx].q;
            VectorXd combined = direction; // default: direct steering
            if (use_apf)
            {
                VectorXd apf = ComputeAPFBias(tree[near_idx].q, q_attract, q_fixed);
                combined = 0.6 * direction + 0.4 * apf * step_size;
            }
            double comb_norm = combined.norm();
            if (comb_norm > step_size)
                combined = combined / comb_norm * step_size;

            VectorXd q_new = tree[near_idx].q + combined;
            q_new = q_new.cwiseMax(q_min).cwiseMin(q_max);

            if (!IsEdgeCollisionFreeSearch(tree[near_idx].q, q_new, q_fixed))
                return -1;

            int new_idx = tree.size();
            tree.push_back({q_new, near_idx, 0.0});
            return new_idx;
        };

        for (int iter = 0; iter < max_iter; ++iter)
        {
            // Adaptive edge step size
            if (collision_checker_)
            {
                double desired = (iter < coarse_iters) ? coarse_edge_step : fine_edge_step;
                if (collision_checker_->edge_step_size() != desired)
                    collision_checker_->set_edge_step_size(desired);
            }

            // Sample with bridge sampling
            VectorXd q_rand(dim);
            double roll = unit(rng);
            if (roll < 0.2)
            {
                // Bridge sampling
                VectorXd q_s1(dim), q_s2(dim);
                for (int j = 0; j < dim; ++j)
                {
                    q_s1(j) = jdist[j](rng);
                    q_s2(j) = jdist[j](rng);
                }
                bool s1_free = IsCollisionFreePlan(q_s1, q_fixed);
                bool s2_free = IsCollisionFreePlan(q_s2, q_fixed);
                if (s1_free != s2_free)
                    q_rand = (q_s1 + q_s2) * 0.5;
                else
                    for (int j = 0; j < dim; ++j)
                        q_rand(j) = jdist[j](rng);
            }
            else
            {
                for (int j = 0; j < dim; ++j)
                    q_rand(j) = jdist[j](rng);
            }

            // Extend start tree toward random sample (with APF — obstacle avoidance)
            int idx_a = extend(tree_s, q_rand, q_goal, true);
            if (idx_a < 0)
                continue;

            // Extend goal tree toward start tree's new node (no APF — direction already targeted)
            const VectorXd &q_new_a = tree_s[idx_a].q;
            int idx_b = extend(tree_g, q_new_a, q_start, false);
            if (idx_b < 0)
                continue;

            // Check if the two new nodes can be connected
            const VectorXd &q_new_b = tree_g[idx_b].q;
            double gap = (q_new_a - q_new_b).norm();
            if (gap < connect_tol &&
                IsEdgeCollisionFreeSearch(q_new_a, q_new_b, q_fixed))
            {
                // Connected!  Build path: start → … → q_new_a → q_new_b → … → goal

                // ============================================================
                // Post-connect rewiring: optimize path cost via RRT* style
                // neighbor rewiring on BOTH trees.  This removes unnecessary
                // detours that the greedy extend produced, yielding shorter,
                // smoother paths.
                // ============================================================

                // Compute costs for both trees (distance from root)
                auto compute_tree_costs = [](std::vector<RRTNode> &tree)
                {
                    tree[0].cost = 0.0;
                    for (int i = 1; i < (int)tree.size(); ++i)
                    {
                        int p = tree[i].parent;
                        if (p >= 0)
                            tree[i].cost = tree[p].cost + WeightedDist(tree[p].q, tree[i].q);
                        else
                            tree[i].cost = 0.0;
                    }
                };
                compute_tree_costs(tree_s);
                compute_tree_costs(tree_g);

                // Rewire each tree: for each node, try to find a lower-cost
                // parent among nearby nodes (within rewire_radius).
                const double rewire_radius = step_size * 2.5;

                auto rewire_tree = [&](std::vector<RRTNode> &tree)
                {
                    int rewire_count = 0;
                    for (int i = 1; i < (int)tree.size(); ++i)
                    {
                        int best_parent = tree[i].parent;
                        double best_cost = tree[i].cost;

                        // Search nearby nodes for a better parent
                        for (int j = 0; j < (int)tree.size(); ++j)
                        {
                            if (j == i || j == tree[i].parent)
                                continue;
                            double d = WeightedDist(tree[j].q, tree[i].q);
                            if (d > rewire_radius)
                                continue;

                            double new_cost = tree[j].cost + d;
                            if (new_cost < best_cost - 1e-6)
                            {
                                if (IsEdgeCollisionFreeSearch(tree[j].q, tree[i].q, q_fixed))
                                {
                                    best_parent = j;
                                    best_cost = new_cost;
                                }
                            }
                        }

                        if (best_parent != tree[i].parent)
                        {
                            tree[i].parent = best_parent;
                            tree[i].cost = best_cost;
                            rewire_count++;

                            // Propagate cost reduction to descendants
                            for (int k = i + 1; k < (int)tree.size(); ++k)
                            {
                                if (tree[k].parent == i)
                                {
                                    int p = tree[k].parent;
                                    tree[k].cost = tree[p].cost +
                                                   WeightedDist(tree[p].q, tree[k].q);
                                }
                            }
                        }
                    }
                    return rewire_count;
                };

                int rewires_s = rewire_tree(tree_s);
                int rewires_g = rewire_tree(tree_g);

                // Extract optimized path
                std::vector<VectorXd> path;

                // Start side (root → q_new_a)
                int idx = idx_a;
                while (idx >= 0)
                {
                    path.push_back(tree_s[idx].q);
                    idx = tree_s[idx].parent;
                }
                std::reverse(path.begin(), path.end());

                // Goal side (q_new_b → root)
                idx = idx_b;
                while (idx >= 0)
                {
                    path.push_back(tree_g[idx].q);
                    idx = tree_g[idx].parent;
                }

                // Compute final path cost
                double path_cost = 0.0;
                for (size_t i = 1; i < path.size(); ++i)
                    path_cost += WeightedDist(path[i - 1], path[i]);

                std::cout << "  [RRT-Connect] Connected at iter " << iter
                          << ", trees: " << tree_s.size() << "+" << tree_g.size()
                          << ", gap: " << gap
                          << ", rewires: " << rewires_s << "+" << rewires_g
                          << ", path_cost: " << path_cost << std::endl;
                return path;
            }

            // Progress report
            if (iter % 1000 == 0 && iter > 0)
            {
                // Estimate closest pair between trees (sample recent nodes)
                double min_gap = std::numeric_limits<double>::infinity();
                int s_begin = std::max(0, (int)tree_s.size() - 80);
                int g_begin = std::max(0, (int)tree_g.size() - 80);
                for (int i = s_begin; i < (int)tree_s.size(); ++i)
                    for (int j = g_begin; j < (int)tree_g.size(); ++j)
                    {
                        double d = (tree_s[i].q - tree_g[j].q).norm();
                        if (d < min_gap)
                            min_gap = d;
                    }
                std::cout << "  [RRT-Connect] iter " << iter
                          << ", trees: " << tree_s.size() << "+" << tree_g.size()
                          << ", min_gap: " << min_gap << std::endl;
            }
        }

        return {};
    }

    // ========================================================================
    // Toppra-guided smooth timing
    //
    // Uses Toppra to find the time-optimal duration T_opt for the path,
    // then builds a C∞ smooth 7th-order polynomial s(t) with that duration.
    // This gives the SPEED of Toppra with the SMOOTHNESS of a bell-curve
    // velocity profile — no acceleration waves from greedy speed-up/slow-down.
    //
    // The C∞ polynomial shape: s(τ) = L·(35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷)
    // has peak velocity = (35/16)·L/T ≈ 2.19·L/T at τ=0.5.
    // To respect joint velocity limits, we need T ≥ (35/16)·L / s_vel_max.
    // We sample the path's q'(s) to find s_vel_max and s_acc_max locally,
    // then take the max of T_toppra and the C∞ constraint durations.
    // ========================================================================
    std::unique_ptr<drake::trajectories::PathParameterizedTrajectory<double>>
    BuildToppraTrajectory(
        const drake::trajectories::Trajectory<double> &path,
        double total_arc_length,
        double max_vel, double max_acc) const
    {
        auto &plant = robot_diagram_->plant();
        const int nq = plant.num_positions();

        // ====================================================================
        // Step 1: Use Toppra to find the time-optimal duration
        // ====================================================================
        drake::multibody::CalcGridPointsOptions grid_opts;
        grid_opts.max_err = 1e-3;
        grid_opts.max_seg_length = 0.05;
        grid_opts.min_points = 50;
        Eigen::VectorXd gridpoints =
            drake::multibody::Toppra::CalcGridPoints(path, grid_opts);

        std::cout << "  [Toppra] Grid points: " << gridpoints.size() << std::endl;

        drake::multibody::Toppra toppra(path, plant, gridpoints);

        VectorXd v_lower = VectorXd::Constant(nq, -max_vel);
        VectorXd v_upper = VectorXd::Constant(nq, max_vel);
        toppra.AddJointVelocityLimit(v_lower, v_upper);

        VectorXd a_lower = VectorXd::Constant(nq, -max_acc);
        VectorXd a_upper = VectorXd::Constant(nq, max_acc);
        toppra.AddJointAccelerationLimit(a_lower, a_upper);

        auto s_of_t = toppra.SolvePathParameterization(0.0, 0.0);

        if (!s_of_t.has_value())
        {
            std::cerr << "  [Toppra] SolvePathParameterization FAILED!" << std::endl;
            return nullptr;
        }

        double T_toppra = s_of_t->end_time();
        std::cout << "  [Toppra] Time-optimal duration: " << T_toppra << " s" << std::endl;

        // ====================================================================
        // Step 2: Compute C∞ polynomial constraint durations from path q'(s)
        //
        // Sample the path derivative at many points to find the maximum |q'(s)|
        // and |q''(s)|.  Then compute the minimum T for the C∞ polynomial:
        //   T_vel = (35/16) · L / min_j(max_vel / |q'_j|)   (velocity)
        //   T_acc = sqrt(7.508 · L / min_j(remaining / |q'_j|))  (acceleration)
        // ====================================================================
        auto path_d1 = path.MakeDerivative(1);
        auto path_d2 = path_d1->MakeDerivative(1);

        const int kSampleCount = 500;
        double s_start = path.start_time();
        double s_end = path.end_time();

        // Find per-joint global max of |q'| and |q''|
        VectorXd max_dqds = VectorXd::Zero(nq);
        VectorXd max_d2qds2 = VectorXd::Zero(nq);
        for (int i = 0; i <= kSampleCount; ++i)
        {
            double s = s_start + (s_end - s_start) * i / kSampleCount;
            VectorXd dq = path_d1->value(s);
            VectorXd ddq = path_d2->value(s);
            for (int j = 0; j < nq; ++j)
            {
                max_dqds(j) = std::max(max_dqds(j), std::abs(dq(j)));
                max_d2qds2(j) = std::max(max_d2qds2(j), std::abs(ddq(j)));
            }
        }

        // C∞ polynomial velocity constraint: peak ṡ = (35/16)·L/T ≤ s_vel_max
        double s_vel_max = std::numeric_limits<double>::infinity();
        for (int j = 0; j < nq; ++j)
            if (max_dqds(j) > 1e-9)
                s_vel_max = std::min(s_vel_max, max_vel / max_dqds(j));
        if (!std::isfinite(s_vel_max))
            s_vel_max = 1.0;

        double T_vel = (35.0 / 16.0) * total_arc_length / s_vel_max;

        // C∞ polynomial acceleration constraint: peak s̈ ≈ 7.508·L/T²
        // Must satisfy |q''|·ṡ² + |q'|·s̈ ≤ max_acc at peak velocity
        // ṡ_peak = (35/16)·L/T, s̈_peak ≈ 7.508·L/T²
        // For each joint: |q''|·ṡ² + |q'|·s̈ ≤ max_acc
        double T_acc = 0.0;
        for (int j = 0; j < nq; ++j)
        {
            if (max_dqds(j) < 1e-9)
                continue;
            // Solve: |q''|·(35/16·L/T)² + |q'|·7.508·L/T² ≤ max_acc
            // = L/T² · (|q''|·(35/16)²·L + |q'|·7.508) ≤ max_acc
            double coeff = max_d2qds2(j) * (35.0 / 16.0) * (35.0 / 16.0) * total_arc_length + max_dqds(j) * 7.508;
            if (coeff > 1e-9)
                T_acc = std::max(T_acc, std::sqrt(total_arc_length * coeff / max_acc));
        }
        if (T_acc < 0.1)
            T_acc = 0.1;

        // Final duration: max of Toppra optimal and C∞ polynomial constraints
        double T_final = std::max({T_toppra, T_vel, T_acc});

        std::cout << "  [Timing] T_toppra: " << T_toppra
                  << ", T_vel: " << T_vel
                  << ", T_acc: " << T_acc
                  << " → T_final: " << T_final << " s (smooth)" << std::endl;

        // ====================================================================
        // Step 3: Build C∞ smooth polynomial s(t) with T_final
        // ====================================================================
        auto smooth_timing = BuildSmoothTiming(total_arc_length, T_final);

        return std::make_unique<drake::trajectories::PathParameterizedTrajectory<double>>(
            path, smooth_timing);
    }

    // APF + Informed RRT* + CCD MoveJ planner

    // Resolve cache path relative to the executable's project root

    // Bidirectional RRT-Connect + APF + CCD MoveJ planner
    // Uses Toppra for time-optimal path parameterization
    std::unique_ptr<drake::trajectories::Trajectory<double>>
    PlanMoveJ(const VectorXd &q_start, const VectorXd &q_goal,
              double max_vel = 1.0, double max_acc = 2.0, int = 10)
    {
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "[PlanMoveJ] RRT-Connect + S-curve timing" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        auto t_start = std::chrono::high_resolution_clock::now();
        const double original_step_size = collision_checker_->edge_step_size();

        try
        {
            auto &plant = robot_diagram_->plant();
            const int nq = plant.num_positions();

            VectorXd q_plan_start = FullToPlan(q_start);
            VectorXd q_plan_goal = FullToPlan(q_goal);
            VectorXd q_fixed = q_start;

            VectorXd plan_lower = ExtractPlanLimits(plant.GetPositionLowerLimits());
            VectorXd plan_upper = ExtractPlanLimits(plant.GetPositionUpperLimits());

            const double kLumberRange = 0.2;
            for (int i = 0; i < 2; ++i)
            {
                double lo = std::min(q_plan_start(i), q_plan_goal(i)) - kLumberRange;
                double hi = std::max(q_plan_start(i), q_plan_goal(i)) + kLumberRange;
                plan_lower(i) = std::max(plan_lower(i), lo);
                plan_upper(i) = std::min(plan_upper(i), hi);
            }

            // Validate start and goal are collision-free
            if (!IsCollisionFreePlan(q_plan_start, q_fixed))
            {
                std::cerr << "  [RRT*] Start configuration is in collision!" << std::endl;
                return nullptr;
            }
            if (!IsCollisionFreePlan(q_plan_goal, q_fixed))
            {
                std::cerr << "  [RRT*] Goal configuration is in collision!" << std::endl;
                return nullptr;
            }

            // Phase 1: Bidirectional RRT-Connect (primary planner)
            std::cout << "  [RRT-Connect] Running bidirectional RRT in " << kPlanDof << "-DOF..." << std::endl;
            auto path = RunBidirectionalRRT(q_plan_start, q_plan_goal,
                                            plan_lower, plan_upper, q_fixed,
                                            4000, 0.3);


            if (path.empty())
            {
                std::cerr << "  [ERROR] All planners failed" << std::endl;
                collision_checker_->set_edge_step_size(original_step_size);
                return nullptr;
            }

            std::cout << "  [RRT*] Raw path: " << path.size() << " waypoints" << std::endl;

            // ====================================================================
            // Trajectory: Catmull-Rom + dense cubic spline + exact S-curve timing
            // ====================================================================
            // Path: Catmull-Rom interpolates through RRT waypoints, producing
            //       smooth curves that pass through all waypoints
            // Timing: Exact 7-phase S-curve polynomial (no resampling approximation)
            // ====================================================================

            // Step 1: Shortcut smoothing + path optimization
            auto smoothed = SmoothPath(path, q_fixed, 30);
            std::cout << "  [Path] Smoothed: " << smoothed.size() << " waypoints" << std::endl;

            // OPTIMIZATION: Use moderate step size for speed, final CCD validates safety
            const double opt_step_size = collision_checker_->edge_step_size();
            const double opt_edge_step = 0.03; // Moderate step for optimization
            collision_checker_->set_edge_step_size(opt_edge_step);

            // CRITICAL FIX: Collect modifications in buffer first, then apply
            // This avoids using already-modified waypoints in subsequent checks
            // OPTIMIZATION: Use conservative step size (0.05 rad) and fewer iterations
            const double max_opt_step = 0.05; // Conservative step per iteration
            const int max_opt_iters = 40;     // Reduced from 80 for speed

            for (int opt = 0; opt < max_opt_iters; ++opt)
            {
                std::vector<VectorXd> smoothed_next = smoothed; // Buffer for next iteration
                bool any_change = false;

                for (int i = 1; i < (int)smoothed.size() - 1; ++i)
                {
                    VectorXd mid = (smoothed[i - 1] + smoothed[i + 1]) * 0.5;
                    VectorXd diff = mid - smoothed[i];
                    double step = std::min(max_opt_step, diff.norm());
                    if (step < 1e-6)
                        continue;
                    VectorXd q_new = smoothed[i] + step * diff.normalized();
                    if (IsEdgeCollisionFreeSearch(smoothed[i - 1], q_new, q_fixed) &&
                        IsEdgeCollisionFreeSearch(q_new, smoothed[i + 1], q_fixed))
                    {
                        smoothed_next[i] = q_new;
                        any_change = true;
                    }
                }

                smoothed = smoothed_next;
                if (!any_change)
                    break; // Converged
            }

            collision_checker_->set_edge_step_size(opt_step_size); // Restore original

            smoothed = SmoothPath(smoothed, q_fixed, 20);
            std::cout << "  [Path] Optimized: " << smoothed.size() << " waypoints" << std::endl;

            // ====================================================================
            // Path: C2 cubic spline (guaranteed continuous velocity & acceleration)
            // Timing: Toppra-guided C∞ smooth polynomial
            // ====================================================================

            const int num_wp = smoothed.size();

            // Convert waypoints to full 19-DOF
            MatrixXd waypoints_19(nq, num_wp);
            for (int i = 0; i < num_wp; ++i)
                waypoints_19.col(i) = PlanToFull(smoothed[i], q_fixed);

            // Cumulative arc length
            VectorXd arc_length(num_wp);
            arc_length(0) = 0.0;
            for (int i = 1; i < num_wp; ++i)
                arc_length(i) = arc_length(i - 1) +
                                (waypoints_19.col(i) - waypoints_19.col(i - 1)).norm();
            double total_arc_length = arc_length(num_wp - 1);

            if (total_arc_length < 1e-9)
            {
                return std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
                    drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
                        std::vector<double>{0.0, 0.001},
                        std::vector<MatrixXd>{q_start, q_start}));
            }

            // C2 cubic spline: q(s) where s ∈ [0, total_arc_length]
            auto path_spline =
                drake::trajectories::PiecewisePolynomial<double>::
                    CubicWithContinuousSecondDerivatives(arc_length, waypoints_19);

            // ====================================================================
            // Toppra-guided smooth timing
            // Uses Toppra to find optimal duration, then applies C∞ polynomial
            // for smooth (wave-free) velocity profile.
            // ====================================================================
            std::cout << "  [Timing] Total arc: " << total_arc_length
                      << ", max_vel: " << max_vel
                      << ", max_acc: " << max_acc
                      << " (Toppra time-optimal)" << std::endl;

            auto trajectory = BuildToppraTrajectory(
                path_spline, total_arc_length, max_vel, max_acc);

            if (!trajectory)
            {
                std::cerr << "  [Toppra] Failed, falling back to smooth polynomial timing" << std::endl;
                auto path_d1 = path_spline.derivative(1);
                double max_path_vel_norm = 0.0;
                for (int i = 0; i <= 200; ++i)
                {
                    double s = total_arc_length * i / 200;
                    s = std::max(path_spline.start_time(),
                                 std::min(path_spline.end_time(), s));
                    max_path_vel_norm = std::max(max_path_vel_norm,
                                                 path_d1.value(s).norm());
                }
                max_path_vel_norm = std::max(max_path_vel_norm, 1e-6);
                double s_vel_max = max_vel / max_path_vel_norm;
                double s_acc_max = max_acc / max_path_vel_norm;
                double T_smooth = ComputeSmoothTimingDuration(
                    total_arc_length, s_vel_max, s_acc_max);
                auto s_curve_timing = BuildSmoothTiming(total_arc_length, T_smooth);
                trajectory =
                    std::make_unique<drake::trajectories::PathParameterizedTrajectory<double>>(
                        path_spline, s_curve_timing);
            }

            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(t_end - t_start).count();

            std::cout << "  [Trajectory] Duration: " << trajectory->end_time()
                      << " s, planning time: " << elapsed << " s" << std::endl;

            // Restore fine edge_step_size for thorough final CCD validation
            collision_checker_->set_edge_step_size(0.005);

            // CCD validation on final trajectory
            std::cout << "  [CCD] Validating trajectory with fine step (0.005 rad)..." << std::endl;
            bool collision_free = ValidateTrajectoryComplete(*trajectory, 0.005);
            if (!collision_free)
            {
                std::cerr << "  [CCD] Trajectory has collisions, returning nullptr" << std::endl;
                collision_checker_->set_edge_step_size(original_step_size);
                return nullptr;
            }

            collision_checker_->set_edge_step_size(original_step_size);
            std::cout << std::string(80, '=') << std::endl;
            return trajectory;
        }
        catch (const std::exception &e)
        {
            std::cerr << "  [RRT*] Exception: " << e.what() << std::endl;
            collision_checker_->set_edge_step_size(original_step_size);
            return nullptr;
        }
    }

    /**
     * PlanMoveJFromCartesian - Plan MoveJ from task-space start/goal poses.
     *
     * Flow:
     *   1. Solve collision-free IK for start pose → q_start_solved
     *   2. Solve collision-free IK for goal pose  → q_goal_solved
     *   3. If q_current ≠ q_start_solved, plan intermediate MoveJ to start
     *   4. Plan MoveJ from q_start_solved to q_goal_solved
     *
     * @param q_current   Current joint configuration (for IK seed)
     * @param start_xyz   Start position in base frame (meters)
     * @param start_rpy   Start orientation as Roll-Pitch-Yaw (radians)
     * @param goal_xyz    Goal position in base frame (meters)
     * @param goal_rpy    Goal orientation as Roll-Pitch-Yaw (radians)
     */
    std::unique_ptr<drake::trajectories::Trajectory<double>>
    PlanMoveJFromCartesian(
        const VectorXd &q_current,
        const Eigen::Vector3d &start_xyz, const Eigen::Vector3d &start_rpy,
        const Eigen::Vector3d &goal_xyz, const Eigen::Vector3d &goal_rpy,
        double max_vel = 1.0, double max_acc = 2.0,
        double pos_tol = 0.002, double rot_tol = 0.02,
        double collision_margin = 0.01)
    {
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "[PlanMoveJFromCartesian] Task-space → IK → MoveJ" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Build start pose
        drake::math::RigidTransformd start_pose(
            drake::math::RollPitchYawd(start_rpy(0), start_rpy(1), start_rpy(2)),
            start_xyz);

        // Build goal pose
        drake::math::RigidTransformd goal_pose(
            drake::math::RollPitchYawd(goal_rpy(0), goal_rpy(1), goal_rpy(2)),
            goal_xyz);

        // ---- Solve IK for start ----
        std::cout << "  [IK] Solving start: xyz=(" << start_xyz.transpose()
                  << "), rpy=(" << start_rpy.transpose() << ")" << std::endl;
        VectorXd q_start_solved = SolveIK(start_pose, q_current, pos_tol, rot_tol, collision_margin);

        {
            auto T_ik = ComputeEEPose(q_start_solved);
            double err = (T_ik.translation() - start_xyz).norm();
            std::cout << "  [IK] Start pos error: " << (err * 1000.0) << " mm" << std::endl;
            if (err > pos_tol * 5)
            {
                std::cerr << "  [ERROR] Start IK failed to converge (" << err << " m)" << std::endl;
                return nullptr;
            }
            if (CheckCollisionUsingChecker(q_start_solved))
            {
                std::cerr << "  [ERROR] Start configuration is in collision!" << std::endl;
                return nullptr;
            }
        }

        // ---- Solve IK for goal (using start solution as seed) ----
        std::cout << "  [IK] Solving goal:  xyz=(" << goal_xyz.transpose()
                  << "), rpy=(" << goal_rpy.transpose() << ")" << std::endl;
        VectorXd q_goal_solved = SolveIK(goal_pose, q_start_solved, pos_tol, rot_tol, collision_margin);

        {
            auto T_ik = ComputeEEPose(q_goal_solved);
            double err = (T_ik.translation() - goal_xyz).norm();
            std::cout << "  [IK] Goal pos error: " << (err * 1000.0) << " mm" << std::endl;
            if (err > pos_tol * 5)
            {
                std::cerr << "  [ERROR] Goal IK failed to converge (" << err << " m)" << std::endl;
                return nullptr;
            }
            if (CheckCollisionUsingChecker(q_goal_solved))
            {
                std::cerr << "  [ERROR] Goal configuration is in collision!" << std::endl;
                return nullptr;
            }
        }

        std::cout << "  [IK] q_start: " << q_start_solved.transpose() << std::endl;
        std::cout << "  [IK] q_goal:  " << q_goal_solved.transpose() << std::endl;

        // ---- Plan MoveJ from start to goal ----
        std::cout << "  [MoveJ] Planning from start to goal..." << std::endl;
        return PlanMoveJ(q_start_solved, q_goal_solved, max_vel, max_acc);
    }

    /**
     * PlanMoveJToPose - Plan MoveJ from current position to a task-space goal.
     * Convenience wrapper: start = q_current (no IK needed for start).
     *
     * @param q_current  Current joint configuration
     * @param goal_xyz   Goal position in base frame (meters)
     * @param goal_rpy   Goal orientation as Roll-Pitch-Yaw (radians)
     */
    std::unique_ptr<drake::trajectories::Trajectory<double>>
    PlanMoveJToPose(
        const VectorXd &q_current,
        const Eigen::Vector3d &goal_xyz, const Eigen::Vector3d &goal_rpy,
        double max_vel = 1.0, double max_acc = 2.0,
        double pos_tol = 0.002, double rot_tol = 0.02,
        double collision_margin = 0.01)
    {
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "[PlanMoveJToPose] Current → Task-space goal" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Build goal pose
        drake::math::RigidTransformd goal_pose(
            drake::math::RollPitchYawd(goal_rpy(0), goal_rpy(1), goal_rpy(2)),
            goal_xyz);

        // Solve IK for goal
        std::cout << "  [IK] Goal: xyz=(" << goal_xyz.transpose()
                  << "), rpy=(" << goal_rpy.transpose() << ")" << std::endl;
        VectorXd q_goal_solved = SolveIK(goal_pose, q_current, pos_tol, rot_tol, collision_margin);

        {
            auto T_ik = ComputeEEPose(q_goal_solved);
            double err = (T_ik.translation() - goal_xyz).norm();
            std::cout << "  [IK] Goal pos error: " << (err * 1000.0) << " mm" << std::endl;
            if (err > pos_tol * 5)
            {
                std::cerr << "  [ERROR] Goal IK failed to converge (" << err << " m)" << std::endl;
                return nullptr;
            }
            if (CheckCollisionUsingChecker(q_goal_solved))
            {
                std::cerr << "  [ERROR] Goal configuration is in collision!" << std::endl;
                return nullptr;
            }
        }

        std::cout << "  [IK] q_goal: " << q_goal_solved.transpose() << std::endl;

        // Plan MoveJ from current to goal
        std::cout << "  [MoveJ] Planning from current to goal..." << std::endl;
        return PlanMoveJ(q_current, q_goal_solved, max_vel, max_acc);
    }

    // Evaluate planned trajectory at given time
    // Accepts Trajectory base class to work with CompositeTrajectory, PiecewisePolynomial, etc.
    VectorXd eval_trajectory(const drake::trajectories::Trajectory<double> &trajectory, double t)
    {
        // Clamp time to trajectory bounds
        double t_clamped = std::max(trajectory.start_time(),
                                    std::min(trajectory.end_time(), t));
        return trajectory.value(t_clamped);
    }

    // ========== INDUSTRIAL-GRADE COLLISION DETECTION ==========

    // ============================================================================
    // INDUSTRIAL-GRADE: Collision Checking using SceneGraphCollisionChecker
    // ============================================================================

    /**
     * CheckCollisionUsingChecker - Industrial-grade collision check using SceneGraphCollisionChecker
     *
     * This method uses Drake's SceneGraphCollisionChecker which provides:
     * - Automatic adjacent link filtering based on kinematic tree topology
     * - Robot-environment collision detection
     * - Robot-robot self-collision detection
     * - Industrial-grade reliability and testing
     *
     * @param q Configuration to check
     * @return true if collision detected, false if collision-free
     */
    bool CheckCollisionUsingChecker(const VectorXd &q)
    {
        if (!collision_checker_)
        {
            std::cerr << "[ERROR] SceneGraphCollisionChecker not available." << std::endl;
            return true; // Assume collision if checker not available (safe default)
        }

        try
        {
            // DEBUG: Print collision detection info for first few calls
            static int debug_call_count = 0;
            if (debug_call_count < 1)
            {
                std::cout << "\n[DEBUG COLLISION CHECK #" << debug_call_count << "]" << std::endl;
                std::cout << "  Using SceneGraphCollisionChecker CalcRobotClearance API" << std::endl;
                debug_call_count++;
            }

            // ============================================================================
            // COLLISION DETECTION using SceneGraphCollisionChecker's CalcRobotClearance:
            // - CalcRobotClearance returns distance measurements between robot bodies
            //   and all other bodies (both robot and environment)
            // - Negative distance = penetration (collision)
            // - Positive distance = clearance (no collision)
            // - CollisionType distinguishes between self-collisions and environment collisions
            // ============================================================================

            const double influence_distance = 1.0; // 1 meter - maximum distance to check
            drake::planning::RobotClearance clearance =
                collision_checker_->CalcRobotClearance(q, influence_distance);

            // Analyze the clearance results to find collisions
            std::vector<drake::multibody::BodyIndex> collision_robot_indices;
            std::vector<drake::multibody::BodyIndex> collision_other_indices;
            std::vector<drake::planning::RobotCollisionType> collision_types_list;
            std::vector<double> collision_distances;

            int self_collision_count = 0;
            int environment_collision_count = 0;

            // Find all colliding pairs (negative distance)
            for (int i = 0; i < clearance.size(); ++i)
            {
                double dist = clearance.distances()(i);

                // Negative distance means penetration (collision)
                if (dist < 0.0)
                {
                    auto collision_type = clearance.collision_types()[i];

                    collision_robot_indices.push_back(clearance.robot_indices()[i]);
                    collision_other_indices.push_back(clearance.other_indices()[i]);
                    collision_types_list.push_back(collision_type);
                    collision_distances.push_back(dist);

                    // Count collision types
                    if (collision_type == drake::planning::RobotCollisionType::kSelfCollision)
                    {
                        self_collision_count++;
                    }
                    else if (collision_type == drake::planning::RobotCollisionType::kEnvironmentCollision)
                    {
                        environment_collision_count++;
                    }
                }
            }

            // Determine if there's a TRUE collision   (any negative distance)
            bool has_collision = !collision_distances.empty();

            // Print detailed collision information if collision detected
            if (has_collision)
            {
                std::cout << "\n[COLLISION DETAILS] TRUE COLLISION detected!" << std::endl;
                std::cout << "  Total collisions: " << collision_distances.size() << std::endl;
                std::cout << "  Self-collisions: " << self_collision_count << std::endl;
                std::cout << "  Environment collisions: " << environment_collision_count << std::endl;

                // Find deepest penetrations for reporting
                std::cout << "\n  Significant collisions (penetration depth > 3mm):" << std::endl;
                int significant_count = 0;
                int max_print = std::min(static_cast<int>(collision_distances.size()), 10);

                // Sort by penetration depth (most negative = deepest)
                std::vector<size_t> sorted_indices(collision_distances.size());
                std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
                std::sort(sorted_indices.begin(), sorted_indices.end(),
                          [&collision_distances](size_t a, size_t b)
                          {
                              return collision_distances[a] < collision_distances[b];
                          });

                for (int idx = 0; idx < max_print; ++idx)
                {
                    size_t i = sorted_indices[idx];
                    double depth_mm = -collision_distances[i] * 1000; // Convert to mm, make positive

                    if (depth_mm > 3.0) // Only report significant penetrations
                    {
                        const auto &robot_body = robot_diagram_->plant().get_body(collision_robot_indices[i]);
                        const auto &other_body = robot_diagram_->plant().get_body(collision_other_indices[i]);
                        const auto &collision_type = collision_types_list[i];

                        std::string type_str = (collision_type == drake::planning::RobotCollisionType::kSelfCollision)
                                                   ? "SELF"
                                                   : "ENV";

                        std::cout << "  [" << (significant_count + 1) << "] "
                                  << robot_body.name() << " <-> " << other_body.name()
                                  << ", depth=" << depth_mm << " mm, type=" << type_str << std::endl;
                        significant_count++;
                    }
                }

                if (significant_count > 0)
                {
                    std::cout << "  " << significant_count << " deep penetration(s) (> 3mm)" << std::endl;
                }
            }

            return has_collision; // true = collision, false = no collision
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ERROR] SceneGraphCollisionChecker failed: " << e.what() << std::endl;
            return true; // Assume collision on error (safe default)
        }
    }

    /**
     * HasCollisionChecker - Check if SceneGraphCollisionChecker is available
     *
     * @return true if SceneGraphCollisionChecker is initialized and available
     */
    bool HasCollisionChecker() const
    {
        return collision_checker_ != nullptr;
    }

    /**
     * ValidateTrajectoryComplete - CCD continuous collision validation
     *
     * Uses Drake's CheckEdgeCollisionFree for continuous edge checking
     * between consecutive trajectory samples. edge_step_size=0.01 ensures
     * ~100 collision checks per radian of joint motion.
     *
     * @param trajectory The trajectory to validate
     * @param min_sampling_interval Sampling interval for edge endpoints (seconds)
     * @param collision_tolerance Penetration depth threshold (meters).
     *                            Penetrations smaller than this are ignored as
     *                            mesh approximation artifacts.
     * @return true if trajectory is collision-free (within tolerance)
     */
    bool ValidateTrajectoryComplete(
        const drake::trajectories::Trajectory<double> &trajectory,
        double min_sampling_interval = 0.002, // 2ms = dense edge endpoints
        double collision_tolerance = 0.003)   // 3mm penetration tolerance
    {
        if (!collision_checker_)
        {
            throw std::runtime_error("[ERROR] SceneGraphCollisionChecker not available!");
        }

        const double duration = trajectory.end_time() - trajectory.start_time();
        int num_samples = static_cast<int>(std::ceil(duration / min_sampling_interval)) + 1;
        num_samples = std::max(num_samples, 100);

        std::cout << "\n[CCD TRAJECTORY VALIDATION]" << std::endl;
        std::cout << "  Duration: " << duration << " s, samples: " << num_samples
                  << ", interval: " << (duration / (num_samples - 1) * 1000.0) << " ms" << std::endl;
        std::cout << "  Edge CCD step size: " << collision_checker_->edge_step_size() << " rad" << std::endl;
        std::cout << "  Collision tolerance: " << (collision_tolerance * 1000.0) << " mm" << std::endl;

        bool valid = true;
        int collisions = 0;
        int ignored = 0;
        double first_collision_time = -1.0;
        double worst_depth = 0.0;

        VectorXd q_prev = trajectory.value(trajectory.start_time());

        // Check first sample
        if (!collision_checker_->CheckConfigCollisionFree(q_prev))
        {
            auto clearance = collision_checker_->CalcRobotClearance(
                q_prev, collision_tolerance + 0.01);
            double min_d = 0;
            for (int j = 0; j < clearance.size(); ++j)
                min_d = std::min(min_d, clearance.distances()(j));
            worst_depth = std::min(worst_depth, min_d);

            if (min_d < -collision_tolerance)
            {
                collisions++;
                first_collision_time = trajectory.start_time();
                valid = false;
            }
            else
            {
                ignored++;
            }
        }

        // Check each edge using Drake's continuous CCD
        for (int i = 1; i < num_samples; ++i)
        {
            double t = trajectory.start_time() + duration * i / (num_samples - 1);
            VectorXd q_curr = trajectory.value(t);

            // Continuous edge collision check between consecutive samples
            if (!collision_checker_->CheckEdgeCollisionFree(q_prev, q_curr))
            {
                // Binary check flagged collision — verify penetration depth
                auto clearance = collision_checker_->CalcRobotClearance(
                    q_curr, collision_tolerance + 0.01);
                double min_d = 0;
                for (int j = 0; j < clearance.size(); ++j)
                    min_d = std::min(min_d, clearance.distances()(j));
                worst_depth = std::min(worst_depth, min_d);

                if (min_d < -collision_tolerance)
                {
                    if (first_collision_time < 0)
                        first_collision_time = t;
                    collisions++;
                    valid = false;

                    if (collisions <= 5)
                    {
                        double depth_mm = -min_d * 1000.0;
                        std::cout << "  [CCD COLLISION] at t=" << t
                                  << " s, depth=" << depth_mm << " mm"
                                  << " (segment " << i << "/" << num_samples << ")" << std::endl;
                    }
                }
                else
                {
                    ignored++;
                }
            }

            q_prev = q_curr;
        }

        if (valid)
        {
            std::cout << "  [CCD SUCCESS] Trajectory is collision-free!" << std::endl;
            std::cout << "  Verified " << (num_samples - 1) << " edges";
            if (ignored > 0)
                std::cout << " (" << ignored << " sub-threshold ignored)";
            std::cout << std::endl;
        }
        else
        {
            std::cout << "\n  [CCD FAILURE] Trajectory has COLLISIONS!" << std::endl;
            std::cout << "  First collision near t=" << first_collision_time << " s" << std::endl;
            std::cout << "  Colliding segments: " << collisions << " / " << (num_samples - 1)
                      << " (" << (100.0 * collisions / (num_samples - 1)) << "%)" << std::endl;
            std::cout << "  Worst penetration: " << (-worst_depth * 1000.0) << " mm" << std::endl;
            if (ignored > 0)
                std::cout << "  Sub-threshold ignored: " << ignored << std::endl;
        }

        return valid;
    }

    /**
     * SetupAdjacentLinkCollisionFiltering - Filter collisions between adjacent links
     *
     * This function filters out collision detection between robot links that are
     * directly connected by joints, as these are not actual collisions but rather
     * the physical connection between links.
     *
     * NOTE: Non-adjacent links on the same kinematic chain (e.g., link1 and link5)
     * are NOT filtered because they CAN collide when the arm is in a folded configuration.
     * These are valid self-collisions that should be detected.
     */
    void SetupAdjacentLinkCollisionFiltering()
    {
        if (!collision_checker_)
        {
            std::cerr << "[ERROR] CollisionChecker not initialized!" << std::endl;
            return;
        }

        auto &plant = robot_diagram_->plant();
        const int num_bodies = plant.num_bodies();

        std::cout << "\n[ADJACENT LINK FILTERING] Scanning for directly connected links..." << std::endl;
        std::cout << "  [INFO] Scanning " << num_bodies << " bodies for adjacent link relationships..." << std::endl;

        int num_filters_added = 0;
        int num_already_filtered = 0;

        // Iterate through all joints to find directly connected bodies
        for (drake::multibody::JointIndex joint_idx(0); joint_idx < plant.num_joints(); ++joint_idx)
        {
            const auto &joint = plant.get_joint(joint_idx);

            // Get the bodies connected by this joint
            drake::multibody::BodyIndex parent_body = joint.parent_body().index();
            drake::multibody::BodyIndex child_body = joint.child_body().index();

            // Skip if either body is not part of the robot
            if (!collision_checker_->IsPartOfRobot(parent_body) ||
                !collision_checker_->IsPartOfRobot(child_body))
            {
                continue;
            }

            // Check if this collision is already filtered
            if (collision_checker_->IsCollisionFilteredBetween(parent_body, child_body))
            {
                num_already_filtered++;
                // Only log first few to avoid spam
                if (num_already_filtered <= 5)
                {
                    std::cout << "  [SKIP] " << plant.get_body(parent_body).name()
                              << " <-> " << plant.get_body(child_body).name()
                              << " (already filtered)" << std::endl;
                }
                continue;
            }

            // Filter the collision between these adjacent bodies
            collision_checker_->SetCollisionFilteredBetween(parent_body, child_body, true);
            num_filters_added++;

            std::cout << "  [FILTER] " << plant.get_body(parent_body).name()
                      << " <-> " << plant.get_body(child_body).name()
                      << " (joint: " << joint.name() << ")" << std::endl;
        }

        std::cout << "  [SUCCESS] Added " << num_filters_added << " adjacent link collision filters" << std::endl;
        if (num_already_filtered > 5)
        {
            std::cout << "  [INFO] " << (num_already_filtered - 5) << " more pairs were already filtered" << std::endl;
        }

        // ========================================================================
        // Auto-detect and filter false-positive mesh overlaps at default pose.
        //
        // Collision meshes are simplified approximations of visual meshes.
        // At the default (zero) pose, links branching from the same parent
        // (e.g., head_link1, left_arm_link1, right_arm_link1 from lumber_link3)
        // may have collision mesh overlap even though there's no visual collision.
        //
        // This dynamically detects which pairs have mesh penetration at q=0
        // and filters ONLY those specific pairs — no hardcoding needed.
        //
        // IMPORTANT: This does NOT filter arm chain pairs (link1↔link3, etc.)
        // unless they actually overlap at the default pose. Arm folding
        // collisions during motion are still correctly detected.
        // ========================================================================
        {
            VectorXd q_default = robot_diagram_->plant().GetPositions(
                *robot_diagram_->plant().CreateDefaultContext());

            const double influence = 1.0;
            auto clearance = collision_checker_->CalcRobotClearance(q_default, influence);

            // Find all pairs with negative distance (mesh penetration) at default pose
            struct PenetratingPair
            {
                drake::multibody::BodyIndex body_a;
                drake::multibody::BodyIndex body_b;
                double depth; // negative = penetration depth
            };
            std::vector<PenetratingPair> penetrations;

            for (int i = 0; i < clearance.size(); ++i)
            {
                if (clearance.distances()(i) < 0.0)
                {
                    // Only consider robot-robot pairs (self-collision)
                    auto ctype = clearance.collision_types()[i];
                    if (ctype == drake::planning::RobotCollisionType::kSelfCollision)
                    {
                        penetrations.push_back({clearance.robot_indices()[i],
                                                clearance.other_indices()[i],
                                                clearance.distances()(i)});
                    }
                }
            }

            if (!penetrations.empty())
            {
                std::cout << "\n[MESH OVERLAP] Detected " << penetrations.size()
                          << " false-positive penetrations at default pose:" << std::endl;

                for (const auto &p : penetrations)
                {
                    const auto &body_a = plant.get_body(p.body_a);
                    const auto &body_b = plant.get_body(p.body_b);

                    if (!collision_checker_->IsCollisionFilteredBetween(p.body_a, p.body_b))
                    {
                        collision_checker_->SetCollisionFilteredBetween(p.body_a, p.body_b, true);
                        double depth_mm = -p.depth * 1000.0;
                        std::cout << "  [FILTER-OVERLAP] " << body_a.name()
                                  << " <-> " << body_b.name()
                                  << " (mesh overlap: " << depth_mm << " mm at q=0)" << std::endl;
                    }
                }
            }
            else
            {
                std::cout << "\n[MESH OVERLAP] No false-positive penetrations at default pose." << std::endl;
            }
        }

        // Verify the filters by checking current filter matrix
        std::cout << "\n[VERIFICATION] Checking filter matrix..." << std::endl;

        int num_filtered = 0;
        int num_checked = 0;
        for (int i = 0; i < num_bodies; ++i)
        {
            for (int j = i + 1; j < num_bodies; ++j)
            {
                // Only check robot-robot pairs
                if (collision_checker_->IsPartOfRobot(drake::multibody::BodyIndex(i)) &&
                    collision_checker_->IsPartOfRobot(drake::multibody::BodyIndex(j)))
                {
                    num_checked++;
                    if (collision_checker_->IsCollisionFilteredBetween(
                            drake::multibody::BodyIndex(i),
                            drake::multibody::BodyIndex(j)))
                    {
                        num_filtered++;
                    }
                }
            }
        }

        std::cout << "  [VERIFICATION] Robot-robot pairs: " << num_filtered << "/" << num_checked << " filtered" << std::endl;
    }

private:
    std::shared_ptr<drake::planning::RobotDiagram<double>> robot_diagram_;
    std::unique_ptr<drake::systems::Simulator<double>> simulator_;
    std::unique_ptr<drake::planning::SceneGraphCollisionChecker> collision_checker_;
    // Standalone plant for IK (no SceneGraph, avoids segfault with connected plant)
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> ik_plant_;
};

// ========== Trajectory Execution Helpers ==========

// Resample a planned trajectory at a fixed control period.
// Returns a vector of (time, q) pairs spaced at exactly control_period intervals.
// This is the standard way industrial controllers consume trajectory data:
//   - Uniform time spacing matching the servo loop rate
//   - The underlying spline (C2 continuous) guarantees smooth acceleration between samples
static std::vector<std::pair<double, VectorXd>> ResampleTrajectory(
    const drake::trajectories::Trajectory<double> &traj,
    double control_period)
{
    double duration = traj.end_time() - traj.start_time();
    int num_points = static_cast<int>(std::round(duration / control_period)) + 1;

    std::vector<std::pair<double, VectorXd>> samples;
    samples.reserve(num_points);

    for (int i = 0; i < num_points; ++i)
    {
        double t = traj.start_time() + std::min(i * control_period, duration);
        samples.emplace_back(t, traj.value(t));
    }

    return samples;
}

// Execute a planned trajectory in MuJoCo with rendering and trajectory point recording.
// Drives MuJoCo at the specified control_period (e.g., 5ms for 200Hz).
static void ExecuteTrajectory(
    DrakeSimulator &drake_sim,
    MuJoCoSimulator &mujoco_sim,
    const drake::trajectories::Trajectory<double> &traj,
    std::vector<float> &traj_points,
    int nv, double control_period, int &total_steps)
{
    double traj_dur = traj.end_time() - traj.start_time();
    const int traj_sample_interval = static_cast<int>(0.01 / control_period);
    int step_count = 0;

    // Build analytic velocity trajectory once (works for both BsplineTrajectory
    // and PathParameterizedTrajectory — no finite-difference noise).
    auto vel_traj = traj.MakeDerivative(1);

    for (double t = 0.0; t < traj_dur && !mujoco_sim.should_close(); t += control_period)
    {
        double t_abs = traj.start_time() + t;
        double t_clamped = std::max(traj.start_time(), std::min(traj.end_time(), t_abs));
        VectorXd q_desired = traj.value(t_clamped);
        VectorXd v_desired = vel_traj->value(t_clamped);

        mujoco_sim.set_state(q_desired, v_desired);
        mujoco_sim.set_ctrl(q_desired);

        // Record EE from kinematic FK (planned position), NOT from
        // post-physics mj_step which introduces gravity/dynamics artifacts
        // that make the trajectory appear as a polyline.
        if (step_count % traj_sample_interval == 0)
        {
            mujoco_sim.forward(); // update FK from q_desired
            Eigen::Vector3d ee_pos_world = mujoco_sim.get_ee_position_world();
            traj_points.push_back(static_cast<float>(ee_pos_world(0)));
            traj_points.push_back(static_cast<float>(ee_pos_world(1)));
            traj_points.push_back(static_cast<float>(ee_pos_world(2)));
        }

        mujoco_sim.step(control_period);
        step_count++;
        total_steps++;

        if (step_count % 5 == 0)
            mujoco_sim.render(traj_points);
    }

    std::cout << "  Executed " << step_count << " steps (period: "
              << (control_period * 1000.0) << " ms)" << std::endl;
}

// Run planning on a background thread while keeping GLFW rendering alive.
// This prevents the OS from detecting the window as "unresponsive" during
// expensive RRT* collision-checking (which can take seconds to minutes).
template <typename PlanFn>
static std::unique_ptr<drake::trajectories::Trajectory<double>>
PlanWithRendering(PlanFn plan_fn, MuJoCoSimulator &mujoco_sim,
                  std::vector<float> &traj_points)
{
    auto future = std::async(std::launch::async, plan_fn);

    while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
    {
        if (mujoco_sim.should_close())
        {
            future.wait();
            return nullptr;
        }
        mujoco_sim.render(traj_points);
    }

    return future.get();
}

// Plan and execute return-to-home trajectory (MoveJ from current to q_start)
static void ReturnToHome(
    DrakeSimulator &drake_sim,
    MuJoCoSimulator &mujoco_sim,
    VectorXd &q_current, const VectorXd &q_start,
    std::vector<float> &traj_points,
    int nv, double control_period)
{
    if ((q_current - q_start).cwiseAbs().maxCoeff() < 1e-4)
    {
        std::cout << "  Already at home position" << std::endl;
        return;
    }

    std::cout << "\n  >>> Returning to home position..." << std::endl;
    auto return_traj = PlanWithRendering(
        [&]()
        { return drake_sim.PlanMoveJ(q_current, q_start, 1.0, 2.0); },
        mujoco_sim, traj_points);

    if (!return_traj)
    {
        std::cerr << "  [ERROR] Return trajectory planning failed! Staying at current position." << std::endl;
        return;
    }

    int dummy = 0;
    ExecuteTrajectory(drake_sim, mujoco_sim, *return_traj, traj_points, nv, control_period, dummy);
    q_current = q_start;
    std::cout << "  [OK] Returned to home position" << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Drake & MuJoCo Circular Trajectory   " << std::endl;
    std::cout << "========================================\n"
              << std::endl;

    std::string project_dir;
    std::string urdf_path;
    std::string mujoco_scene_path;

    // Method 1: Check environment variable
    if (const char *env_root = std::getenv("DMR_PROJECT_ROOT"))
    {
        project_dir = env_root;
        std::cout << "[PATH] Using project root from environment: " << project_dir << std::endl;
    }
    // Method 2: Relative to executable (portable)
    else
    {
        // Get executable path (platform-specific)
        std::string exe_path;
#ifdef __linux__
        char exe_buf[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
        if (len != -1)
        {
            exe_buf[len] = '\0';
            exe_path = exe_buf;
        }
#elif __APPLE__
        char exe_buf[PATH_MAX];
        uint32_t len = sizeof(exe_buf);
        if (_NSGetExecutablePath(exe_buf, &len) == 0)
        {
            exe_path = exe_buf;
        }
#endif

        if (!exe_path.empty())
        {
            // Remove executable name to get directory
            size_t last_sep = exe_path.find_last_of('/');
            if (last_sep != std::string::npos)
            {
                std::string exe_dir = exe_path.substr(0, last_sep);
                // Go up to project root (assuming executable is in build/ or demo/)
                size_t build_pos = exe_dir.find_last_of('/');
                if (build_pos != std::string::npos)
                {
                    project_dir = exe_dir.substr(0, build_pos);
                    std::cout << "[PATH] Detected project root from executable: " << project_dir << std::endl;
                }
            }
        }
    }

    // Method 3: Fallback to relative paths from working directory
    if (project_dir.empty())
    {
        project_dir = ".."; // Assume we're in build/ or demo/
        std::cout << "[PATH] Using relative path from working directory" << std::endl;
    }

    // Construct model paths (relative to project root)
    urdf_path = project_dir + "/model/V3/urdf/RobotV3.urdf";
    mujoco_scene_path = project_dir + "/model/V3/urdf/scene.xml";

    std::ifstream urdf_check(urdf_path);
    std::ifstream scene_check(mujoco_scene_path);

    if (!urdf_check.good())
    {
        std::cerr << "\n[ERROR] URDF file not found: " << urdf_path << std::endl;
        std::cerr << "[INFO] Set DMR_PROJECT_ROOT environment variable or run from project directory" << std::endl;
        return -1;
    }
    urdf_check.close();

    if (!scene_check.good())
    {
        std::cerr << "\n[ERROR] MuJoCo scene file not found: " << mujoco_scene_path << std::endl;
        std::cerr << "[INFO] Set DMR_PROJECT_ROOT environment variable or run from project directory" << std::endl;
        return -1;
    }
    scene_check.close();

    double time_step = 0.001;  // MuJoCo physics timestep
    double control_hz = 200.0; // Trajectory control frequency (Hz)
    bool enable_visualization = true;

    // Parse optional flags only
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--no-visual")
            enable_visualization = false;
        else if (arg == "--dt" && i + 1 < argc)
            time_step = std::stod(argv[++i]);
        else if (arg == "--hz" && i + 1 < argc)
            control_hz = std::stod(argv[++i]);
    }

    double control_period = 1.0 / control_hz;
    std::cout << "[CONTROL] Frequency: " << control_hz << " Hz"
              << ", Period: " << (control_period * 1000.0) << " ms" << std::endl;

    try
    {
        // ========== STEP 1: DRAKE CARTESIAN TRAJECTORY PLANNING ==========
        std::cout << "\n>>> Step 1: Loading Drake Model for Planning" << std::endl;
        DrakeSimulator drake_sim(urdf_path);

        // Define starting joint configuration for RobotV3 (19 DOF)
        // Joint indices: lumber1[0], lumber[1-2], head[3-4], left_arm[5-11], right_arm[12-18]

        // Initial joint configuration
        VectorXd q_start = VectorXd::Zero(19);
        // Lumber lift
        q_start(0) = 0.0; // lumber_joint1 (prismatic, -0.3~0m) — upper limit (home)
        // Head joints: tilt head up to avoid self-collision with lumber_link3
        q_start(3) = 0.0;  // head_joint1
        q_start(4) = -0.3; // head_joint2 (tilt forward slightly)
        // Right arm: home position (comfortable pose, no self-collision)
        q_start(12) = 0.0;
        q_start(13) = 0.0;
        q_start(14) = 0.0;
        q_start(15) = 0.0;
        q_start(16) = 0.0;
        q_start(17) = 0.0;
        q_start(18) = 0.0;

        // Compute initial EE position using Forward Kinematics
        drake::math::RigidTransformd T_ee_start = drake_sim.ComputeEEPose(q_start);
        Eigen::Vector3d ee_start = T_ee_start.translation(); // 末端初始位置

        std::cout << "Initial EE Position (agv_link frame): " << ee_start.transpose() << std::endl;
        std::cout << "Configuration uses agv_link (base) coordinate frame as reference" << std::endl;

        // Perform initial collision check on starting configuration
        std::cout << "\n>>> Safety Check: Verifying Initial Configuration" << std::endl;
        bool initial_has_collision = drake_sim.CheckCollisionUsingChecker(q_start);

        if (initial_has_collision)
        {
            std::cout << "\n[WARNING] Starting configuration is in collision!" << std::endl;
            std::cout << "Trajectory planning will attempt to find safe path..." << std::endl;
        }
        else
        {
            std::cout << "[OK] Initial configuration is safe" << std::endl;
        }

        // ========== STEP 2: INIT MUJOCO ==========
        std::cout << "\n>>> Initializing MuJoCo for Visualization" << std::endl;
        drake_sim.reset();
        std::cout << std::flush;
        std::cerr << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        MuJoCoSimulator mujoco_sim(mujoco_scene_path, enable_visualization);
        mujoco_sim.reset();
        int nq = mujoco_sim.get_num_positions();
        int nv = mujoco_sim.get_num_dofs();
        std::vector<float> traj_points;
        int total_step_count = 0;

        VectorXd q_current = q_start;
        VectorXd v_zero_init = VectorXd::Zero(nv);
        mujoco_sim.set_state(q_current, v_zero_init);
        mujoco_sim.set_ctrl(q_current);
        mujoco_sim.step(0);

        // Verify initial EE position
        Eigen::Vector3d mujoco_ee_start = mujoco_sim.get_ee_position();
        std::cout << "Drake EE start: " << ee_start.transpose() << std::endl;
        std::cout << "MuJoCo EE start: " << mujoco_ee_start.transpose() << std::endl;

        // ========== STEP 3: INTERACTIVE MENU ==========
        auto sim_start_time = std::chrono::high_resolution_clock::now();
        int action_counter = 0;

        // Print menu once at start
        auto print_menu = [&]()
        {
            std::cout << "\n"
                      << std::string(50, '=') << std::endl;
            std::cout << "  Trajectory Planning Menu" << std::endl;
            std::cout << std::string(50, '=') << std::endl;
            std::cout << "  1. MoveJ - To goal pose xyz rpy (start = current)" << std::endl;
            std::cout << "  2. MoveJ - From start to goal pose xyz rpy" << std::endl;
            std::cout << "  3. FK Explorer - joints -> xyz rpy (find reachable pose)" << std::endl;
            std::cout << "  4. MoveJ - To goal joints (lumber2 lumber3 arm1..7)" << std::endl;
            std::cout << "  0. Exit" << std::endl;
            std::cout << std::string(50, '=') << std::endl;
            std::cout << ">> Select: " << std::flush;
        };

        while (!mujoco_sim.should_close())
        {
            print_menu();

            // Non-blocking wait for stdin: keep rendering while waiting for input
            int choice = 0;
            bool got_input = false;
            while (!mujoco_sim.should_close())
            {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(STDIN_FILENO, &fds);
                struct timeval tv = {0, 100000}; // 100ms timeout
                int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
                if (ret > 0)
                {
                    got_input = true;
                    break;
                }
                // No input yet — keep rendering so mouse/camera works
                mujoco_sim.render(traj_points);
            }
            if (mujoco_sim.should_close())
                break;
            if (!got_input)
                continue;

            std::cin >> choice;

            if (std::cin.eof() || choice == 0)
                break;
            if (choice < 1 || choice > 4)
            {
                std::cout << "  [Invalid] Please select 0-4" << std::endl;
                continue;
            }

            // Plan trajectory based on selection
            std::unique_ptr<drake::trajectories::Trajectory<double>> traj;
            std::string action_name;

            // Return to home before planning next trajectory (if not already there)
            // Skip for FK explorer (option 3) — no movement needed
            if (choice != 3 && choice != 4)
                ReturnToHome(drake_sim, mujoco_sim, q_current, q_start, traj_points, nv, control_period);

            // Clear previous trajectory visualization before new one
            traj_points.clear();

            switch (choice)
            {
            case 1: // MoveJ — current position → goal pose (xyz rpy)
            {
                action_name = "MoveJ";
                auto T_cur = drake_sim.ComputeEEPose(q_current);
                Eigen::Vector3d cur_xyz = T_cur.translation();
                Eigen::Vector3d cur_rpy =
                    drake::math::RollPitchYawd(T_cur.rotation()).vector();

                std::cout << "\n>>> [MoveJ] Current EE: xyz=(" << cur_xyz.transpose()
                          << "), rpy=(" << cur_rpy.transpose() << ")" << std::endl;
                std::cout << "  Enter goal - x y z roll pitch yaw: " << std::flush;

                double gx, gy, gz, gr, gp, gyaw;
                std::cin >> gx >> gy >> gz >> gr >> gp >> gyaw;

                Eigen::Vector3d goal_xyz(gx, gy, gz);
                Eigen::Vector3d goal_rpy(gr, gp, gyaw);

                traj = PlanWithRendering(
                    [&]() -> std::unique_ptr<drake::trajectories::Trajectory<double>>
                    {
                        return drake_sim.PlanMoveJToPose(
                            q_current, goal_xyz, goal_rpy, 1.0, 2.0);
                    },
                    mujoco_sim, traj_points);
                break;
            }
            case 2: // MoveJ — start pose → goal pose (both xyz rpy)
            {
                action_name = "MoveJ";

                auto T_cur = drake_sim.ComputeEEPose(q_current);
                Eigen::Vector3d cur_xyz = T_cur.translation();
                Eigen::Vector3d cur_rpy =
                    drake::math::RollPitchYawd(T_cur.rotation()).vector();

                std::cout << "\n>>> [MoveJ] Current EE: xyz=(" << cur_xyz.transpose()
                          << "), rpy=(" << cur_rpy.transpose() << ")" << std::endl;

                // Start pose
                double sx, sy, sz, sr, sp, syaw;
                std::cout << "  Enter start - x y z roll pitch yaw: " << std::flush;
                std::cin >> sx >> sy >> sz >> sr >> sp >> syaw;
                Eigen::Vector3d start_xyz(sx, sy, sz);
                Eigen::Vector3d start_rpy(sr, sp, syaw);

                // Goal pose
                double gx, gy, gz, gr, gp, gyaw;
                std::cout << "  Enter goal  - x y z roll pitch yaw: " << std::flush;
                std::cin >> gx >> gy >> gz >> gr >> gp >> gyaw;
                Eigen::Vector3d goal_xyz(gx, gy, gz);
                Eigen::Vector3d goal_rpy(gr, gp, gyaw);

                // If current ≠ start, first move to start via MoveJ
                traj = PlanWithRendering(
                    [&]() -> std::unique_ptr<drake::trajectories::Trajectory<double>>
                    {
                        return drake_sim.PlanMoveJFromCartesian(
                            q_current, start_xyz, start_rpy,
                            goal_xyz, goal_rpy, 1.0, 2.0);
                    },
                    mujoco_sim, traj_points);

                // Note: PlanMoveJFromCartesian solves IK for both start and goal
                // then plans MoveJ from q_start_solved to q_goal_solved.
                // If q_current ≠ q_start_solved, the returned trajectory starts
                // from q_start_solved. The caller should first execute a move
                // from q_current to q_start_solved (handled by ReturnToHome
                // if start is close to home, otherwise a separate move).
                break;
            }
            case 3: // FK Explorer - input joint angles, see Cartesian pose
            {
                // Show current joint state as reference
                std::cout << "\n>>> [FK Explorer] Joint angle -> Cartesian pose" << std::endl;
                std::cout << "  Planning joints (9-DOF):" << std::endl;
                std::cout << "    [1] lumber_joint2  [2] lumber_joint3" << std::endl;
                std::cout << "    [12..18] right_arm_joint1..7" << std::endl;
                std::cout << "  Current q (planning): ";
                for (int j = 0; j < DrakeSimulator::kPlanDof; ++j)
                    std::cout << q_current(DrakeSimulator::kPlanIndices[j]) << " ";
                std::cout << std::endl;

                // Show current EE as reference
                auto T_cur = drake_sim.ComputeEEPose(q_current);
                Eigen::Vector3d cur_xyz = T_cur.translation();
                Eigen::Vector3d cur_rpy =
                    drake::math::RollPitchYawd(T_cur.rotation()).vector();
                std::cout << "  Current EE: xyz=(" << cur_xyz.transpose()
                          << "), rpy=(" << cur_rpy.transpose() << ")" << std::endl;

                // Loop: repeatedly query joint angles until empty input
                while (!mujoco_sim.should_close())
                {
                    std::cout << "\n  Enter 9 joint angles (lumber2 lumber3 arm1..7), or q to quit: " << std::flush;

                    // Non-blocking input wait
                    bool fk_got_input = false;
                    std::string line;
                    while (!mujoco_sim.should_close())
                    {
                        fd_set fds;
                        FD_ZERO(&fds);
                        FD_SET(STDIN_FILENO, &fds);
                        struct timeval tv = {0, 100000};
                        int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
                        if (ret > 0) { fk_got_input = true; break; }
                        mujoco_sim.render(traj_points);
                    }
                    if (mujoco_sim.should_close()) break;
                    if (!fk_got_input) continue;

                    std::getline(std::cin, line);
                    // Skip leading whitespace/newline from previous cin>>
                    size_t start = line.find_first_not_of(" \t\r\n");
                    if (start == std::string::npos) continue;
                    if (line[start] == 'q' || line[start] == 'Q') break;

                    // Parse 9 doubles
                    std::istringstream iss(line);
                    std::vector<double> vals;
                    double v;
                    while (iss >> v) vals.push_back(v);
                    if ((int)vals.size() != DrakeSimulator::kPlanDof)
                    {
                        std::cout << "  [Error] Need exactly 9 values, got " << vals.size() << std::endl;
                        continue;
                    }

                    // Build q from planning joints
                    VectorXd q_test = q_current;
                    for (int j = 0; j < DrakeSimulator::kPlanDof; ++j)
                        q_test(DrakeSimulator::kPlanIndices[j]) = vals[j];

                    // FK
                    auto T_fk = drake_sim.ComputeEEPose(q_test);
                    Eigen::Vector3d fk_xyz = T_fk.translation();
                    Eigen::Vector3d fk_rpy =
                        drake::math::RollPitchYawd(T_fk.rotation()).vector();

                    // Collision check
                    bool in_collision = drake_sim.CheckCollisionUsingChecker(q_test);

                    std::cout << "  EE: xyz=(" << fk_xyz.transpose()
                              << "), rpy=(" << fk_rpy.transpose() << ")"
                              << (in_collision ? "  ** COLLISION **" : "  [OK] collision-free")
                              << std::endl;
                    std::cout << "  => Copy for MoveJ: " << fk_xyz(0) << " " << fk_xyz(1) << " " << fk_xyz(2)
                              << " " << fk_rpy(0) << " " << fk_rpy(1) << " " << fk_rpy(2) << std::endl;
                }
                continue; // Don't execute any trajectory
            }
            case 4: // MoveJ — direct joint angle input (no IK)
            {
                action_name = "MoveJ";

                std::cout << "\n>>> [MoveJ] Direct joint angle input" << std::endl;
                std::cout << "  Current q (planning): ";
                for (int j = 0; j < DrakeSimulator::kPlanDof; ++j)
                    std::cout << q_current(DrakeSimulator::kPlanIndices[j]) << " ";
                std::cout << std::endl;

                auto T_cur = drake_sim.ComputeEEPose(q_current);
                Eigen::Vector3d cur_xyz = T_cur.translation();
                Eigen::Vector3d cur_rpy =
                    drake::math::RollPitchYawd(T_cur.rotation()).vector();
                std::cout << "  Current EE: xyz=(" << cur_xyz.transpose()
                          << "), rpy=(" << cur_rpy.transpose() << ")" << std::endl;

                std::cout << "  Enter 9 joints (lumber2 lumber3 arm1..7): " << std::flush;

                std::vector<double> vals;
                double v;
                for (int j = 0; j < DrakeSimulator::kPlanDof; ++j)
                {
                    std::cin >> v;
                    vals.push_back(v);
                }

                // Build q_goal from planning joints
                VectorXd q_goal = q_current;
                for (int j = 0; j < DrakeSimulator::kPlanDof; ++j)
                    q_goal(DrakeSimulator::kPlanIndices[j]) = vals[j];

                // Show goal EE
                auto T_goal = drake_sim.ComputeEEPose(q_goal);
                Eigen::Vector3d goal_xyz = T_goal.translation();
                Eigen::Vector3d goal_rpy =
                    drake::math::RollPitchYawd(T_goal.rotation()).vector();
                std::cout << "  Goal EE:   xyz=(" << goal_xyz.transpose()
                          << "), rpy=(" << goal_rpy.transpose() << ")" << std::endl;

                // Collision check
                if (drake_sim.CheckCollisionUsingChecker(q_goal))
                {
                    std::cout << "  [WARNING] Goal is in collision! Planning anyway..." << std::endl;
                }
                else
                {
                    std::cout << "  [OK] Goal is collision-free" << std::endl;
                }

                // Plan MoveJ directly from current to goal (no IK)
                traj = PlanWithRendering(
                    [&]() -> std::unique_ptr<drake::trajectories::Trajectory<double>>
                    {
                        return drake_sim.PlanMoveJ(q_current, q_goal, 1.0, 2.0);
                    },
                    mujoco_sim, traj_points);
                break;
            }
            }

            if (!traj)
            {
                std::cout << "  [FAIL] Planning failed!" << std::endl;
                continue;
            }

            double traj_dur = traj->end_time() - traj->start_time();
            if (traj_dur < 1e-6)
            {
                std::cout << "  [SKIP] Empty trajectory!" << std::endl;
                continue;
            }

            std::cout << "  Duration: " << traj_dur << " s" << std::endl;

            // Execute trajectory in MuJoCo at control_period rate
            ExecuteTrajectory(drake_sim, mujoco_sim, *traj, traj_points, nv, control_period, total_step_count);

            // Update current position
            q_current = traj->value(traj->end_time());
            auto T_ee_final = drake_sim.ComputeEEPose(q_current);
            std::cout << "  Final EE: " << T_ee_final.translation().transpose() << std::endl;

            // Resample trajectory at control period and save JSON for real robot
            {
                auto resampled = ResampleTrajectory(*traj, control_period);
                std::string json_filename = "trajectory_action_" + std::to_string(action_counter) + ".json";

                std::ofstream json_file(json_filename);
                if (json_file.is_open())
                {
                    json_file << "{\n    \"cycle\": 0,\n    \"actions\": [{\n";
                    json_file << "        \"taskId\": \"" << action_name << "_action" << action_counter << "\",\n";
                    json_file << "        \"taskType\": \"Play\",\n";
                    json_file << "        \"taskParameters\": {\n";
                    json_file << "            \"continue\": false,\n";
                    json_file << "            \"updateId\": 0,\n";
                    json_file << "            \"controlHz\": " << control_hz << ",\n";
                    json_file << "            \"controlPeriod\": " << std::scientific << control_period << ",\n";
                    json_file << "            \"rightHand\": [\n";
                    for (size_t i = 0; i < resampled.size(); ++i)
                    {
                        const auto &q_t = resampled[i].second;
                        json_file << "                [";
                        for (int j = 12; j <= 18; ++j)
                        {
                            json_file << std::scientific << std::setprecision(15) << q_t(j);
                            if (j < 18)
                                json_file << ", ";
                        }
                        json_file << "]" << (i < resampled.size() - 1 ? ",\n" : "\n");
                    }
                    json_file << "            ]\n        }\n    }]\n}\n";
                    json_file.close();
                    std::cout << "  [JSON] Saved " << resampled.size() << " points to "
                              << json_filename << " (@" << control_hz << "Hz)" << std::endl;
                }
            }

            action_counter++;
        }

        auto sim_end_time = std::chrono::high_resolution_clock::now();
        auto sim_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(sim_end_time - sim_start_time);
        std::cout << "\n=== Session Complete ===" << std::endl;
        std::cout << "Actions executed: " << action_counter << ", Total steps: " << total_step_count
                  << ", Time: " << sim_elapsed.count() << " ms" << std::endl;

        while (!mujoco_sim.should_close())
        {
            mujoco_sim.render(traj_points);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "\n=== Test Completed ===" << std::endl;
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n=== ERROR ===" << std::endl;
        std::cerr << e.what() << std::endl;
        return 1;
    }
}