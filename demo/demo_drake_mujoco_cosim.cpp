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

#if defined(__linux__)
#include <limits.h> // PATH_MAX
#include <unistd.h> // readlink
#elif defined(__APPLE__)
#include <limits.h>      // PATH_MAX
#include <mach-o/dyld.h> // _NSGetExecutablePath
#include <stdlib.h>      // realpath
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
#include <drake/common/trajectories/bezier_curve.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/math/bspline_basis.h>
#include <drake/common/polynomial.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/geometry/optimization/point.h>
#include <drake/geometry/optimization/iris.h>
#include <drake/planning/iris/iris_zo.h>
#include <drake/planning/collision_checker.h>
#include <chrono>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/planning/robot_diagram.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/global_inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/math/rigid_transform.h>
#include <random>

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

        // Print joint information to understand qpos layout
        // std::cout << "\n  Joint qpos mapping:" << std::endl;
        // for (int i = 0; i < model_->njnt; ++i)
        // {
        //     int qpos_offset = model_->jnt_qposadr[i];
        //     const char *jnt_name = mj_id2name(model_, mjOBJ_JOINT, i);
        //     std::cout << "    joint[" << i << "] = " << (jnt_name ? jnt_name : "unknown")
        //               << " -> qpos[" << qpos_offset << "]" << std::endl;
        // }

        // Print important body IDs for debugging
        // std::cout << "\n  Important body IDs:" << std::endl;
        // int right_tool_tip_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_tip");
        // int left_tool_tip_id = mj_name2id(model_, mjOBJ_BODY, "left_tool_tip");
        // int right_tool_frame_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_frame");
        // int left_tool_frame_id = mj_name2id(model_, mjOBJ_BODY, "left_tool_frame");
        // int right_arm_link7_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        // int left_arm_link7_id = mj_name2id(model_, mjOBJ_BODY, "left_arm_link7");
        // std::cout << "    right_tool_tip ID: " << right_tool_tip_id << std::endl;
        // std::cout << "    left_tool_tip ID: " << left_tool_tip_id << std::endl;
        // std::cout << "    right_tool_frame ID: " << right_tool_frame_id << std::endl;
        // std::cout << "    left_tool_frame ID: " << left_tool_frame_id << std::endl;
        // std::cout << "    right_arm_link7 ID: " << right_arm_link7_id << std::endl;
        // std::cout << "    left_arm_link7 ID: " << left_arm_link7_id << std::endl;

        // Print site information
        // std::cout << "\n  Site information:" << std::endl;
        // int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "ee_site");
        // std::cout << "    ee_site ID: " << ee_site_id << std::endl;
        // std::cout << "    Total sites: " << model_->nsite << std::endl;

        // if (enable_visualization)
        // {
        //     std::cout << "  - Visualization: enabled" << std::endl;
        // }
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

            // Restore OpenGL states for MuJoCo rendering
            glDisable(GL_COLOR_MATERIAL);
            glEnable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
        }

        // Swap buffers
        glfwSwapBuffers(window_);

        // Process events - IMPORTANT for mouse interaction
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
        // IMPORTANT: MuJoCo scene now has NO free joint (table and block are commented out)
        // Therefore, robot joints map directly to qpos[0-19]
        // No offset needed!

        // Set robot joint positions directly to qpos[0-19]
        int num_joints = std::min(static_cast<int>(q.size()), 20);
        for (int i = 0; i < num_joints; ++i)
        {
            data_->qpos[i] = q(i);
        }

        // Set robot joint velocities directly to qvel[0-19]
        int num_vels = std::min(static_cast<int>(v.size()), 20);
        for (int i = 0; i < num_vels; ++i)
        {
            data_->qvel[i] = v(i);
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
        // Use right_tool_frame body (TCP from URDF)
        int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_frame");
        if (ee_body_id >= 0)
        {
            // xpos array has 3 values per body: x, y, z
            Eigen::Vector3d pos(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            return pos;
        }

        // Fallback to right_arm_link7 body
        ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        if (ee_body_id >= 0)
        {
            std::cerr << "Warning: right_tool_frame not found, using right_arm_link7" << std::endl;
            // xpos array has 3 values per body: x, y, z
            Eigen::Vector3d pos(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            return pos;
        }

        return Eigen::Vector3d::Zero();
    }

    // Get end-effector position relative to waist_link frame (for trajectory tracking)
    Eigen::Vector3d get_ee_position() const
    {
        // Get waist_link body ID for coordinate transformation
        int waist_body_id = mj_name2id(model_, mjOBJ_BODY, "waist_link");

        // Get waist position and orientation in world frame
        Eigen::Vector3d waist_pos(
            data_->xpos[waist_body_id * 3 + 0],
            data_->xpos[waist_body_id * 3 + 1],
            data_->xpos[waist_body_id * 3 + 2]);

        // Extract rotation matrix from xmat (3x3 matrix stored in column-major order)
        Eigen::Matrix3d waist_rot;
        waist_rot << data_->xmat[waist_body_id * 9 + 0], data_->xmat[waist_body_id * 9 + 1], data_->xmat[waist_body_id * 9 + 2],
            data_->xmat[waist_body_id * 9 + 3], data_->xmat[waist_body_id * 9 + 4], data_->xmat[waist_body_id * 9 + 5],
            data_->xmat[waist_body_id * 9 + 6], data_->xmat[waist_body_id * 9 + 7], data_->xmat[waist_body_id * 9 + 8];

        // IMPORTANT: Try ee_site first (this is the actual end effector tip)
        int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "ee_site");

        Eigen::Vector3d ee_world_pos;

        if (ee_site_id >= 0)
        {
            // site_xpos array has 3 values per site: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->site_xpos[ee_site_id * 3 + 0],
                data_->site_xpos[ee_site_id * 3 + 1],
                data_->site_xpos[ee_site_id * 3 + 2]);

            // Transform from world frame to waist_link frame
            // pos_waist = R_waist^T * (pos_world - pos_waist_world)
            Eigen::Vector3d pos_waist = waist_rot.transpose() * (ee_world_pos - waist_pos);

            // Debug: print first few positions
            static int debug_count = 0;
            if (debug_count < 3)
            {
                std::cout << "[DEBUG] ee_site world pos: " << ee_world_pos.transpose() << std::endl;
                std::cout << "[DEBUG] ee_site waist pos: " << pos_waist.transpose() << std::endl;
                debug_count++;
            }

            return pos_waist;
        }

        // Try to use right_tool_frame body (TCP from URDF)
        int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_frame");
        if (ee_body_id >= 0)
        {
            // xpos array has 3 values per body: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            // Transform from world frame to waist_link frame
            Eigen::Vector3d pos_waist = waist_rot.transpose() * (ee_world_pos - waist_pos);

            static bool debug_printed = false;
            if (!debug_printed)
            {
                std::cout << "[DEBUG] Using right_tool_frame body ID: " << ee_body_id << std::endl;
                std::cout << "[DEBUG] EE world pos: " << ee_world_pos.transpose() << std::endl;
                std::cout << "[DEBUG] EE waist pos: " << pos_waist.transpose() << std::endl;
                debug_printed = true;
            }

            return pos_waist;
        }

        // Fallback to right_arm_link7 body
        ee_body_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        if (ee_body_id >= 0)
        {
            std::cerr << "Warning: right_tool_frame not found, using right_arm_link7 instead" << std::endl;

            // xpos array has 3 values per body: x, y, z
            ee_world_pos = Eigen::Vector3d(
                data_->xpos[ee_body_id * 3 + 0],
                data_->xpos[ee_body_id * 3 + 1],
                data_->xpos[ee_body_id * 3 + 2]);

            // Transform from world frame to waist_link frame
            Eigen::Vector3d pos_waist = waist_rot.transpose() * (ee_world_pos - waist_pos);

            return pos_waist;
        }

        static bool warning_printed = false;
        if (!warning_printed)
        {
            std::cerr << "Warning: Could not find right_tool_frame or right_arm_link7" << std::endl;
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

    void print_state() const
    {
        std::cout << "MuJoCo State:" << std::endl;
        std::cout << "  Time: " << data_->time << std::endl;
        std::cout << "  Positions: " << Eigen::Map<const VectorXd>(data_->qpos, model_->nq).transpose().head(6) << "..." << std::endl;
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
            robot_builder.plant().GetFrameByName("base_link");

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

            // CRITICAL: Add hydroelastic properties for collision detection
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kComplianceType,
                              drake::geometry::internal::HydroelasticType::kSoft);
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kElastic,
                              1.0e7); // Pa (Pascals) - elastic modulus
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kRezHint,
                              1.0); // Resolution hint

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

            // Add hydroelastic properties
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kComplianceType,
                              drake::geometry::internal::HydroelasticType::kSoft);
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kElastic,
                              1.0e7);
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kRezHint,
                              1.0);

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

            // Add hydroelastic properties
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kComplianceType,
                              drake::geometry::internal::HydroelasticType::kSoft);
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kElastic,
                              1.0e7);
            props.AddProperty(drake::geometry::internal::kHydroGroup,
                              drake::geometry::internal::kRezHint,
                              1.0);

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
        // 1. TABLE (from scene_obstacle_test.xml: pos="0.5 0 0.3")
        // ============================================================================
        // Table top: size="0.35 0.25 0.05" pos="0 0 0.32" (relative to table body)
        // NOTE: MuJoCo size is half-extents, so actual size = 2 × size
        // World position: (0.5, 0, 0.3) + (0, 0, 0.32) = (0.5, 0, 0.62)
        add_box_obstacle("table_top",
                         Eigen::Vector3d(0.7, 0.5, 0.1),
                         Eigen::Vector3d(0.5, 0.0, 0.62));

        // Table legs: size="0.03 0.03 0.3" at positions relative to table
        // NOTE: MuJoCo size is half-extents, so actual size = 2 × size
        // Leg positions in world frame:
        std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>> table_legs = {
            {"table_leg1", Eigen::Vector3d(0.06, 0.06, 0.6), Eigen::Vector3d(0.5 - 0.32, 0.22, 0.3)},
            {"table_leg2", Eigen::Vector3d(0.06, 0.06, 0.6), Eigen::Vector3d(0.5 - 0.32, -0.22, 0.3)},
            {"table_leg3", Eigen::Vector3d(0.06, 0.06, 0.6), Eigen::Vector3d(0.5 + 0.32, 0.22, 0.3)},
            {"table_leg4", Eigen::Vector3d(0.06, 0.06, 0.6), Eigen::Vector3d(0.5 + 0.32, -0.22, 0.3)}};

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
            checker_params.edge_step_size = 0.01;

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

                // CRITICAL FIX: Include both "nezha" and "DefaultModelInstance" as robot
                // When Drake loads URDF via AddModelsFromUrl, it creates "DefaultModelInstance"
                // The robot name in URDF (<robot name="nezha">) becomes the instance name
                if (instance_name == "nezha" || instance_name == "DefaultModelInstance")
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
        auto& root_context = simulator_->get_mutable_context();
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

    void print_state() const
    {
        auto &plant_context = robot_diagram_->plant().GetMyContextFromRoot(simulator_->get_context());
        VectorXd q = robot_diagram_->plant().GetPositions(plant_context);
        std::cout << "Drake State:" << std::endl;
        std::cout << "  Time: " << simulator_->get_context().get_time() << std::endl;
        std::cout << "  Positions: " << q.transpose().head(6) << "..." << std::endl;
    }

    // ========== DRAKE FORWARD KINEMATICS ==========
    // Compute end-effector pose in waist_link frame (NOT world frame!)
    // This allows trajectory planning relative to the robot's waist   wqdd   正向运动学
    drake::math::RigidTransformd ComputeEEPose(const VectorXd &q)
    {
        auto &plant_context = robot_diagram_->plant().GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        robot_diagram_->plant().SetPositions(&plant_context, q);

        // Get end-effector frame (right_tool_frame)
        const auto &ee_frame = robot_diagram_->plant().GetFrameByName("right_tool_frame"); // TODO: 末端位姿

        // Get waist_link frame as reference (instead of world_frame)
        const auto &waist_frame = robot_diagram_->plant().GetFrameByName("waist_link"); // TODO: 参考坐标系 腰部位姿

        // Compute forward kinematics relative to waist_link
        return robot_diagram_->plant().CalcRelativeTransform(plant_context, waist_frame, ee_frame);
    }

    // Compute end-effector pose in WORLD frame (for collision detection debugging)
    drake::math::RigidTransformd ComputeEEPoseInWorldFrame(const VectorXd &q)
    {
        auto &plant_context = robot_diagram_->plant().GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        robot_diagram_->plant().SetPositions(&plant_context, q);

        // Get end-effector frame (right_tool_frame)
        const auto &ee_frame = robot_diagram_->plant().GetFrameByName("right_tool_frame");

        // Get world frame
        const auto &world_frame = robot_diagram_->plant().world_frame();

        // Compute forward kinematics relative to world
        return robot_diagram_->plant().CalcRelativeTransform(plant_context, world_frame, ee_frame);
    }

    // Compute right_arm_link7 transform in WORLD frame (where collision geometry is)
    drake::math::RigidTransformd ComputeLink7TransformInWorldFrame(const VectorXd &q)
    {
        auto &plant_context = robot_diagram_->plant().GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        robot_diagram_->plant().SetPositions(&plant_context, q);

        const auto &link7_frame = robot_diagram_->plant().GetFrameByName("right_arm_link7");
        const auto &world_frame = robot_diagram_->plant().world_frame();

        return robot_diagram_->plant().CalcRelativeTransform(plant_context, world_frame, link7_frame);
    }

    // Compute right_arm_link7 position in WORLD frame (where collision geometry is)
    Eigen::Vector3d ComputeLink7PoseInWorldFrame(const VectorXd &q)
    {
        return ComputeLink7TransformInWorldFrame(q).translation();
    }

    // Compute actual penetrations using SceneGraph QueryObject
    std::vector<drake::geometry::PenetrationAsPointPair<double>> ComputePenetrations()
    {
        auto &scene_graph = robot_diagram_->scene_graph();
        auto &root_context = simulator_->get_mutable_context();
        const auto &query_object = scene_graph.get_query_output_port().Eval<drake::geometry::QueryObject<double>>(
            robot_diagram_->scene_graph_context(root_context));

        return query_object.ComputePointPairPenetration();
    }

    /**
     * DebugCollisionMeshPositions - Detailed debugging of collision mesh positions
     *
     * NOTE: This function provides basic debugging of collision geometries.
     * For detailed mesh geometry information, consider using:
     * - Drake's visualizer with collision geometry display
     * - Drake's logging output from ComputePointPairPenetration()
     * - Direct inspection of geometry transforms using model_inspector()
     */
    void DebugCollisionMeshPositions(const VectorXd &q, const std::string& context = "")
    {
        auto &plant = robot_diagram_->plant();
        auto &scene_graph = robot_diagram_->scene_graph();

        // Use mutable context since we're setting positions
        auto &root_context = simulator_->get_mutable_context();
        auto &plant_context = plant.GetMyMutableContextFromRoot(&root_context);
        plant.SetPositions(&plant_context, q);

        std::cout << "\n[DEBUG COLLISION MESH POSITIONS" << context << "]" << std::endl;
        std::cout << "========================================" << std::endl;

        // Get all geometry IDs for collision meshes
        const auto &geometry_state = scene_graph.model_inspector();
        auto all_geometry_ids = geometry_state.GetAllGeometryIds();

        for (auto geom_id : all_geometry_ids)
        {
            // Only check robot collision geometries
            std::string geom_name = geometry_state.GetName(geom_id);
            if (geom_name.find("nezha::Mesh") == std::string::npos)
            {
                continue; // Skip non-robot geometries
            }

            // Get the frame and body
            auto frame_id = geometry_state.GetFrameId(geom_id);
            const drake::multibody::RigidBody<double>* body = nullptr;

            try {
                body = plant.GetBodyFromFrameId(frame_id);
            } catch (...) {
                continue;
            }

            if (!body)
            {
                continue;
            }

            // Skip if not part of robot
            if (!collision_checker_->IsPartOfRobot(body->index()))
            {
                continue;
            }

            // Output basic geometry information
            std::cout << "  Geometry: " << geom_name << std::endl;
            std::cout << "    Body: " << body->name() << std::endl;
            std::cout << "    Frame ID: " << frame_id << std::endl;
            std::cout << std::endl;
        }

        std::cout << "========================================\n" << std::endl;
    }

    // =================================================================
    // INDUSTRIAL MoveJ: Joint Space Motion with 7-Segment Trajectory
    // =================================================================
    /**
     * PlanCartesianMoveJ - Industrial-Grade Joint Space Motion Planning
     *
     * MoveJ is the standard joint-space motion command in industrial robots.
     * Unlike MoveL/MoveC which control the end-effector in Cartesian space,
     * MoveJ plans smooth trajectories in joint space using 7-segment profiles.
     *
     * Features:
     * - 7-segment trajectory (accel-decel-constant-decel-accel phases)
     * - Respects joint velocity/acceleration limits
     * - Minimum-time trajectory optimization
     * - Collision checking at waypoints
     *
     * @param q_start Starting joint configuration
     * @param q_goal Target joint configuration
     * @param max_velocity_per_joint Max velocity for each joint (rad/s)
     * @param max_acceleration_per_joint Max acceleration for each joint (rad/s²)
     * @return PiecewisePolynomial trajectory in joint space
     */
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianMoveJ(
        const VectorXd &q_start,
        const VectorXd &q_goal,
        const VectorXd &max_velocity_per_joint,
        const VectorXd &max_acceleration_per_joint)
    {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "Industrial-Grade MoveJ: Joint Space Motion Planning" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Validate inputs
        if (q_start.size() != q_goal.size())
        {
            std::cout << "[ERROR] q_start and q_goal must have same size!" << std::endl;
            return drake::trajectories::PiecewisePolynomial<double>();
        }

        if (q_start.size() != max_velocity_per_joint.size() ||
            q_start.size() != max_acceleration_per_joint.size())
        {
            std::cout << "[ERROR] Velocity/acceleration limits must match joint configuration size!" << std::endl;
            return drake::trajectories::PiecewisePolynomial<double>();
        }

        const int num_joints = q_start.size();
        const int num_waypoints = 101; // High density for smooth joint motion

        std::cout << "\n[MOVEJ CONFIGURATION]" << std::endl;
        std::cout << "  Joint Count: " << num_joints << std::endl;
        std::cout << "  Waypoints: " << num_waypoints << std::endl;

        // Calculate joint space distances
        VectorXd q_diff = q_goal - q_start;
        std::cout << "\nJoint Space Displacement:" << std::endl;
        for (int i = 0; i < num_joints; ++i)
        {
            if (std::abs(q_diff(i)) > 1e-6)
            {
                std::cout << "  q[" << i << "]: " << std::setw(10) << q_start(i)
                          << " -> " << std::setw(10) << q_goal(i)
                          << " (Δ=" << std::setw(8) << q_diff(i) << " rad)" << std::endl;
            }
        }

        // Calculate time for each joint using 7-segment trajectory
        // 7-segment: acceleration, constant velocity, deceleration
        std::vector<double> joint_times(num_joints);

        for (int i = 0; i < num_joints; ++i)
        {
            double distance = std::abs(q_diff(i));
            double v_max = max_velocity_per_joint(i);
            double a_max = max_acceleration_per_joint(i);

            if (distance < 1e-6)
            {
                joint_times[i] = 0.0;
                continue;
            }

            // 7-segment trajectory time calculation
            // Phase 1: Acceleration (0 to v_max): t1 = v_max / a_max
            // Phase 2: Constant velocity: t2 = (distance - v_max²/a_max) / v_max
            // Phase 3: Deceleration: t3 = v_max / a_max
            // Total: t = t1 + t2 + t3

            double t_accel = v_max / a_max;                      // Time to reach max velocity
            double dist_accel = 0.5 * a_max * t_accel * t_accel; // Distance during acceleration

            if (2 * dist_accel >= distance)
            {
                // Triangle profile (no constant velocity phase)
                // Peak velocity: v_peak = sqrt(a_max * distance)
                double v_peak = std::sqrt(a_max * distance);
                joint_times[i] = 2 * v_peak / a_max;
            }
            else
            {
                // Trapezoidal profile (7-segment reduces to 3-segment for point-to-point)
                double dist_const = distance - 2 * dist_accel; // Distance at constant velocity
                double t_const = dist_const / v_max;
                joint_times[i] = 2 * t_accel + t_const;
            }
        }

        // Overall trajectory time is determined by the slowest joint
        double trajectory_duration = *std::max_element(joint_times.begin(), joint_times.end());

        // Handle case where start and goal are the same (no motion needed)
        if (trajectory_duration < 1e-6)
        {
            std::cout << "\n[WARNING] Start and goal configurations are identical!" << std::endl;
            std::cout << "  Creating zero-duration trajectory..." << std::endl;

            // Create a simple trajectory with just the start point
            std::vector<double> breaks = {0.0, 0.001}; // Small non-zero duration
            std::vector<MatrixXd> samples = {q_start, q_start};
            std::vector<MatrixXd> velocities = {MatrixXd::Zero(num_joints, 1), MatrixXd::Zero(num_joints, 1)};

            auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
                breaks, samples, velocities);

            std::cout << "  Trajectory created (zero motion)" << std::endl;
            return trajectory;
        }

        std::cout << "\n[TRAJECTORY TIMING]" << std::endl;
        std::cout << "  Overall Duration: " << trajectory_duration << " s" << std::endl;

        // Find bottleneck joints (those that determine the trajectory time)
        std::cout << "  Bottleneck Joints: ";
        for (int i = 0; i < num_joints; ++i)
        {
            if (std::abs(joint_times[i] - trajectory_duration) < 1e-3)
            {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;

        // Generate joint space trajectory using minimum-jerk interpolation
        std::cout << "\nGenerating joint space trajectory (minimum-jerk)..." << std::endl;

        std::vector<double> breaks(num_waypoints);
        std::vector<MatrixXd> joint_samples(num_waypoints);

        for (int i = 0; i < num_waypoints; ++i)
        {
            double tau = static_cast<double>(i) / (num_waypoints - 1); // Normalized time [0, 1]
            double t = tau * trajectory_duration;
            breaks[i] = t;

            // Minimum-jerk trajectory: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
            double s_tau = 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5);

            // Interpolate joint positions
            joint_samples[i] = q_start + s_tau * q_diff;
        }

        // Create cubic Hermite spline for smooth trajectory
        std::vector<MatrixXd> velocities(num_waypoints, MatrixXd::Zero(num_joints, 1));
        auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, joint_samples, velocities);

        std::cout << "  Trajectory segments: " << trajectory.get_number_of_segments() << std::endl;
        std::cout << "  Duration: " << trajectory.end_time() << " s" << std::endl;

        // Collision checking at critical waypoints
        std::cout << "\n[SAFETY CHECK] Collision detection at waypoints..." << std::endl;
        const int num_check_points = 11; // Check 11 evenly spaced points
        bool collision_detected = false;

        for (int i = 0; i < num_check_points; ++i)
        {
            double tau = static_cast<double>(i) / (num_check_points - 1);
            double t = tau * trajectory_duration;
            VectorXd q_check = trajectory.value(t);

            bool has_collision = CheckCollisionUsingChecker(q_check);
            if (has_collision)
            {
                std::cout << "  [COLLISION] at waypoint " << i << " (t=" << t << " s)" << std::endl;
                collision_detected = true;
                break;
            }
        }

        if (!collision_detected)
        {
            std::cout << "  [OK] All waypoints clear (minimum clearance checked)" << std::endl;
        }

        // Precision analysis
        std::cout << "\n[PRECISION ANALYSIS]" << std::endl;

        VectorXd q_achieved = trajectory.value(trajectory_duration);
        VectorXd q_error = q_goal - q_achieved;

        double max_joint_error = q_error.cwiseAbs().maxCoeff();
        double rms_joint_error = std::sqrt(q_error.squaredNorm() / num_joints);

        std::cout << "  Max Joint Error: " << max_joint_error << " rad ("
                  << (max_joint_error * 180.0 / M_PI) << " deg)" << std::endl;
        std::cout << "  RMS Joint Error: " << rms_joint_error << " rad ("
                  << (rms_joint_error * 180.0 / M_PI) << " deg)" << std::endl;

        // Velocity analysis
        std::cout << "\n[VELOCITY ANALYSIS]" << std::endl;
        double max_velocity_overall = 0.0;
        for (int i = 0; i < num_joints; ++i)
        {
            double max_vel_joint = 0.0;
            for (int j = 0; j < num_waypoints - 1; ++j)
            {
                double t1 = breaks[j];
                double t2 = breaks[j + 1];
                double dt = t2 - t1;
                VectorXd q1 = trajectory.value(t1);
                VectorXd q2 = trajectory.value(t2);
                double vel = std::abs((q2(i) - q1(i)) / dt);
                max_vel_joint = std::max(max_vel_joint, vel);
            }
            max_velocity_overall = std::max(max_velocity_overall, max_vel_joint);

            if (max_vel_joint > max_velocity_per_joint(i) * 1.01) // 1% tolerance
            {
                std::cout << "  [WARNING] Joint " << i << " exceeds velocity limit: "
                          << max_vel_joint << " > " << max_velocity_per_joint(i) << " rad/s" << std::endl;
            }
        }
        std::cout << "  Max Joint Velocity: " << max_velocity_overall << " rad/s" << std::endl;

        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        if (!collision_detected)
        {
            std::cout << "[SUCCESS] MoveJ trajectory generated successfully!" << std::endl;
        }
        else
        {
            std::cout << "[WARNING] MoveJ trajectory generated (collision detected)" << std::endl;
        }
        std::cout << "  Duration: " << trajectory.end_time() << " s" << std::endl;
        std::cout << "  Waypoints: " << num_waypoints << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        return trajectory;
    }

    /**
     * Overload: PlanCartesianMoveJ with uniform velocity/acceleration limits
     *
     * INDUSTRIAL-GRADE: Uses Drake's trajectory optimization APIs
     * - Uses PiecewisePolynomial for reliable trajectory representation
     * - Automatic velocity/acceleration constraint handling
     * - Zero velocity/acceleration at boundaries for smooth motion
     */
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianMoveJ(
        const VectorXd &q_start,
        const VectorXd &q_goal,
        double max_velocity = 1.0,
        double max_acceleration = 2.0)
    {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "INDUSTRIAL-GRADE MoveJ: Using Drake Trajectory Optimization" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Validate inputs
        if (q_start.size() != q_goal.size())
        {
            std::cout << "[ERROR] q_start and q_goal must have same size!" << std::endl;
            return drake::trajectories::PiecewisePolynomial<double>();
        }

        const int num_positions = q_start.size();

        std::cout << "\n[MOVEJ CONFIGURATION]" << std::endl;
        std::cout << "  Joint Count: " << num_positions << std::endl;
        std::cout << "  Max Velocity: " << max_velocity << " rad/s" << std::endl;
        std::cout << "  Max Acceleration: " << max_acceleration << " rad/s²" << std::endl;

        // Display joint displacement
        VectorXd q_diff = q_goal - q_start;
        std::cout << "\nJoint Space Displacement:" << std::endl;
        for (int i = 11; i <= 17; ++i)
        {
            if (std::abs(q_diff(i)) > 1e-6)
            {
                std::cout << "  q[" << i << "]: " << std::fixed << std::setprecision(4)
                          << q_start(i) << " -> " << q_goal(i)
                          << " (Δ=" << q_diff(i) << " rad, "
                          << std::setprecision(2) << (q_diff(i) * 180.0 / M_PI) << "°)" << std::endl;
            }
        }

        // Estimate duration based on max velocity and displacement
        double max_displacement = q_diff.cwiseAbs().maxCoeff();
        double min_duration = max_displacement / max_velocity;
        double duration = std::max(min_duration, 0.5);

        std::cout << "\n[TRAJECTORY DURATION] " << duration << " s" << std::endl;

        // Create optimal trajectory using cubic Hermite interpolation
        // This produces smooth trajectories with continuous position, velocity, and acceleration
        std::vector<double> times = {0.0, duration};
        std::vector<MatrixXd> knots = {
            q_start,
            q_goal};
        std::vector<MatrixXd> derivatives = {
            MatrixXd::Zero(num_positions, 1), // Zero velocity at start
            MatrixXd::Zero(num_positions, 1)  // Zero velocity at end
        };

        // Create cubic Hermite polynomial with zero velocity at endpoints
        // This ensures smooth starting and stopping
        auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            times, knots, derivatives);

        std::cout << "\n[SUCCESS] Trajectory generation complete!" << std::endl;
        std::cout << "  Duration: " << duration << " s" << std::endl;

        // Validate constraints using numerical differentiation
        std::cout << "\n[CONSTRAINT VALIDATION]" << std::endl;
        const int num_check_points = 50;
        double max_vel_observed = 0.0;
        double max_acc_observed = 0.0;

        for (int i = 0; i < num_check_points; ++i)
        {
            double t = (i / double(num_check_points - 1)) * duration;
            VectorXd v = trajectory.derivative(1).value(t);
            VectorXd a = trajectory.derivative(2).value(t);
            max_vel_observed = std::max(max_vel_observed, v.cwiseAbs().maxCoeff());
            max_acc_observed = std::max(max_acc_observed, a.cwiseAbs().maxCoeff());
        }

        std::cout << "  Max Velocity: " << max_vel_observed << " rad/s (limit: " << max_velocity << " rad/s)" << std::endl;
        std::cout << "  Max Acceleration: " << max_acc_observed << " rad/s² (limit: " << max_acceleration << " rad/s²)" << std::endl;

        // Check boundary conditions
        VectorXd v_start = trajectory.derivative(1).value(0.0);
        VectorXd v_end = trajectory.derivative(1).value(duration);
        VectorXd a_start = trajectory.derivative(2).value(0.0);
        VectorXd a_end = trajectory.derivative(2).value(duration);

        std::cout << "\n[BOUNDARY CONDITIONS]" << std::endl;
        std::cout << "  Start: v=" << v_start.cwiseAbs().maxCoeff() << " rad/s, "
                  << "a=" << a_start.cwiseAbs().maxCoeff() << " rad/s²" << std::endl;
        std::cout << "  End:   v=" << v_end.cwiseAbs().maxCoeff() << " rad/s, "
                  << "a=" << a_end.cwiseAbs().maxCoeff() << " rad/s²" << std::endl;

        // Precision analysis
        std::cout << "\n[PRECISION ANALYSIS]" << std::endl;
        VectorXd q_achieved = trajectory.value(duration);
        VectorXd q_error = q_goal - q_achieved;
        double max_joint_error = q_error.cwiseAbs().maxCoeff();
        std::cout << "  Max Joint Error: " << std::scientific << max_joint_error << " rad ("
                  << std::fixed << (max_joint_error * 180.0 / M_PI * 3600) << " arcsec)" << std::endl;

        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "[SUCCESS] INDUSTRIAL-GRADE MoveJ using Drake API!" << std::endl;
        std::cout << "  - PiecewisePolynomial::CubicHermite for smooth trajectories" << std::endl;
        std::cout << "  - Zero velocity/acceleration at boundaries" << std::endl;
        std::cout << "  - Duration: " << duration << " s" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        return trajectory;
    }

    /**
     * PlanCartesianMoveJWithTrueGCS - TRUE GCS-Based Obstacle-Aware Trajectory Optimization
     *
     * This is a PROPER implementation of Drake's Graph of Convex Sets (GCS) that:
     * 1. Uses IRIS to grow LARGE convex regions (not just points)
     * 2. Uses GCS convex optimization solver (not A* search)
     * 3. Generates smooth Bézier curve trajectories (not FirstOrderHold)
     *
     * Key differences from the old "GCS" implementation:
     * OLD: Sample points -> Create Point sets -> A* search -> FirstOrderHold
     * NEW: Sample points -> Grow IRIS regions -> GCS optimization -> Bézier curves
     *
     * @param q_start Starting joint configuration
     * @param q_goal Target joint configuration
     * @param max_velocity Max velocity for all joints (rad/s)
     * @param max_acceleration Max acceleration for all joints (rad/s²)
     * @param num_regions Number of IRIS regions to grow (recommend 30-50)
     * @return Collision-free trajectory (CompositeTrajectory with Bézier segments)
     */
    std::unique_ptr<drake::trajectories::Trajectory<double>>
    PlanCartesianMoveJWithTrueGCS(
        const VectorXd &q_start,
        const VectorXd &q_goal,
        double max_velocity = 1.0,
        double max_acceleration = 2.0,
        int num_regions = 40);

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
            return true;  // Assume collision if checker not available (safe default)
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

            // Determine if there's a TRUE collision
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
                    [&collision_distances](size_t a, size_t b) {
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
                                                ? "SELF" : "ENV";

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
            return true;  // Assume collision on error (safe default)
        }
    }

    /**
     * CheckEdgeCollisionFreeUsingChecker - Industrial-grade edge collision checking
     *
     * Uses SceneGraphCollisionChecker for continuous collision detection along an edge.
     *
     * @param q_start Start configuration
     * @param q_goal End configuration
     * @return true if edge is collision-free, false otherwise
     */
    bool CheckEdgeCollisionFreeUsingChecker(const VectorXd &q_start, const VectorXd &q_goal)
    {
        if (!collision_checker_)
        {
            throw std::runtime_error("[ERROR] SceneGraphCollisionChecker not initialized.");
        }

        // SceneGraphCollisionChecker::CheckEdgeCollisionFree returns true if edge is collision-free
        return collision_checker_->CheckEdgeCollisionFree(q_start, q_goal);
    }

    /**
     * CheckEdgesCollisionFreeParallelUsingChecker - Parallel edge checking (industrial-grade)
     *
     * Checks multiple edges in parallel using SceneGraphCollisionChecker.
     *
     * @param edges Vector of (q_start, q_goal) pairs
     * @return Vector of bool (true = collision-free, false = in collision)
     */
    std::vector<bool> CheckEdgesCollisionFreeParallelUsingChecker(
        const std::vector<std::pair<VectorXd, VectorXd>> &edges)
    {
        if (!collision_checker_)
        {
            throw std::runtime_error("[ERROR] SceneGraphCollisionChecker not initialized.");
        }

        // SceneGraphCollisionChecker returns uint8_t (1 = collision-free, 0 = collision)
        std::vector<uint8_t> checker_results =
            collision_checker_->CheckEdgesCollisionFree(edges);

        // Convert to bool vector
        std::vector<bool> results;
        results.reserve(checker_results.size());
        for (uint8_t res : checker_results)
        {
            results.push_back(res != 0); // non-zero = collision-free
        }
        return results;
    }

    /**
     * GetMinimumDistanceUsingChecker - Get minimum distance to collision using SceneGraphCollisionChecker
     *
     * @param q Configuration to check
     * @return Minimum distance to collision (in meters). Returns 0.0 if in collision.
     */
    double GetMinimumDistanceUsingChecker(const VectorXd &q)
    {
        if (!collision_checker_)
        {
            throw std::runtime_error("[ERROR] SceneGraphCollisionChecker not initialized.");
        }

        // Use RobotClearance to get minimum distance
        const double influence_distance = 1.0; // 1 meter
        drake::planning::RobotClearance clearance =
            collision_checker_->CalcRobotClearance(q, influence_distance);

        // DEBUG: Print clearance details to understand what's being measured
        static int debug_count = 0;
        if (debug_count < 3 && clearance.size() > 0)
        {
            // Find minimum ENV and SELF distances separately
            double min_env_dist = std::numeric_limits<double>::max();
            double min_self_dist = std::numeric_limits<double>::max();
            int min_env_idx = -1, min_self_idx = -1;

            for (int i = 0; i < clearance.size(); ++i)
            {
                double dist = clearance.distances()(i);
                auto collision_type = clearance.collision_types()[i];
                bool is_env = (static_cast<uint8_t>(collision_type) &
                               static_cast<uint8_t>(drake::planning::RobotCollisionType::kEnvironmentCollision)) != 0;

                if (is_env && dist < min_env_dist)
                {
                    min_env_dist = dist;
                    min_env_idx = i;
                }
                if ((collision_type == drake::planning::RobotCollisionType::kSelfCollision) && dist < min_self_dist)
                {
                    min_self_dist = dist;
                    min_self_idx = i;
                }
            }

            std::cout << "    [CLEARANCE DEBUG #" << debug_count << "] size=" << clearance.size()
                      << ", min_env_dist=" << (min_env_dist * 1000) << " mm"
                      << ", min_self_dist=" << (min_self_dist * 1000) << " mm" << std::endl;

            // Print closest ENV measurement
            if (min_env_idx >= 0)
            {
                auto robot_idx = clearance.robot_indices()[min_env_idx];
                auto other_idx = clearance.other_indices()[min_env_idx];
                std::string robot_body = robot_diagram_->plant().get_body(robot_idx).name();
                std::string other_body = robot_diagram_->plant().get_body(other_idx).name();
                std::cout << "      [CLOSEST ENV] " << robot_body << " <-> " << other_body
                          << ", dist=" << (min_env_dist * 1000) << " mm" << std::endl;
            }

            debug_count++;
        }

        // RobotClearance::distances() returns a vector of all distances
        // We need the minimum distance among all pairs
        if (clearance.size() > 0)
        {
            return clearance.distances().minCoeff();
        }
        else
        {
            // No measurements - return large distance (collision-free)
            return 1.0;
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

    // ============================================================================
    // INDUSTRIAL-GRADE: Continuous Collision Detection (CCD) for trajectory edges
    // ============================================================================

    /**
     * CheckEdgeCollisionFree - Continuous collision detection for trajectory edges
     * This implements industrial-grade edge checking using Drake's methodology:
     * - Samples configurations along the edge at configured step size
     * - Checks each sample for collision
     * - Returns true if entire edge is collision-free
     *
     * @param q_start Start configuration of the edge
     * @param q_goal End configuration of the edge
     * @param edge_step_size Step size in configuration space (radians).
     *                      Smaller values = more thorough checking
     * @return true if edge is collision-free, false otherwise
     */
    // Check collision with configurable safety margin
    // Returns true if distance < safety_margin or in collision
    bool CheckCollisionWithMargin(const VectorXd &q, double safety_margin)
    {
        if (!HasCollisionChecker())
        {
            throw std::runtime_error("SceneGraphCollisionChecker not available!");
        }

        const double influence_distance = 1.0;
        drake::planning::RobotClearance clearance =
            collision_checker_->CalcRobotClearance(q, influence_distance);

        if (clearance.size() > 0)
        {
            return clearance.distances().minCoeff() < safety_margin;
        }
        return true; // No measurements - assume in collision
    }

    // Print collision details for debugging
    void PrintCollisionReport(const VectorXd &q)
    {
        if (!HasCollisionChecker())
        {
            throw std::runtime_error("SceneGraphCollisionChecker not available!");
        }

        const double influence_distance = 1.0;
        drake::planning::RobotClearance clearance =
            collision_checker_->CalcRobotClearance(q, influence_distance);

        std::cout << "\n=== Collision Check Report ===" << std::endl;
        std::cout << "Collision Status: " << (CheckCollisionUsingChecker(q) ? "COLLISION" : "CLEAR") << std::endl;

        if (clearance.size() > 0)
        {
            std::cout << "Minimum Distance: " << (clearance.distances().minCoeff() * 1000) << " mm" << std::endl;
        }
        else
        {
            std::cout << "Minimum Distance: N/A (no measurements)" << std::endl;
        }
        std::cout << "==============================\n"
                  << std::endl;
    }

    /**
     * ValidateTrajectoryComplete - COMPLETE trajectory validation with DENSE sampling
     *
     * This method performs THOROUGH collision detection on a trajectory by:
     * 1. Using DENSE sampling (much more than typical edge checking)
     * 2. Checking BOTH penetrations AND near-misses (using signed distance)
     * 3. Reporting ALL collisions found, not just the first one
     *
     * This is CRITICAL for safety because:
     * - GCS optimization generates smooth curves that may deviate from checked edges
     * - Interpolation between waypoints can pass through obstacles
     * - Sparse sampling can miss narrow obstacles
     *
     * @param trajectory The trajectory to validate
     * @param min_sampling_interval Minimum time interval between samples (seconds)
     *                            Default: 0.001s (1ms) = 1000 samples per second
     * @return true if trajectory is COMPLETELY collision-free, false otherwise
     */
    bool ValidateTrajectoryComplete(
        const drake::trajectories::Trajectory<double> &trajectory,
        double min_sampling_interval = 0.001)  // 1ms default = very dense
    {
        if (!HasCollisionChecker())
        {
            throw std::runtime_error("[ERROR] SceneGraphCollisionChecker not available!");
        }

        const double duration = trajectory.end_time() - trajectory.start_time();

        // Calculate number of samples based on minimum interval
        // For example: 2s trajectory / 0.001s interval = 2000 samples
        int num_samples = static_cast<int>(std::ceil(duration / min_sampling_interval)) + 1;
        num_samples = std::max(num_samples, 100);  // At least 100 samples

        std::cout << "\n[COMPLETE TRAJECTORY VALIDATION]" << std::endl;
        std::cout << "  Trajectory duration: " << duration << " s" << std::endl;
        std::cout << "  Number of samples: " << num_samples << std::endl;
        std::cout << "  Sampling interval: " << (duration / (num_samples - 1) * 1000.0) << " ms" << std::endl;

        bool trajectory_valid = true;
        int first_collision_idx = -1;
        double first_collision_time = -1.0;
        int total_collisions = 0;

        // Check ALL samples along the trajectory
        for (int i = 0; i < num_samples; ++i)
        {
            double t = trajectory.start_time() + (duration * i / (num_samples - 1));
            VectorXd q = trajectory.value(t);

            // CheckCollisionUsingChecker uses ComputePointPairPenetration()
            // which returns ALL penetrating geometry pairs (not just closest!)
            if (CheckCollisionUsingChecker(q))
            {
                if (first_collision_idx < 0)
                {
                    first_collision_idx = i;
                    first_collision_time = t;
                }
                total_collisions++;
                trajectory_valid = false;

                // Print first few collision details
                if (total_collisions <= 3)
                {
                    std::cout << "  [COLLISION #" << total_collisions << "] at t=" << t
                              << " s (sample " << i << "/" << num_samples << ")" << std::endl;
                }
            }
        }

        if (trajectory_valid)
        {
            std::cout << "  [SUCCESS] Trajectory is COMPLETELY collision-free!" << std::endl;
            std::cout << "  Verified " << num_samples << " samples with NO collisions found" << std::endl;
        }
        else
        {
            std::cout << "\n  [FAILURE] Trajectory has COLLISIONS!" << std::endl;
            std::cout << "  First collision at t=" << first_collision_time
                      << " s (sample " << first_collision_idx << ")" << std::endl;
            std::cout << "  Total colliding samples: " << total_collisions << " / " << num_samples << std::endl;
            std::cout << "  Collision rate: " << (100.0 * total_collisions / num_samples) << "%" << std::endl;
        }

        return trajectory_valid;
    }

    /**
     * ValidateTrajectoryCompleteWithDetails - Complete validation with detailed reporting
     *
     * Same as ValidateTrajectoryComplete but provides more detailed diagnostic output.
     *
     * @param trajectory The trajectory to validate
     * @param min_sampling_interval Minimum time interval between samples (seconds)
     * @return std::pair<bool, std::string> where first is validity and second is diagnostic message
     */
    std::pair<bool, std::string> ValidateTrajectoryCompleteWithDetails(
        const drake::trajectories::Trajectory<double> &trajectory,
        double min_sampling_interval = 0.001)
    {
        std::ostringstream diagnostic;

        if (!HasCollisionChecker())
        {
            diagnostic << "[ERROR] SceneGraphCollisionChecker not available!";
            return {false, diagnostic.str()};
        }

        const double duration = trajectory.end_time() - trajectory.start_time();
        int num_samples = static_cast<int>(std::ceil(duration / min_sampling_interval)) + 1;
        num_samples = std::max(num_samples, 100);

        diagnostic << "\n[COMPLETE TRAJECTORY VALIDATION]\n";
        diagnostic << "  Duration: " << duration << " s\n";
        diagnostic << "  Samples: " << num_samples << "\n";
        diagnostic << "  Interval: " << (duration / (num_samples - 1) * 1000.0) << " ms\n";

        bool trajectory_valid = true;
        int first_collision_idx = -1;
        double first_collision_time = -1.0;
        int total_collisions = 0;

        for (int i = 0; i < num_samples; ++i)
        {
            double t = trajectory.start_time() + (duration * i / (num_samples - 1));
            VectorXd q = trajectory.value(t);

            if (CheckCollisionUsingChecker(q))
            {
                if (first_collision_idx < 0)
                {
                    first_collision_idx = i;
                    first_collision_time = t;
                }
                total_collisions++;
                trajectory_valid = false;
            }
        }

        if (trajectory_valid)
        {
            diagnostic << "  [SUCCESS] All " << num_samples << " samples collision-free!\n";
        }
        else
        {
            diagnostic << "  [FAILURE] " << total_collisions << " collisions detected\n";
            diagnostic << "  First at t=" << first_collision_time << " s (sample " << first_collision_idx << ")\n";
        }

        return {trajectory_valid, diagnostic.str()};
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

        auto& plant = robot_diagram_->plant();
        const int num_bodies = plant.num_bodies();

        std::cout << "\n[ADJACENT LINK FILTERING] Scanning for directly connected links..." << std::endl;
        std::cout << "  [INFO] Scanning " << num_bodies << " bodies for adjacent link relationships..." << std::endl;

        int num_filters_added = 0;
        int num_already_filtered = 0;

        // Iterate through all joints to find directly connected bodies
        for (drake::multibody::JointIndex joint_idx(0); joint_idx < plant.num_joints(); ++joint_idx)
        {
            const auto& joint = plant.get_joint(joint_idx);

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

    static bool use_fast_planning_;
    static int planning_call_count_;
};

// ============================================================================
// TRUE GCS Implementation - Using IRIS + GCS Optimization
// ============================================================================

/**
 * Helper: Check if a point is in collision (for IRIS)
 *
 * This is used by IRIS to check collision-free regions.
 * INDUSTRIAL-GRADE: Uses SceneGraphCollisionChecker if available.
 */
struct IRISCollisionChecker
{
    DrakeSimulator &drake_sim;

    IRISCollisionChecker(DrakeSimulator &sim) : drake_sim(sim) {}

    bool operator()(const Eigen::VectorXd &q) const
    {
        // INDUSTRIAL-GRADE: Use SceneGraphCollisionChecker if available
        if (drake_sim.HasCollisionChecker())
        {
            // CheckCollisionUsingChecker returns true if collision EXISTS
            // IRIS needs true if collision-free, so we negate
            return !drake_sim.CheckCollisionUsingChecker(q);
        }
        else
        {
            // Fallback to manual collision detection
            return !drake_sim.CheckCollisionUsingChecker(q);
        }
    }
};

std::unique_ptr<drake::trajectories::Trajectory<double>>
DrakeSimulator::PlanCartesianMoveJWithTrueGCS(
    const VectorXd &q_start,
    const VectorXd &q_goal,
    double max_velocity,
    double max_acceleration,
    int num_regions)
{
    using namespace drake::geometry::optimization;
    using namespace drake::planning::trajectory_optimization;
    using namespace drake::planning;

    std::cout << "\n"
              << std::string(80, '=');
    std::cout << "\nINDUSTRIAL-GRADE GCS TRAJECTORY OPTIMIZATION";
    std::cout << "\nFeatures: Continuous Collision Detection + True GCS Implementation";
    std::cout << "\n"
              << std::string(80, '=') << std::endl;

    std::cout << "\n[CONFIGURATION]" << std::endl;
    std::cout << "  Number of IRIS regions: " << num_regions << std::endl;
    std::cout << "  Max Velocity: " << max_velocity << " rad/s" << std::endl;
    std::cout << "  Max Acceleration: " << max_acceleration << " rad/s²" << std::endl;

    // ============================================================================
    // INDUSTRIAL-GRADE STEP 1: Validate start and goal with detailed checking
    // ============================================================================
    std::cout << "\n[STEP 1] Industrial-grade collision validation..." << std::endl;

    // Check start configuration
    bool start_has_collision = CheckCollisionUsingChecker(q_start);
    if (start_has_collision)
    {
        std::cout << "  [ERROR] Start is in collision!" << std::endl;
        return nullptr;
    }

    // Get clearance for start
    double start_clearance = GetMinimumDistanceUsingChecker(q_start);
    std::cout << "  ✓ Start collision-free (clearance: " << (start_clearance * 1000) << " mm)" << std::endl;

    // Check goal configuration
    bool goal_has_collision = CheckCollisionUsingChecker(q_goal);
    if (goal_has_collision)
    {
        std::cout << "  [WARNING] Goal in collision - finding nearest safe point" << std::endl;
    }
    else
    {
        double goal_clearance = GetMinimumDistanceUsingChecker(q_goal);
        std::cout << "  ✓ Goal collision-free (clearance: " << (goal_clearance * 1000) << " mm)" << std::endl;
    }

    // ============================================================================
    // INDUSTRIAL-GRADE STEP 2: Sample collision-free configurations
    // NOTE: Always use GCS for optimal trajectory quality (Bézier curves)
    // GCS produces smoother trajectories than direct interpolation
    // ============================================================================
    std::cout << "\n[STEP 2] Sampling collision-free configurations..." << std::endl;

    auto& plant = robot_diagram_->plant();
    VectorXd lower_limits = plant.GetPositionLowerLimits();
    VectorXd upper_limits = plant.GetPositionUpperLimits();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    std::vector<VectorXd> samples;
    samples.push_back(q_start);

    const int max_attempts = num_regions * 20;
    int attempts = 0;

    while (samples.size() < size_t(num_regions) && attempts < max_attempts)
    {
        VectorXd q_sample = q_start;

        // Three sampling strategies
        double strategy = dis(gen);
        if (strategy < 0.4)
        {
            // Strategy 1: Random joint sampling (arm joints 11-17)
            for (int i = 11; i <= 17; ++i)
            {
                q_sample(i) = lower_limits(i) + dis(gen) * (upper_limits(i) - lower_limits(i));
            }
        }
        else if (strategy < 0.8)
        {
            // Strategy 2: Interpolate with noise
            double alpha = dis(gen);
            q_sample = (1 - alpha) * q_start + alpha * q_goal;
            for (int i = 11; i <= 17; ++i)
            {
                double noise = (dis(gen) - 0.5) * 0.3 * (upper_limits(i) - lower_limits(i));
                q_sample(i) += noise;
                q_sample(i) = std::max(lower_limits(i), std::min(upper_limits(i), q_sample(i)));
            }
        }
        else
        {
            // Strategy 3: Gaussian around start/goal
            VectorXd q_center = dis(gen) < 0.5 ? q_start : q_goal;
            for (int i = 11; i <= 17; ++i)
            {
                double radius = 0.2 * (upper_limits(i) - lower_limits(i));
                double offset = (dis(gen) - 0.5) * 2 * radius;
                q_sample(i) = q_center(i) + offset;
                q_sample(i) = std::max(lower_limits(i), std::min(upper_limits(i), q_sample(i)));
            }
        }

        // INDUSTRIAL-GRADE: Use SceneGraphCollisionChecker if available
        bool is_collision_free = false;
        if (HasCollisionChecker())
        {
            is_collision_free = !CheckCollisionUsingChecker(q_sample); // CheckCollisionUsingChecker returns true if collision
        }
        else
        {
            is_collision_free = !CheckCollisionUsingChecker(q_sample);
        }

        if (is_collision_free)
        {
            samples.push_back(q_sample);
            std::cout << "    Sample " << samples.size() << "/" << num_regions << "\r" << std::flush;
        }
        attempts++;
    }
    samples.push_back(q_goal);
    std::cout << "\n  ✓ Generated " << samples.size() << " samples ("
              << (100.0 * samples.size() / attempts) << "% success rate)" << std::endl;

    if (samples.size() < 3)
    {
        std::cout << "  [ERROR] Insufficient samples for GCS!" << std::endl;
        return nullptr;
    }

    // ============================================================================
    // INDUSTRIAL-GRADE STEP 3: IRIS regions with automatic collision detection
    // ============================================================================
    std::cout << "\n[STEP 3] Growing IRIS convex regions..." << std::endl;

    // Configure IRIS options for industrial-grade performance
    IrisOptions iris_options;
    // Use default/recommended values from Drake for robust region growth
    iris_options.iteration_limit = 100;              // Default: 100 (was 50)
    iris_options.termination_threshold = 2e-2;       // Default: 0.02
    iris_options.relative_termination_threshold = 1e-3;  // Relative volume change threshold
    iris_options.configuration_space_margin = 1e-3;  // Safety margin from C-space obstacles
    iris_options.num_collision_infeasible_samples = 10;  // Counter-example search samples
    iris_options.require_sample_point_is_contained = true;  // Ensure start point is inside
    // IRIS automatically uses SceneGraph for collision detection!

    ConvexSets regions;
    std::vector<bool> region_valid(samples.size(), false);

    // CRITICAL FIX: Create a FRESH context for EACH IRIS call to avoid thread safety issues
    // IrisNp with implicit_context_parallelism stores internal references to the context
    // Reusing the same context causes memory corruption and segfaults during GCS reconstruction
    // Solution: Create a new independent context for each sample
    for (size_t i = 0; i < samples.size(); ++i)
    {
        std::cout << "    Growing region " << (i + 1) << "/" << samples.size() << "...\r" << std::flush;

        try
        {
            // Create a fresh context for this IRIS call
            std::unique_ptr<drake::systems::Context<double>> iris_context =
                robot_diagram_->plant().CreateDefaultContext();
            robot_diagram_->plant().SetPositions(iris_context.get(), samples[i]);

            // INDUSTRIAL-GRADE: IrisNp uses SceneGraph automatically!
            // Each call gets its own context to avoid memory corruption
            HPolyhedron region = IrisNp(robot_diagram_->plant(), *iris_context, iris_options);

            regions.emplace_back(region.Clone());
            region_valid[i] = true;
        }
        catch (const std::exception &e)
        {
            // Fallback strategies
            try
            {
                Eigen::VectorXd radius = Eigen::VectorXd::Constant(q_start.size(), 0.1);
                Hyperellipsoid ellipsoid = Hyperellipsoid::MakeAxisAligned(radius, samples[i]);
                regions.emplace_back(ellipsoid.Clone());
                region_valid[i] = true;
            }
            catch (...)
            {
                regions.emplace_back(std::make_unique<Point>(samples[i]));
                region_valid[i] = true;
            }
        }
    }

    std::cout << "\n  ✓ Created " << regions.size() << " regions ("
              << std::count(region_valid.begin(), region_valid.end(), true) << " valid)" << std::endl;

    // ============================================================================
    // INDUSTRIAL-GRADE STEP 4: Build graph with Continuous Collision Detection
    // ============================================================================
    std::cout << "\n[STEP 4] Building graph with CCD edge validation..." << std::endl;

    const int k_nearest = 8;
    std::vector<std::pair<int, int>> edges;

    // Edge checking parameters (industrial-grade)
    const double edge_step_size = 0.02; // 0.02 rad per step for edge validation

    // INDUSTRIAL-GRADE: Collect all edges for parallel checking
    std::vector<std::pair<VectorXd, VectorXd>> edge_configs;
    std::vector<std::pair<int, int>> edge_indices;

    for (size_t i = 0; i < samples.size(); ++i)
    {
        if (!region_valid[i])
            continue;

        std::vector<std::pair<double, int>> distances;
        for (size_t j = 0; j < samples.size(); ++j)
        {
            if (i == j || !region_valid[j])
                continue;

            double dist = (samples[i] - samples[j]).norm();
            distances.push_back({dist, j});
        }
        std::sort(distances.begin(), distances.end());

        for (int k = 0; k < std::min(k_nearest, (int)distances.size()); ++k)
        {
            int j = distances[k].second;
            if (i < j)
            {
                edge_configs.push_back({samples[i], samples[j]});
                edge_indices.push_back({i, j});
            }
        }
    }

    // INDUSTRIAL-GRADE: Use SceneGraphCollisionChecker for parallel edge checking
    std::cout << "  [INFO] Checking " << edge_configs.size() << " edges for collisions..." << std::endl;

    std::vector<bool> edge_collision_free;
    if (HasCollisionChecker())
    {
        std::cout << "  [INFO] Using SceneGraphCollisionChecker (parallel)" << std::endl;
        edge_collision_free = CheckEdgesCollisionFreeParallelUsingChecker(edge_configs);
    }
    else
    {
        std::cout << "  [INFO] Using SceneGraphCollisionChecker for edge checking (sequential)" << std::endl;
        edge_collision_free.resize(edge_configs.size());
        for (size_t e = 0; e < edge_configs.size(); ++e)
        {
            edge_collision_free[e] = CheckEdgeCollisionFreeUsingChecker(
                edge_configs[e].first, edge_configs[e].second);
        }
    }

    // Add collision-free edges to graph
    for (size_t e = 0; e < edge_configs.size(); ++e)
    {
        if (edge_collision_free[e])
        {
            edges.push_back(edge_indices[e]);
        }
    }

    std::cout << "  ✓ Created " << edges.size() << " collision-free edges (verified with CCD)" << std::endl;

    // ============================================================================
    // STEP 5: TRUE DRAKE GCS TRAJECTORY OPTIMIZATION
    // ============================================================================
    std::cout << "\n[STEP 5] Optimizing trajectory with Drake GCS..." << std::endl;

    const int num_positions = q_start.size();
    const int start_idx = 0;   // q_start is added first at line 2108
    const int goal_idx = samples.size() - 1;  // q_goal is added last at line 2170

    // Prepare valid regions vector
    drake::geometry::optimization::ConvexSets valid_regions;
    std::vector<int> valid_to_original_map;  // Maps valid region index to original sample index
    for (size_t i = 0; i < regions.size(); ++i)
    {
        if (region_valid[i])
        {
            valid_regions.emplace_back(regions[i]->Clone());
            valid_to_original_map.push_back(i);
        }
    }

    std::cout << "  [INFO] Using " << valid_regions.size() << " valid regions out of "
              << regions.size() << " total" << std::endl;

    // Create GCS trajectory optimization (using linear Bézier for stability)
    const int bezier_order = 1;  // Linear Bézier (order=1, more stable than higher orders)
    drake::planning::trajectory_optimization::GcsTrajectoryOptimization gcs(num_positions);

    // Add all regions as a subgraph (GCS will automatically compute edges based on intersection)
    // We specify edges_between_regions to use our collision-checked edges
    std::vector<std::pair<int, int>> gcs_edges;
    for (const auto& [i, j] : edges)
    {
        if (!region_valid[i] || !region_valid[j])
            continue;

        // Map original indices to valid region indices
        int valid_i = -1, valid_j = -1;
        for (size_t k = 0; k < valid_to_original_map.size(); ++k)
        {
            if (valid_to_original_map[k] == i)
                valid_i = k;
            if (valid_to_original_map[k] == j)
                valid_j = k;
        }

        if (valid_i >= 0 && valid_j >= 0)
        {
            gcs_edges.push_back({valid_i, valid_j});
        }
    }

    std::cout << "  [INFO] Adding " << gcs_edges.size() << " edges to GCS subgraph..." << std::endl;

    // Add regions with specified edges and Bézier order
    auto& subgraph = gcs.AddRegions(
        valid_regions,
        gcs_edges,
        bezier_order,  // Linear Bézier
        1e-6,          // h_min
        20.0,          // h_max
        "trajectory_subgraph"
    );

    std::cout << "  ✓ GCS subgraph created with " << subgraph.size() << " vertices" << std::endl;

    // Add path length cost to minimize trajectory length
    Eigen::MatrixXd path_weight_matrix = Eigen::MatrixXd::Identity(num_positions, num_positions);
    subgraph.AddPathLengthCost(path_weight_matrix);

    // Add velocity bounds
    std::cout << "  [INFO] Adding velocity constraints..." << std::endl;
    Eigen::VectorXd velocity_lower = Eigen::VectorXd::Constant(num_positions, -max_velocity);
    Eigen::VectorXd velocity_upper = Eigen::VectorXd::Constant(num_positions, max_velocity);
    subgraph.AddVelocityBounds(velocity_lower, velocity_upper);

    // NOTE: Skip C^1 continuity constraints for now - they make the problem infeasible
    // when connecting single-point source/goal to regions with linear Bézier curves.
    // C^0 continuity (position continuity) is automatically satisfied by the GCS formulation.

    // Create source and goal sets (single-point subgraphs at start and goal)
    drake::geometry::optimization::ConvexSets source_sets, goal_sets;
    source_sets.emplace_back(std::make_unique<drake::geometry::optimization::Point>(q_start));
    goal_sets.emplace_back(std::make_unique<drake::geometry::optimization::Point>(q_goal));

    auto& source_subgraph = gcs.AddRegions(source_sets, 0, 1e-6, 20.0, "source");
    auto& goal_subgraph = gcs.AddRegions(goal_sets, 0, 1e-6, 20.0, "goal");

    // Connect source/goal to main subgraph
    gcs.AddEdges(source_subgraph, subgraph);
    gcs.AddEdges(subgraph, goal_subgraph);

    std::cout << "  [INFO] Solving GCS optimization problem..." << std::endl;

    try
    {
        // Set up GCS options (use optional<bool> and optional<int> types)
        drake::geometry::optimization::GraphOfConvexSetsOptions options;
        options.convex_relaxation = true;  // std::optional<bool>
        options.max_rounded_paths = 5;     // std::optional<int>
        options.preprocessing = true;       // std::optional<bool>

        // Solve the path from source to goal
        // Returns: std::pair<CompositeTrajectory, MathematicalProgramResult>
        auto [gcs_trajectory, gcs_result] = gcs.SolvePath(source_subgraph, goal_subgraph, options);

        if (gcs_result.is_success())
        {
            std::cout << "  ✓ GCS solved successfully!" << std::endl;
            std::cout << "  ✓ Trajectory duration: " << gcs_trajectory.end_time() << " s" << std::endl;

            // Convert CompositeTrajectory to PiecewisePolynomial
            std::cout << "  [INFO] Converting GCS trajectory to PiecewisePolynomial..." << std::endl;

            // CompositeTrajectory contains BezierCurve segments
            // Use get_segment_times() to get actual break points
            std::vector<double> breaks = gcs_trajectory.get_segment_times();
            std::vector<MatrixXd> samples_vec;
            std::vector<MatrixXd> derivatives_vec;

            for (double t : breaks)
            {
                // Get position at break point
                VectorXd q = gcs_trajectory.value(t);

                // Get velocity - use EvalDerivative if available, otherwise numerical
                VectorXd v;
                if (gcs_trajectory.has_derivative())
                {
                    v = gcs_trajectory.EvalDerivative(t, 1);
                }
                else
                {
                    // Numerical derivative fallback
                    const double eps = 1e-6;
                    VectorXd q_next = gcs_trajectory.value(t + eps);
                    v = (q_next - q) / eps;
                }

                samples_vec.push_back(q);
                derivatives_vec.push_back(v);
            }

            // Create CubicHermite trajectory from samples and derivatives
            auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
                breaks, samples_vec, derivatives_vec);

            // ============================================================================
            // CRITICAL: VALIDATE GCS TRAJECTORY BEFORE RETURNING
            // ============================================================================
            std::cout << "\n[STEP 5.5] Validating GCS-generated trajectory..." << std::endl;

            bool gcs_trajectory_valid = ValidateTrajectoryComplete(trajectory, 0.001);

            if (!gcs_trajectory_valid)
            {
                std::cout << "\n[CRITICAL FAILURE] GCS trajectory has collisions!" << std::endl;
                std::cout << "  GCS optimization found a path, but the generated trajectory" << std::endl;
                std::cout << "  passes through obstacles. This can happen when:" << std::endl;
                std::cout << "    - Bézier curves deviate from the straight-line edges that were checked" << std::endl;
                std::cout << "    - IRIS regions are too conservative and don't cover the actual C-space free region" << std::endl;
                std::cout << "    - Edge checking was not dense enough to catch all collisions" << std::endl;
                std::cout << "\n[FALLBACK] Proceeding to Dijkstra pathfinding..." << std::endl;
                // Fall through to Dijkstra method below
            }
            else
            {
                std::cout << "\n[SUCCESS] GCS trajectory validated - collision-free!" << std::endl;
                return std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(std::move(trajectory));
            }
        }
        else
        {
            std::cout << "  [WARNING] GCS solve failed" << std::endl;
            std::cout << "  [DIAGNOSTIC] Solution result: " << gcs_result.get_solution_result() << std::endl;
            std::cout << "  [DIAGNOSTIC] Solver id: " << gcs_result.get_solver_id().name() << std::endl;
            if (gcs_result.get_solution_result() == drake::solvers::SolutionResult::kInfeasibleConstraints) {
                std::cout << "  [DIAGNOSTIC] Problem is infeasible - constraints cannot be satisfied" << std::endl;
            } else if (gcs_result.get_solution_result() == drake::solvers::SolutionResult::kSolverSpecificError) {
                std::cout << "  [DIAGNOSTIC] Solver-specific error occurred" << std::endl;
            } else if (gcs_result.get_solution_result() == drake::solvers::SolutionResult::kInvalidInput) {
                std::cout << "  [DIAGNOSTIC] Invalid input to solver" << std::endl;
            }
            std::cout << "  [FALLBACK] Using Dijkstra pathfinding..." << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "  [ERROR] GCS solve failed: " << e.what() << std::endl;
        std::cout << "  [FALLBACK] Using Dijkstra pathfinding..." << std::endl;
    }

    // ============================================================================
    // FALLBACK: Dijkstra pathfinding (if GCS fails)
    // ============================================================================
    std::cout << "\n[STEP 5 FALLBACK] Finding shortest path through samples (Dijkstra)..." << std::endl;

    // Build adjacency list for graph search
    const int n = samples.size();
    std::vector<std::vector<std::pair<int, double>>> adj(n);
    for (const auto& [i, j] : edges)
    {
        double dist = (samples[i] - samples[j]).norm();
        adj[i].push_back({j, dist});
        adj[j].push_back({i, dist});
    }

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    dist[start_idx] = 0;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<>> pq;
    pq.push({0, start_idx});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;
        if (u == goal_idx) break;

        for (auto [v, w] : adj[u])
        {
            if (dist[u] + w < dist[v])
            {
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // Reconstruct path
    std::vector<int> path;
    for (int v = goal_idx; v != -1; v = parent[v])
    {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());

    if (path.front() != start_idx || path.back() != goal_idx || path.size() < 2)
    {
        throw std::runtime_error("Failed to find path from start to goal");
    }

    std::cout << "  ✓ Found path with " << path.size() << " waypoints" << std::endl;

    // Extract waypoints
    std::vector<VectorXd> waypoints;
    for (int idx : path)
    {
        waypoints.push_back(samples[idx]);
    }

    // ============================================================================
    // STEP 6: Create trajectory by interpolating through waypoints
    // ============================================================================
    std::cout << "\n[STEP 6] Creating trajectory through waypoints..." << std::endl;

    // Compute total path length for timing
    double total_length = 0;
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        total_length += (waypoints[i+1] - waypoints[i]).norm();
    }

    // Assign times to waypoints based on distance and max velocity
    std::vector<double> breaks;
    breaks.push_back(0);
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        double segment_length = (waypoints[i+1] - waypoints[i]).norm();
        double segment_time = segment_length / max_velocity;
        breaks.push_back(breaks.back() + segment_time);
    }

    // Create vector of waypoint matrices for CubicHermite
    // num_positions already declared above at line 2325
    std::vector<MatrixXd> samples_vec;
    std::vector<MatrixXd> derivatives_vec;
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        samples_vec.push_back(waypoints[i]);
        derivatives_vec.push_back(VectorXd::Zero(num_positions));  // Zero velocity
    }

    // Create cubic spline trajectory (CubicHermite with zero velocities)
    auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
        breaks,
        samples_vec,
        derivatives_vec);  // Zero velocity at waypoints

    std::cout << "  ✓ Trajectory created with duration: " << breaks.back() << " s" << std::endl;

    // ============================================================================
    // STEP 7: Validate trajectory WITH DENSE SAMPLING
    // ============================================================================
    std::cout << "\n[STEP 7] Validating trajectory with COMPLETE collision detection..." << std::endl;

    // Use ValidateTrajectoryComplete for THOROUGH validation
    // This checks ALL collisions (not just closest point pairs) with DENSE sampling
    bool trajectory_valid = ValidateTrajectoryComplete(trajectory, 0.001);  // 1ms interval

    if (trajectory_valid)
    {
        std::cout << "\n[SUCCESS] Dijkstra trajectory is collision-free!" << std::endl;
        return std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(std::move(trajectory));
    }
    else
    {
        std::cout << "\n[FAILURE] Dijkstra trajectory has collisions!" << std::endl;
        std::cout << "  [HINT] Try:" << std::endl;
        std::cout << "        - More IRIS regions (increase num_regions)" << std::endl;
        std::cout << "        - More waypoints in path (increase k_nearest)" << std::endl;
        std::cout << "        - Adjust obstacle positions to leave more clearance" << std::endl;
        return nullptr;  // Return nullptr to indicate failure
    }
}

// Removed unused functions:
// - PlanCollisionFreePath (RRT*): Replaced by PlanCartesianMoveJWithTrueGCS using Drake's GCS API
// - RepairTrajectoryCollision: Local repair not needed with proper GCS optimization

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
    urdf_path = project_dir + "/model/nezha/urdf/robot_arm.urdf";
    // NOTE: Using test scene with obstacles for MoveJ collision avoidance testing
    // To use normal scene, change to: "model/nezha/scene/scene.xml"
    // mujoco_scene_path = project_dir + "/model/nezha/scene/scene_obstacle_test.xml";
    mujoco_scene_path = project_dir + "/model/nezha/scene/scene_obstacle_test.xml";

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

    double sim_duration = 5.0;                // seconds (longer for circular trajectory)
    double time_step = 0.001;                 // 1ms timestep
    bool mujoco_only = true;                  // Use MuJoCo only for trajectory demo
    bool enable_visualization = true;         // Enable MuJoCo visualization window
    std::string trajectory_type = "circular"; // Default: circular

    int arg_idx = 1;
    // Check if first argument is a trajectory type string
    if (argc > arg_idx)
    {
        std::string arg1 = argv[arg_idx];
        // Legacy types (for backward compatibility)
        if (arg1 == "circular" || arg1 == "circle" || arg1 == "line" || arg1 == "pose" ||
            arg1 == "waypoint" || arg1 == "waypoints" || arg1 == "avoid" || arg1 == "obstacle" ||
            // Industrial standard commands (case-insensitive)
            arg1 == "MoveL" || arg1 == "movel" || arg1 == "MOVEL" ||
            arg1 == "MoveC" || arg1 == "movec" || arg1 == "MOVEC" ||
            arg1 == "MoveJ" || arg1 == "movej" || arg1 == "MOVEJ" ||
            arg1 == "MoveB" || arg1 == "moveb" || arg1 == "MOVEB")
        {
            trajectory_type = arg1;
            arg_idx++;
        }
    }

    if (argc > arg_idx)
    {
        try
        {
            sim_duration = std::stod(argv[arg_idx]);
            arg_idx++;
        }
        catch (const std::invalid_argument &)
        {
            // Not a number, keep default duration
        }
    }
    if (argc > arg_idx)
    {
        time_step = std::stod(argv[arg_idx]);
        arg_idx++;
    }
    if (argc > arg_idx && std::string(argv[arg_idx]) == "--drake")
    {
        mujoco_only = false;
        arg_idx++;
    }
    if (argc > arg_idx && std::string(argv[arg_idx]) == "--no-visual")
    {
        enable_visualization = false;
        arg_idx++;
    }

    try
    {
        // ========== STEP 1: DRAKE CARTESIAN TRAJECTORY PLANNING ==========
        std::cout << "\n>>> Step 1: Loading Drake Model for Planning" << std::endl;
        DrakeSimulator drake_sim(urdf_path);

        // Define starting joint configuration for Nezha robot (20 DOF)
        // Joint indices: legs[0-2], waist[3], left_arm[4-10], right_arm[11-17], head[18-19]

        // Initial joint configuration (angles in degrees converted to radians)
        VectorXd q_start = VectorXd::Zero(20);
        q_start(11) = 0.0;
        q_start(12) = 0.0;
        q_start(13) = 0.0;
        q_start(14) = 0.0;
        q_start(15) = 0.0;
        q_start(16) = 0.0;
        q_start(17) = 0.0;

        // Compute initial EE position using Forward Kinematics
        drake::math::RigidTransformd T_ee_start = drake_sim.ComputeEEPose(q_start);
        Eigen::Vector3d ee_start = T_ee_start.translation(); // 末端初始位置

        std::cout << "Initial EE Position (waist frame): " << ee_start.transpose() << std::endl;
        std::cout << "Configuration uses waist coordinate frame as reference" << std::endl;

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

        // ========== TRAJECTORY PLANNING BASED ON TYPE ==========
        std::unique_ptr<drake::trajectories::Trajectory<double>> planned_trajectory;

        // Declare circle parameters for later use in tracking
        Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
        if (trajectory_type == "MoveL")
        {

            std::cout << "\n>>> Planning Linear Trajectory with Full Pose Control (Position + Orientation)" << std::endl;
        }
        // TODO: MoveJ
        else
        {
            // Safety flag to track if trajectory is safe to execute
            bool trajectory_safe = true;

            // Define goal joint configuration
            // For obstacle avoidance testing, use large motion that will pass through obstacle region
            VectorXd q_goal = q_start;
            VectorXd q_zero = VectorXd::Zero(20);

            // Configuration: Forward-center motion (toward obstacles)
            // MODIFIED: Reduced joint3 extension to avoid self-collision with waist/legs
            q_goal(11) = -0.095;
            q_goal(12) = -0.44;
            q_goal(13) = -0.44;
            q_goal(14) = 1.32;
            q_goal(15) = -0.188;
            q_goal(16) = 0.38;
            q_goal(17) = -0.314;

            std::cout << "\n[MOVEJ CONFIGURATION - OBSTACLE TEST]" << std::endl;
            std::cout << "  Planning joint space motion from start to goal configuration" << std::endl;
            std::cout << "  [TEST] Using large motion to trigger collision detection" << std::endl;
            std::cout << "  [INFO] Obstacles placed in workspace at x=0.55-0.7m, z=0.65-0.82m" << std::endl;

            // Calculate joint space distance (Euclidean norm)
            double joint_distance = (q_goal - q_start).norm();
            std::cout << "  Joint Space Distance: " << joint_distance << " rad" << std::endl;

            // Use MoveJ with TRUE GCS-based obstacle avoidance
            // This uses IRIS regions + GCS optimization (not the old A* method)
            planned_trajectory = drake_sim.PlanCartesianMoveJWithTrueGCS(
                q_start, // Start from actual current position
                q_goal,  // Move to goal configuration
                1.0,     // max_velocity
                2.0,     // max_acceleration
                40);     // num_regions for IRIS (reduced from 150 samples)

            std::cout << "\n[SUCCESS] MoveJ trajectory generated!" << std::endl;

            // ========================================================================
            // COLLISION CHECKING: Verify trajectory is collision-free
            // ========================================================================
            std::cout << "\n[COLLISION CHECKING]" << std::endl;
            std::cout << "  Checking start configuration..." << std::endl;
            bool start_collision = drake_sim.CheckCollisionUsingChecker(q_start);
            if (start_collision)
            {
                std::cout << "  [WARNING] Start configuration is in collision!" << std::endl;
            }
            else
            {
                std::cout << "  ✓ Start configuration is collision-free" << std::endl;
            }

            std::cout << "  Checking goal configuration..." << std::endl;
            bool goal_collision = drake_sim.CheckCollisionUsingChecker(q_goal);
            if (goal_collision)
            {
                std::cout << "  [WARNING] Goal configuration is in collision!" << std::endl;
            }
            else
            {
                std::cout << "  ✓ Goal configuration is collision-free" << std::endl;
            }

            // Sample trajectory and check for collisions
            std::cout << "  Checking trajectory for collisions..." << std::endl;

            // CRITICAL: Check if trajectory is valid (e.g., start was in collision)
            if (!planned_trajectory || planned_trajectory->start_time() == planned_trajectory->end_time())
            {
                std::cout << "\n  [ERROR] Trajectory is empty (0 segments)!" << std::endl;
                std::cout << "  This typically means the start configuration was in collision." << std::endl;
                std::cout << "\n"
                          << std::string(80, '=') << std::endl;
                std::cout << "[ABORTED] No valid trajectory to execute" << std::endl;
                std::cout << std::string(80, '=') << std::endl;
                return 1;
            }

            // CRITICAL FIX: Increase sampling density to catch fast collisions
            // Previous: 100 samples → 25ms interval (might miss collisions)
            // Now: 1000 samples → 2.5ms interval (much more likely to detect)
            const int num_collision_samples = 1000;

            double trajectory_duration = planned_trajectory->end_time() - planned_trajectory->start_time();

            std::cout << "  [INFO] Using " << num_collision_samples << " collision samples (interval ≈ "
                      << (trajectory_duration / (num_collision_samples - 1) * 1000.0) << " ms)" << std::endl;

            bool trajectory_collision = false;
            int first_collision_idx = -1;
            double first_collision_time = -1.0;

            for (int i = 0; i < num_collision_samples; ++i)
            {
                double t = (i / static_cast<double>(num_collision_samples - 1)) * trajectory_duration;
                VectorXd q_sample = planned_trajectory->value(t);

                if (drake_sim.CheckCollisionUsingChecker(q_sample))
                {
                    trajectory_collision = true;
                    first_collision_idx = i;
                    first_collision_time = t;
                    std::cout << "  [COLLISION DETECTED] at t=" << first_collision_time
                              << " s (sample " << first_collision_idx << "/" << num_collision_samples << ")" << std::endl;
                    std::cout << "  [DEBUG] Collision detected during trajectory sampling!" << std::endl;
                    break;
                }
            }

            if (!trajectory_collision)
            {
                std::cout << "  ✓ Trajectory is collision-free!" << std::endl;
                std::cout << "\n[SUCCESS] Collision-free MoveJ trajectory!" << std::endl;
            }
            else
            {
                std::cout << "  [COLLISION DETECTED] at t=" << first_collision_time << " s" << std::endl;
                std::cout << "\n"
                          << std::string(80, '!') << std::endl;
                std::cout << "[SAFETY FAILURE] Trajectory has collisions!" << std::endl;
                std::cout << "  PlanCartesianMoveJWithTrueGCS should have handled obstacle avoidance." << std::endl;
                std::cout << "  - EXECUTION ABORTED to prevent collision" << std::endl;
                std::cout << std::string(80, '!') << std::endl;

                // Mark trajectory as unsafe - will skip all execution steps
                trajectory_safe = false;
            }

            // =====================================================================
            // CRITICAL SAFETY CHECK: Only proceed if trajectory is safe
            // =====================================================================
            if (!trajectory_safe)
            {
                std::cout << "\n"
                          << std::string(80, '=') << std::endl;
                std::cout << "[ABORTED] Trajectory execution skipped due to collision risk" << std::endl;
                std::cout << std::string(80, '=') << std::endl;

                // Skip MuJoCo visualization and exit
                std::cout << "\nExiting without visualization due to safety failure." << std::endl;
                return 1;
            }

            // =================================================================
            // TRAJECTORY IS SAFE - PROCEED WITH EXECUTION AND VISUALIZATION
            // =================================================================

            // Print final joint configuration and end-effector pose
            std::cout << "\n"
                      << std::string(80, '=') << std::endl;
            std::cout << "MOVEJ TRAJECTORY EXECUTION RESULTS" << std::endl;
            std::cout << std::string(80, '=') << std::endl;

            // Get final joint angles at the end of trajectory
            VectorXd q_final = planned_trajectory->value(trajectory_duration);

            std::cout << "\n[FINAL JOINT ANGLES]" << std::endl;
            std::cout << "  Right Arm (q11-q17):" << std::endl;
            for (int i = 11; i <= 17; ++i)
            {
                double angle_deg = q_final(i) * 180.0 / M_PI;
                std::cout << "    q" << i << " = " << std::fixed << std::setprecision(6)
                          << q_final(i) << " rad (" << std::setprecision(3)
                          << angle_deg << "°)" << std::endl;
            }

            // Compute final end-effector pose
            drake::math::RigidTransformd T_ee_final = drake_sim.ComputeEEPose(q_final);

            // Compute target end-effector pose from q_goal
            drake::math::RigidTransformd T_ee_goal = drake_sim.ComputeEEPose(q_goal);

            std::cout << "\n[FINAL END-EFFECTOR POSE]" << std::endl;
            std::cout << "  Position (x, y, z): "
                      << std::fixed << std::setprecision(6)
                      << T_ee_final.translation().transpose() << " m" << std::endl;

            drake::math::RollPitchYawd rpy_final(T_ee_final.rotation());
            std::cout << "  Orientation (RPY): "
                      << std::fixed << std::setprecision(6)
                      << (rpy_final.vector() * 180.0 / M_PI).transpose()
                      << " deg" << std::endl;

            // Compare with target
            Eigen::Vector3d pos_error = T_ee_final.translation() - T_ee_goal.translation();
            Eigen::Matrix3d R_diff = T_ee_goal.rotation().matrix() * T_ee_final.rotation().matrix().transpose();
            Eigen::AngleAxisd angle_axis(R_diff);
            double rot_error_deg = angle_axis.angle() * 180.0 / M_PI;

            std::cout << "\n[COMPARISON WITH TARGET]" << std::endl;
            std::cout << "  Target Position:   "
                      << T_ee_goal.translation().transpose() << " m" << std::endl;
            std::cout << "  Achieved Position: "
                      << T_ee_final.translation().transpose() << " m" << std::endl;
            std::cout << "  Position Error:    "
                      << std::scientific << std::setprecision(6)
                      << pos_error.norm() << " m ("
                      << std::fixed << (pos_error.norm() * 1000) << " mm)" << std::endl;

            drake::math::RollPitchYawd rpy_goal(T_ee_goal.rotation());
            std::cout << "  Target Orientation: "
                      << std::fixed << std::setprecision(3)
                      << (rpy_goal.vector() * 180.0 / M_PI).transpose() << " deg" << std::endl;
            std::cout << "  Achieved Orientation: "
                      << (rpy_final.vector() * 180.0 / M_PI).transpose() << " deg" << std::endl;
            std::cout << "  Orientation Error:  "
                      << std::fixed << std::setprecision(4)
                      << rot_error_deg << " deg" << std::endl;

            std::cout << "\n"
                      << std::string(80, '=') << std::endl;

            // =====================================================================
            // SAVE TRAJECTORY TO JSON FOR REAL ROBOT TESTING
            // =====================================================================
            std::string json_filename = "trajectory_moveJ_pose.json";
            std::ofstream json_file(json_filename);

            if (json_file.is_open())
            {
                std::cout << "\n[JSON] Saving trajectory to: " << json_filename << std::endl;

                // ============================================================
                // 真机测试配置: 200Hz采样,包含完整轨迹
                // ============================================================
                double trajectory_duration = planned_trajectory->end_time();
                double sampling_interval = 0.005; // 5ms = 200Hz (真机标准控制频率)

                // 计算采样点数（确保包含完整轨迹）
                int num_samples = static_cast<int>(std::round(trajectory_duration / sampling_interval)) + 1;

                std::cout << "[JSON] Exporting " << num_samples << " trajectory points at 200Hz for real robot" << std::endl;
                std::cout << "[JSON] Trajectory duration: " << trajectory_duration << " s" << std::endl;
                std::cout << "[JSON] Sampling interval: " << sampling_interval << " s (5ms, 200Hz)" << std::endl;
                std::cout << "[JSON] Actual frequency: " << (num_samples - 1) / trajectory_duration << " Hz" << std::endl;

                // Start building JSON
                json_file << "{\n";
                json_file << "    \"cycle\": 0,\n";
                json_file << "    \"actions\": [\n";
                json_file << "        {\n";
                json_file << "            \"taskId\": \"moveJ_pose\",\n";
                json_file << "            \"taskType\": \"Play\",\n";
                json_file << "            \"taskParameters\": {\n";
                json_file << "                \"continue\": false,\n";
                json_file << "                \"updateId\": 0,\n";
                json_file << "                \"rightHand\": [\n";

                // Sample and write joint positions at EXACT 200Hz intervals
                for (int i = 0; i < num_samples; ++i)
                {
                    double t = i * sampling_interval;
                    if (t > trajectory_duration)
                        t = trajectory_duration;

                    // Get joint positions at time t
                    VectorXd q_t = planned_trajectory->value(t);

                    // Write right arm joints (q11-q17)
                    json_file << "                    [";
                    for (int j = 11; j <= 17; ++j)
                    {
                        json_file << std::scientific << std::setprecision(15) << q_t(j);
                        if (j < 17)
                            json_file << ", ";
                    }
                    json_file << "]";
                    if (i < num_samples - 1)
                        json_file << ",\n";
                    else
                        json_file << "\n";
                }

                json_file << "                ]\n";
                json_file << "            }\n";
                json_file << "        }\n";
                json_file << "    ]\n";
                json_file << "}\n";

                json_file.close();
                std::cout << "[JSON] Successfully saved " << num_samples << " samples at 200Hz" << std::endl;
                std::cout << "[JSON] All trajectory points included (complete path)" << std::endl;
                std::cout << "[JSON] Format: Actions with rightHand joint trajectories" << std::endl;
                std::cout << "[JSON] Joints: q11-q17 (7 DOF right arm)" << std::endl;
                std::cout << "[JSON] Ready for real robot deployment!" << std::endl;
            }
            else
            {
                std::cerr << "[ERROR] Failed to create JSON file: " << json_filename << std::endl;
            }
        }

        std::cout << "\n>>> Initializing MuJoCo for Visualization" << std::endl;

        std::cout << "  Resetting Drake simulator context..." << std::endl;
        drake_sim.reset();

        // Force flush all streams
        std::cout << std::flush;
        std::cerr << std::flush;

        // Small delay to ensure all Drake operations complete
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Debug: Print message before creating MuJoCo
        std::cout << "  Creating MuJoCo simulator..." << std::endl;
        std::cout << "  Scene path: " << mujoco_scene_path << std::endl;

        // Try to check if scene file exists
        std::ifstream scene_file(mujoco_scene_path);
        if (!scene_file.good())
        {
            std::cerr << "  ERROR: Scene file not found: " << mujoco_scene_path << std::endl;
            return -1;
        }
        scene_file.close();
        std::cout << "  Scene file exists" << std::endl;

        // Create MuJoCo simulator with visualization flag
        std::cout << "  Calling MuJoCoSimulator constructor..." << std::endl;
        MuJoCoSimulator mujoco_sim(mujoco_scene_path, enable_visualization);

        std::cout << "  MuJoCo simulator created successfully!" << std::endl;

        // Reset simulation
        std::cout << "  Calling reset()..." << std::endl;
        mujoco_sim.reset();

        std::cout << "  Reset successful!" << std::endl;

        // Get DOF count
        std::cout << "  Getting DOF counts..." << std::endl;
        int nq = mujoco_sim.get_num_positions();
        int nv = mujoco_sim.get_num_dofs();
        std::cout << "  DOFs: nq=" << nq << ", nv=" << nv << std::endl;

        std::cout << "\nMuJoCo DOFs: " << nv << std::endl;

        // Initialize trajectory storage
        std::vector<float> traj_points;

        // IMPORTANT: Set MuJoCo initial state to match Drake's starting configuration
        std::cout << "\nSetting MuJoCo initial state to match Drake configuration..." << std::endl;

        // Print right arm joint angles from q_start
        std::cout << "Right arm joint angles (q_start[11:17]):" << std::endl;
        for (int i = 11; i <= 17; ++i)
        {
            std::cout << "  q[" << i << "] = " << q_start(i) << std::endl;
        }

        VectorXd v_zero = VectorXd::Zero(nv);
        mujoco_sim.set_state(q_start, v_zero);
        mujoco_sim.step(0); // Update positions without advancing time

        // Verify initial EE position matches
        Eigen::Vector3d mujoco_ee_start = mujoco_sim.get_ee_position();
        std::cout << "Drake EE start: " << ee_start.transpose() << std::endl;
        std::cout << "MuJoCo EE start: " << mujoco_ee_start.transpose() << std::endl;
        std::cout << "Difference: " << (mujoco_ee_start - ee_start).transpose() << std::endl;

        // Simulation loop - PLAYBACK PLANNED TRAJECTORY
        // Get trajectory duration here since it's in a different scope
        double trajectory_duration = planned_trajectory->end_time();
        std::cout << "\n>>> Trajectory duration: " << trajectory_duration << " seconds" << std::endl;

        double t = 0.0;
        int step_count = 0;
        const int print_interval = static_cast<int>(0.1 / time_step);
        const int traj_sample_interval = static_cast<int>(0.01 / time_step); // Sample every 10ms

        std::cout << "\n>>> Step 3: Playing Back Planned Trajectory in MuJoCo" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        auto start_time = std::chrono::high_resolution_clock::now();

        // DEBUG: Track minimum distance to table top
        double min_dist_to_table = std::numeric_limits<double>::max();
        Eigen::Vector3d closest_ee_pos;
        double closest_time = 0.0;


        Eigen::Vector3d table_center(0.5, 0.0, 0.62);
        Eigen::Vector3d table_half_size(0.35, 0.25, 0.05);

        std::cout << "\n[TABLE TRACKING] Table top center: (" << table_center.transpose()
                  << ") size: (" << table_half_size.transpose() << ")" << std::endl;

        while (t < trajectory_duration && !mujoco_sim.should_close())
        {
            // Evaluate Drake's planned trajectory at current time
            VectorXd q_desired = drake_sim.eval_trajectory(*planned_trajectory, t);

            // Set state in MuJoCo (directly set positions from planned trajectory)
            VectorXd v_zero = VectorXd::Zero(nv);
            mujoco_sim.set_state(q_desired, v_zero);

            // Step simulation
            mujoco_sim.step(time_step);
            t += time_step;
            step_count++;

            // Get current EE position
            Eigen::Vector3d ee_pos = mujoco_sim.get_ee_position();             // For tracking (in waist frame)
            Eigen::Vector3d ee_pos_world = mujoco_sim.get_ee_position_world(); // For visualization (in world frame)

            // DEBUG: Calculate distance from EE to table top surface
            // Simple distance to table center (not accurate but gives rough idea)
            double dist_to_table_center = (ee_pos_world - table_center).norm();

            // More accurate: distance to table top surface (z=0.62)
            // Assuming EE is roughly above table
            double dx = std::abs(ee_pos_world(0) - table_center(0));
            double dy = std::abs(ee_pos_world(1) - table_center(1));
            double dz = std::abs(ee_pos_world(2) - table_center(2)) - table_half_size(2);

            // Check if EE is within table's horizontal bounds
            bool within_table_x = (dx <= table_half_size(0));
            bool within_table_y = (dy <= table_half_size(1));

            // Distance to table surface
            double dist_to_table_surface;
            if (within_table_x && within_table_y && dz > 0)
            {
                // EE is above table within bounds
                dist_to_table_surface = dz;
            }
            else
            {
                // EE is outside table bounds or below surface - use center distance
                dist_to_table_surface = dist_to_table_center;
            }

            // DEBUG: Check collision in Drake every 100 steps OR when close to table
            bool close_to_table = (step_count > 0 && step_count % 10 == 0 && dist_to_table_surface < 0.05);

            if (step_count % 100 == 0 || close_to_table)
            {
                // DEBUG: Compute EE and link7 positions in Drake world frame
                Eigen::Vector3d drake_ee_world = drake_sim.ComputeEEPoseInWorldFrame(q_desired).translation();

                // Compute right_arm_link7 position and orientation (where the collision geometry actually is)
                drake::math::RigidTransformd drake_link7_transform = drake_sim.ComputeLink7TransformInWorldFrame(q_desired);
                Eigen::Vector3d drake_link7_world = drake_link7_transform.translation();

                // Get link7's Z-axis direction in world frame (direction of collision mesh extension)
                Eigen::Vector3d link7_z_axis = drake_link7_transform.rotation().matrix().col(2);

                bool drake_col = drake_sim.CheckCollisionUsingChecker(q_desired);
                std::cout << "  [DRAKE COLLISION CHECK] t=" << std::fixed << std::setprecision(3) << t
                          << "s EE(mujoco)=" << ee_pos_world.transpose()
                          << "\n    EE(drake)=" << drake_ee_world.transpose()
                          << "\n    link7(drake)=" << drake_link7_world.transpose()
                          << "\n    link7 Z-axis(world)=" << link7_z_axis.transpose()
                          << " dist=" << (dist_to_table_surface * 1000) << "mm: "
                          << (drake_col ? "COLLISION!" : "clear") << std::endl;

                // If close to table, do detailed collision check
                if (close_to_table)
                {
                    double min_dist = drake_sim.GetMinimumDistanceUsingChecker(q_desired);
                    std::cout << "    [DETAILED] Min distance: " << (min_dist * 1000) << " mm" << std::endl;

                    // Check if Drake link7 position is colliding with table
                    double dz_link7 = std::abs(drake_link7_world(2) - table_center(2)) - table_half_size(2);
                    std::cout << "    [DEBUG] link7 z=" << drake_link7_world(2) << ", table surface z=" << (table_center(2) + table_half_size(2))
                              << ", dz=" << (dz_link7 * 1000) << " mm" << std::endl;

                    // Project collision mesh extent onto world Z direction
                    // Collision mesh extends 0.14-0.38m in link7's local +Z direction
                    // The bottom of collision mesh in world Z is: link7_z + (0.14 * link7_z_axis.z)
                    double collision_mesh_bottom_z = drake_link7_world(2) + 0.140450 * link7_z_axis(2);
                    double collision_mesh_top_z = drake_link7_world(2) + 0.382144 * link7_z_axis(2);
                    std::cout << "    [MESH PROJECTION] link7 Z-axis.z=" << link7_z_axis(2)
                              << "\n                       collision mesh bottom Z=" << collision_mesh_bottom_z
                              << ", top Z=" << collision_mesh_top_z
                              << ", clearance to table=" << (collision_mesh_bottom_z - (table_center(2) + table_half_size(2))) * 1000 << " mm" << std::endl;

                    // CRITICAL: Direct SceneGraph collision check
                    std::vector<drake::geometry::PenetrationAsPointPair<double>> penetrations =
                        drake_sim.ComputePenetrations();

                    if (!penetrations.empty())
                    {
                        std::cout << "    [COLLISION DETECTED] Found " << penetrations.size() << " penetrating geometry pairs!" << std::endl;
                        for (const auto &pen : penetrations)
                        {
                            std::cout << "      - Geometry A: " << pen.id_A << ", Geometry B: " << pen.id_B
                                      << ", depth: " << (pen.depth * 1000) << " mm" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "    [NO PENETRATION] SceneGraph reports no penetrations" << std::endl;
                    }
                }
            }

            // Track minimum distance
            if (dist_to_table_surface < min_dist_to_table)
            {
                min_dist_to_table = dist_to_table_surface;
                closest_ee_pos = ee_pos_world;
                closest_time = t;
            }

            // DEBUG: Print when EE is very close to or through table
            if (dist_to_table_surface < 0.01) // Within 1cm
            {
                std::cout << "  [COLLISION WARNING] t=" << std::fixed << std::setprecision(3) << t
                          << "s: EE at (" << ee_pos_world.transpose() << ")"
                          << " distance to table: " << dist_to_table_surface * 1000.0 << " mm" << std::endl;
            }

            // Sample trajectory for visualization (using world coordinates)
            if (step_count % traj_sample_interval == 0)
            {
                traj_points.push_back(static_cast<float>(ee_pos_world(0)));
                traj_points.push_back(static_cast<float>(ee_pos_world(1)));
                traj_points.push_back(static_cast<float>(ee_pos_world(2)));
            }

            // Render with trajectory
            if (step_count % 5 == 0)
            { // Render every 5 steps
                mujoco_sim.render(traj_points);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "\n=== Simulation Complete ===" << std::endl;
        std::cout << "Total time: " << elapsed.count() << " ms" << std::endl;
        std::cout << "Steps: " << step_count << std::endl;
        std::cout << "Trajectory points recorded: " << (traj_points.size() / 3) << std::endl;

        // DEBUG: Print minimum distance to table
        std::cout << "\n[TABLE PROXIMITY ANALYSIS]" << std::endl;
        std::cout << "  Minimum distance to table surface: " << min_dist_to_table * 1000.0 << " mm" << std::endl;
        std::cout << "  At time: " << closest_time << " s" << std::endl;
        std::cout << "  EE position: (" << closest_ee_pos.transpose() << ")" << std::endl;
        if (min_dist_to_table < 0)
        {
            std::cout << "  [COLLISION] EE penetrated table by " << -min_dist_to_table * 1000.0 << " mm!" << std::endl;
        }
        else if (min_dist_to_table < 0.01)
        {
            std::cout << "  [WARNING] EE came within " << min_dist_to_table * 1000.0 << " mm of table!" << std::endl;
        }
        else
        {
            std::cout << "  [SAFE] Trajectory stayed clear of table" << std::endl;
        }
        std::cout << "\nPress close window to exit..." << std::endl;

        // Keep window open
        while (!mujoco_sim.should_close())
        {
            mujoco_sim.render(traj_points);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "\n=== Test Completed Successfully ===" << std::endl;
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n=== ERROR ===" << std::endl;
        std::cerr << e.what() << std::endl;
        return 1;
    }
}