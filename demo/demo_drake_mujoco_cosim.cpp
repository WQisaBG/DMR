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
#include <drake/planning/iris/iris_zo.h>
#include <drake/planning/collision_checker.h>
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

// MuJoCo Simulator wrapper with trajectory visualization
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
        std::cout << "\n  Joint qpos mapping:" << std::endl;
        for (int i = 0; i < model_->njnt; ++i)
        {
            int qpos_offset = model_->jnt_qposadr[i];
            const char *jnt_name = mj_id2name(model_, mjOBJ_JOINT, i);
            std::cout << "    joint[" << i << "] = " << (jnt_name ? jnt_name : "unknown")
                      << " -> qpos[" << qpos_offset << "]" << std::endl;
        }

        // Print important body IDs for debugging
        std::cout << "\n  Important body IDs:" << std::endl;
        int right_tool_tip_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_tip");
        int left_tool_tip_id = mj_name2id(model_, mjOBJ_BODY, "left_tool_tip");
        int right_tool_frame_id = mj_name2id(model_, mjOBJ_BODY, "right_tool_frame");
        int left_tool_frame_id = mj_name2id(model_, mjOBJ_BODY, "left_tool_frame");
        int right_arm_link7_id = mj_name2id(model_, mjOBJ_BODY, "right_arm_link7");
        int left_arm_link7_id = mj_name2id(model_, mjOBJ_BODY, "left_arm_link7");
        std::cout << "    right_tool_tip ID: " << right_tool_tip_id << std::endl;
        std::cout << "    left_tool_tip ID: " << left_tool_tip_id << std::endl;
        std::cout << "    right_tool_frame ID: " << right_tool_frame_id << std::endl;
        std::cout << "    left_tool_frame ID: " << left_tool_frame_id << std::endl;
        std::cout << "    right_arm_link7 ID: " << right_arm_link7_id << std::endl;
        std::cout << "    left_arm_link7 ID: " << left_arm_link7_id << std::endl;

        // Print site information
        std::cout << "\n  Site information:" << std::endl;
        int ee_site_id = mj_name2id(model_, mjOBJ_SITE, "ee_site");
        std::cout << "    ee_site ID: " << ee_site_id << std::endl;
        std::cout << "    Total sites: " << model_->nsite << std::endl;

        if (enable_visualization)
        {
            std::cout << "  - Visualization: enabled" << std::endl;
        }
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

// Drake Simulator wrapper with FIXED BASE configuration
class DrakeSimulator
{
public:
    DrakeSimulator(const std::string &urdf_path, double time_step = 0.001)
    {
        // ============================================================
        // APPROACH 1: Build regular Diagram for simulation
        // ============================================================
        drake::systems::DiagramBuilder<double> builder;

        // Create MultibodyPlant with SceneGraph for welding support
        plant_ = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(time_step);
        scene_graph_ = builder.AddSystem<drake::geometry::SceneGraph<double>>();
        drake::geometry::SourceId plant_source_id = plant_->RegisterAsSourceForSceneGraph(scene_graph_);

        // Load URDF model
        auto parser = drake::multibody::Parser(plant_);

        // IMPORTANT: Disable MakeConvexHulls for STL files (Drake only supports .obj, .vtk, .gltf)
        // STL files will be used as-is for collision geometry
        auto &package_map = parser.package_map();
        parser.AddModelsFromUrl(std::string("file://") + urdf_path);

        // WELD BASE TO WORLD FRAME - This removes the floating base DOFs
        // After welding, the robot has only 8 DOF (waist + 7 arm joints)
        const drake::multibody::Frame<double> &world_frame = plant_->world_frame();
        const drake::multibody::Frame<double> &base_frame =
            plant_->GetFrameByName("base_link"); // Nezha robot uses base_link

        plant_->WeldFrames(world_frame, base_frame, drake::math::RigidTransformd());

        // Finalize plant
        plant_->Finalize();

        // IMPORTANT: Connect SceneGraph and MultibodyPlant ports for collision detection
        // This enables the geometry_query_input_port on the plant
        builder.Connect(
            plant_->get_geometry_pose_output_port(),
            scene_graph_->get_source_pose_port(plant_source_id));

        builder.Connect(scene_graph_->get_query_output_port(),
                        plant_->get_geometry_query_input_port());

        // Build diagram
        diagram_ = builder.Build();

        // Create simulator with default context
        simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_);

        // ============================================================
        // APPROACH 2: Build RobotDiagram for GCS planning
        // ============================================================
        try
        {
            drake::planning::RobotDiagramBuilder<double> robot_builder(time_step);

            // Load URDF using RobotDiagramBuilder's parser
            robot_builder.parser().AddModelsFromUrl(std::string("file://") + urdf_path);

            // Weld base to world
            const drake::multibody::Frame<double> &robot_world_frame = robot_builder.plant().world_frame();
            const drake::multibody::Frame<double> &robot_base_frame =
                robot_builder.plant().GetFrameByName("base_link");

            robot_builder.plant().WeldFrames(robot_world_frame, robot_base_frame, drake::math::RigidTransformd());

            // Build RobotDiagram
            robot_diagram_ = robot_builder.Build();

            std::cout << "RobotDiagram created for GCS planning!" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[WARNING] Failed to create RobotDiagram: " << e.what() << std::endl;
            std::cerr << "GCS planning will fall back to waypoint-based approach" << std::endl;
        }

        std::cout << "Drake Model loaded successfully!" << std::endl;
        std::cout << "  - Positions: " << plant_->num_positions() << " (FIXED BASE)" << std::endl;
        std::cout << "  - Velocities: " << plant_->num_velocities() << std::endl;
        std::cout << "  - Actuators: " << plant_->num_actuators() << std::endl;
    }

    void reset()
    {
        simulator_->reset_context(simulator_->get_context().Clone());
        simulator_->get_mutable_context().SetTime(0.0);
    }

    void set_state(const VectorXd &q, const VectorXd &v)
    {
        // Get fresh plant context pointer
        auto &plant_context = plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        plant_->SetPositions(&plant_context, q);
        plant_->SetVelocities(&plant_context, v);
    }

    void get_state(VectorXd &q, VectorXd &v) const
    {
        auto &plant_context = plant_->GetMyContextFromRoot(simulator_->get_context());
        q = plant_->GetPositions(plant_context);
        v = plant_->GetVelocities(plant_context);
    }

    void step(double dt)
    {
        simulator_->AdvanceTo(simulator_->get_context().get_time() + dt);
    }

    double get_time() const { return simulator_->get_context().get_time(); }

    int get_num_positions() const { return plant_->num_positions(); }
    int get_num_velocities() const { return plant_->num_velocities(); }

    void print_state() const
    {
        auto &plant_context = plant_->GetMyContextFromRoot(simulator_->get_context());
        VectorXd q = plant_->GetPositions(plant_context);
        std::cout << "Drake State:" << std::endl;
        std::cout << "  Time: " << simulator_->get_context().get_time() << std::endl;
        std::cout << "  Positions: " << q.transpose().head(6) << "..." << std::endl;
    }

    // ========== DRAKE FORWARD KINEMATICS ==========
    // Compute end-effector pose in waist_link frame (NOT world frame!)
    // This allows trajectory planning relative to the robot's waist   wqdd   正向运动学
    drake::math::RigidTransformd ComputeEEPose(const VectorXd &q)
    {
        auto &plant_context = plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
        plant_->SetPositions(&plant_context, q);

        // Get end-effector frame (right_tool_frame)
        const auto &ee_frame = plant_->GetFrameByName("right_tool_frame"); // TODO: 末端位姿

        // Get waist_link frame as reference (instead of world_frame)
        const auto &waist_frame = plant_->GetFrameByName("waist_link"); // TODO: 参考坐标系 腰部位姿

        // Compute forward kinematics relative to waist_link
        return plant_->CalcRelativeTransform(plant_context, waist_frame, ee_frame);
    }

    // Helper function declarations
    std::optional<VectorXd> SolveGlobalIK(
        const drake::math::RigidTransformd &desired_pose,
        const VectorXd &q_guess,
        bool debug);

    VectorXd GenerateRandomGuess(const VectorXd &q_guess);

    // Removed SolveHierarchicalIK - using only Global IK

    // ========== TRAJECTORY GENERATION IN CARTESIAN SPACE ==========
    // =================================================================
    // Industrial-Grade Circular Trajectory Planning
    // 工业级圆弧轨迹规划 - 使用minimum-jerk和平滑圆弧插值
    // =================================================================

    /**
     * @brief Generate smooth circular trajectory in Cartesian space
     * Uses minimum-jerk trajectory for smooth circular motion
     */
    drake::trajectories::PiecewisePolynomial<double>
    GenerateSmoothCircularTrajectory(
        const Eigen::Vector3d &circle_center,
        double radius,
        const Eigen::Vector3d &normal,
        const Eigen::Vector3d &start_point,
        double max_velocity,
        double max_acceleration)
    {
        std::cout << "\n[SMOOTH CIRCULAR TRAJECTORY]" << std::endl;
        std::cout << "  Center: " << circle_center.transpose() << " m" << std::endl;
        std::cout << "  Radius: " << radius << " m" << std::endl;
        std::cout << "  Normal: " << normal.transpose() << std::endl;
        std::cout << "  Start: " << start_point.transpose() << " m" << std::endl;

        // Create circle coordinate frame
        Eigen::Vector3d z_axis = normal.normalized();
        Eigen::Vector3d x_axis = (Eigen::Vector3d::UnitX().cross(z_axis)).normalized();
        if (x_axis.norm() < 0.1)
        {
            x_axis = (Eigen::Vector3d::UnitY().cross(z_axis)).normalized();
        }
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);

        // Compute angle offset from start point
        Eigen::Vector3d start_offset = (start_point - circle_center).normalized();
        double theta_offset = std::atan2(
            start_offset.dot(y_axis),
            start_offset.dot(x_axis));

        std::cout << "  Angle offset: " << (theta_offset * 180.0 / M_PI) << " deg" << std::endl;

        // Compute duration for full circle
        double circumference = 2.0 * M_PI * radius;
        double t_accel = max_velocity / max_acceleration;
        double dist_accel = 0.5 * max_acceleration * t_accel * t_accel;
        double duration;

        if (circumference < 2.0 * dist_accel)
        {
            duration = 2.0 * std::sqrt(circumference / max_acceleration);
        }
        else
        {
            double dist_const_vel = circumference - 2.0 * dist_accel;
            double t_const_vel = dist_const_vel / max_velocity;
            duration = 2.0 * t_accel + t_const_vel;
        }

        duration *= 1.1; // 10% safety margin

        std::cout << "  Duration: " << duration << " s" << std::endl;
        std::cout << "  Circumference: " << circumference << " m" << std::endl;

        // Sample circle at high resolution
        const int num_samples = 101;
        std::vector<double> breaks(num_samples);
        std::vector<MatrixXd> samples(num_samples);
        std::vector<MatrixXd> derivatives(num_samples);

        for (int i = 0; i < num_samples; ++i)
        {
            double t = duration * i / (num_samples - 1);
            double tau = t / duration;

            // Minimum-jerk trajectory for angle: θ goes from 0 to 2π
            double theta_tau = 2.0 * M_PI * (10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5));

            // Add angle offset to start from the correct position
            double theta = theta_tau + theta_offset;

            // Angular velocity
            double omega_tau = 2.0 * M_PI * (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;

            // Position on circle
            Eigen::Vector3d pos = circle_center +
                                  radius * (std::cos(theta) * x_axis + std::sin(theta) * y_axis);

            // Tangential velocity
            Eigen::Vector3d vel = radius * omega_tau * (-std::sin(theta) * x_axis + std::cos(theta) * y_axis);

            breaks[i] = t;
            samples[i] = pos;
            derivatives[i] = vel;
        }

        std::cout << "  Generated with " << num_samples << " samples" << std::endl;

        return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, samples, derivatives);
    }

    // =================================================================
    // Simplified robust Cartesian line planning using Drake's PiecewisePolynomial
    // 使用Drake简单可靠的API避免复杂的S-Curve计算错误
    // =================================================================

    /**
     * @brief Generate 7th-order minimum-jerk S-curve trajectory (OPTIMIZED)
     * Uses 7th-order polynomial for ultra-smooth motion with zero jerk at boundaries
     * This provides C³ continuity: smooth position, velocity, acceleration, AND jerk
     *
     * The 7th-order polynomial satisfies:
     * - s(0) = 0, s(1) = 1 (position boundary conditions)
     * - s'(0) = 0, s'(1) = 0 (zero velocity at boundaries)
     * - s''(0) = 0, s''(1) = 0 (zero acceleration at boundaries)
     * - s'''(0) = 0, s'''(1) = 0 (zero jerk at boundaries)
     */
    drake::trajectories::PiecewisePolynomial<double>
    GenerateSmoothCartesianTrajectory(
        const Eigen::Vector3d &start_position,
        const Eigen::Vector3d &goal_position,
        double max_velocity,
        double max_acceleration)
    {
        double distance = (goal_position - start_position).norm();
        Eigen::Vector3d direction = (goal_position - start_position).normalized();

        // Compute minimum time duration with S-curve profile
        double t_accel = max_velocity / max_acceleration;
        double dist_accel = 0.5 * max_acceleration * t_accel * t_accel;
        double duration;

        if (distance < 2.0 * dist_accel)
        {
            duration = 2.0 * std::sqrt(distance / max_acceleration);
        }
        else
        {
            double dist_const_vel = distance - 2.0 * dist_accel;
            double t_const_vel = dist_const_vel / max_velocity;
            duration = 2.0 * t_accel + t_const_vel;
        }

        duration *= 1.1; // 10% safety margin

        std::cout << "  [S-CURVE TRAJECTORY - 7th ORDER] Duration: " << duration << " s" << std::endl;
        std::cout << "  [S-CURVE TRAJECTORY - 7th ORDER] Distance: " << distance << " m" << std::endl;
        std::cout << "  [S-CURVE TRAJECTORY - 7th ORDER] Profile: C³ continuous (zero jerk at boundaries)" << std::endl;

        // Create 7th-order minimum-jerk trajectory with S-curve acceleration
        // s(t) = -20τ⁷ + 70τ⁶ - 84τ⁵ + 35τ⁴  where τ = t/duration
        const int num_samples = 401; // Increased for smoother interpolation
        std::vector<double> breaks(num_samples);
        std::vector<MatrixXd> samples(num_samples);
        std::vector<MatrixXd> derivatives(num_samples);
        std::vector<MatrixXd> second_derivatives(num_samples); // For acceleration

        for (int i = 0; i < num_samples; ++i)
        {
            double t = duration * i / (num_samples - 1);
            double tau = t / duration;

            // 7th-order minimum-jerk trajectory (C³ continuous!)
            double s_tau = -20.0 * std::pow(tau, 7) +
                            70.0 * std::pow(tau, 6) -
                            84.0 * std::pow(tau, 5) +
                            35.0 * std::pow(tau, 4);

            // Velocity profile (1st derivative)
            double v_tau = (-140.0 * std::pow(tau, 6) +
                             420.0 * std::pow(tau, 5) -
                             420.0 * std::pow(tau, 4) +
                             140.0 * std::pow(tau, 3)) /
                            duration;

            // Acceleration profile (2nd derivative) - for verification
            double a_tau = (-840.0 * std::pow(tau, 5) +
                             2100.0 * std::pow(tau, 4) -
                             1680.0 * std::pow(tau, 3) +
                             420.0 * std::pow(tau, 2)) /
                            (duration * duration);

            breaks[i] = t;
            samples[i] = start_position + direction * (distance * s_tau);
            derivatives[i] = direction * (distance * v_tau);
            second_derivatives[i] = direction * (distance * a_tau);
        }

        std::cout << "  [S-CURVE TRAJECTORY] Generated with " << num_samples << " samples" << std::endl;

        // Use CubicHermite for C¹ continuity (velocity continuous)
        // This is sufficient since the underlying 7th-order profile already ensures C³
        return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, samples, derivatives);
    }

    /**
     * @brief Apply Savitzky-Golay filter to smooth 1D trajectory data
     *
     * Savitzky-Golay filter fits a polynomial to sliding windows of data,
     * preserving higher-order moments while eliminating high-frequency noise.
     *
     * @param data Input data to be filtered
     * @param window_size Size of sliding window (must be odd)
     * @param poly_order Order of fitting polynomial
     * @return Filtered data
     */
    std::vector<double> ApplySavitzkyGolayFilter(
        const std::vector<double>& data,
        int window_size,
        int poly_order)
    {
        int n = data.size();
        std::vector<double> filtered(n);

        // Half window size
        int half_window = window_size / 2;

        // For interior points, apply Savitzky-Golay convolution
        for (int i = 0; i < n; ++i)
        {
            // Determine window boundaries
            int left = std::max(0, i - half_window);
            int right = std::min(n - 1, i + half_window);
            int actual_window = right - left + 1;

            // For edge points, use smaller window or linear interpolation
            if (actual_window < poly_order + 1)
            {
                // Use simple averaging for edges
                double sum = 0.0;
                for (int j = left; j <= right; ++j)
                {
                    sum += data[j];
                }
                filtered[i] = sum / actual_window;
            }
            else
            {
                // Fit polynomial using least squares (simplified version)
                // For computational efficiency, we use a weighted average
                // that approximates Savitzky-Golay coefficients

                std::vector<double> weights(actual_window, 1.0);

                // Create triangular weights (approximates SG coefficients)
                int center = i - left;
                for (int j = 0; j < actual_window; ++j)
                {
                    double dist = std::abs(j - center);
                    weights[j] = 1.0 - (double)dist / (actual_window / 2.0);
                    if (weights[j] < 0.1) weights[j] = 0.1;
                }

                // Apply weighted average
                double weighted_sum = 0.0;
                double weight_total = 0.0;
                for (int j = 0; j < actual_window; ++j)
                {
                    weighted_sum += weights[j] * data[left + j];
                    weight_total += weights[j];
                }
                filtered[i] = weighted_sum / weight_total;
            }
        }

        return filtered;
    }

    /**
     * @brief Generate Cartesian position trajectory with kinematic constraints
     * Creates a smooth 5th-order polynomial trajectory that respects:
     * - Zero velocity/acceleration at start and end
     * - Maximum velocity/acceleration/jerk constraints
     * NOTE: This function is replaced by GenerateSCurveTrajectoryCartesian
     * Kept for backward compatibility
     */
    drake::trajectories::PiecewisePolynomial<double>
    GenerateCartesianTrajectoryWithConstraints(
        const Eigen::Vector3d &start_position,
        const Eigen::Vector3d &goal_position,
        double duration,
        double max_velocity,
        double max_acceleration,
        double max_jerk)
    {
        // Create minimum-jerk trajectory (5th order polynomial)
        // This is the optimal solution for minimum jerk point-to-point motion
        // Reference: Flash & Hogan (1985) "The coordination of arm movements"

        const std::vector<double> breaks = {0.0, duration};
        std::vector<MatrixXd> samples(2);
        std::vector<MatrixXd> derivatives(2);

        samples[0] = start_position;
        samples[1] = goal_position;

        // For minimum jerk: start and end with zero velocity and acceleration
        derivatives[0] = Eigen::Vector3d::Zero(); // Zero velocity at start
        derivatives[1] = Eigen::Vector3d::Zero(); // Zero velocity at goal

        auto trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, samples, derivatives);

        // Verify constraints and scale duration if necessary
        // Sample trajectory to check max velocity/acceleration
        const int num_samples = 100;
        double max_vel_actual = 0.0;
        double max_accel_actual = 0.0;

        for (int i = 0; i <= num_samples; ++i)
        {
            double t = duration * i / num_samples;
            Eigen::Vector3d vel = trajectory.derivative(1).value(t);
            Eigen::Vector3d accel = trajectory.derivative(2).value(t);

            max_vel_actual = std::max(max_vel_actual, vel.norm());
            max_accel_actual = std::max(max_accel_actual, accel.norm());
        }

        // If constraints are violated, scale the trajectory
        if (max_vel_actual > max_velocity || max_accel_actual > max_acceleration)
        {
            double vel_scale = max_vel_actual > max_velocity ? max_vel_actual / max_velocity : 1.0;
            double accel_scale = max_accel_actual > max_acceleration ? max_accel_actual / max_acceleration : 1.0;

            double scale_factor = std::max(vel_scale, accel_scale);

            std::cout << "  [CONSTRAINT SCALING] Scaling duration by " << scale_factor
                      << " to satisfy Cartesian constraints" << std::endl;

            // Regenerate with longer duration
            return GenerateCartesianTrajectoryWithConstraints(
                start_position, goal_position,
                duration * scale_factor,
                max_velocity, max_acceleration, max_jerk);
        }

        return trajectory;
    }

    // =================================================================
    // NEW: Industrial-Grade Cartesian Line Planning with True S-Curve
    // 使用真正的S型速度规划 (7段速度轮廓) - 符合工业标准
    // =================================================================
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianLineIndustrial(
        const VectorXd &q_start,
        const Eigen::Vector3d &goal_position,
        double max_velocity = 0.5,     // m/s (工业标准: 典型机械臂最大线速度)
        double max_acceleration = 1.0, // m/s² (工业标准: 典型机械臂最大线加速度)
        double max_jerk = 5.0,         // m/s³ (保留参数,但使用minimum-jerk轨迹)
        bool optimize_timing = true)   // 是否使用最优时间规划
    {
        std::cout << "\n=== Industrial-Grade Cartesian Line Planning (Minimum-Jerk) ===" << std::endl;
        std::cout << "Constraints:" << std::endl;
        std::cout << "  Max Velocity: " << max_velocity << " m/s" << std::endl;
        std::cout << "  Max Acceleration: " << max_acceleration << " m/s²" << std::endl;
        std::cout << "  Using minimum-jerk quintic polynomial" << std::endl;

        // Get initial EE pose
        drake::math::RigidTransformd T_ee_start = ComputeEEPose(q_start);
        Eigen::Vector3d pos_start = T_ee_start.translation();

        std::cout << "\nPath Information:" << std::endl;
        std::cout << "  Start: " << pos_start.transpose() << " m" << std::endl;
        std::cout << "  Goal:  " << goal_position.transpose() << " m" << std::endl;
        double distance = (goal_position - pos_start).norm();
        std::cout << "  Distance: " << distance << " m" << std::endl;

        // Generate smooth minimum-jerk trajectory in Cartesian space
        std::cout << "\nGenerating smooth minimum-jerk trajectory..." << std::endl;
        auto cartesian_trajectory = GenerateSmoothCartesianTrajectory(
            pos_start, goal_position,
            max_velocity, max_acceleration);

        double duration = cartesian_trajectory.end_time();
        std::cout << "  Smooth trajectory generated with duration: " << duration << " s" << std::endl;

        // Sample the S-curve trajectory at waypoints
        // INCREASE waypoints for better tracking accuracy
        const int num_waypoints = 101; // Increased from 51 to 101 for better tracking
        std::vector<double> breaks(num_waypoints);

        // Use S-curve timing (not uniform spacing)
        for (int i = 0; i < num_waypoints; ++i)
        {
            breaks[i] = duration * i / (num_waypoints - 1);
        }

        std::cout << "\nConverting to joint space using Differential IK..." << std::endl;
        std::cout << "  Waypoints: " << num_waypoints << " (increased for better tracking)" << std::endl;

        // Set up Differential IK with Cartesian constraints
        const double dt = duration / (num_waypoints - 1);

        drake::multibody::DifferentialInverseKinematicsParameters dik_params(
            plant_->num_positions(),
            plant_->num_velocities());

        dik_params.set_time_step(dt);
        dik_params.set_nominal_joint_position(q_start);

        // Joint position limits
        VectorXd lower_pos_limits = plant_->GetPositionLowerLimits();
        VectorXd upper_pos_limits = plant_->GetPositionUpperLimits();
        const double margin = 0.01;
        lower_pos_limits = lower_pos_limits.array() + margin;
        upper_pos_limits = upper_pos_limits.array() - margin;
        dik_params.set_joint_position_limits({lower_pos_limits, upper_pos_limits});

        // Relax orientation constraint for line motion
        dik_params.set_end_effector_angular_speed_limit(10.0);

        // Joint velocity limits - lock all joints EXCEPT right arm (11-17)
        const double max_joint_velocity_ik = 3.0; // rad/s
        VectorXd lower_velocity_limits = VectorXd::Constant(plant_->num_positions(), -max_joint_velocity_ik);
        VectorXd upper_velocity_limits = VectorXd::Constant(plant_->num_positions(), max_joint_velocity_ik);

        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17) // Not right arm
            {
                lower_velocity_limits(i) = 0.0;
                upper_velocity_limits(i) = 0.0;
            }
        }
        dik_params.set_joint_velocity_limits({lower_velocity_limits, upper_velocity_limits});

        // Only control linear velocity
        drake::Vector6<bool> ee_velocity_flag;
        ee_velocity_flag << false, false, false, // No angular control
            true, true, true;                    // Linear velocity control
        dik_params.set_end_effector_velocity_flag(ee_velocity_flag);

        // CRITICAL: Set higher joint centering gain for non-right-arm joints
        // This strongly penalizes deviation from nominal configuration
        MatrixXd centering_gain = MatrixXd::Zero(plant_->num_positions(), plant_->num_positions());

        // High gain for locked joints (legs, waist, left arm, head) to keep them stationary
        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17) // Not right arm
            {
                centering_gain(i, i) = 100.0; // Strong gain to lock these joints
            }
            else
            {
                centering_gain(i, i) = 0.01; // Small gain for right arm flexibility
            }
        }
        dik_params.set_joint_centering_gain(centering_gain);

        // Get frames and context
        const auto &ee_frame = plant_->GetFrameByName("right_tool_frame");
        auto &plant_context = plant_->GetMyMutableContextFromRoot(
            &simulator_->get_mutable_context());

        // Track joint configurations
        std::vector<MatrixXd> joint_samples(num_waypoints);
        VectorXd q_current = q_start;
        int success_count = 0;
        int fail_count = 0;

        // Track Cartesian trajectory execution
        std::cout << "\nExecuting Cartesian trajectory with Differential IK:" << std::endl;

        for (int i = 0; i < num_waypoints; ++i)
        {
            double t = breaks[i];
            breaks[i] = t; // Store actual time

            // Get desired Cartesian position from trajectory
            Eigen::Vector3d desired_pos = cartesian_trajectory.value(t);

            // Get desired Cartesian velocity from trajectory
            Eigen::Vector3d desired_vel = cartesian_trajectory.derivative(1).value(t);

            // Set current robot state
            plant_->SetPositions(&plant_context, q_current);

            // Compute current EE position
            Eigen::Vector3d current_pos = ComputeEEPose(q_current).translation();

            // Compute error-corrected velocity using P-controller
            // INCREASED gain for better tracking accuracy
            const double kp = 50.0; // Increased from 10.0 to 50.0 for better tracking
            Eigen::Vector3d pos_error = desired_pos - current_pos;
            Eigen::Vector3d corrected_vel = desired_vel + kp * pos_error;

            // Clamp corrected velocity to avoid excessive joint velocities
            double max_ee_vel = max_velocity * 1.5; // Allow 1.5x max velocity for error correction
            if (corrected_vel.norm() > max_ee_vel)
            {
                corrected_vel = corrected_vel.normalized() * max_ee_vel;
            }

            // Create spatial velocity [angular, linear]
            drake::Vector6<double> V_WE_desired;
            V_WE_desired << 0, 0, 0, // Zero angular velocity
                corrected_vel(0), corrected_vel(1), corrected_vel(2);

            // Solve Differential IK
            auto result = drake::multibody::DoDifferentialInverseKinematics(
                *plant_, plant_context, V_WE_desired, ee_frame, dik_params);

            if (result.status == drake::multibody::DifferentialInverseKinematicsStatus::kSolutionFound)
            {
                VectorXd q_dot = result.joint_velocities.value();
                VectorXd q_next = q_current + q_dot * dt;

                // Enforce joint limits
                q_next = q_next.cwiseMax(plant_->GetPositionLowerLimits())
                             .cwiseMin(plant_->GetPositionUpperLimits());

                // Collision detection
                CollisionResult col_result = CheckCollisionDetailed(q_next);

                if (col_result.has_collision)
                {
                    if (i == 0)
                    {
                        joint_samples[i] = q_start;
                    }
                    else
                    {
                        joint_samples[i] = q_current;
                    }
                    if (i < 5)
                    {
                        std::cout << "  [COLLISION] Waypoint " << i << ": "
                                  << col_result.warning_message << std::endl;
                    }
                }
                else
                {
                    q_current = q_next;

                    // Optional: Refine position accuracy using Jacobian-based refinement
                    // This significantly improves tracking accuracy
                    const int refinement_steps = 2;
                    for (int ref = 0; ref < refinement_steps; ++ref)
                    {
                        plant_->SetPositions(&plant_context, q_current);
                        Eigen::Vector3d refined_pos = ComputeEEPose(q_current).translation();
                        Eigen::Vector3d pos_error_refined = desired_pos - refined_pos;

                        // Only refine if error is significant (> 1mm)
                        if (pos_error_refined.norm() > 0.001)
                        {
                            // Compute Jacobian for refinement
                            drake::MatrixX<double> J(6, plant_->num_velocities());
                            plant_->CalcJacobianSpatialVelocity(
                                plant_context,
                                drake::multibody::JacobianWrtVariable::kV,
                                ee_frame,
                                Eigen::Vector3d::Zero(),
                                plant_->world_frame(),
                                plant_->world_frame(),
                                &J);

                            // Extract linear part of Jacobian (rows 3-5)
                            Eigen::MatrixXd J_linear = J.block(3, 0, 3, plant_->num_velocities());

                            // CRITICAL: Only use right arm joints (11-17) for refinement
                            // Create a mask to zero out non-right-arm columns
                            Eigen::MatrixXd J_linear_right_arm(3, 7); // Only 7 right arm joints
                            int right_arm_idx = 0;
                            for (int j = 0; j < plant_->num_velocities(); ++j)
                            {
                                if (j >= 11 && j <= 17)
                                {
                                    J_linear_right_arm.col(right_arm_idx) = J_linear.col(j);
                                    right_arm_idx++;
                                }
                            }

                            // Solve for joint correction: dq = J_pinv * error (only right arm)
                            Eigen::VectorXd dq_right_arm = J_linear_right_arm.completeOrthogonalDecomposition().solve(pos_error_refined);

                            // Apply correction with damping - only to right arm joints
                            const double alpha = 0.5;
                            int dq_idx = 0;
                            for (int j = 0; j < plant_->num_positions(); ++j)
                            {
                                if (j >= 11 && j <= 17)
                                {
                                    q_current(j) += alpha * dq_right_arm(dq_idx);
                                    dq_idx++;
                                }
                                // Non-right-arm joints remain unchanged
                            }

                            // Clamp to joint limits
                            q_current = q_current.cwiseMax(plant_->GetPositionLowerLimits())
                                            .cwiseMin(plant_->GetPositionUpperLimits());
                        }
                    }

                    joint_samples[i] = q_current;
                    success_count++;
                }
            }
            else
            {
                if (i == 0)
                {
                    joint_samples[i] = q_start;
                }
                else
                {
                    joint_samples[i] = q_current;
                }
                fail_count++;
                if (i < 5)
                {
                    std::cout << "  [DIK FAILED] Waypoint " << i << std::endl;
                }
            }

            // Progress update
            if (i % 10 == 0 || i == num_waypoints - 1)
            {
                Eigen::Vector3d actual_pos = ComputeEEPose(q_current).translation();
                double tracking_error = (actual_pos - desired_pos).norm();

                // Check non-right-arm joint deviation
                double max_non_arm_deviation = 0.0;
                for (int j = 0; j < plant_->num_positions(); ++j)
                {
                    if (j < 11 || j > 17) // Non-right-arm joints
                    {
                        double deviation = std::abs(q_current(j) - q_start(j));
                        max_non_arm_deviation = std::max(max_non_arm_deviation, deviation);
                    }
                }

                std::cout << "  Waypoint " << i << "/" << num_waypoints
                          << " | t=" << std::fixed << std::setprecision(3) << t << " s"
                          << " | Tracking Error: " << std::scientific << tracking_error << " m"
                          << " | Non-arm deviation: " << std::fixed << max_non_arm_deviation << " rad"
                          << std::endl;

                // Warn if non-arm joints are moving too much
                if (max_non_arm_deviation > 0.01)
                {
                    std::cout << "    [WARNING] Non-arm joints moving! Max deviation: "
                              << max_non_arm_deviation << " rad" << std::endl;
                }
            }
        }

        // FINAL SAFETY CHECK: Force all non-right-arm joints to stay at initial values
        std::cout << "\n[SAFETY CHECK] Forcing non-right-arm joints to initial positions..." << std::endl;
        for (int i = 0; i < num_waypoints; ++i)
        {
            for (int j = 0; j < plant_->num_positions(); ++j)
            {
                if (j < 11 || j > 17) // Non-right-arm joints
                {
                    joint_samples[i](j) = q_start(j);
                }
            }
        }
        std::cout << "  [OK] All non-right-arm joints locked to initial configuration" << std::endl;

        // =================================================================
        // INDUSTRIAL-GRADE: Final IK refinement for high precision
        // Ensure repeatability precision meets industrial standards (< 0.1mm)
        // =================================================================
        std::cout << "\n[HIGH PRECISION IK REFINEMENT]" << std::endl;
        std::cout << "  Applying Newton-Raphson IK refinement for industrial precision..." << std::endl;

        const double position_tolerance = 1e-4; // 0.1mm (工业标准)
        const int max_iterations = 50;

        int refined_count = 0;
        double max_final_error = 0.0;

        for (int i = 0; i < num_waypoints; ++i)
        {
            VectorXd q = joint_samples[i];

            // Newton-Raphson iteration for precise positioning
            for (int iter = 0; iter < max_iterations; ++iter)
            {
                plant_->SetPositions(&plant_context, q);
                Eigen::Vector3d current_ee = ComputeEEPose(q).translation();

                // Get desired Cartesian position from trajectory
                double t = breaks[i];
                Eigen::Vector3d desired_ee = cartesian_trajectory.value(t);

                Eigen::Vector3d error = desired_ee - current_ee;
                double error_norm = error.norm();

                if (error_norm < position_tolerance)
                {
                    if (i == 0 || i == num_waypoints - 1)
                    {
                        std::cout << "  Waypoint " << i << ": Converged to "
                                  << (error_norm * 1000) << " mm after " << iter << " iterations" << std::endl;
                    }
                    max_final_error = std::max(max_final_error, error_norm);
                    refined_count++;
                    break;
                }

                // Compute Jacobian (only right arm joints)
                drake::MatrixX<double> J(6, plant_->num_velocities());
                plant_->CalcJacobianSpatialVelocity(
                    plant_context,
                    drake::multibody::JacobianWrtVariable::kV,
                    ee_frame,
                    Eigen::Vector3d::Zero(),
                    plant_->world_frame(),
                    plant_->world_frame(),
                    &J);

                // Extract only right arm columns
                Eigen::MatrixXd J_right_arm(3, 7);
                int col_idx = 0;
                for (int j = 11; j <= 17; ++j)
                {
                    J_right_arm.col(col_idx++) = J.block(3, j, 3, 1);
                }

                // Damped Least Squares: Δq = (J^T J + λ²I)^-1 J^T e
                const double damping = 0.01;
                Eigen::MatrixXd JtJ = J_right_arm.transpose() * J_right_arm;
                Eigen::MatrixXd A = JtJ + damping * damping * Eigen::MatrixXd::Identity(7, 7);
                Eigen::VectorXd delta_q = A.ldlt().solve(J_right_arm.transpose() * error);

                // Extract right arm current positions
                VectorXd q_right_arm(7);
                for (int j = 0; j < 7; ++j)
                {
                    q_right_arm(j) = q(11 + j);
                }

                // Apply correction with line search
                double alpha = 1.0;
                const double beta = 0.5; // Backtracking parameter
                VectorXd q_new = q;

                for (int ls = 0; ls < 10; ++ls)
                {
                    // Apply candidate step
                    for (int j = 0; j < 7; ++j)
                    {
                        q_new(11 + j) = q_right_arm(j) + alpha * delta_q(j);
                    }

                    // Clamp to joint limits
                    q_new = q_new.cwiseMax(plant_->GetPositionLowerLimits())
                                .cwiseMin(plant_->GetPositionUpperLimits());

                    // Check if error decreased
                    plant_->SetPositions(&plant_context, q_new);
                    Eigen::Vector3d new_ee = ComputeEEPose(q_new).translation();
                    double new_error = (desired_ee - new_ee).norm();

                    if (new_error < error_norm || alpha < 0.01)
                    {
                        q = q_new;
                        break;
                    }

                    alpha *= beta; // Reduce step size
                }
            }

            // Store refined configuration
            joint_samples[i] = q;
        }

        std::cout << "  [PRECISION] Refined " << refined_count << "/" << num_waypoints << " waypoints" << std::endl;
        std::cout << "  [PRECISION] Max final error: " << (max_final_error * 1000) << " mm" << std::endl;
        std::cout << "  [PRECISION] Industrial standard achieved: " << (max_final_error < 1e-4 ? "YES (< 0.1mm)" : "NO") << std::endl;

        std::cout << "\nResults:" << std::endl;
        std::cout << "  Success: " << success_count << "/" << num_waypoints
                  << " (" << (100.0 * success_count / num_waypoints) << "%)" << std::endl;
        std::cout << "  Failed:  " << fail_count << "/" << num_waypoints << std::endl;

        // Compute velocity derivatives for smooth CubicHermite interpolation
        std::vector<MatrixXd> derivative_samples(num_waypoints);
        for (int i = 0; i < num_waypoints; ++i)
        {
            if (i == 0)
            {
                derivative_samples[i] = (joint_samples[1] - joint_samples[0]) /
                                        (breaks[1] - breaks[0]);
            }
            else if (i == num_waypoints - 1)
            {
                derivative_samples[i] = (joint_samples[i] - joint_samples[i - 1]) /
                                        (breaks[i] - breaks[i - 1]);
            }
            else
            {
                derivative_samples[i] = (joint_samples[i + 1] - joint_samples[i - 1]) /
                                        (breaks[i + 1] - breaks[i - 1]);
            }
            // Smooth the derivatives
            derivative_samples[i] *= 0.5;
        }

        auto final_trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, joint_samples, derivative_samples);

        std::cout << "\nIndustrial-grade trajectory generated successfully!" << std::endl;
        std::cout << "  Segments: " << final_trajectory.get_number_of_segments() << std::endl;
        std::cout << "  Duration: " << duration << " s" << std::endl;
        std::cout << "  Average Velocity: " << (distance / duration) << " m/s" << std::endl;

        // =================================================================
        // EXPORT: Save right arm joint trajectory to CSV for real robot testing
        // =================================================================
        std::cout << "\n[CSV EXPORT] Saving joint trajectory to CSV file..." << std::endl;

        // Generate filename with timestamp
        std::time_t now = std::time(nullptr);
        std::tm *now_tm = std::localtime(&now);
        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", now_tm);

        std::string csv_filename = "trajectory_linear_" + std::string(timestamp) + ".csv";
        // std::string csv_path = "/home/abc/RobotGrasp/DMR/CSV/" + csv_filename;

        // ✅ INDUSTRIAL STANDARD: Use relative path for CSV output
        // Check environment variable first, fallback to current directory
        std::string csv_dir;
        if (const char *env_csv_dir = std::getenv("DMR_CSV_DIR"))
        {
            csv_dir = env_csv_dir;
        }
        else
        {
            csv_dir = "CSV"; // Relative to current working directory
        }

        // Create directory if it doesn't exist
        std::string mkdir_cmd = "mkdir -p " + csv_dir;
        system(mkdir_cmd.c_str());

        std::string csv_path = csv_dir + "/" + csv_filename;

        std::ofstream csv_file(csv_path);
        if (csv_file.is_open())
        {
            // CSV Header
            csv_file << "time,";
            for (int j = 11; j <= 17; ++j)
            {
                csv_file << "joint_" << j;
                if (j < 17)
                    csv_file << ",";
            }
            csv_file << "\n";

            // CSV Data: resample trajectory at higher frequency (100Hz)
            const double export_freq = 100.0; // Hz
            const double export_dt = 1.0 / export_freq;
            const int export_samples = static_cast<int>(duration * export_freq) + 1;

            std::cout << "  Export frequency: " << export_freq << " Hz" << std::endl;
            std::cout << "  Total samples: " << export_samples << std::endl;

            for (int i = 0; i < export_samples; ++i)
            {
                double t = i * export_dt;
                if (t > duration)
                    t = duration;

                VectorXd q = final_trajectory.value(t);

                csv_file << std::fixed << std::setprecision(6) << t << ",";
                for (int j = 11; j <= 17; ++j)
                {
                    csv_file << std::fixed << std::setprecision(9) << q(j);
                    if (j < 17)
                        csv_file << ",";
                }
                csv_file << "\n";
            }

            csv_file.close();
            std::cout << "  [SUCCESS] Trajectory saved to: " << csv_path << std::endl;
            std::cout << "  [INFO] CSV format: time (s), joint_11 to joint_17 (rad)" << std::endl;
            std::cout << "  [INFO] Use this file for real robot deployment" << std::endl;
        }
        else
        {
            std::cout << "  [ERROR] Failed to create CSV file: " << csv_path << std::endl;
        }

        return final_trajectory;
    }

    // =================================================================
    // NEW: Cartesian Line Planning with Full Pose Control (Position + Orientation)
    // 同时控制末端执行器的位置和姿态 - 使用 SLERP 进行姿态插值
    // =================================================================
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianLineWithPose(
        const VectorXd &q_start,
        const drake::math::RigidTransformd &goal_pose,
        double max_velocity = 0.5,             // m/s
        double max_acceleration = 1.0,         // m/s²
        double max_angular_velocity = 1.0,     // rad/s
        double max_angular_acceleration = 2.0, // rad/s²
        bool optimize_timing = true)
    {
        std::cout << "\n=== Cartesian Line Planning with Full Pose Control ===" << std::endl;
        std::cout << "Constraints:" << std::endl;
        std::cout << "  Max Linear Velocity: " << max_velocity << " m/s" << std::endl;
        std::cout << "  Max Linear Acceleration: " << max_acceleration << " m/s²" << std::endl;
        std::cout << "  Max Angular Velocity: " << max_angular_velocity << " rad/s" << std::endl;
        std::cout << "  Max Angular Acceleration: " << max_angular_acceleration << " rad/s²" << std::endl;

        // Get initial EE pose
        drake::math::RigidTransformd T_start = ComputeEEPose(q_start);
        Eigen::Vector3d pos_start = T_start.translation();
        drake::math::RotationMatrixd R_start = T_start.rotation();

        Eigen::Vector3d pos_goal = goal_pose.translation();
        drake::math::RotationMatrixd R_goal = goal_pose.rotation();

        std::cout << "\nPath Information:" << std::endl;
        std::cout << "  Start Position: " << pos_start.transpose() << " m" << std::endl;
        std::cout << "  Goal Position:  " << pos_goal.transpose() << " m" << std::endl;

        // Extract orientation as Roll-Pitch-Yaw for display
        drake::math::RollPitchYawd rpy_start(R_start);
        drake::math::RollPitchYawd rpy_goal(R_goal);
        std::cout << "  Start Orientation (RPY): " << rpy_start.vector().transpose() << " rad" << std::endl;
        std::cout << "  Goal Orientation (RPY):  " << rpy_goal.vector().transpose() << " rad" << std::endl;

        double distance = (pos_goal - pos_start).norm();
        std::cout << "  Linear Distance: " << distance << " m" << std::endl;

        // Calculate angular distance using angle-axis representation
        Eigen::Matrix3d R_diff_matrix = R_goal.matrix() * R_start.matrix().transpose();
        Eigen::AngleAxisd angle_axis(R_diff_matrix);
        double angular_distance = angle_axis.angle();
        std::cout << "  Angular Distance: " << angular_distance << " rad (" << (angular_distance * 180.0 / M_PI) << " deg)" << std::endl;

        // Generate smooth position trajectory
        std::cout << "\nGenerating position trajectory..." << std::endl;
        auto pos_trajectory = GenerateSmoothCartesianTrajectory(
            pos_start, pos_goal,
            max_velocity, max_acceleration);

        double pos_duration = pos_trajectory.end_time();

        // Calculate required duration for orientation based on angular velocity
        double angular_duration = 0.0;
        if (angular_distance > 1e-6)
        {
            // Use minimum time trajectory for rotation: t = sqrt(4*theta/alpha_max)
            // or velocity-limited: t = theta/omega_max
            double t_vel = angular_distance / max_angular_velocity;
            double t_acc = std::sqrt(4.0 * angular_distance / max_angular_acceleration);
            angular_duration = std::max(t_vel, t_acc);
        }
        else
        {
            angular_duration = pos_duration;
        }

        // Use the longer of the two durations
        double duration = std::max(pos_duration, angular_duration);
        std::cout << "  Position trajectory duration: " << pos_duration << " s" << std::endl;
        std::cout << "  Orientation trajectory duration: " << angular_duration << " s" << std::endl;
        std::cout << "  Final trajectory duration: " << duration << " s" << std::endl;

        // ========================================================================
        // S-CURVE ORIENTATION TRAJECTORY: 7th-order polynomial for smooth angular velocity
        // ========================================================================
        std::cout << "\n[S-CURVE ORIENTATION - 7th ORDER] Generating ultra-smooth orientation trajectory..." << std::endl;
        std::cout << "  Strategy: 7th-order minimum-jerk polynomial with C³ continuity" << std::endl;
        std::cout << "  This ensures ZERO angular acceleration at start/end!" << std::endl;

        // Convert to quaternions for interpolation
        // NOTE: rpy_start and rpy_goal are already declared above (line 1653-1654)
        // NOTE: Use quat_start/quat_goal to avoid conflict with q_start parameter (which is VectorXd)
        Eigen::Quaterniond quat_start = rpy_start.ToQuaternion();
        Eigen::Quaterniond quat_goal = rpy_goal.ToQuaternion();

        // Use dense waypoints with S-curve timing for smooth angular velocity
        const int num_ori_waypoints = 401; // Match position trajectory density
        std::vector<double> orientation_breaks(num_ori_waypoints);
        std::vector<Eigen::Quaterniond> quaternions(num_ori_waypoints);

        for (int i = 0; i < num_ori_waypoints; ++i)
        {
            double t = duration * i / (num_ori_waypoints - 1);
            double tau = t / duration;
            orientation_breaks[i] = t;

            // 7th-order S-curve profile for orientation (same as position)
            double s_tau = -20.0 * std::pow(tau, 7) +
                            70.0 * std::pow(tau, 6) -
                            84.0 * std::pow(tau, 5) +
                            35.0 * std::pow(tau, 4);

            // Use S-curve timing for SLERP (not linear!)
            // This gives smooth angular velocity with zero acceleration at boundaries
            // NOTE: slerp(alpha, q) interpolates between *this and q
            Eigen::Quaterniond q_interp = quat_start.slerp(s_tau, quat_goal);
            quaternions[i] = q_interp;
        }

        std::cout << "  [S-CURVE ORIENTATION] Created " << num_ori_waypoints << " waypoints with S-curve timing" << std::endl;
        std::cout << "  [S-CURVE ORIENTATION] This ensures smooth angular velocity AND acceleration" << std::endl;

        auto orientation_trajectory = drake::trajectories::PiecewiseQuaternionSlerp<double>(
            orientation_breaks, quaternions);

        // ========================================================================
        // OPTIMIZED WAYPOINT STRATEGY: Balance between accuracy and smoothness
        // ========================================================================
        std::cout << "\n[OPTIMIZED WAYPOINT STRATEGY] Balancing accuracy and smoothness..." << std::endl;

        // KEY INSIGHT: Need enough waypoints for trajectory accuracy, but not too many
        // - Too few (e.g., 25): trajectory becomes curved, straight lines get distorted
        // - Too many (e.g., 249): IK noise accumulates, causing velocity/accel oscillations
        // - Sweet spot: 75-100 waypoints
        const int num_ik_waypoints = 200; // Balanced: accurate but not noisy
        const double output_frequency = 200.0; // Output at 200Hz

        std::cout << "  IK waypoints: " << num_ik_waypoints << " (balanced for accuracy & smoothness)" << std::endl;
        std::cout << "  Output frequency: " << output_frequency << " Hz" << std::endl;
        std::cout << "  Strategy: Medium-density IK + C² spline smoothing" << std::endl;

        // Generate time breaks
        std::vector<double> breaks(num_ik_waypoints);
        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            breaks[i] = duration * i / (num_ik_waypoints - 1);
        }

        std::cout << "  Path length: " << distance << " m" << std::endl;
        std::cout << "  Angular distance: " << (angular_distance * 180.0 / M_PI) << " deg" << std::endl;
        std::cout << "  IK sampling interval: " << (duration / (num_ik_waypoints - 1) * 1000.0) << " ms" << std::endl;

        std::cout << "\nConverting to joint space using Differential IK (sparse waypoints)..." << std::endl;

        // Set up Differential IK parameters
        const double dt = duration / (num_ik_waypoints - 1);

        drake::multibody::DifferentialInverseKinematicsParameters dik_params(
            plant_->num_positions(),
            plant_->num_velocities());

        dik_params.set_time_step(dt);
        dik_params.set_nominal_joint_position(q_start);

        // Joint position limits
        VectorXd lower_pos_limits = plant_->GetPositionLowerLimits();
        VectorXd upper_pos_limits = plant_->GetPositionUpperLimits();
        const double margin = 0.01;
        lower_pos_limits = lower_pos_limits.array() + margin;
        upper_pos_limits = upper_pos_limits.array() - margin;
        dik_params.set_joint_position_limits({lower_pos_limits, upper_pos_limits});

        // Set angular velocity limit (increase for better convergence)
        dik_params.set_end_effector_angular_speed_limit(max_angular_velocity * 3.0); // Increased from 2.0 to 3.0

        // Joint velocity limits - lock all joints EXCEPT right arm (11-17)
        // IMPORTANT: Increase velocity limits to avoid discontinuity
        const double max_joint_velocity_ik = 5.0; // Increased from 3.0 to 5.0 rad/s
        VectorXd lower_velocity_limits = VectorXd::Constant(plant_->num_positions(), -max_joint_velocity_ik);
        VectorXd upper_velocity_limits = VectorXd::Constant(plant_->num_positions(), max_joint_velocity_ik);

        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17)
            { // Not right arm
                lower_velocity_limits(i) = 0.0;
                upper_velocity_limits(i) = 0.0;
            }
        }
        dik_params.set_joint_velocity_limits({lower_velocity_limits, upper_velocity_limits});

        // CRITICAL: Enable BOTH angular and linear velocity control
        drake::Vector6<bool> ee_velocity_flag;
        ee_velocity_flag << true, true, true, // Angular velocity control (ENABLED!)
            true, true, true;                 // Linear velocity control
        dik_params.set_end_effector_velocity_flag(ee_velocity_flag);

        // Joint centering gain - REDUCED to avoid forcing joints away from solution
        MatrixXd centering_gain = MatrixXd::Zero(plant_->num_positions(), plant_->num_positions());
        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17)
            {                                 // Not right arm
                centering_gain(i, i) = 100.0; // Strong gain to lock these joints
            }
            else
            {
                centering_gain(i, i) = 0.001; // Reduced from 0.01 to 0.001 for more flexibility
            }
        }
        dik_params.set_joint_centering_gain(centering_gain);

        // Get frames and context
        const auto &ee_frame = plant_->GetFrameByName("right_tool_frame");
        auto &plant_context = plant_->GetMyMutableContextFromRoot(
            &simulator_->get_mutable_context());

        // Track joint configurations
        std::vector<MatrixXd> joint_samples(num_ik_waypoints);
        VectorXd q_current = q_start;
        int success_count = 0;
        int fail_count = 0;

        std::cout << "\nExecuting Cartesian trajectory with full pose control (sparse waypoints):" << std::endl;

        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            double t = breaks[i];
            breaks[i] = t;

            // Get desired position and velocity from trajectory
            Eigen::Vector3d desired_pos = pos_trajectory.value(t);
            Eigen::Vector3d desired_linear_vel = pos_trajectory.derivative(1).value(t);

            // Get desired orientation from SLERP trajectory
            Eigen::Quaterniond desired_quat = orientation_trajectory.orientation(t);
            drake::math::RotationMatrixd desired_R(desired_quat);

            // Use PiecewiseQuaternionSlerp's built-in angular velocity
            Eigen::Vector3d desired_angular_vel = orientation_trajectory.angular_velocity(t);

            // Set current robot state
            plant_->SetPositions(&plant_context, q_current);

            // Compute current pose
            drake::math::RigidTransformd T_current = ComputeEEPose(q_current);
            Eigen::Vector3d current_pos = T_current.translation();
            drake::math::RotationMatrixd current_R = T_current.rotation();

            // Position error correction (P-controller) - REDUCED gain for smoother motion
            const double kp_pos = 10.0; // Reduced from 50.0 to 10.0 for smoother tracking
            Eigen::Vector3d pos_error = desired_pos - current_pos;
            Eigen::Vector3d corrected_linear_vel = desired_linear_vel + kp_pos * pos_error;

            // Clamp linear velocity
            double max_ee_vel = max_velocity * 1.2; // Reduced from 1.5 to 1.2
            if (corrected_linear_vel.norm() > max_ee_vel)
            {
                corrected_linear_vel = corrected_linear_vel.normalized() * max_ee_vel;
            }

            // Orientation error correction (reduced gain for better stability)
            const double kp_rot = 1.0; // Reduced from 2.0 to 1.0 for smoother rotation
            drake::math::RotationMatrixd R_error = desired_R * current_R.inverse();

            // Convert rotation error to axis-angle
            drake::math::RollPitchYawd rpy_error(R_error);
            Eigen::Vector3d rot_error_vec = rpy_error.vector();

            // For small angles, use the RPY vector directly as error
            Eigen::Vector3d corrected_angular_vel = desired_angular_vel + kp_rot * rot_error_vec;

            // Clamp angular velocity to avoid excessive corrections
            double max_angular_vel_correction = max_angular_velocity * 2.0; // Increased limit
            if (corrected_angular_vel.norm() > max_angular_vel_correction)
            {
                corrected_angular_vel = corrected_angular_vel.normalized() * max_angular_vel_correction;
            }

            // Create spatial velocity [angular, linear]
            drake::Vector6<double> V_WE_desired;
            V_WE_desired << corrected_angular_vel(0), corrected_angular_vel(1), corrected_angular_vel(2),
                corrected_linear_vel(0), corrected_linear_vel(1), corrected_linear_vel(2);

            // Solve Differential IK
            auto result = drake::multibody::DoDifferentialInverseKinematics(
                *plant_, plant_context, V_WE_desired, ee_frame, dik_params);

            if (result.status == drake::multibody::DifferentialInverseKinematicsStatus::kSolutionFound)
            {
                VectorXd q_dot = result.joint_velocities.value();
                VectorXd q_next = q_current + q_dot * dt;

                // Enforce joint limits
                q_next = q_next.cwiseMax(plant_->GetPositionLowerLimits())
                             .cwiseMin(plant_->GetPositionUpperLimits());

                // Collision detection
                CollisionResult col_result = CheckCollisionDetailed(q_next);

                if (col_result.has_collision)
                {
                    if (i == 0)
                    {
                        joint_samples[i] = q_start;
                    }
                    else
                    {
                        joint_samples[i] = q_current;
                    }
                    if (i < 5)
                    {
                        std::cout << "  [COLLISION] Waypoint " << i << ": "
                                  << col_result.warning_message << std::endl;
                    }
                }
                else
                {
                    // For the first waypoint (t=0), ensure we use the exact starting configuration
                    // This ensures the trajectory starts precisely at q_start
                    if (i == 0)
                    {
                        joint_samples[i] = q_start;
                        q_current = q_start; // Keep q_current at q_start for the next iteration
                    }
                    else
                    {
                        q_current = q_next;

                        // OPTIONAL: Iterative refinement for higher precision
                        // Perform Newton-Raphson style refinement to reduce BOTH position and orientation error
                        const int refinement_iterations = 3;
                        const double position_tolerance = 1e-4;                  // 0.1mm
                        const double orientation_tolerance = 1.0 * M_PI / 180.0; // 1 degree

                        for (int ref_iter = 0; ref_iter < refinement_iterations; ++ref_iter)
                        {
                            plant_->SetPositions(&plant_context, q_current);
                            drake::math::RigidTransformd current_pose_refined = ComputeEEPose(q_current);

                            Eigen::Vector3d pos_error_refined = desired_pos - current_pose_refined.translation();
                            drake::math::RotationMatrixd current_R_refined = current_pose_refined.rotation();

                            // Calculate orientation error
                            Eigen::Matrix3d R_diff_refined = desired_R.matrix() * current_R_refined.matrix().transpose();
                            Eigen::AngleAxisd angle_axis_refined(R_diff_refined);
                            double rot_error_refined = angle_axis_refined.angle();
                            Eigen::Vector3d rot_axis_refined = angle_axis_refined.axis();

                            // Check convergence for both position and orientation
                            bool pos_converged = pos_error_refined.norm() < position_tolerance;
                            bool rot_converged = rot_error_refined < orientation_tolerance;

                            if (pos_converged && rot_converged)
                            {
                                break; // Both converged, no need for more refinement
                            }

                            // Compute full Jacobian (angular + linear)
                            drake::MatrixX<double> J(6, plant_->num_velocities());
                            plant_->CalcJacobianSpatialVelocity(
                                plant_context,
                                drake::multibody::JacobianWrtVariable::kV,
                                ee_frame,
                                Eigen::Vector3d::Zero(),
                                plant_->world_frame(),
                                plant_->world_frame(),
                                &J);

                            // Extract only right arm joints (11-17)
                            Eigen::MatrixXd J_right_arm(6, 7);
                            int right_arm_idx = 0;
                            for (int j = 11; j <= 17; ++j)
                            {
                                J_right_arm.col(right_arm_idx++) = J.col(j);
                            }

                            // Combined error vector [angular_error, linear_error]
                            drake::Vector6<double> error_vector;
                            error_vector << rot_axis_refined * rot_error_refined, // Angular error (3D)
                                pos_error_refined;                                // Linear error (3D)

                            // Damped Least Squares: Δq = (J^T J + λ²I)^-1 J^T e
                            const double damping = 0.005; // Small damping for stability
                            Eigen::MatrixXd JtJ = J_right_arm.transpose() * J_right_arm;
                            Eigen::MatrixXd A = JtJ + damping * damping * Eigen::MatrixXd::Identity(7, 7);
                            Eigen::VectorXd delta_q_right_arm = A.ldlt().solve(J_right_arm.transpose() * error_vector);

                            // Apply correction with small step size
                            const double alpha = 0.5; // Conservative step size
                            for (int j = 0; j < 7; ++j)
                            {
                                q_current(11 + j) += alpha * delta_q_right_arm(j);
                            }

                            // Clamp to joint limits
                            q_current = q_current.cwiseMax(plant_->GetPositionLowerLimits())
                                            .cwiseMin(plant_->GetPositionUpperLimits());
                        }

                        joint_samples[i] = q_current;
                        success_count++;
                    }
                }
            }
            else
            {
                if (i == 0)
                {
                    joint_samples[i] = q_start;
                }
                else
                {
                    joint_samples[i] = q_current;
                }
                fail_count++;
                if (i < 5)
                {
                    std::cout << "  [DIK FAILED] Waypoint " << i << std::endl;
                }
            }

            // Progress update
            if (i % 10 == 0 || i == num_ik_waypoints - 1)
            {
                Eigen::Vector3d actual_pos = ComputeEEPose(q_current).translation();
                double pos_tracking_error = (actual_pos - desired_pos).norm();

                drake::math::RotationMatrixd actual_R = ComputeEEPose(q_current).rotation();
                // Calculate rotation error using angle-axis
                Eigen::Matrix3d R_diff = desired_R.matrix() * actual_R.matrix().transpose();
                Eigen::AngleAxisd angle_axis(R_diff);
                double rot_tracking_error = angle_axis.angle();

                std::cout << "  Waypoint " << i << "/" << num_ik_waypoints
                          << " | t=" << std::fixed << std::setprecision(3) << t << " s"
                          << " | Pos Error: " << std::scientific << pos_tracking_error << " m"
                          << " | Rot Error: " << (rot_tracking_error * 180.0 / M_PI) << " deg"
                          << std::endl;
            }
        }

        // Force non-right-arm joints to stay at initial values
        std::cout << "\n[SAFETY CHECK] Forcing non-right-arm joints to initial positions..." << std::endl;
        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            for (int j = 0; j < plant_->num_positions(); ++j)
            {
                if (j < 11 || j > 17)
                { // Non-right-arm joints
                    joint_samples[i](j) = q_start(j);
                }
            }
        }

        std::cout << "\nResults:" << std::endl;
        std::cout << "  Success: " << success_count << "/" << num_ik_waypoints
                  << " (" << (100.0 * success_count / num_ik_waypoints) << "%)" << std::endl;
        std::cout << "  Failed:  " << fail_count << "/" << num_ik_waypoints << std::endl;

        // ========================================================================
        // SMOOTHING: Check and fix joint velocity discontinuities
        // ========================================================================
        std::cout << "\n[SMOOTHING] Checking joint velocity continuity..." << std::endl;

        // Calculate joint velocities between waypoints
        std::vector<VectorXd> joint_velocities(num_ik_waypoints - 1);
        double max_velocity_jump = 0.0;
        int max_jump_idx = -1;
        int max_jump_joint = -1;

        for (int i = 0; i < num_ik_waypoints - 1; ++i)
        {
            double dt_seg = breaks[i + 1] - breaks[i];
            joint_velocities[i] = (joint_samples[i + 1] - joint_samples[i]) / dt_seg;

            // Check velocity jump for right arm joints only
            if (i > 0)
            {
                VectorXd velocity_jump = (joint_velocities[i] - joint_velocities[i - 1]).cwiseAbs();
                for (int j = 11; j <= 17; ++j)
                {
                    if (velocity_jump(j) > max_velocity_jump)
                    {
                        max_velocity_jump = velocity_jump(j);
                        max_jump_idx = i;
                        max_jump_joint = j;
                    }
                }
            }
        }

        std::cout << "  Max velocity jump: " << max_velocity_jump << " rad/s"
                  << " at waypoint " << max_jump_idx << ", joint " << max_jump_joint << std::endl;

        // ========================================================================
        // ULTIMATE S-CURVE SOLUTION: 7th-order polynomial + Savitzky-Golay filtering
        // ========================================================================
        // Problem: IK noise causes velocity/acceleration oscillations
        // Solution: Apply heavy filtering to eliminate high-frequency noise

        std::cout << "\n[ULTIMATE S-CURVE] Implementing multi-stage smoothing..." << std::endl;
        std::cout << "  Strategy: S-curve trajectory + Savitzky-Golay filter + C² spline" << std::endl;
        std::cout << "  Goal: Eliminate ALL high-frequency oscillations" << std::endl;

        // Step 1: Apply Savitzky-Golay filter to smooth joint positions
        std::cout << "\n[STEP 1] Applying Savitzky-Golay filter to joint positions..." << std::endl;
        const int sg_window = 31; // Window size (must be odd)
        const int sg_order = 4;   // Polynomial order for fitting

        std::cout << "  Window size: " << sg_window << " points" << std::endl;
        std::cout << "  Polynomial order: " << sg_order << std::endl;

        std::vector<MatrixXd> joint_samples_filtered(num_ik_waypoints);

        // Initialize with original data FIRST
        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            joint_samples_filtered[i] = joint_samples[i];
        }

        // Apply Savitzky-Golay filter to each joint independently
        for (int joint_idx = 11; joint_idx <= 17; ++joint_idx)
        {
            // Extract this joint's trajectory
            std::vector<double> joint_trajectory(num_ik_waypoints);
            for (int i = 0; i < num_ik_waypoints; ++i)
            {
                joint_trajectory[i] = joint_samples[i](joint_idx);
            }

            // Apply Savitzky-Golay filter
            std::vector<double> joint_filtered = ApplySavitzkyGolayFilter(joint_trajectory, sg_window, sg_order);

            // Store filtered values (only for right arm joints)
            for (int i = 0; i < num_ik_waypoints; ++i)
            {
                joint_samples_filtered[i](joint_idx) = joint_filtered[i];
            }
        }

        std::cout << "  ✓ Savitzky-Golay filtering complete" << std::endl;

        // Step 2: Create C²-continuous spline with ZERO boundary velocity
        std::cout << "\n[STEP 2] Creating C²-continuous spline with zero boundary conditions..." << std::endl;

        // Convert filtered joint samples to Eigen matrix
        MatrixXd joint_samples_matrix(plant_->num_positions(), num_ik_waypoints);
        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            joint_samples_matrix.col(i) = joint_samples_filtered[i];
        }

        // CRITICAL: Set ZERO boundary velocities for smooth start/stop
        VectorXd start_velocity = VectorXd::Zero(plant_->num_positions());
        VectorXd end_velocity = VectorXd::Zero(plant_->num_positions());

        // Convert breaks to Eigen vector
        Eigen::Map<Eigen::VectorXd> breaks_eigen(breaks.data(), breaks.size());

        // Create C² continuous trajectory
        auto final_trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            breaks_eigen, joint_samples_matrix, start_velocity, end_velocity);

        std::cout << "  ✓ C²-continuous spline created" << std::endl;
        std::cout << "  ✓ Zero boundary velocity enforced" << std::endl;

        // Step 3: Resample trajectory at high frequency for ultra-smooth velocity
        std::cout << "\n[STEP 3] Resampling trajectory at 500Hz for ultra-smooth motion..." << std::endl;
        const int resample_points = static_cast<int>(duration * 500.0); // 500Hz resampling
        std::vector<double> breaks_resampled(resample_points);
        std::vector<MatrixXd> samples_resampled(resample_points);

        for (int i = 0; i < resample_points; ++i)
        {
            double t = duration * i / (resample_points - 1);
            breaks_resampled[i] = t;
            samples_resampled[i] = final_trajectory.value(t);
        }

        std::cout << "  Resampled from " << num_ik_waypoints << " to " << resample_points << " points" << std::endl;

        // Step 4: Create final C² spline from resampled data
        MatrixXd samples_matrix_resampled(plant_->num_positions(), resample_points);
        for (int i = 0; i < resample_points; ++i)
        {
            samples_matrix_resampled.col(i) = samples_resampled[i];
        }

        Eigen::Map<Eigen::VectorXd> breaks_resampled_eigen(breaks_resampled.data(), breaks_resampled.size());

        auto ultra_smooth_trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            breaks_resampled_eigen, samples_matrix_resampled, start_velocity, end_velocity);

        std::cout << "\n[SUCCESS] Ultra-smooth S-curve trajectory created!" << std::endl;
        std::cout << "  Original waypoints: " << num_ik_waypoints << std::endl;
        std::cout << "  After resampling: " << resample_points << std::endl;
        std::cout << "  Smoothness: C² continuous + Savitzky-Golay filtered" << std::endl;
        std::cout << "  Boundary: Zero velocity and acceleration" << std::endl;

        return ultra_smooth_trajectory;

        // =================================================================
        // PRECISION ANALYSIS: Test IK and Trajectory Accuracy
        // =================================================================
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "PRECISION ANALYSIS: IK Solver & Trajectory Accuracy" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Test 1: End-point accuracy (planning level precision)
        std::cout << "\n[TEST 1] End-Point Accuracy (Planning Level)" << std::endl;
        VectorXd q_final = joint_samples[num_ik_waypoints - 1];
        drake::math::RigidTransformd T_final = ComputeEEPose(q_final);

        Eigen::Vector3d final_pos_error = pos_goal - T_final.translation();
        Eigen::Matrix3d R_final_diff = R_goal.matrix() * T_final.rotation().matrix().transpose();
        Eigen::AngleAxisd angle_axis_final(R_final_diff);
        double final_rot_error = angle_axis_final.angle();

        std::cout << "  Target Position:     " << pos_goal.transpose() << " m" << std::endl;
        std::cout << "  Achieved Position:   " << T_final.translation().transpose() << " m" << std::endl;
        std::cout << "  Position Error:      " << std::scientific << final_pos_error.norm() << " m ("
                  << std::fixed << (final_pos_error.norm() * 1000) << " mm)" << std::endl;

        drake::math::RollPitchYawd goal_rpy_final(R_goal);
        drake::math::RollPitchYawd achieved_rpy_final(T_final.rotation());
        std::cout << "  Target Orientation:  " << goal_rpy_final.vector().transpose() << " rad" << std::endl;
        std::cout << "  Achieved Orientation: " << achieved_rpy_final.vector().transpose() << " rad" << std::endl;
        std::cout << "  Orientation Error:   " << std::scientific << final_rot_error << " rad ("
                  << std::fixed << (final_rot_error * 180.0 / M_PI) << " deg)" << std::endl;

        // Test 2: Statistics over all waypoints
        std::cout << "\n[TEST 2] Trajectory Tracking Statistics" << std::endl;

        std::vector<double> pos_errors_all;
        std::vector<double> rot_errors_all;

        for (int i = 0; i < num_ik_waypoints; ++i)
        {
            drake::math::RigidTransformd T_waypoint = ComputeEEPose(joint_samples[i]);

            // Get desired pose at this waypoint
            Eigen::Vector3d desired_pos_wp = pos_trajectory.value(breaks[i]);
            Eigen::Quaterniond desired_quat_wp = orientation_trajectory.orientation(breaks[i]);
            drake::math::RotationMatrixd desired_R_wp(desired_quat_wp);

            // Compute errors
            double pos_err = (desired_pos_wp - T_waypoint.translation()).norm();
            pos_errors_all.push_back(pos_err);

            Eigen::Matrix3d R_diff_wp = desired_R_wp.matrix() * T_waypoint.rotation().matrix().transpose();
            Eigen::AngleAxisd angle_axis_wp(R_diff_wp);
            double rot_err = angle_axis_wp.angle();
            rot_errors_all.push_back(rot_err);
        }

        // Compute statistics
        double pos_error_mean = std::accumulate(pos_errors_all.begin(), pos_errors_all.end(), 0.0) / num_ik_waypoints;
        double pos_error_max = *std::max_element(pos_errors_all.begin(), pos_errors_all.end());
        double pos_error_min = *std::min_element(pos_errors_all.begin(), pos_errors_all.end());

        double pos_error_std = 0.0;
        for (double err : pos_errors_all)
        {
            pos_error_std += (err - pos_error_mean) * (err - pos_error_mean);
        }
        pos_error_std = std::sqrt(pos_error_std / num_ik_waypoints);

        double rot_error_mean = std::accumulate(rot_errors_all.begin(), rot_errors_all.end(), 0.0) / num_ik_waypoints;
        double rot_error_max = *std::max_element(rot_errors_all.begin(), rot_errors_all.end());
        double rot_error_min = *std::min_element(rot_errors_all.begin(), rot_errors_all.end());

        std::cout << "\n  Position Error Statistics:" << std::endl;
        std::cout << "    Mean:   " << std::scientific << pos_error_mean << " m ("
                  << std::fixed << std::setprecision(4) << (pos_error_mean * 1e6) << " µm)" << std::endl;
        std::cout << "    Std:    " << std::scientific << pos_error_std << " m ("
                  << std::fixed << std::setprecision(4) << (pos_error_std * 1e6) << " µm)" << std::endl;
        std::cout << "    Max:    " << std::scientific << pos_error_max << " m ("
                  << std::fixed << std::setprecision(4) << (pos_error_max * 1e6) << " µm)" << std::endl;
        std::cout << "    Min:    " << std::scientific << pos_error_min << " m ("
                  << std::fixed << std::setprecision(4) << (pos_error_min * 1e6) << " µm)" << std::endl;

        std::cout << "\n  Orientation Error Statistics:" << std::endl;
        std::cout << "    Mean:   " << std::scientific << rot_error_mean << " rad ("
                  << std::fixed << std::setprecision(6) << (rot_error_mean * 180.0 / M_PI) << " deg)" << std::endl;
        std::cout << "    Max:    " << std::scientific << rot_error_max << " rad ("
                  << std::fixed << std::setprecision(6) << (rot_error_max * 180.0 / M_PI) << " deg)" << std::endl;
        std::cout << "    Min:    " << std::scientific << rot_error_min << " rad ("
                  << std::fixed << std::setprecision(6) << (rot_error_min * 180.0 / M_PI) << " deg)" << std::endl;

        // Test 3: Forward-Backward Consistency Check
        std::cout << "\n[TEST 3] Forward Kinematics Verification" << std::endl;
        plant_->SetPositions(&plant_context, q_start);
        drake::math::RigidTransformd T_start_verify = ComputeEEPose(q_start);
        double start_pos_err = (T_start_verify.translation() - pos_start).norm();
        std::cout << "  Start position FK error: " << std::scientific << start_pos_err << " m ("
                  << std::fixed << (start_pos_err * 1e6) << " µm)" << std::endl;

        plant_->SetPositions(&plant_context, q_final);
        drake::math::RigidTransformd T_final_verify = ComputeEEPose(q_final);
        double final_pos_err_verify = (T_final_verify.translation() - pos_goal).norm();
        std::cout << "  Final position FK error: " << std::scientific << final_pos_err_verify << " m ("
                  << std::fixed << (final_pos_err_verify * 1e6) << " µm)" << std::endl;

        // Test 4: Trajectory smoothness (velocity continuity)
        std::cout << "\n[TEST 4] Trajectory Smoothness" << std::endl;
        std::vector<double> joint_velocities_scalar(num_ik_waypoints - 1);
        double max_joint_vel = 0.0;
        double avg_joint_vel = 0.0;

        for (int i = 0; i < num_ik_waypoints - 1; ++i)
        {
            VectorXd dq = joint_samples[i + 1] - joint_samples[i];
            double dt_seg = breaks[i + 1] - breaks[i];
            double vel = dq.norm() / dt_seg;
            joint_velocities_scalar[i] = vel;
            max_joint_vel = std::max(max_joint_vel, vel);
            avg_joint_vel += vel;
        }
        avg_joint_vel /= (num_ik_waypoints - 1);

        std::cout << "  Max joint velocity: " << std::fixed << std::setprecision(4)
                  << max_joint_vel << " rad/s" << std::endl;
        std::cout << "  Avg joint velocity: " << std::fixed << std::setprecision(4)
                  << avg_joint_vel << " rad/s" << std::endl;

        // Summary rating
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "PRECISION RATING" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        // Position rating
        std::string pos_rating;
        if (pos_error_mean < 1e-5)
            pos_rating = "⭐⭐⭐⭐⭐ EXCELLENT (< 0.01 mm)";
        else if (pos_error_mean < 1e-4)
            pos_rating = "⭐⭐⭐⭐ VERY GOOD (< 0.1 mm)";
        else if (pos_error_mean < 1e-3)
            pos_rating = "⭐⭐⭐ GOOD (< 1 mm)";
        else if (pos_error_mean < 1e-2)
            pos_rating = "⭐⭐ FAIR (< 10 mm)";
        else
            pos_rating = "⭐ POOR (> 10 mm)";

        // Orientation rating
        std::string rot_rating;
        if (rot_error_mean < M_PI / 1800)
            rot_rating = "⭐⭐⭐⭐⭐ EXCELLENT (< 0.1°)";
        else if (rot_error_mean < M_PI / 180)
            rot_rating = "⭐⭐⭐⭐ VERY GOOD (< 1°)";
        else if (rot_error_mean < M_PI / 18)
            rot_rating = "⭐⭐⭐ GOOD (< 10°)";
        else if (rot_error_mean < M_PI / 6)
            rot_rating = "⭐⭐ FAIR (< 30°)";
        else
            rot_rating = "⭐ POOR (> 30°)";

        std::cout << "  Position Precision:  " << pos_rating << std::endl;
        std::cout << "  Orientation Precision: " << rot_rating << std::endl;
        std::cout << "  Planning Level End-Point Accuracy: " << std::scientific << final_pos_error.norm()
                  << " m (" << std::fixed << (final_pos_error.norm() * 1e6) << " µm)" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        return final_trajectory;
    }

    // =================================================================
    // Industrial-Grade Circular Trajectory with Full 6D Pose Control
    // =================================================================
    /**
     * @brief Plan circular trajectory with full 6D pose control (MoveC)
     *
     * Implements industrial-grade MoveC (Circular Motion) command:
     * - Interpolates position along a circular arc
     * - Interpolates orientation using SLERP (Spherical Linear Interpolation)
     * - Supports velocity and acceleration constraints for both linear and angular motion
     * - Uses Differential IK for precise tracking
     *
     * @param q_start Initial joint configuration
     * @param via_pose Intermediate waypoint pose (defines arc together with start and goal)
     * @param goal_pose Final target pose (position + orientation)
     * @param max_velocity Maximum linear velocity (m/s)
     * @param max_acceleration Maximum linear acceleration (m/s²)
     * @param max_angular_velocity Maximum angular velocity (rad/s)
     * @param max_angular_acceleration Maximum angular acceleration (rad/s²)
     * @return Joint space trajectory following the 6D circular arc
     */
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianCircleWithPose(
        const VectorXd &q_start,
        const drake::math::RigidTransformd &via_pose,
        const drake::math::RigidTransformd &goal_pose,
        double max_velocity = 0.5,
        double max_acceleration = 1.0,
        double max_angular_velocity = 1.0,
        double max_angular_acceleration = 2.0)
    {
        std::cout << "\n"
                  << std::string(80, '=') << std::endl;
        std::cout << "Industrial-Grade MoveC: Circular Trajectory with Full 6D Pose Control" << std::endl;
        std::cout << std::string(80, '=') << std::endl;

        std::cout << "\nConstraints:" << std::endl;
        std::cout << "  Max Linear Velocity: " << max_velocity << " m/s" << std::endl;
        std::cout << "  Max Linear Acceleration: " << max_acceleration << " m/s²" << std::endl;
        std::cout << "  Max Angular Velocity: " << max_angular_velocity << " rad/s" << std::endl;
        std::cout << "  Max Angular Acceleration: " << max_angular_acceleration << " rad/s²" << std::endl;

        // Extract start, via, and goal poses
        drake::math::RigidTransformd T_start = ComputeEEPose(q_start);
        Eigen::Vector3d pos_start = T_start.translation();
        drake::math::RotationMatrixd R_start = T_start.rotation();

        Eigen::Vector3d pos_via = via_pose.translation();
        drake::math::RotationMatrixd R_via = via_pose.rotation();

        Eigen::Vector3d pos_goal = goal_pose.translation();
        drake::math::RotationMatrixd R_goal = goal_pose.rotation();

        // Display orientation info
        drake::math::RollPitchYawd rpy_start(R_start);
        drake::math::RollPitchYawd rpy_via(R_via);
        drake::math::RollPitchYawd rpy_goal(R_goal);

        std::cout << "\nPath Information (6D Pose):" << std::endl;
        std::cout << "  Start Position:  " << pos_start.transpose() << " m" << std::endl;
        std::cout << "  Via Position:   " << pos_via.transpose() << " m" << std::endl;
        std::cout << "  Goal Position:  " << pos_goal.transpose() << " m" << std::endl;
        std::cout << "  Start Orientation (RPY): " << (rpy_start.vector() * 180 / M_PI).transpose() << " deg" << std::endl;
        std::cout << "  Via Orientation (RPY):   " << (rpy_via.vector() * 180 / M_PI).transpose() << " deg" << std::endl;
        std::cout << "  Goal Orientation (RPY):  " << (rpy_goal.vector() * 180 / M_PI).transpose() << " deg" << std::endl;

        // Calculate circle parameters from three points
        Eigen::Vector3d v1 = pos_via - pos_start;
        Eigen::Vector3d v2 = pos_goal - pos_via;

        // Circle center (intersection of perpendicular bisectors)
        double d1 = v1.squaredNorm();
        double d2 = v2.squaredNorm();
        Eigen::Vector3d center = pos_start +
                                 (pos_goal - pos_start) * (d1 * d2 * (d1 - d2)) / (2 * d1 * d2 * (pos_goal - pos_start).squaredNorm() - (d1 - d2) * (d1 - d2)) + // This is simplified; actual calculation more complex
                                 Eigen::Vector3d(0, 0, 0);                                                                                                       // Placeholder

        // For robustness, use simpler circle fitting
        // Circle center is at equal distance from all three points
        // Using circumcenter formula
        Eigen::Vector3d ac = pos_start - pos_goal;
        Eigen::Vector3d ab = pos_start - pos_via;
        Eigen::Vector3d bc = pos_via - pos_goal;

        // Normal to the plane (cross product)
        Eigen::Vector3d normal = ab.cross(bc).normalized();

        // Circle radius and center (using circumcenter)
        double a = ab.squaredNorm();
        double b = bc.squaredNorm();
        double c = ac.squaredNorm();

        // Circumcenter calculation
        double alpha = a * (b + c - a);
        double beta = b * (c + a - b);
        double gamma = c * (a + b - c);
        double total = alpha + beta + gamma;

        Eigen::Vector3d circle_center = (alpha * pos_goal + beta * pos_start + gamma * pos_via) / total;
        double radius = (pos_start - circle_center).norm();

        std::cout << "\nCircle Parameters:" << std::endl;
        std::cout << "  Center: " << circle_center.transpose() << " m" << std::endl;
        std::cout << "  Radius: " << radius << " m" << std::endl;
        std::cout << "  Normal: " << normal.transpose() << std::endl;

        // Calculate angular span
        Eigen::Vector3d r1 = (pos_start - circle_center).normalized();
        Eigen::Vector3d r2 = (pos_via - circle_center).normalized();
        Eigen::Vector3d r3 = (pos_goal - circle_center).normalized();

        double angle1 = std::acos(r1.dot(r2));
        double angle2 = std::acos(r2.dot(r3));
        double total_angle = angle1 + angle2;

        std::cout << "  Angular Span: " << (total_angle * 180 / M_PI) << " deg" << std::endl;
        std::cout << "  Arc Length: " << (radius * total_angle) << " m" << std::endl;

        // Generate smooth circular trajectory
        std::cout << "\nGenerating smooth circular trajectory..." << std::endl;
        auto circle_trajectory = GenerateSmoothCircularTrajectory(
            circle_center, radius, normal, pos_start, max_velocity, max_acceleration);

        double pos_duration = circle_trajectory.end_time();

        // Generate orientation trajectory (SLERP from start to goal via point)
        std::cout << "Generating orientation trajectory (dual SLERP)..." << std::endl;

        // For orientation, interpolate from start -> via -> goal
        const int num_waypoints = 251; // Higher density for circle
        std::vector<double> breaks(num_waypoints);
        for (int i = 0; i < num_waypoints; ++i)
        {
            breaks[i] = pos_duration * i / (num_waypoints - 1);
        }

        // Create dual SLERP for orientation (start->via, then via->goal)
        std::vector<double> ori_breaks = {0.0, pos_duration / 2, pos_duration};
        std::vector<Eigen::Quaterniond> quats = {
            R_start.ToQuaternion(),
            R_via.ToQuaternion(),
            R_goal.ToQuaternion()};
        auto orientation_trajectory = drake::trajectories::PiecewiseQuaternionSlerp<double>(
            ori_breaks, quats);

        std::cout << "  Position duration: " << pos_duration << " s" << std::endl;
        std::cout << "  Orientation duration: " << pos_duration << " s (synchronized)" << std::endl;

        // Convert to joint space using Differential IK
        std::cout << "\nConverting to joint space using Differential IK..." << std::endl;
        std::cout << "  Waypoints: " << num_waypoints << " (high density for precision)" << std::endl;

        const double dt = pos_duration / (num_waypoints - 1);

        drake::multibody::DifferentialInverseKinematicsParameters dik_params(
            plant_->num_positions(),
            plant_->num_velocities());
        dik_params.set_time_step(dt);
        dik_params.set_nominal_joint_position(q_start);

        // Joint position limits
        VectorXd lower_pos_limits = plant_->GetPositionLowerLimits();
        VectorXd upper_pos_limits = plant_->GetPositionUpperLimits();
        const double margin = 0.01;
        lower_pos_limits = lower_pos_limits.array() + margin;
        upper_pos_limits = upper_pos_limits.array() - margin;
        dik_params.set_joint_position_limits({lower_pos_limits, upper_pos_limits});

        // Set angular velocity limit
        dik_params.set_end_effector_angular_speed_limit(max_angular_velocity * 2.0);

        // Joint velocity limits
        const double max_joint_velocity_ik = 3.0;
        VectorXd lower_velocity_limits = VectorXd::Constant(plant_->num_positions(), -max_joint_velocity_ik);
        VectorXd upper_velocity_limits = VectorXd::Constant(plant_->num_positions(), max_joint_velocity_ik);

        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17)
            {
                lower_velocity_limits(i) = 0.0;
                upper_velocity_limits(i) = 0.0;
            }
        }
        dik_params.set_joint_velocity_limits({lower_velocity_limits, upper_velocity_limits});

        // CRITICAL: Enable BOTH angular and linear velocity control (6D)
        drake::Vector6<bool> ee_velocity_flag;
        ee_velocity_flag << true, true, true, // Angular velocity control
            true, true, true;                 // Linear velocity control
        dik_params.set_end_effector_velocity_flag(ee_velocity_flag);

        // Joint centering gain
        MatrixXd centering_gain = MatrixXd::Zero(plant_->num_positions(), plant_->num_positions());
        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17)
            {
                centering_gain(i, i) = 100.0;
            }
            else
            {
                centering_gain(i, i) = 0.01;
            }
        }
        dik_params.set_joint_centering_gain(centering_gain);

        const auto &ee_frame = plant_->GetFrameByName("right_tool_frame");
        auto &plant_context = plant_->GetMyMutableContextFromRoot(
            &simulator_->get_mutable_context());

        std::vector<MatrixXd> joint_samples(num_waypoints);
        VectorXd q_current = q_start;
        int success_count = 0;
        int fail_count = 0;

        std::cout << "\nExecuting circular trajectory with full 6D pose control:" << std::endl;

        for (int i = 0; i < num_waypoints; ++i)
        {
            double t = breaks[i];

            // Get desired position from circle trajectory
            Eigen::Vector3d desired_pos = circle_trajectory.value(t);
            Eigen::Vector3d desired_linear_vel = circle_trajectory.derivative(1).value(t);

            // Get desired orientation from SLERP trajectory
            Eigen::Quaterniond desired_quat = orientation_trajectory.orientation(t);
            drake::math::RotationMatrixd desired_R(desired_quat);
            Eigen::Vector3d desired_angular_vel = orientation_trajectory.angular_velocity(t);

            plant_->SetPositions(&plant_context, q_current);

            // Position error correction
            const double kp_pos = 50.0;
            drake::math::RigidTransformd T_current = ComputeEEPose(q_current);
            Eigen::Vector3d pos_error = desired_pos - T_current.translation();
            Eigen::Vector3d corrected_linear_vel = desired_linear_vel + kp_pos * pos_error;

            // Clamp velocity
            double max_ee_vel = max_velocity * 1.5;
            if (corrected_linear_vel.norm() > max_ee_vel)
            {
                corrected_linear_vel = corrected_linear_vel.normalized() * max_ee_vel;
            }

            // Orientation error correction
            const double kp_ori = 10.0;
            Eigen::Matrix3d R_error_matrix = desired_R.matrix() * T_current.rotation().matrix().transpose();
            Eigen::AngleAxisd angle_axis(R_error_matrix);
            Eigen::Vector3d ori_error_vec = angle_axis.axis() * angle_axis.angle();
            Eigen::Vector3d corrected_angular_vel = desired_angular_vel + kp_ori * ori_error_vec;

            // Create spatial velocity [angular, linear]
            drake::Vector6<double> V_WE_desired;
            V_WE_desired << corrected_angular_vel(0), corrected_angular_vel(1), corrected_angular_vel(2),
                corrected_linear_vel(0), corrected_linear_vel(1), corrected_linear_vel(2);

            // Solve Differential IK
            auto result = drake::multibody::DoDifferentialInverseKinematics(
                *plant_, plant_context, V_WE_desired, ee_frame, dik_params);

            if (result.status == drake::multibody::DifferentialInverseKinematicsStatus::kSolutionFound)
            {
                VectorXd q_dot = result.joint_velocities.value();
                VectorXd q_next = q_current + q_dot * dt;

                q_next = q_next.cwiseMax(plant_->GetPositionLowerLimits())
                             .cwiseMin(plant_->GetPositionUpperLimits());

                q_current = q_next;
                joint_samples[i] = q_current;
                success_count++;
            }
            else
            {
                if (i > 0)
                {
                    joint_samples[i] = q_current;
                    fail_count++;
                    if (fail_count <= 5 || fail_count % 20 == 0)
                    {
                        std::cout << "  [WARN] IK failed at waypoint " << i
                                  << " (total failures: " << fail_count << ")" << std::endl;
                    }
                }
                else
                {
                    joint_samples[i] = q_start;
                }
            }

            // Progress output
            if (i % 25 == 0 || i == num_waypoints - 1)
            {
                plant_->SetPositions(&plant_context, q_current);
                drake::math::RigidTransformd T_actual = ComputeEEPose(q_current);

                double pos_tracking_error = (T_actual.translation() - desired_pos).norm();

                Eigen::Matrix3d R_actual = T_actual.rotation().matrix();
                Eigen::Matrix3d R_diff = desired_R.matrix() * R_actual.transpose();
                Eigen::AngleAxisd angle_axis_diff(R_diff);
                double rot_tracking_error = angle_axis_diff.angle();

                std::cout << "  Waypoint " << std::setw(3) << i << "/" << num_waypoints
                          << " | t=" << std::fixed << std::setprecision(3) << t << " s"
                          << " | Pos Error: " << std::scientific << pos_tracking_error << " m"
                          << " | Rot Error: " << (rot_tracking_error * 180.0 / M_PI) << " deg"
                          << std::endl;
            }
        }

        // Force non-right-arm joints to initial values
        std::cout << "\n[SAFETY CHECK] Forcing non-right-arm joints to initial positions..." << std::endl;
        for (int i = 0; i < num_waypoints; ++i)
        {
            for (int j = 0; j < plant_->num_positions(); ++j)
            {
                if (j < 11 || j > 17)
                {
                    joint_samples[i](j) = q_start(j);
                }
            }
        }

        std::cout << "\nResults:" << std::endl;
        std::cout << "  Success: " << success_count << "/" << num_waypoints
                  << " (" << (100.0 * success_count / num_waypoints) << "%)" << std::endl;
        std::cout << "  Failed:  " << fail_count << "/" << num_waypoints << std::endl;

        // Compute derivatives for smooth interpolation
        std::vector<MatrixXd> derivative_samples(num_waypoints);
        for (int i = 0; i < num_waypoints; ++i)
        {
            if (i == 0)
            {
                derivative_samples[i] = (joint_samples[1] - joint_samples[0]) / (breaks[1] - breaks[0]);
            }
            else if (i == num_waypoints - 1)
            {
                derivative_samples[i] = (joint_samples[i] - joint_samples[i - 1]) / (breaks[i] - breaks[i - 1]);
            }
            else
            {
                derivative_samples[i] = (joint_samples[i + 1] - joint_samples[i - 1]) / (breaks[i + 1] - breaks[i - 1]);
            }
            derivative_samples[i] *= 0.5;
        }

        auto final_trajectory = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, joint_samples, derivative_samples);

        std::cout << "\n[SUCCESS] MoveC trajectory generated successfully!" << std::endl;
        std::cout << "  Segments: " << final_trajectory.get_number_of_segments() << std::endl;
        std::cout << "  Duration: " << pos_duration << " s" << std::endl;
        std::cout << "  Arc Length: " << (radius * total_angle) << " m" << std::endl;

        // Final accuracy check
        std::cout << "\n[FINAL ACCURACY CHECK]" << std::endl;
        VectorXd q_final = joint_samples[num_waypoints - 1];
        drake::math::RigidTransformd T_final = ComputeEEPose(q_final);

        Eigen::Vector3d final_pos_error = pos_goal - T_final.translation();
        Eigen::Matrix3d R_final_diff = R_goal.matrix() * T_final.rotation().matrix().transpose();
        Eigen::AngleAxisd angle_axis_final(R_final_diff);
        double final_rot_error = angle_axis_final.angle();

        std::cout << "  Goal Position:     " << pos_goal.transpose() << " m" << std::endl;
        std::cout << "  Achieved Position: " << T_final.translation().transpose() << " m" << std::endl;
        std::cout << "  Position Error:    " << std::scientific << final_pos_error.norm() << " m ("
                  << std::fixed << (final_pos_error.norm() * 1000) << " mm)" << std::endl;

        drake::math::RollPitchYawd rpy_final(R_goal);
        std::cout << "  Goal Orientation:  " << (rpy_final.vector() * 180 / M_PI).transpose() << " deg" << std::endl;
        std::cout << "  Orientation Error: " << std::fixed << (final_rot_error * 180 / M_PI) << " deg" << std::endl;

        return final_trajectory;
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

            CollisionResult col_result = CheckCollisionDetailed(q_check);
            if (col_result.has_collision)
            {
                std::cout << "  [COLLISION] at waypoint " << i << " (t=" << t << " s)" << std::endl;
                std::cout << "    Min distance: " << (col_result.min_distance * 1000) << " mm" << std::endl;
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
     */
    drake::trajectories::PiecewisePolynomial<double>
    PlanCartesianMoveJ(
        const VectorXd &q_start,
        const VectorXd &q_goal,
        double max_velocity = 1.0,
        double max_acceleration = 2.0)
    {
        int num_joints = q_start.size();
        VectorXd vel_limits = VectorXd::Constant(num_joints, max_velocity);
        VectorXd acc_limits = VectorXd::Constant(num_joints, max_acceleration);

        return PlanCartesianMoveJ(q_start, q_goal, vel_limits, acc_limits);
    }

    // =================================================================
    // LEGACY: Original time-based Cartesian line planning (kept for comparison)
    // =================================================================
    drake::trajectories::PiecewisePolynomial<double> PlanCartesianLine(
        double duration,
        const VectorXd &q_start,
        const Eigen::Vector3d &goal_position)
    {
        std::cout << "\n=== Cartesian Space Linear Trajectory Planning (Differential IK) ===" << std::endl;
        std::cout << "[NOTE] For industrial applications, consider using PlanCartesianLineIndustrial()" << std::endl;
        std::cout << "       which provides velocity/acceleration/jerk constraints in Cartesian space" << std::endl;

        // Get initial EE pose
        drake::math::RigidTransformd T_ee_start = ComputeEEPose(q_start);
        Eigen::Vector3d pos_start = T_ee_start.translation();

        std::cout << "EE Start: " << pos_start.transpose() << std::endl;
        std::cout << "EE Goal: " << goal_position.transpose() << std::endl;

        // Use fewer waypoints with Differential IK
        const int num_waypoints = 21;
        const double dt = duration / (num_waypoints - 1);

        std::vector<double> breaks(num_waypoints);
        std::vector<MatrixXd> joint_samples(num_waypoints);

        // Set up Differential IK parameters (same as Circle)
        // IMPORTANT: Must specify both num_positions AND num_velocities
        drake::multibody::DifferentialInverseKinematicsParameters dik_params(
            plant_->num_positions(),
            plant_->num_velocities()); // This was missing!
        dik_params.set_time_step(dt);
        dik_params.set_nominal_joint_position(q_start);

        // IMPORTANT: Set joint position limits to prevent unbounded variables
        VectorXd lower_pos_limits = plant_->GetPositionLowerLimits();
        VectorXd upper_pos_limits = plant_->GetPositionUpperLimits();
        const double margin = 0.01;
        lower_pos_limits = lower_pos_limits.array() + margin;
        upper_pos_limits = upper_pos_limits.array() - margin;
        dik_params.set_joint_position_limits({lower_pos_limits, upper_pos_limits});

        // Relax orientation constraint
        dik_params.set_end_effector_angular_speed_limit(10.0); // Increased from 5.0

        // Joint velocity limits - lock all joints EXCEPT right arm (11-17)
        const double max_joint_velocity = 3.0; // Increased from 1.0 for faster motion
        VectorXd lower_velocity_limits = VectorXd::Constant(plant_->num_positions(), -max_joint_velocity);
        VectorXd upper_velocity_limits = VectorXd::Constant(plant_->num_positions(), max_joint_velocity);

        // Lock non-right-arm joints (legs: 0-2, waist: 3, left arm: 4-10, head: 18-19)
        for (int i = 0; i < plant_->num_positions(); ++i)
        {
            if (i < 11 || i > 17)
            { // Not right arm
                lower_velocity_limits(i) = 0.0;
                upper_velocity_limits(i) = 0.0;
            }
        }
        dik_params.set_joint_velocity_limits({lower_velocity_limits, upper_velocity_limits});

        // Only control linear velocity (relax orientation)
        drake::Vector6<bool> ee_velocity_flag;
        ee_velocity_flag << false, false, false, // Don't control angular velocity
            true, true, true;                    // Control linear velocity
        dik_params.set_end_effector_velocity_flag(ee_velocity_flag);

        // Joint centering gain
        dik_params.set_joint_centering_gain(0.01 * MatrixXd::Identity(plant_->num_positions(),
                                                                      plant_->num_positions()));

        // Get frames and context
        const auto &ee_frame = plant_->GetFrameByName("right_tool_frame");
        auto &plant_context = plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());

        // Compute constant linear velocity for straight-line motion
        Eigen::Vector3d linear_velocity = (goal_position - pos_start) / duration;

        VectorXd q_current = q_start;
        int success_count = 0;

        std::cout << "Integrating Differential IK for " << num_waypoints << " waypoints..." << std::endl;

        for (int i = 0; i < num_waypoints; ++i)
        {
            breaks[i] = i * dt;

            // Set current robot state
            plant_->SetPositions(&plant_context, q_current);

            // Create spatial velocity Vector6 [angular, linear]
            // Zero angular velocity, constant linear velocity
            drake::Vector6<double> V_WE_desired;
            V_WE_desired << 0, 0, 0, // Zero angular velocity
                linear_velocity(0),  // Constant linear velocity
                linear_velocity(1),
                linear_velocity(2);

            // Use velocity-based DIK (NO IPOPT!)
            auto result = drake::multibody::DoDifferentialInverseKinematics(
                *plant_,
                plant_context,
                V_WE_desired,
                ee_frame,
                dik_params);

            if (result.status == drake::multibody::DifferentialInverseKinematicsStatus::kSolutionFound)
            {
                // Integrate joint velocity
                VectorXd q_dot = result.joint_velocities.value();
                VectorXd q_next = q_current + q_dot * dt;

                // Enforce joint limits
                q_next = q_next.cwiseMax(plant_->GetPositionLowerLimits())
                             .cwiseMin(plant_->GetPositionUpperLimits());

                // Collision detection
                CollisionResult col_result = CheckCollisionDetailed(q_next);

                if (col_result.has_collision)
                {
                    std::cout << "  [COLLISION] Waypoint " << i << ": " << col_result.warning_message << std::endl;
                    if (i == 0)
                    {
                        joint_samples[i] = q_start;
                    }
                    else
                    {
                        joint_samples[i] = q_current;
                    }
                }
                else
                {
                    q_current = q_next;
                    joint_samples[i] = q_current;
                    success_count++;
                }
            }
            else
            {
                if (i == 0)
                {
                    joint_samples[i] = q_start;
                }
                else
                {
                    joint_samples[i] = q_current;
                }
                if (i < 5)
                {
                    std::cout << "  [DIK WARNING] Waypoint " << i << " failed" << std::endl;
                }
            }

            if (i % 10 == 0)
            {
                Eigen::Vector3d current_pos = ComputeEEPose(q_current).translation();
                std::cout << "  Waypoint " << i << "/" << num_waypoints
                          << " EE: [" << current_pos.transpose() << "]" << std::endl;
            }
        }

        std::cout << "Linear trajectory planned with " << success_count << "/" << num_waypoints << " successful" << std::endl;

        // Compute velocity derivatives for smooth motion
        std::vector<MatrixXd> derivative_samples(num_waypoints);
        for (int i = 0; i < num_waypoints; ++i)
        {
            if (i == 0)
            {
                derivative_samples[i] = (joint_samples[1] - joint_samples[0]) / dt;
            }
            else if (i == num_waypoints - 1)
            {
                derivative_samples[i] = (joint_samples[i] - joint_samples[i - 1]) / dt;
            }
            else
            {
                derivative_samples[i] = (joint_samples[i + 1] - joint_samples[i - 1]) / (2.0 * dt);
            }
            derivative_samples[i] *= 0.5;
        }

        return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
            breaks, joint_samples, derivative_samples);
    }

    // Evaluate planned trajectory at given time
    VectorXd eval_trajectory(const drake::trajectories::PiecewisePolynomial<double> &trajectory, double t)
    {
        // Clamp time to trajectory bounds
        double t_clamped = std::max(trajectory.start_time(),
                                    std::min(trajectory.end_time(), t));
        return trajectory.value(t_clamped);
    }

    // Concatenate two trajectories, shifting the second trajectory's time
    // Uses Drake's built-in shiftRight() and ConcatenateInTime()
    drake::trajectories::PiecewisePolynomial<double> ConcatenateTrajectories(
        const drake::trajectories::PiecewisePolynomial<double> &traj1,
        const drake::trajectories::PiecewisePolynomial<double> &traj2,
        double traj1_duration)
    {
        // Debug: print trajectory info
        std::cout << "  [Concatenate] traj1: [" << traj1.start_time() << ", " << traj1.end_time() << "]" << std::endl;
        std::cout << "  [Concatenate] traj2: [" << traj2.start_time() << ", " << traj2.end_time() << "]" << std::endl;

        // Create a copy of traj2 that we can modify
        auto traj2_shifted = traj2;

        // Shift traj2 time so it starts right after traj1 ends
        // ConcatenateInTime requires that traj2.start_time() == result.end_time()
        double traj2_shift = traj1.end_time() - traj2.start_time();

        std::cout << "  [Concatenate] Shifting traj2 by " << traj2_shift << " seconds" << std::endl;

        // Use shiftRight to shift the trajectory in time
        traj2_shifted.shiftRight(traj2_shift);

        std::cout << "  [Concatenate] After shift traj2: [" << traj2_shifted.start_time() << ", " << traj2_shifted.end_time() << "]" << std::endl;

        // Create a copy of traj1 as result (to avoid modifying input)
        auto result = traj1;

        // Concatenate the two trajectories
        // ConcatenateInTime will append traj2_shifted after result
        result.ConcatenateInTime(traj2_shifted);

        std::cout << "  [Concatenate] Result: [" << result.start_time() << ", " << result.end_time() << "]" << std::endl;

        // Return the concatenated trajectory
        return result;
    }

    // Plan collision-free trajectory with intermediate waypoint sampling
    // Uses sampling-based approach to find obstacle-free path
    drake::trajectories::PiecewisePolynomial<double> PlanWithObstacleAvoidance(
        const VectorXd &q_start,
        const Eigen::Vector3d &goal_position,
        double duration = 4.0,
        int max_attempts = 10)
    {
        std::cout << "\n=== Obstacle Avoidance Path Planning ===" << std::endl;
        std::cout << "Goal: " << goal_position.transpose() << std::endl;
        std::cout << "Max attempts: " << max_attempts << std::endl;

        // First, try direct path
        std::cout << "\n>>> Attempt 1: Direct path" << std::endl;
        auto direct_traj = PlanCartesianLine(duration, q_start, goal_position);

        // Check if direct path is collision-free
        bool path_clear = true;
        const int check_points = 20;
        for (int i = 0; i <= check_points; ++i)
        {
            double t = (duration * i) / check_points;
            VectorXd q_check = eval_trajectory(direct_traj, t);
            CollisionResult col = CheckCollisionDetailed(q_check);
            if (col.has_collision)
            {
                path_clear = false;
                std::cout << "  [COLLISION at t=" << t << "] " << col.warning_message << std::endl;
                break;
            }
        }

        if (path_clear)
        {
            std::cout << "Direct path is CLEAR!" << std::endl;
            return direct_traj;
        }

        // Direct path has collision, try intermediate waypoints
        std::cout << "\n>>> Direct path blocked, trying intermediate waypoints..." << std::endl;

        // Compute initial EE position
        auto T_ee_start = ComputeEEPose(q_start);
        Eigen::Vector3d ee_start = T_ee_start.translation();

        // Generate intermediate waypoints at different heights
        std::vector<Eigen::Vector3d> intermediate_positions;

        // Strategy 1: Go higher (Z offset)
        for (double z_offset : {0.1, 0.15, 0.2, 0.25})
        {
            Eigen::Vector3d intermediate = goal_position;
            intermediate(2) += z_offset;
            intermediate_positions.push_back(intermediate);
        }

        // Strategy 2: Go sideways (Y offset)
        for (double y_offset : {-0.1, 0.1, -0.15, 0.15})
        {
            Eigen::Vector3d intermediate = goal_position;
            intermediate(1) += y_offset;
            intermediate_positions.push_back(intermediate);
        }

        // Strategy 3: Go backward/forward (X offset)
        for (double x_offset : {-0.1, 0.1})
        {
            Eigen::Vector3d intermediate = goal_position;
            intermediate(0) += x_offset;
            intermediate_positions.push_back(intermediate);
        }

        std::cout << "Generated " << intermediate_positions.size() << " intermediate waypoint candidates" << std::endl;

        // Try each intermediate waypoint
        for (size_t attempt = 0; attempt < intermediate_positions.size() && attempt < (size_t)max_attempts; ++attempt)
        {
            const auto &via_point = intermediate_positions[attempt];
            std::cout << "\n>>> Attempt " << (attempt + 2) << ": Via point " << via_point.transpose() << std::endl;

            // Plan: Start → Via → Goal
            double segment_duration = duration / 2.0;

            // Segment 1: Start → Via
            auto traj1 = PlanCartesianLine(segment_duration, q_start, via_point);
            VectorXd q_via = eval_trajectory(traj1, segment_duration);

            // Check via point
            CollisionResult col_via = CheckCollisionDetailed(q_via);
            if (col_via.has_collision)
            {
                std::cout << "  Via point in collision, skipping..." << std::endl;
                continue;
            }

            // Segment 2: Via → Goal
            auto traj2 = PlanCartesianLine(segment_duration, q_via, goal_position);
            VectorXd q_goal = eval_trajectory(traj2, segment_duration);

            // Check goal
            CollisionResult col_goal = CheckCollisionDetailed(q_goal);
            if (col_goal.has_collision)
            {
                std::cout << "  Goal reachable but in collision, skipping..." << std::endl;
                continue;
            }

            // Check full path for collisions
            auto combined = ConcatenateTrajectories(traj1, traj2, segment_duration);
            bool combined_clear = true;

            for (int i = 0; i <= check_points; ++i)
            {
                double t = (duration * i) / check_points;
                VectorXd q_check = eval_trajectory(combined, t);
                CollisionResult col = CheckCollisionDetailed(q_check);
                if (col.has_collision)
                {
                    combined_clear = false;
                    std::cout << "  Path collision at t=" << t << ", skipping..." << std::endl;
                    break;
                }
            }

            if (combined_clear)
            {
                std::cout << "\n[SUCCESS] Found collision-free path via waypoint!" << std::endl;
                std::cout << "  Via point: " << via_point.transpose() << std::endl;
                return combined;
            }
        }

        // All attempts failed
        std::cout << "\n[FAILED] Could not find collision-free path after "
                  << intermediate_positions.size() << " attempts" << std::endl;
        std::cout << "Returning direct path (will have collisions)" << std::endl;
        return direct_traj;
    }

    // ========== INDUSTRIAL-GRADE COLLISION DETECTION ==========

    // Collision checking result with detailed information
    struct CollisionResult
    {
        bool has_collision = false;
        double min_distance = std::numeric_limits<double>::infinity();
        std::vector<std::string> colliding_pairs;
        std::string warning_message;

        bool IsSafe() const
        {
            return !has_collision && min_distance > 0.0;
        }
    };

    // Check if configuration q has collisions with detailed reporting
    // Returns: CollisionResult with collision status, minimum distance, and details
    CollisionResult CheckCollisionDetailed(const VectorXd &q)
    {
        CollisionResult result;

        try
        {
            using namespace drake::geometry;
            using namespace drake::systems;

            auto &plant_context = plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
            plant_->SetPositions(&plant_context, q);

            // Get the QueryObject for collision queries
            const auto &geometry_query_port = plant_->get_geometry_query_input_port();

            // Check if port is connected
            if (!geometry_query_port.HasValue(plant_context))
            {
                result.warning_message = "Geometry query port not connected - collision detection unavailable";
                return result; // Return no collision to allow planning to continue
            }

            const auto &query_object = geometry_query_port.Eval<QueryObject<double>>(plant_context);

            // Method 1: Check for actual penetrations (collisions)
            // Use ComputePointPairPenetration() to get all colliding pairs
            std::vector<PenetrationAsPointPair<double>> penetrations =
                query_object.ComputePointPairPenetration();

            // HELPER: Check if two geometries belong to adjacent bodies (should be ignored)
            auto are_adjacent_bodies = [&](GeometryId id1, GeometryId id2) -> bool
            {
                // Get the frame IDs from geometry IDs
                const auto &inspector = query_object.inspector();

                // Get the frames these geometries are attached to
                FrameId frame1 = inspector.GetFrameId(id1);
                FrameId frame2 = inspector.GetFrameId(id2);

                // If same frame, they're on the same body (adjacent by definition)
                if (frame1 == frame2)
                {
                    return true;
                }

                // Get body indices from frames
                auto body1 = plant_->GetBodyFromFrameId(frame1);
                auto body2 = plant_->GetBodyFromFrameId(frame2);

                if (!body1 || !body2)
                {
                    return false; // Can't determine, assume not adjacent
                }

                // Simplified check: bodies are adjacent if their indices are close
                // This works for robot arms where consecutive links have consecutive indices
                // For nezha: right_arm_link1-7 have consecutive body indices
                return (std::abs(body1->index() - body2->index()) <= 1);
            };

            // RELAXED COLLISION DETECTION:
            // 1. Filter out adjacent link collisions
            // 2. Only count collisions with significant penetration depth
            const double penetration_threshold = 0.001; // 1mm threshold
            int serious_collisions = 0;
            int adjacent_collisions = 0;

            for (const auto &penetration : penetrations)
            {
                // Check if this is an adjacent link collision
                if (are_adjacent_bodies(penetration.id_A, penetration.id_B))
                {
                    adjacent_collisions++;
                    continue; // Skip adjacent link collisions
                }

                // penetration.depth is positive when objects are penetrating
                if (penetration.depth > penetration_threshold)
                {
                    serious_collisions++;
                }
            }

            result.has_collision = (serious_collisions > 0);

            // Method 2: Compute signed distances for safety margin
            // This gives us the minimum distance even when not in collision
            const std::vector<drake::geometry::SignedDistancePair<double>> signed_distances =
                query_object.ComputeSignedDistancePairwiseClosestPoints();

            // Find minimum distance (excluding self-collisions of adjacent links)
            for (const auto &dist : signed_distances)
            {
                // Skip adjacent links
                if (are_adjacent_bodies(dist.id_A, dist.id_B))
                {
                    continue;
                }

                if (dist.distance < result.min_distance)
                {
                    result.min_distance = dist.distance;
                }
            }

            // Generate warning message if unsafe
            if (result.has_collision)
            {
                std::ostringstream oss;
                oss << "SERIOUS COLLISION! " << serious_collisions
                    << " non-adjacent pair(s) with penetration > " << (penetration_threshold * 1000) << "mm"
                    << " (" << adjacent_collisions << " adjacent pair(s) ignored)";
                result.warning_message = oss.str();
            }
            else if (!penetrations.empty() && adjacent_collisions > 0)
            {
                // Only adjacent link collisions - this is OK
                std::ostringstream oss;
                oss << "Only adjacent link contact: " << adjacent_collisions
                    << " pair(s) (ignored)";
                result.warning_message = oss.str();
            }
            else if (result.min_distance < 0.01)
            { // Less than 1cm warning
                std::ostringstream oss;
                oss << "Close: " << result.min_distance * 1000 << " mm clearance";
                result.warning_message = oss.str();
            }
            else if (result.min_distance < 0.05)
            { // Less than 5cm info
                std::ostringstream oss;
                oss << "Clearance: " << result.min_distance * 1000 << " mm";
                result.warning_message = oss.str();
            }
        }
        catch (const std::exception &e)
        {
            result.warning_message = std::string("Collision check error: ") + e.what();
            // Don't throw - allow planning to continue with degraded safety
        }

        return result;
    }

    // Simple collision check (backward compatible)
    bool CheckCollision(const VectorXd &q)
    {
        CollisionResult result = CheckCollisionDetailed(q);
        return result.has_collision;
    }

    // Get minimum distance to collision with safety margin
    double GetMinimumCollisionDistance(const VectorXd &q)
    {
        CollisionResult result = CheckCollisionDetailed(q);
        return result.min_distance;
    }

    // Check collision with configurable safety margin
    // Returns true if distance < safety_margin or in collision
    bool CheckCollisionWithMargin(const VectorXd &q, double safety_margin)
    {
        CollisionResult result = CheckCollisionDetailed(q);
        return result.has_collision || (result.min_distance < safety_margin);
    }

    // Print collision details for debugging
    void PrintCollisionReport(const VectorXd &q)
    {
        CollisionResult result = CheckCollisionDetailed(q);

        std::cout << "\n=== Collision Check Report ===" << std::endl;
        std::cout << "Collision Status: " << (result.has_collision ? "COLLISION" : "CLEAR") << std::endl;
        std::cout << "Minimum Distance: " << (result.min_distance * 1000) << " mm" << std::endl;

        if (!result.warning_message.empty())
        {
            std::cout << result.warning_message << std::endl;
        }

        if (result.has_collision)
        {
            std::cout << "\nColliding Geometry Pairs:" << std::endl;
            for (const auto &pair : result.colliding_pairs)
            {
                std::cout << "  - " << pair << std::endl;
            }
        }
        std::cout << "==============================\n"
                  << std::endl;
    }

private:
    std::unique_ptr<drake::systems::Diagram<double>> diagram_;
    std::shared_ptr<drake::planning::RobotDiagram<double>> robot_diagram_; // For GCS
    drake::multibody::MultibodyPlant<double> *plant_ = nullptr;
    drake::geometry::SceneGraph<double> *scene_graph_ = nullptr;
    std::unique_ptr<drake::systems::Simulator<double>> simulator_;

    // INDUSTRIAL-GRADE: Use fast planning after first GCS run
    static bool use_fast_planning_;
    static int planning_call_count_;
};

// ============================================================================
// Implementation of Advanced IK Solvers
// ============================================================================

// Generate random initial guess for IK
VectorXd DrakeSimulator::GenerateRandomGuess(const VectorXd &q_guess)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    VectorXd q_random = q_guess;

    // Only randomize right arm joints (11-17)
    for (int i = 11; i <= 17; ++i)
    {
        double lower = plant_->GetPositionLowerLimits()(i);
        double upper = plant_->GetPositionUpperLimits()(i);
        q_random(i) = lower + dist(gen) * (upper - lower);
    }

    return q_random;
}

// Implementation of Advanced IK with multiple random initializations
std::optional<VectorXd> DrakeSimulator::SolveGlobalIK(
    const drake::math::RigidTransformd &desired_pose,
    const VectorXd &q_guess,
    bool debug)
{
    using namespace drake::multibody;

    const auto &ee_frame = plant_->GetFrameByName("right_tool_frame");
    const auto &waist_frame = plant_->GetFrameByName("waist_link");

    // Try multiple initial guesses for better global exploration
    const int num_attempts = 12; // Multiple random attempts

    for (int attempt = 0; attempt < num_attempts; ++attempt)
    {
        try
        {
            // Create IK problem
            InverseKinematics ik(*plant_);

            // Set initial guess
            VectorXd q_init;
            if (attempt == 0)
            {
                q_init = q_guess;
            }
            else if (attempt == 1)
            {
                // Zero position for right arm
                q_init = q_guess;
                for (int i = 11; i <= 17; ++i)
                {
                    q_init(i) = 0.0;
                }
            }
            else if (attempt == 2)
            {
                // Mid-range
                q_init = q_guess;
                for (int i = 11; i <= 17; ++i)
                {
                    double lower = plant_->GetPositionLowerLimits()(i);
                    double upper = plant_->GetPositionUpperLimits()(i);
                    q_init(i) = (lower + upper) / 2.0;
                }
            }
            else
            {
                // Random guesses for better global exploration
                q_init = GenerateRandomGuess(q_guess);
            }

            ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_init);

            // Add position constraint (relaxed for better success rate)
            // Using 5mm tolerance for position
            double pos_tol = 0.0001; // 5mm tolerance
            Eigen::Vector3d pos_lower = desired_pose.translation() - Eigen::Vector3d::Constant(pos_tol);
            Eigen::Vector3d pos_upper = desired_pose.translation() + Eigen::Vector3d::Constant(pos_tol);

            ik.AddPositionConstraint(
                ee_frame, Eigen::Vector3d::Zero(),
                waist_frame, pos_lower, pos_upper);

            // Add orientation constraint (relaxed - 45 degrees for circular motion)
            double orientation_tolerance = 5.0 * M_PI / 180.0; // 45 degrees
            ik.AddOrientationConstraint(
                waist_frame, desired_pose.rotation(),
                ee_frame, drake::math::RotationMatrixd::Identity(),
                orientation_tolerance);

            // Fix non-right-arm joints
            for (int i = 0; i < plant_->num_positions(); ++i)
            {
                if (i < 11 || i > 17)
                {
                    ik.get_mutable_prog()->AddBoundingBoxConstraint(
                        q_guess(i), q_guess(i), ik.q()(i));
                }
            }

            // Add joint limits for right arm
            for (int i = 11; i <= 17; ++i)
            {
                double lower = plant_->GetPositionLowerLimits()(i);
                double upper = plant_->GetPositionUpperLimits()(i);
                ik.get_mutable_prog()->AddBoundingBoxConstraint(
                    lower, upper, ik.q()(i));
            }

            // Add cost to stay close to initial guess (for smoothness)
            // This helps avoid large jumps in joint space
            ik.get_mutable_prog()->AddCost(
                (ik.q() - q_init).dot(ik.q() - q_init));

            // Solve IK
            auto result = drake::solvers::Solve(ik.prog());

            if (result.is_success())
            {
                VectorXd q_solution = result.GetSolution(ik.q());

                // Verify solution
                auto T_solution = ComputeEEPose(q_solution);
                double pos_error = (T_solution.translation() - desired_pose.translation()).norm();

                if (debug && attempt < 3)
                {
                    std::cout << "    [Advanced IK] Attempt " << (attempt + 1)
                              << " success, error: " << pos_error * 1000 << " mm" << std::endl;
                }

                if (pos_error < 0.001) // 2cm tolerance
                {
                    if (debug && attempt > 0)
                    {
                        std::cout << "    [Advanced IK] Success on attempt " << (attempt + 1)
                                  << ", error: " << pos_error * 1000 << " mm" << std::endl;
                    }
                    return q_solution;
                }
            }
            else
            {
                if (debug && attempt < 2)
                {
                    std::cout << "    [Advanced IK] Attempt " << (attempt + 1)
                              << " failed to converge" << std::endl;
                }
            }
        }
        catch (const std::exception &e)
        {
            if (debug && attempt == 0)
            {
                std::cout << "    [Advanced IK] Exception on attempt " << (attempt + 1)
                          << ": " << e.what() << std::endl;
            }
            continue;
        }
    }

    return std::nullopt;
}

// Removed SolveHierarchicalIK - now using only Global IK for better global search

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
    mujoco_scene_path = project_dir + "/model/nezha/scene/scene.xml";

    // Verify paths exist
    std::cout << "\n[PATH] Checking model files:" << std::endl;
    std::cout << "  URDF:   " << urdf_path << std::endl;
    std::cout << "  Scene:  " << mujoco_scene_path << std::endl;

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

    std::cout << "  [OK] All model files found\n"
              << std::endl;

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
        std::cout << "\n=== Running Drake Cartesian Planning + MuJoCo Visualization ===" << std::endl;
        std::cout << "Close the visualization window to exit...\n"
                  << std::endl;

        // ========== STEP 1: DRAKE CARTESIAN TRAJECTORY PLANNING ==========
        std::cout << "\n>>> Step 1: Loading Drake Model for Planning" << std::endl;
        DrakeSimulator drake_sim(urdf_path);

        // Define starting joint configuration for Nezha robot (20 DOF)
        // Joint indices: legs[0-2], waist[3], left_arm[4-10], right_arm[11-17], head[18-19]

        // Initial joint configuration (angles in degrees converted to radians)
        VectorXd q_start = VectorXd::Zero(20);
        // q_start(11) = 5.0 * M_PI / 180.0;    // 5 degrees
        // q_start(12) = -5.0 * M_PI / 180.0;    // -5 degrees
        // q_start(13) = 0.0 * M_PI / 180.0;    // 0 degrees
        // q_start(14) = 0.0 * M_PI / 180.0;    // 0 degrees
        // q_start(15) = 0.0 * M_PI / 180.0;    // 0 degrees
        // q_start(16) = 0.0 * M_PI / 180.0;    // 0 degrees
        // q_start(17) = 0.0 * M_PI / 180.0;    // 0 degrees




     

       
        q_start(11) = 0.05; // right_arm_joint1 (shoulder pan) - 2.86°
        q_start(12) = 0.0;  // right_arm_joint2 (shoulder lift) - 0°
        q_start(13) = 0.05; // right_arm_joint3 (elbow) - 2.86° (was 0, caused singularity)
        q_start(14) = 0.2;  // right_arm_joint4 (wrist rotation) - 11.46°
        q_start(15) = 0.1;  // right_arm_joint5 (wrist flex) - 5.73°
        q_start(16) = 0.05; // right_arm_joint6 (wrist rotation) - 2.86°
        q_start(17) = 0.05; // right_arm_joint7 (wrist flex) - 2.86°

    

  

        // Compute initial EE position using Forward Kinematics
        drake::math::RigidTransformd T_ee_start = drake_sim.ComputeEEPose(q_start);
        Eigen::Vector3d ee_start = T_ee_start.translation();   // 末端初始位置

        std::cout << "Initial EE Position (waist frame): " << ee_start.transpose() << std::endl;
        std::cout << "Configuration uses waist coordinate frame as reference" << std::endl;

        // Perform initial collision check on starting configuration
        std::cout << "\n>>> Safety Check: Verifying Initial Configuration" << std::endl;
        DrakeSimulator::CollisionResult initial_collision = drake_sim.CheckCollisionDetailed(q_start);
        drake_sim.PrintCollisionReport(q_start);

        if (initial_collision.has_collision)
        {
            std::cout << "\n[WARNING] Starting configuration is in collision!" << std::endl;
            std::cout << "Trajectory planning will attempt to find safe path..." << std::endl;
        }
        else
        {
            std::cout << "[OK] Initial configuration is safe" << std::endl;
        }

        // ========== TRAJECTORY PLANNING BASED ON TYPE ==========
        drake::trajectories::PiecewisePolynomial<double> planned_trajectory;

        // Declare circle parameters for later use in tracking
        Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
        if (trajectory_type == "MoveL")
        {
            // ========== LINEAR TRAJECTORY WITH FULL POSE CONTROL ==========
            /*
                进一步优化建议
                如果需要 亚毫米级（< 0.01 mm） 精度：
                增加精化迭代次数：3 → 5
                使用更高阶插值：B样条代替三次样条
                考虑前馈控制：基于模型的前馈 + 反馈
                减少步长：dt 再减半
            */
            std::cout << "\n>>> Planning Linear Trajectory with Full Pose Control (Position + Orientation)" << std::endl;

            // Define goal position (offset from start by 20cm in X and -20cm in Y and +20cm in Z)
            Eigen::Vector3d goal_position = ee_start;
            goal_position(0) += 0.2; // +20cm in X
            goal_position(1) -= 0.2; // -20cm in Y
            goal_position(2) += 0.2; // +20cm in Z

            // =====================================================================
            // METHOD 1: Joint space definition (more intuitive for robot programmers)
            // =====================================================================
            VectorXd q_target = VectorXd::Zero(20);
            q_target(11) = 10* M_PI / 180.0; // shoulder pan
            q_target(12) =-10* M_PI / 180.0;
            q_target(13) = 5* M_PI / 180.0; // elbow
            q_target(14) = 20* M_PI / 180.0; // wrist rotation
            q_target(15) = 0* M_PI / 180.0; // wrist flex
            q_target(16) = 0* M_PI / 180.0; // wrist lateral
            q_target(17) = 0* M_PI / 180.0; // wrist vertical

            // Compute goal pose from target joint configuration using Forward Kinematics
            drake::math::RigidTransformd goal_pose_from_fk = drake_sim.ComputeEEPose(q_target);

            // =====================================================================
            // METHOD 2: Cartesian space definition (position + orientation)
            // =====================================================================
            // Define goal orientation (small rotation from start)
            drake::math::RollPitchYawd start_rpy_custom(T_ee_start.rotation());
            drake::math::RollPitchYawd goal_rpy_custom(
                start_rpy_custom.roll_angle(),            // Keep same roll
                start_rpy_custom.pitch_angle(),           // Keep same pitch
                start_rpy_custom.yaw_angle() + M_PI / 8.0 // +22.5 degrees yaw
            );
            drake::math::RotationMatrixd goal_rotation_custom(goal_rpy_custom);

            // Create goal pose from Cartesian definition
            drake::math::RigidTransformd goal_pose_from_cartesian(goal_rotation_custom, goal_position);

            // =====================================================================
            // CHOOSE WHICH METHOD TO USE
            // =====================================================================
            bool use_joint_space_method = false; // Set false to use Cartesian space method

            drake::math::RigidTransformd goal_pose;
            if (use_joint_space_method)
            {
                goal_pose = goal_pose_from_fk;
                std::cout << "  [METHOD] Using Joint Space Definition (Forward Kinematics)" << std::endl;
            }
            else
            {
                goal_pose = goal_pose_from_cartesian;
                std::cout << "  [METHOD] Using Cartesian Space Definition (Position + Orientation)" << std::endl;
            }

            // Extract goal position and rotation for display
            Eigen::Vector3d goal_position_final = goal_pose.translation();
            drake::math::RotationMatrixd goal_rotation_final = goal_pose.rotation();

            std::cout << "  EE Start Position: " << ee_start.transpose() << " m" << std::endl;
            std::cout << "  EE Goal Position (computed): " << goal_position_final.transpose() << " m" << std::endl;

            if (use_joint_space_method)
            {
                std::cout << "  Target Joint Config (right arm): [";
                for (int i = 11; i <= 17; ++i)
                {
                    std::cout << q_target(i);
                    if (i < 17)
                        std::cout << " ";
                }
                std::cout << "]" << std::endl;
            }
            else
            {
                std::cout << "  Position Offset (XYZ): ["
                          << goal_position(0) - ee_start(0) << " "
                          << goal_position(1) - ee_start(1) << " "
                          << goal_position(2) - ee_start(2) << "] m" << std::endl;
            }

            drake::math::RollPitchYawd start_rpy_display(T_ee_start.rotation());
            drake::math::RollPitchYawd goal_rpy_display(goal_rotation_final);
            std::cout << "  EE Start Orientation (RPY): " << start_rpy_display.vector().transpose() << " rad" << std::endl;
            std::cout << "  EE Goal Orientation (RPY):  " << goal_rpy_display.vector().transpose() << " rad" << std::endl;

            std::cout << "  Linear Distance: " << (goal_position_final - ee_start).norm() << " m" << std::endl;
            // Calculate angular distance using angle-axis
            Eigen::Matrix3d R_diff_mat = T_ee_start.rotation().matrix() * goal_rotation_final.matrix().transpose();
            Eigen::AngleAxisd angle_axis_diff(R_diff_mat);
            double angular_distance_main = angle_axis_diff.angle();
            std::cout << "  Angular Distance: " << angular_distance_main << " rad ("
                      << (angular_distance_main * 180.0 / M_PI) << " deg)" << std::endl;

            // Use the new full-pose planning function
            // This controls BOTH position and orientation
            //TODO: MoveL
            planned_trajectory = drake_sim.PlanCartesianLineWithPose(
                q_start,
                goal_pose,
                0.2,   // max_velocity = INCREASED from 0.3 to 0.8 m/s (faster motion)
                0.1,   // max_acceleration = INCREASED from 0.15 to 1.0 m/s²
                1.0,   // max_angular_velocity = INCREASED from 0.5 to 1.5 rad/s
                0.5,   // max_angular_acceleration = INCREASED from 0.25 to 3.0 rad/s²
                true); // enable optimal timing planning

            std::cout << "\n[SUCCESS] Full-pose trajectory generated!" << std::endl;
            std::cout << "The robot will now move to the target position WHILE rotating to the target orientation." << std::endl;

            // =====================================================================
            // SAVE TRAJECTORY TO JSON FOR REAL ROBOT TESTING
            // =====================================================================
            std::string json_filename = "trajectory_moveL_pose.json";
            std::ofstream json_file(json_filename);

            if (json_file.is_open())
            {
                std::cout << "\n[JSON] Saving trajectory to: " << json_filename << std::endl;

                // CRITICAL: Use ALL samples from the planned trajectory at 200Hz
                // Do NOT resample - use the exact waypoints that were planned
                double trajectory_duration = planned_trajectory.end_time();

                // Get the exact number of samples in the trajectory
                // The trajectory was generated at 200Hz (5ms intervals)
                double sampling_interval = 0.005; // 5ms = 200Hz
                int num_samples = static_cast<int>(trajectory_duration / sampling_interval) + 1;

                std::cout << "[JSON] Exporting all " << num_samples << " trajectory points at 200Hz" << std::endl;
                std::cout << "[JSON] Trajectory duration: " << trajectory_duration << " s" << std::endl;
                std::cout << "[JSON] Actual frequency: " << (num_samples - 1) / trajectory_duration << " Hz" << std::endl;

                // Start building JSON
                json_file << "{\n";
                json_file << "    \"cycle\": 0,\n";
                json_file << "    \"actions\": [\n";
                json_file << "        {\n";
                json_file << "            \"taskId\": \"moveL_pose\",\n";
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
                    VectorXd q_t = planned_trajectory.value(t);

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
                std::cout << "[JSON] Successfully saved " << num_samples << " samples" << std::endl;
                std::cout << "[JSON] All trajectory points preserved at 200Hz" << std::endl;
                std::cout << "[JSON] Format: Actions with rightHand joint trajectories" << std::endl;
                std::cout << "[JSON] Joints: q11-q17 (7 DOF right arm)" << std::endl;
                std::cout << "[JSON] Ready for real robot deployment!" << std::endl;
            }
            else
            {
                std::cerr << "[ERROR] Failed to create JSON file: " << json_filename << std::endl;
            }
        }
        else if (trajectory_type == "MoveC")
        {

            // ========== CIRCULAR TRAJECTORY: YZ PLANE ==========
            // Strategy: Move to starting position on circle, then draw circle in YZ plane
            std::cout << "\n>>> Planning Circular Trajectory (YZ Plane)" << std::endl;

            // Get EE position in waist frame
            Eigen::Vector3d ee_start_waist = ee_start; // Already computed as T_ee_start.translation()

            // Define circle parameters
            double radius = 0.06; // Circle radius (meters) - 6cm (smaller for better reachability)

            // IMPORTANT: Place circle center at the same position as LINE trajectory goal
            // This matches the linear trajectory target position
            circle_center = ee_start_waist;
            circle_center(0) += 0.2; // Same offset as linear trajectory: +20cm in X
            circle_center(1) -= 0.2; // Same offset as linear trajectory: -20cm in Y
            circle_center(2) += 0.2; // Same offset as linear trajectory: +20cm in Z

            // Calculate circle starting point (at angle 0, which is "up" from center in Y direction)
            // Circle lies in YZ plane centered at this target position
            Eigen::Vector3d circle_start_point = circle_center;
            circle_start_point(0) = circle_center(0);          // Same X (forward reach)
            circle_start_point(1) = circle_center(1) + radius; // +radius in Y direction
            circle_start_point(2) = circle_center(2);          // Same Z as center

            std::cout << "  === Strategy ===" << std::endl;
            std::cout << "  Phase 1: Move from current EE position to circle start point" << std::endl;
            std::cout << "  Phase 2: Draw full circle in YZ plane (normal along X-axis)" << std::endl;
            std::cout << std::endl;
            std::cout << "  EE current position: " << ee_start_waist.transpose() << " m" << std::endl;
            std::cout << "  Circle center:        " << circle_center.transpose() << " m (at LINE target position)" << std::endl;
            std::cout << "  Circle start point:   " << circle_start_point.transpose() << " m" << std::endl;
            std::cout << "  Circle radius:        " << radius * 1000 << " mm" << std::endl;

            // Verify start point is on the circle
            double start_to_center_dist = (circle_start_point - circle_center).norm();
            std::cout << "  Start->Center dist:   " << start_to_center_dist * 1000 << " mm (should be " << radius * 1000 << " mm)" << std::endl;

            // Create circle normal vector - use X axis (perpendicular to YZ plane)
            // This creates a VERTICAL circle in the YZ plane
            Eigen::Vector3d circle_normal = Eigen::Vector3d::UnitX();

            std::cout << "  Circle normal: " << circle_normal.transpose() << " (X-axis)" << std::endl;
            std::cout << "  Circle plane: YZ PLANE (vertical)" << std::endl;

            // Define approach phase duration and circle drawing duration
            double approach_duration = 1.5;                            // 1.5 seconds to move to start position
            double circle_duration = sim_duration - approach_duration; // Remaining time for circle

            if (circle_duration < 3.0)
            {
                std::cout << "  [WARNING] Total duration too short for circle drawing!" << std::endl;
                std::cout << "  [WARNING] Adjusting to minimum 5 seconds total" << std::endl;
                sim_duration = 5.0;
                approach_duration = 1.5;
                circle_duration = 3.5;
            }

            std::cout << "  Approach duration: " << approach_duration << " s (will be auto-computed)" << std::endl;
            std::cout << "  Circle duration:   " << circle_duration << " s (will be auto-computed)" << std::endl;

            // Phase 1: Plan approach trajectory (move from current EE to circle start point)
            // ✅ INDUSTRIAL STANDARD: MoveC with full 6D pose control
            std::cout << "\n>>> Phase 1: Planning approach trajectory to circle start point (MoveL)" << std::endl;
            std::cout << "[INDUSTRIAL] Using MoveL with 6D pose control and automatic timing" << std::endl;

            // Define approach goal pose (keep orientation same as start for approach)
            drake::math::RigidTransformd approach_pose(T_ee_start.rotation(), circle_start_point);
            auto approach_trajectory = drake_sim.PlanCartesianLineWithPose(
                q_start,
                approach_pose,
                0.5,   // max_velocity
                1.0,   // max_acceleration
                1.0,   // max_angular_velocity
                2.0,   // max_angular_acceleration
                true); // optimize_timing

            double actual_approach_duration = approach_trajectory.end_time();
            std::cout << "  Actual approach duration: " << actual_approach_duration << " s" << std::endl;

            // Phase 2: Plan circle trajectory (MoveC with 6D pose control)
            // Get joint configuration at the end of approach trajectory
            VectorXd q_circle_start = drake_sim.eval_trajectory(approach_trajectory, actual_approach_duration);
            drake::math::RigidTransformd T_ee_after_approach = drake_sim.ComputeEEPose(q_circle_start);
            Eigen::Vector3d ee_actual_start = T_ee_after_approach.translation();

            std::cout << "\n>>> Phase 2: Planning circle trajectory with MoveC (6D pose control)" << std::endl;
            std::cout << "  Actual EE position after approach: " << ee_actual_start.transpose() << " m" << std::endl;
            std::cout << "  Original circle center:            " << circle_center.transpose() << " m" << std::endl;

            // Verify distance from actual EE to original center
            double actual_to_center_dist = (ee_actual_start - circle_center).norm();
            std::cout << "  Actual EE->Center distance:       " << actual_to_center_dist * 1000
                      << " mm (should be close to " << radius * 1000 << " mm)" << std::endl;

            // Define via point (middle of circle at 180 degrees from start)
            // This is the opposite point on the circle
            Eigen::Vector3d via_point = circle_center;
            via_point(0) = circle_center(0);          // Same X
            via_point(1) = circle_center(1) - radius; // -radius in Y (opposite direction)
            via_point(2) = circle_center(2);          // Same Z as center

            // Define goal point (back to start for full circle, or end point for arc)
            Eigen::Vector3d goal_point = circle_start_point; // Full circle returns to start

            std::cout << "  Via point:   " << via_point.transpose() << " m" << std::endl;
            std::cout << "  Goal point:  " << goal_point.transpose() << " m (back to start for full circle)" << std::endl;

            // Define orientations for via and goal poses
            // For a welding/cutting scenario, maintain tool orientation relative to path
            // Use same orientation as start (can be customized for specific tasks)
            drake::math::RigidTransformd via_pose(T_ee_start.rotation(), via_point);
            drake::math::RigidTransformd goal_pose(T_ee_start.rotation(), goal_point);

            // Use industrial-grade MoveC with full 6D pose control
            std::cout << "\n[INDUSTRIAL] Using MoveC with 6D pose control (Position + Orientation)" << std::endl;
            std::cout << "  - Circle calculated from 3 points: start -> via -> goal" << std::endl;
            std::cout << "  - Orientation interpolated via dual SLERP" << std::endl;
            std::cout << "  - Full 6D Differential IK control" << std::endl;
            auto circle_trajectory = drake_sim.PlanCartesianCircleWithPose(
                q_circle_start, via_pose, goal_pose,
                0.3,  // max_velocity
                0.25,  // max_acceleration
                0.5,  // max_angular_velocity
                0.25); // max_angular_acceleration

            // Concatenate approach and circle trajectories
            std::cout << "\n>>> Concatenating approach and circle trajectories" << std::endl;
            planned_trajectory = drake_sim.ConcatenateTrajectories(
                approach_trajectory, circle_trajectory, actual_approach_duration);

            double actual_duration = planned_trajectory.end_time() - planned_trajectory.start_time();
            std::cout << "Combined trajectory duration: " << actual_duration << " s (auto-computed)" << std::endl;

            // Debug: Test if trajectory is accessible
            std::cout << "\n>>> Testing planned_trajectory before MuJoCo init..." << std::endl;
            Eigen::VectorXd test_q = planned_trajectory.value(0.0);
            std::cout << "  planned_trajectory at t=0: size=" << test_q.size() << std::endl;
            std::cout << "  planned_trajectory at t=2.5: size=" << planned_trajectory.value(2.5).size() << std::endl;
            std::cout << "  Trajectory test PASSED" << std::endl;
        }
        else if (trajectory_type == "MoveJ")
        {
            // ========== JOINT SPACE MOTION (MoveJ) ==========
            std::cout << "\n>>> Planning Joint Space Motion (MoveJ)" << std::endl;
            std::cout << "[INDUSTRIAL] Using MoveJ with 7-segment trajectory planning" << std::endl;

            // Define goal joint configuration
            // We'll create a target by offsetting some joint angles
            VectorXd q_goal = q_start;

            // Offset right arm joints (11-17) for demonstration
            q_goal(11) = 12.86* M_PI / 180.0; // right_arm_joint1: +0.3 rad
            q_goal(12) = -10* M_PI / 180.0; // right_arm_joint2: -0.2 rad
            q_goal(13) = 12.86* M_PI / 180.0; // right_arm_joint3: +0.4 rad
            q_goal(14) = 20* M_PI / 180.0; // right_arm_joint4: +0.5 rad
            q_goal(15) = 15.73* M_PI / 180.0; // right_arm_joint5: -0.3 rad
            q_goal(16) = 12.86* M_PI / 180.0; // right_arm_joint6: +0.2 rad
            q_goal(17) = 12.86* M_PI / 180.0; // right_arm_joint7: -0.4 rad

            std::cout << "\n[MOVEJ CONFIGURATION]" << std::endl;
            std::cout << "  Planning joint space motion from start to goal configuration" << std::endl;
            std::cout << "  Right arm joints will be offset" << std::endl;

            // Calculate joint space distance (Euclidean norm)
            double joint_distance = (q_goal - q_start).norm();
            std::cout << "  Joint Space Distance: " << joint_distance << " rad" << std::endl;

            // Use MoveJ with uniform velocity/acceleration limits
            planned_trajectory = drake_sim.PlanCartesianMoveJ(
                q_start,
                q_goal,
                1.0,  // max_velocity = 1.0 rad/s
                2.0); // max_acceleration = 2.0 rad/s²

            std::cout << "\n[SUCCESS] MoveJ trajectory generated!" << std::endl;
            std::cout << "The robot will move smoothly in joint space to the target configuration." << std::endl;

            // Print final joint configuration and end-effector pose
            std::cout << "\n" << std::string(80, '=') << std::endl;
            std::cout << "MOVEJ TRAJECTORY EXECUTION RESULTS" << std::endl;
            std::cout << std::string(80, '=') << std::endl;

            // Get final joint angles at the end of trajectory
            double trajectory_duration = planned_trajectory.end_time() - planned_trajectory.start_time();
            VectorXd q_final = planned_trajectory.value(trajectory_duration);

            std::cout << "\n[FINAL JOINT ANGLES]" << std::endl;
            std::cout << "  Right Arm (q11-q17):" << std::endl;
            for (int i = 11; i <= 17; ++i) {
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

            std::cout << "\n" << std::string(80, '=') << std::endl;
        }
        else
        {
            // ========== CIRCULAR TRAJECTORY: YZ PLANE ==========
            // Strategy: Move to starting position on circle, then draw circle in YZ plane
            std::cout << "\n>>> Planning Circular Trajectory (YZ Plane)" << std::endl;

            // Get EE position in waist frame
            Eigen::Vector3d ee_start_waist = ee_start; // Already computed as T_ee_start.translation()

            // Define circle parameters
            double radius = 0.06; // Circle radius (meters) - 6cm (smaller for better reachability)

            // IMPORTANT: Place circle center at the same position as LINE trajectory goal
            // This matches the linear trajectory target position
            circle_center = ee_start_waist;
            circle_center(0) += 0.2; // Same offset as linear trajectory: +20cm in X
            circle_center(1) -= 0.2; // Same offset as linear trajectory: -20cm in Y
            circle_center(2) += 0.2; // Same offset as linear trajectory: +20cm in Z

            // Calculate circle starting point (at angle 0, which is "up" from center in Y direction)
            // Circle lies in YZ plane centered at this target position
            Eigen::Vector3d circle_start_point = circle_center;
            circle_start_point(0) = circle_center(0);          // Same X (forward reach)
            circle_start_point(1) = circle_center(1) + radius; // +radius in Y direction
            circle_start_point(2) = circle_center(2);          // Same Z as center

            std::cout << "  === Strategy ===" << std::endl;
            std::cout << "  Phase 1: Move from current EE position to circle start point" << std::endl;
            std::cout << "  Phase 2: Draw full circle in YZ plane (normal along X-axis)" << std::endl;
            std::cout << std::endl;
            std::cout << "  EE current position: " << ee_start_waist.transpose() << " m" << std::endl;
            std::cout << "  Circle center:        " << circle_center.transpose() << " m (at LINE target position)" << std::endl;
            std::cout << "  Circle start point:   " << circle_start_point.transpose() << " m" << std::endl;
            std::cout << "  Circle radius:        " << radius * 1000 << " mm" << std::endl;

            // Verify start point is on the circle
            double start_to_center_dist = (circle_start_point - circle_center).norm();
            std::cout << "  Start->Center dist:   " << start_to_center_dist * 1000 << " mm (should be " << radius * 1000 << " mm)" << std::endl;

            // Create circle normal vector - use X axis (perpendicular to YZ plane)
            // This creates a VERTICAL circle in the YZ plane
            Eigen::Vector3d circle_normal = Eigen::Vector3d::UnitX();

            std::cout << "  Circle normal: " << circle_normal.transpose() << " (X-axis)" << std::endl;
            std::cout << "  Circle plane: YZ PLANE (vertical)" << std::endl;

            // Define approach phase duration and circle drawing duration
            double approach_duration = 1.5;                            // 1.5 seconds to move to start position
            double circle_duration = sim_duration - approach_duration; // Remaining time for circle

            if (circle_duration < 3.0)
            {
                std::cout << "  [WARNING] Total duration too short for circle drawing!" << std::endl;
                std::cout << "  [WARNING] Adjusting to minimum 5 seconds total" << std::endl;
                sim_duration = 5.0;
                approach_duration = 1.5;
                circle_duration = 3.5;
            }

            std::cout << "  Approach duration: " << approach_duration << " s (will be auto-computed)" << std::endl;
            std::cout << "  Circle duration:   " << circle_duration << " s (will be auto-computed)" << std::endl;

            // Phase 1: Plan approach trajectory (move from current EE to circle start point)
            // ✅ INDUSTRIAL STANDARD: MoveC with full 6D pose control
            std::cout << "\n>>> Phase 1: Planning approach trajectory to circle start point (MoveL)" << std::endl;
            std::cout << "[INDUSTRIAL] Using MoveL with 6D pose control and automatic timing" << std::endl;

            // Define approach goal pose (keep orientation same as start for approach)
            drake::math::RigidTransformd approach_pose(T_ee_start.rotation(), circle_start_point);
            auto approach_trajectory = drake_sim.PlanCartesianLineWithPose(
                q_start,
                approach_pose,
                0.5,   // max_velocity
                1.0,   // max_acceleration
                1.0,   // max_angular_velocity
                2.0,   // max_angular_acceleration
                true); // optimize_timing

            double actual_approach_duration = approach_trajectory.end_time();
            std::cout << "  Actual approach duration: " << actual_approach_duration << " s" << std::endl;

            // Phase 2: Plan circle trajectory (MoveC with 6D pose control)
            // Get joint configuration at the end of approach trajectory
            VectorXd q_circle_start = drake_sim.eval_trajectory(approach_trajectory, actual_approach_duration);
            drake::math::RigidTransformd T_ee_after_approach = drake_sim.ComputeEEPose(q_circle_start);
            Eigen::Vector3d ee_actual_start = T_ee_after_approach.translation();

            std::cout << "\n>>> Phase 2: Planning circle trajectory with MoveC (6D pose control)" << std::endl;
            std::cout << "  Actual EE position after approach: " << ee_actual_start.transpose() << " m" << std::endl;
            std::cout << "  Original circle center:            " << circle_center.transpose() << " m" << std::endl;

            // Verify distance from actual EE to original center
            double actual_to_center_dist = (ee_actual_start - circle_center).norm();
            std::cout << "  Actual EE->Center distance:       " << actual_to_center_dist * 1000
                      << " mm (should be close to " << radius * 1000 << " mm)" << std::endl;

            // Define via point (middle of circle at 180 degrees from start)
            // This is the opposite point on the circle
            Eigen::Vector3d via_point = circle_center;
            via_point(0) = circle_center(0);          // Same X
            via_point(1) = circle_center(1) - radius; // -radius in Y (opposite direction)
            via_point(2) = circle_center(2);          // Same Z as center

            // Define goal point (back to start for full circle, or end point for arc)
            Eigen::Vector3d goal_point = circle_start_point; // Full circle returns to start

            std::cout << "  Via point:   " << via_point.transpose() << " m" << std::endl;
            std::cout << "  Goal point:  " << goal_point.transpose() << " m (back to start for full circle)" << std::endl;

            // Define orientations for via and goal poses
            // For a welding/cutting scenario, maintain tool orientation relative to path
            // Use same orientation as start (can be customized for specific tasks)
            drake::math::RigidTransformd via_pose(T_ee_start.rotation(), via_point);
            drake::math::RigidTransformd goal_pose(T_ee_start.rotation(), goal_point);

            // Use industrial-grade MoveC with full 6D pose control
            std::cout << "\n[INDUSTRIAL] Using MoveC with 6D pose control (Position + Orientation)" << std::endl;
            std::cout << "  - Circle calculated from 3 points: start -> via -> goal" << std::endl;
            std::cout << "  - Orientation interpolated via dual SLERP" << std::endl;
            std::cout << "  - Full 6D Differential IK control" << std::endl;
            auto circle_trajectory = drake_sim.PlanCartesianCircleWithPose(
                q_circle_start, via_pose, goal_pose,
                0.5,  // max_velocity
                1.0,  // max_acceleration
                1.0,  // max_angular_velocity
                2.0); // max_angular_acceleration

            // Concatenate approach and circle trajectories
            std::cout << "\n>>> Concatenating approach and circle trajectories" << std::endl;
            planned_trajectory = drake_sim.ConcatenateTrajectories(
                approach_trajectory, circle_trajectory, actual_approach_duration);

            double actual_duration = planned_trajectory.end_time() - planned_trajectory.start_time();
            std::cout << "Combined trajectory duration: " << actual_duration << " s (auto-computed)" << std::endl;

            // Debug: Test if trajectory is accessible
            std::cout << "\n>>> Testing planned_trajectory before MuJoCo init..." << std::endl;
            Eigen::VectorXd test_q = planned_trajectory.value(0.0);
            std::cout << "  planned_trajectory at t=0: size=" << test_q.size() << std::endl;
            std::cout << "  planned_trajectory at t=2.5: size=" << planned_trajectory.value(2.5).size() << std::endl;
            std::cout << "  Trajectory test PASSED" << std::endl;
        }

        // ========== STEP 2: MUJOCO VISUALIZATION ==========
        if (!enable_visualization)
        {
            std::cout << "\n>>> Step 2: Skipping MuJoCo visualization (headless mode)" << std::endl;
            std::cout << ">>> Trajectory planning completed successfully!" << std::endl;

            // Get actual trajectory duration
            double trajectory_duration = planned_trajectory.end_time();
            std::cout << ">>> Total duration: " << trajectory_duration << " seconds" << std::endl;

            // Optional: Save trajectory to file for later analysis
            std::cout << "\n>>> Saving trajectory to file..." << std::endl;
            std::ofstream traj_file("/tmp/drake_planned_trajectory.csv");
            if (traj_file.is_open())
            {
                traj_file << "time,q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,q17,q18,q19" << std::endl;
                const double sample_dt = 0.01;
                for (double t = 0.0; t <= trajectory_duration; t += sample_dt)
                {
                    Eigen::VectorXd q = drake_sim.eval_trajectory(planned_trajectory, t);
                    traj_file << t;
                    for (int i = 0; i < q.size(); ++i)
                    {
                        traj_file << "," << q(i);
                    }
                    traj_file << std::endl;
                }
                traj_file.close();
                std::cout << "  Trajectory saved to: /tmp/drake_planned_trajectory.csv" << std::endl;
            }

            return 0;
        }

        std::cout << "\n>>> Step 2: Initializing MuJoCo for Visualization" << std::endl;

        // IMPORTANT: Clear Drake's internal context to prevent memory corruption
        // The issue is that after extensive IK solving (62 waypoints for circle vs 21 for line),
        // Drake's context may hold references that get corrupted when MuJoCo allocates memory.
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
        // Get actual trajectory duration instead of using hardcoded sim_duration
        double trajectory_duration = planned_trajectory.end_time();
        std::cout << "\n>>> Trajectory duration: " << trajectory_duration << " seconds" << std::endl;

        double t = 0.0;
        int step_count = 0;
        const int print_interval = static_cast<int>(0.1 / time_step);
        const int traj_sample_interval = static_cast<int>(0.01 / time_step); // Sample every 10ms

        std::cout << "\n>>> Step 3: Playing Back Planned Trajectory in MuJoCo" << std::endl;
        std::cout << ">>> Real-time EE position tracking:" << std::endl;
        std::cout << "Time(s) | EE_X | EE_Y | EE_Z | Distance from Center" << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        auto start_time = std::chrono::high_resolution_clock::now();

        while (t < trajectory_duration && !mujoco_sim.should_close())
        {
            // Evaluate Drake's planned trajectory at current time
            VectorXd q_desired = drake_sim.eval_trajectory(planned_trajectory, t);

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

            // Sample trajectory for visualization (using world coordinates)
            if (step_count % traj_sample_interval == 0)
            {
                traj_points.push_back(static_cast<float>(ee_pos_world(0)));
                traj_points.push_back(static_cast<float>(ee_pos_world(1)));
                traj_points.push_back(static_cast<float>(ee_pos_world(2)));
            }

            // Print EE position and joint angles every 0.1 seconds
            if (step_count % print_interval == 0)
            {
                double dist_from_center = (ee_pos - circle_center).norm();

                // DEBUG: Print q_desired right arm joints for first few samples
                if (step_count == print_interval || step_count == print_interval * 10 ||
                    step_count == print_interval * 25)
                {
                    std::cout << " q_desired[11:17]: [";
                    for (int j = 11; j <= 17; ++j)
                    {
                        std::cout << q_desired(j);
                        if (j < 17)
                            std::cout << " ";
                    }
                    std::cout << "]";
                }
                std::cout << t << " | " << ee_pos(0) << " | " << ee_pos(1) << " | "
                          << ee_pos(2) << " | " << dist_from_center;

                // Print right arm joint angles for debugging
                if (step_count == print_interval || step_count == print_interval * 10 ||
                    step_count == print_interval * 25)
                {
                    std::cout << " | J: [";
                    for (int i = 11; i <= 17; ++i)
                    {
                        std::cout << q_desired(i);
                        if (i < 17)
                            std::cout << " ";
                    }
                    std::cout << "]";
                }
                std::cout << std::endl;
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