cmake_minimum_required(VERSION 4.1.2)
project(force_control)

# 设置 C++ 标准和构建类型
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# 添加 Boost MPL 限制宏定义
add_definitions(
    -DBOOST_MPL_LIMIT_LIST_SIZE=50
    -DBOOST_MPL_LIMIT_VECTOR_SIZE=50
    -DBOOST_MPL_LIMIT_MAP_SIZE=50
    -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
)

# 定义第三方库路径
set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/thirdparty/install")
set(MUJOCO_PATH "${CMAKE_SOURCE_DIR}/thirdparty/install/mujoco")

# 查找依赖包
set(Python_ADDITIONAL_VERSIONS 3.10)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigenpy REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glfw3 REQUIRED)
# find_package(qpOASES REQUIRED)
find_package(OpenGL REQUIRED) # 查找 OpenGL
# find_package(hpp-fcl REQUIRED)


# set(COAL_LIBRARY "${CMAKE_SOURCE_DIR}/thirdparty/install/lib/libcoal.so")
# if(EXISTS ${COAL_LIBRARY})
#     message(STATUS "Found coal library: ${COAL_LIBRARY}")
# else()
#     message(FATAL_ERROR "Could not find coal library at: ${COAL_LIBRARY}")
# endif()

# 动态查找 mujoco 库
find_library(MUJOCO_LIBRARY
    NAMES mujoco
    PATHS ${MUJOCO_PATH}/lib
    NO_DEFAULT_PATH
)
if(NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "无法找到 mujoco 库，请检查路径: ${MUJOCO_PATH}/lib")
endif()

# 添加 ImGui 核心源文件
# set(IMGUI_SOURCES
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui.cpp
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_demo.cpp
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_draw.cpp
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_tables.cpp
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_widgets.cpp
# )

# # 添加 ImGui 后端源文件
# set(IMGUI_BACKEND_SOURCES
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.cpp
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3.cpp
# )

# # sim_interface 库
# file(GLOB SIM_SRC
#     "${CMAKE_SOURCE_DIR}/sim_interface/src/*.cpp"
# )
# add_library(sim_interface STATIC ${SIM_SRC})

# target_sources(sim_interface PRIVATE
#     ${IMGUI_SOURCES}
#     ${IMGUI_BACKEND_SOURCES}
# )
# target_include_directories(sim_interface PUBLIC
#     ${MUJOCO_PATH}/include
#     ${CMAKE_SOURCE_DIR}/sim_interface/include
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui
#     ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
# )
# target_link_libraries(sim_interface PUBLIC
#     ${MUJOCO_LIBRARY}
#     glfw
#     OpenGL::GL
# )


# control_algorithm
# file(GLOB CON_SRC ${CMAKE_SOURCE_DIR}/control_algorithm/src/*.cpp)
# add_library(control_algorithm STATIC ${CON_SRC})
# target_include_directories(control_algorithm PUBLIC
#     ${MUJOCO_PATH}/include
#     ${CMAKE_SOURCE_DIR}/sim_interface/include
#     ${CMAKE_SOURCE_DIR}/control_algorithm/include
#     ${CMAKE_SOURCE_DIR}/thirdparty/install/include/eigen3
#     ${CMAKE_SOURCE_DIR}/thirdparty/json/include/nlohmann
#     ${CMAKE_SOURCE_DIR}/thirdparty/install/include  # 添加这个以包含 coal 头文件

# )

# # 链接所有必要的库
# target_link_libraries(control_algorithm PUBLIC
#     pinocchio::pinocchio
#     eigenpy::eigenpy
#     ${Python3_LIBRARIES}
#     hpp-fcl::hpp-fcl
#     ${COAL_LIBRARY}  # 添加 coal 库
# )

# 可执行文件
# add_executable(demo_mujoco ${CMAKE_SOURCE_DIR}/demo/demo_mujoco.cpp)
# target_link_libraries(demo_mujoco PRIVATE
#     sim_interface
#     control_algorithm
#     pinocchio::pinocchio
#     eigenpy::eigenpy
#     ${Python3_LIBRARIES}
# )

# add_executable(demo_IK ${CMAKE_SOURCE_DIR}/demo/demo_IK.cpp)
# target_link_libraries(demo_IK PRIVATE
#     sim_interface
#     control_algorithm
#     pinocchio::pinocchio
#     eigenpy::eigenpy
#     ${Python3_LIBRARIES}
#     hpp-fcl::hpp-fcl
#     ${COAL_LIBRARY}  # 添加 coal 库
# )

# add_executable(demo_mujoco_trajectory ${CMAKE_SOURCE_DIR}/demo/demo_mujoco_trajectory.cpp)
# target_link_libraries(demo_mujoco_trajectory PRIVATE
#     sim_interface
#     control_algorithm
#     pinocchio::pinocchio
#     eigenpy::eigenpy
#     ${Python3_LIBRARIES}
#     hpp-fcl::hpp-fcl
#     ${COAL_LIBRARY}  # 添加 coal 库
# )

# ========================================
# Drake + MuJoCo Co-Simulation Demo
# ========================================

# 设置 Drake 路径
set(DRAKE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/install/drake")

# 查找 Drake
find_package(Drake REQUIRED CONFIG
    HINTS ${DRAKE_DIR}/lib/cmake/drake
    NO_DEFAULT_PATH
)

message(STATUS "Found Drake: ${DRAKE_DIR}")

# 创建 Drake + MuJoCo 联合仿真可执行文件
add_executable(demo_drake_mujoco_cosim ${CMAKE_SOURCE_DIR}/demo/demo_drake_mujoco_cosim.cpp)

# 设置 C++20 标准 (Drake 要求)
target_compile_features(demo_drake_mujoco_cosim PRIVATE cxx_std_20)

# 设置包含目录
message(STATUS "MUJOCO_PATH: ${MUJOCO_PATH}")
message(STATUS "MUJOCO_INCLUDE: ${MUJOCO_PATH}/include")
target_include_directories(demo_drake_mujoco_cosim PRIVATE
    ${MUJOCO_PATH}/include
    ${CMAKE_SOURCE_DIR}/thirdparty/install/include
)

# 链接库 (注意: 使用小写 drake::drake)
target_link_libraries(demo_drake_mujoco_cosim PRIVATE
    drake::drake
    ${MUJOCO_LIBRARY}
    glfw
    OpenGL::GL
    GL
)

message(STATUS "Drake + MuJoCo co-simulation demo configured")
