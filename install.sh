#!/bin/bash
# 自动安装脚本，彻底解决 Pinocchio、EigenPy、hpp-fcl 等依赖构建时 jrl-cmakemodules Doxygen 文档丢失错误   
# 支持文档关闭、无需安装无用 doxygen-html

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRDPARTY_DIR="$SCRIPT_DIR/thirdparty"
INSTALL_DIR="$THIRDPARTY_DIR/install"
BUILD_DIR="$THIRDPARTY_DIR/build"
LOG_FILE="$THIRDPARTY_DIR/install.log"

CMAKE_VERSION="4.1.2"
EIGEN_VERSION="3.4.0"
PINOCCHIO_VERSION="v3.8.0"
HPP_FCL_VERSION="v3.0.0"
EIGENPY_VERSION="v3.10.0"

# ----- 基础工具 -----
log(){ echo -e "[$(date +'%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"; }
error(){ log "ERROR: $1" >&2; exit 1; }
command_exists(){ command -v "$1" &>/dev/null; }

wait_for_apt_lock(){
    log "检查 APT 锁状态..."
    local max_retries=20 retry_delay=2 retries=0
    while [ $retries -lt $max_retries ]; do
        if sudo lsof /var/lib/dpkg/lock-frontend &>/dev/null; then
            log "检测到 APT 锁，等待 $retry_delay 秒后重试 ($((retries+1))/$max_retries)"
            sleep $retry_delay; ((retries++))
        else
            log "APT 锁已释放"
            return 0
        fi
    done
    error "APT 锁长时间占用，请检查 dpkg/apt"
}

setup_dirs(){
    log "=== 初始化目录 ==="
    mkdir -p "$THIRDPARTY_DIR" "$INSTALL_DIR" "$BUILD_DIR"
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    : > "$LOG_FILE"
    log "安装目录: $THIRDPARTY_DIR, 安装前缀: $INSTALL_DIR"
}

install_sys_deps(){
    log "=== 安装系统依赖 ==="
    [ -f /etc/os-release ] && . /etc/os-release
    [ "${ID-}" != "ubuntu" ] && error "仅支持 Ubuntu 当前: $ID"
    wait_for_apt_lock
    log "更新 APT 缓存..."
    sudo apt-get update || error "APT 缓存更新失败"
    log "安装依赖..."
    pkgs=(build-essential git pkg-config wget curl python3 python3-dev python3-pip libssl-dev doxygen graphviz)
    devlibs=(libeigen3-dev libboost-system-dev libboost-filesystem-dev libboost-thread-dev liburdfdom-dev libassimp-dev libccd-dev liboctomap-dev libqhull-dev libtinyxml2-dev libltdl-dev libopenblas-dev libsuitesparse-dev freeglut3-dev)
    # IPOPT dependencies for non-convex optimization (GCS with Bézier curves)
    ipopt_deps=(coinor-libcoinutils-dev coinor-libosi-dev coinor-libclp-dev coinor-libcgl-dev coinor-libipopt-dev)
    sudo apt-get install -y "${pkgs[@]}" "${devlibs[@]}" "${ipopt_deps[@]}" || error "系统依赖安装失败"
    python3 -m pip install --upgrade pip
    python3 -m pip install --upgrade numpy matplotlib imageio pyglet lxml
    log "✓ 系统依赖完成 (含IPOPT)"
}

# ensure_cmake(){
#     log "=== 检查或安装 CMake $CMAKE_VERSION ==="
#     export PATH="$INSTALL_DIR/bin:$PATH"
#     if command_exists "$INSTALL_DIR/bin/cmake"; then 
#         log "✓ 已有 CMake $($INSTALL_DIR/bin/cmake --version)"
#         return 0
#     fi
#     if command_exists cmake; then
#         local system_cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
#         if [[ "$system_cmake_version" > "4.1.0" ]]; then
#             export PATH="$(dirname "$(command -v cmake)"):$PATH"
#             log "✓ 系统兼容 CMake: $system_cmake_version"
#             return 0
#         fi
#     fi
#     log "下载并安装 CMake ..."
#     mkdir -p "$INSTALL_DIR"
#     cd "$THIRDPARTY_DIR"
#     local tar="cmake-$CMAKE_VERSION-linux-x86_64.tar.gz"
#     local url="https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/$tar"
#     wget "$url" -O "$tar" || error "CMake 下载失败"
#     tar -xzf "$tar"
#     cp -r cmake-$CMAKE_VERSION-linux-x86_64/* "$INSTALL_DIR/"
#     rm -rf cmake-$CMAKE_VERSION-linux-x86_64
#     export PATH="$INSTALL_DIR/bin:$PATH"
#     command_exists cmake || error "CMake 未正常安装"
#     log "当前使用的 CMake: $(which cmake) -> $(cmake --version | head -n1)"
# }

ensure_cmake(){
    log "=== 检查或安装 CMake $CMAKE_VERSION ==="
    export PATH="$INSTALL_DIR/bin:$PATH"
    if command_exists "$INSTALL_DIR/bin/cmake"; then 
        log "✓ 已有 CMake $($INSTALL_DIR/bin/cmake --version)"
        return 0
    fi
    if command_exists cmake; then
        local system_cmake_version=$(cmake --version | head -n1 | awk '{print $3}')
        if [[ "$system_cmake_version" > "4.1.0" ]]; then
            log "⚠️ 系统兼容 CMake: $system_cmake_version，但将优先使用本地版本"
        fi
    fi
    log "下载并安装 CMake ..."
    mkdir -p "$INSTALL_DIR"
    cd "$THIRDPARTY_DIR"
    local tar="cmake-$CMAKE_VERSION-linux-x86_64.tar.gz"
    local url="https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/$tar"
    wget "$url" -O "$tar" || error "CMake 下载失败"
    tar -xzf "$tar"
    cp -r cmake-$CMAKE_VERSION-linux-x86_64/* "$INSTALL_DIR/"
    rm -rf cmake-$CMAKE_VERSION-linux-x86_64
    export PATH="$INSTALL_DIR/bin:$PATH"
    command_exists cmake || error "CMake 未正常安装"
    log "当前使用的 CMake: $(which cmake) -> $(cmake --version | head -n1)"
}

fix_cmakelists_minver(){
    find "$THIRDPARTY_DIR" -type f -name 'CMakeLists.txt' | while read -r f; do
        if grep -qi 'cmake_minimum_required' "$f"; then
            sed -i 's|cmake_minimum_required.*|cmake_minimum_required(VERSION 3.22)|g' "$f"
        else
            sed -i '1i cmake_minimum_required(VERSION 3.22)' "$f"
        fi
    done
}

download_source(){
    if [ "$#" -ne 3 ]; then error "download_source 必须3参数 name url tag"; fi
    local name="$1" url="$2" tag="$3"
    local dir="$THIRDPARTY_DIR/$name"
    log "=> 获取 $name ($tag)"
    [ -d "$dir" ] && rm -rf "$dir"
    git clone --depth 1 --branch "$tag" "$url" "$dir" || error "$name 克隆失败 $url"
    cd "$dir"
    if [ "$name" = "pinocchio" ]; then log "初始化 $name 子模块"; git submodule update --init --recursive || log "警告：pinocchio 子模块初始化异常"; fi
    cd "$THIRDPARTY_DIR"
}

download_eigen_latest(){
    log "=== 下载并安装 Eigen $EIGEN_VERSION ==="
    cd "$THIRDPARTY_DIR"
    local url="https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.tar.gz"
    local tar="eigen-$EIGEN_VERSION.tar.gz"
    [ -d eigen-$EIGEN_VERSION ] && rm -rf eigen-$EIGEN_VERSION
    wget "$url" -O "$tar" || error "Eigen 下载失败"
    tar -xzf "$tar"
    mkdir -p eigen-$EIGEN_VERSION/build && cd eigen-$EIGEN_VERSION/build
    cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
    make -j2 install
    cd "$THIRDPARTY_DIR"
    log "✓ Eigen $EIGEN_VERSION 安装成功"
}

# ----- 构建EigenPy、hpp-fcl、Pinocchio只安装必要部分，关闭所有文档 -----

install_qpoases(){
    log "=== 安装 qpOASES releases/3.2.2 (关闭文档) ==="
    download_source "qpOASES" "https://github.com/coin-or/qpOASES.git" "releases/3.2.2"
    fix_cmakelists_minver
    cd "$THIRDPARTY_DIR/qpOASES"
    mkdir -p build && cd build
    local cmake_cmd=$(which cmake)
    log "qpOASES CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    make -j$(nproc) -k install
    cd "$THIRDPARTY_DIR"
    log "✓ qpOASES 安装成功"
}

install_topp_ra(){
    log "=== 安装 TOPP-RA v0.6.2 (关闭文档) ==="
    local name="topp-ra"
    local url="https://github.com/hungpham2511/toppra.git"
    local tag="v0.6.2"
    local dir="$THIRDPARTY_DIR/$name"

    # 检测是否已存在对应文件夹
    if [ -d "$dir" ]; then
        log "✓ $name 已存在，跳过下载和克隆"
    else
        download_source "$name" "$url" "$tag"
        fix_cmakelists_minver
    fi

    cd "$dir/cpp"
    mkdir -p build && cd build
    local cmake_cmd=$(which cmake)
    log "TOPP-RA CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    make -j$(nproc) -k install
    cd "$THIRDPARTY_DIR"
    log "✓ TOPP-RA v0.6.2 安装成功"
}

install_json(){
    log "=== 安装 JSON v3.12.0 (关闭文档) ==="
    local name="json"
    local url="https://github.com/nlohmann/json.git"
    local tag="v3.12.0"
    local dir="$THIRDPARTY_DIR/$name"

    # 检测是否已存在对应文件夹
    if [ -d "$dir" ]; then
        log "✓ $name 已存在，跳过下载和克隆"
    else
        download_source "$name" "$url" "$tag"
        fix_cmakelists_minver
    fi

    cd "$dir"
    mkdir -p build && cd build
    local cmake_cmd=$(which cmake)
    log "JSON CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    make -j$(nproc) -k install
    cd "$THIRDPARTY_DIR"
    log "✓ JSON v3.12.0 安装成功"
}

install_mujoco() {
    log "=== 安装 mujoco 3.3.7 (二进制) ==="
    local version="3.3.7"
    local platform="linux-x86_64"
    local tar="mujoco-${version}-${platform}.tar.gz"
    local url="https://github.com/google-deepmind/mujoco/releases/download/${version}/${tar}"
    local sha256_url="https://github.com/google-deepmind/mujoco/releases/download/${version}/${tar}.sha256"
    local sha256=$(wget -qO- "$sha256_url" | awk '{print $1}')

    cd "$THIRDPARTY_DIR"
    [ -f "$tar" ] || wget "$url" -O "$tar" || error "MuJoCo 二进制下载失败"

    # 校验 SHA256
    echo "${sha256}  $tar" | sha256sum --check || error "SHA256 校验失败"

    # 解压到安装目录
    mkdir -p "$INSTALL_DIR"
    tar -xzf "$tar" -C "$INSTALL_DIR" || error "MuJoCo 解压失败"
    mv "$INSTALL_DIR/mujoco-${version}" "$INSTALL_DIR/mujoco" || error "MuJoCo 目录重命名失败"

    log "✓ MuJoCo 二进制安装成功"
}

install_drake() {
    log "=== 编译安装 Drake v1.47.0 (配置 MuJoCo 后端) ==="
    local drake_version="v1.47.0"
    local drake_src_dir="$THIRDPARTY_DIR/drake_source"
    local drake_install_dir="$INSTALL_DIR/drake"
    local mujoco_dir="$INSTALL_DIR/mujoco"

    # 检查 MuJoCo 是否已安装
    if [ ! -d "$mujoco_dir" ]; then
        error "MuJoCo 未找到，请先安装 MuJoCo"
    fi

    # 创建目录
    mkdir -p "$drake_src_dir"
    mkdir -p "$drake_install_dir"

    # 克隆 Drake 源码
    log "=> 获取 Drake 源码 ($drake_version)..."
    if [ ! -d "$drake_src_dir/.git" ]; then
        git clone https://github.com/RobotLocomotion/drake.git "$drake_src_dir" || error "Drake 克隆失败"
    else
        log "✓ Drake 源码已存在，跳过克隆"
    fi

    cd "$drake_src_dir"

    # 更新源码
    log "=> 检出 Drake $drake_version 及更新子模块..."
    git fetch origin
    git checkout "$drake_version" || error "Drake 切换到 $drake_version 失败"
    git submodule update --init --recursive || error "Drake 子模块初始化失败"

    # 创建构建目录
    local build_dir="$drake_src_dir/build"
    mkdir -p "$build_dir"
    cd "$build_dir"

    local cmake_cmd=$(which cmake)
    local python_exec=$(which python3)

    log "=> 配置 CMake (MuJoCo 后端 + IPOPT 求解器)..."
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$drake_install_dir" \
        -DWITH_MUJOCO=ON \
        -DMUJOCO_PATH="$mujoco_dir" \
        -DWITH_MOSEK=OFF \
        -DWITH_GUROBI=OFF \
        -DWITH_SNOPT=OFF \
        -DWITH_IPOPT=ON \
        -DPython_EXECUTABLE="$python_exec" \
        -DCMAKE_VERBOSE_MAKEFILE=ON || error "Drake CMake 配置失败"

    log "=> 编译 Drake..."
    make -j$(nproc) || error "Drake 编译失败"

    log "=> 安装到: $drake_install_dir"
    make install || error "Drake 安装失败"

    cd "$THIRDPARTY_DIR"
    log "✓ Drake 编译安装成功"
}



# install_imgui(){
#     log "=== 安装 ImGui v1.92.0 ==="
#     local name="imgui"
#     local url="https://github.com/ocornut/imgui.git"
#     local tag="v1.92.0"
#     local dir="$THIRDPARTY_DIR/$name"

#     # 检测是否已存在对应文件夹
#     if [ -d "$dir" ]; then
#         log "✓ $name 已存在，跳过下载和克隆"
#     else
#         download_source "$name" "$url" "$tag"
#         fix_cmakelists_minver
#     fi

#     cd "$dir"
#     mkdir -p build && cd build
#     local cmake_cmd=$(which cmake)
#     log "ImGui CMake 路径: $cmake_cmd"
#     "$cmake_cmd" .. \
#         -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
#         -DBUILD_SHARED_LIBS=ON \
#         -DIMGUI_BUILD_EXAMPLES=OFF \
#         -DIMGUI_BUILD_TESTS=OFF \
#         -DCMAKE_VERBOSE_MAKEFILE=ON
#     make -j$(nproc) -k install
#     cd "$THIRDPARTY_DIR"
#     log "✓ ImGui v1.92.0 安装成功"
# }

install_imgui(){
    log "=== 安装 ImGui v1.92.0 ==="
    local name="imgui"
    local url="https://github.com/ocornut/imgui.git"
    local tag="v1.92.0"
    local dir="$THIRDPARTY_DIR/$name"

    # 检测是否已存在对应文件夹
    if [ -d "$dir" ]; then
        log "✓ $name 已存在，跳过下载和克隆"
    else
        download_source "$name" "$url" "$tag"
    fi

    # 创建 CMakeLists.txt (ImGui 原仓库不包含主 CMakeLists.txt)
    log "创建 ImGui CMakeLists.txt"
    cat > "$dir/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.22)
project(imgui CXX)

# ImGui as a library
add_library(imgui STATIC
    imgui.cpp
    imgui_draw.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Installation
include(GNUInstallDirs)
install(TARGETS imgui
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES
    imgui.h
    imgui_internal.h
    imconfig.h
    imstb_rectpack.h
    imstb_textedit.h
    imstb_truetype.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/imgui
)
EOF

    cd "$dir"
    mkdir -p build && cd build
    local cmake_cmd=$(which cmake)
    log "ImGui CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    make -j$(nproc) install
    cd "$THIRDPARTY_DIR"
    log "✓ ImGui v1.92.0 安装成功"
}

install_eigenpy(){
    log "=== 安装 EigenPy $EIGENPY_VERSION (关闭文档) ==="
    download_source "eigen-py" "https://github.com/stack-of-tasks/eigenpy.git" "$EIGENPY_VERSION"
    fix_cmakelists_minver
    cd "$THIRDPARTY_DIR/eigen-py"
    mkdir -p build && cd build
    local python_exec=$(which python3)
    local cmake_cmd=$(which cmake)
    log "EigenPy CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DPYTHON_EXECUTABLE="$python_exec" \
        -DBUILD_TESTING=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DUSE_SYSTEM_CMAKE_MODULES=ON
    make -j2 install
    cd "$THIRDPARTY_DIR"
    log "✓ EigenPy 安装成功"
}

install_hpp_fcl(){
    log "=== 安装 hpp-fcl $HPP_FCL_VERSION (关闭文档) ==="
    download_source "hpp-fcl" "https://github.com/humanoid-path-planner/hpp-fcl.git" "$HPP_FCL_VERSION"
    fix_cmakelists_minver
    cd "$THIRDPARTY_DIR/hpp-fcl"
    mkdir -p build && cd build
    local python_exec=$(which python3)
    # local cmake_cmd=$(which cmake)
    local cmake_cmd="$INSTALL_DIR/bin/cmake"
    log "hpp-fcl CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DBUILD_PYTHON_INTERFACE=ON \
        -DPYTHON_EXECUTABLE="$python_exec" \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DBUILD_TESTING=OFF \
        -DCMAKE_PREFIX_PATH="$INSTALL_DIR" \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    make -j2 install
    cd "$THIRDPARTY_DIR"
    log "✓ hpp-fcl 安装成功"
}

install_pinocchio(){
    log "=== 安装 Pinocchio $PINOCCHIO_VERSION (关闭文档) ==="
    download_source "pinocchio" "https://github.com/stack-of-tasks/pinocchio.git" "$PINOCCHIO_VERSION"
    fix_cmakelists_minver
    cd "$THIRDPARTY_DIR/pinocchio"
    mkdir -p build && cd build
    local python_exec=$(which python3)
    local cmake_cmd=$(which cmake)
    log "Pinocchio CMake 路径: $cmake_cmd"
    "$cmake_cmd" .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DCMAKE_PREFIX_PATH="$INSTALL_DIR" \
        -DBUILD_WITH_URDF_SUPPORT=ON \
        -DBUILD_WITH_COLLISION_SUPPORT=ON \
        -DBUILD_WITH_HPP_FCL=ON \
        -DBUILD_PYTHON_INTERFACE=ON \
        -DPYTHON_EXECUTABLE="$python_exec" \
        -DBUILD_DOCUMENTATION=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
        -DBUILD_TESTING=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_WITH_EXAMPLES=OFF \
        -DCMAKE_CXX_FLAGS="-O2 -march=native" \
        -DCMAKE_VERBOSE_MAKEFILE=ON
    grep -q "BUILD_WITH_URDF_SUPPORT:BOOL=ON" CMakeCache.txt || error "Pinocchio 必须开启 URDF 支持"
    grep -q "BUILD_WITH_COLLISION_SUPPORT:BOOL=ON" CMakeCache.txt || error "Pinocchio 必须开启碰撞检测"
    make -j2 install
    cd "$THIRDPARTY_DIR"
    log "✓ Pinocchio 安装成功"
}

verify_installation(){
    log "=== 验证安装 ==="
    cat > "$THIRDPARTY_DIR/test_pinocchio.cpp" << EOF
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <hpp/fcl/collision_object.h>
#include <iostream>
int main(){
    std::cout << "=== Pinocchio 安装验证 ===" << std::endl;
    pinocchio::Model model;
    pinocchio::JointModelFreeFlyer joint_placement;
    pinocchio::FrameIndex base_frame_id = model.addJoint(0, joint_placement, pinocchio::SE3::Identity(), "root_joint");
    std::cout << "✓ 模型创建成功\\n  关节数量: " << model.njoints << "\\n  自由度: " << model.nv << std::endl;
    pinocchio::Data data(model);
    Eigen::VectorXd q = pinocchio::neutral(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    std::cout << "✓ 正向运动学计算成功" << std::endl;
    Eigen::MatrixXd J(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q, base_frame_id, J);
    std::cout << "✓ 雅可比计算成功\\n  雅可比矩阵维度: " << J.rows() << " x " << J.cols() << std::endl;
    pinocchio::GeometryModel geometry_model; pinocchio::GeometryData geometry_data(geometry_model);
    std::cout << "✓ 碰撞检测数据结构创建成功" << std::endl;
    auto sphere = std::make_shared<hpp::fcl::Sphere>(0.1);
    hpp::fcl::CollisionObject collision_object(sphere);
    std::cout << "✓ hpp-fcl 集成测试成功" << std::endl;
    std::cout << "🎉 安装验证通过！" << std::endl;
    return 0;
}
EOF
    g++ -std=c++17 -I"$INSTALL_DIR/include" -I/usr/include/eigen3 -L"$INSTALL_DIR/lib" -Wl,-rpath,"$INSTALL_DIR/lib" "$THIRDPARTY_DIR/test_pinocchio.cpp" -lpinocchio -lhpp-fcl -lboost_system -lboost_filesystem -o "$THIRDPARTY_DIR/test_pinocchio" || error "测试程序编译失败"
    log "运行测试程序..."
    LD_LIBRARY_PATH="$INSTALL_DIR/lib:$LD_LIBRARY_PATH" "$THIRDPARTY_DIR/test_pinocchio"
}

# create_setup_env(){
#     log "=== 创建环境设置脚本 ==="
#     cat > "$THIRDPARTY_DIR/setup_env.sh" << EOF
# #!/bin/bash
# SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
# INSTALL_DIR="\$SCRIPT_DIR/install"
# CMAKE_BIN_DIR="\$INSTALL_DIR/bin"
# [ -d "\$CMAKE_BIN_DIR" ] && export PATH="\$CMAKE_BIN_DIR:\$PATH"
# export PINOCCHIO_ROOT="\$INSTALL_DIR"
# export CMAKE_PREFIX_PATH="\$INSTALL_DIR:\$CMAKE_PREFIX_PATH"

# # 添加 mujoco 的库路径
# MUJOCO_LIB_DIR="\$INSTALL_DIR/mujoco/lib"
# [ -d "\$MUJOCO_LIB_DIR" ] && export LD_LIBRARY_PATH="\$MUJOCO_LIB_DIR:\$LD_LIBRARY_PATH"

# # 添加 mujoco 的头文件路径
# MUJOCO_INCLUDE_DIR="\$INSTALL_DIR/mujoco/include"
# [ -d "\$MUJOCO_INCLUDE_DIR" ] && export CPLUS_INCLUDE_PATH="\$MUJOCO_INCLUDE_DIR:\$CPLUS_INCLUDE_PATH"

# export PYTHONPATH="\$INSTALL_DIR/lib/python3/dist-packages:\$INSTALL_DIR/lib/python3/site-packages:\$PYTHONPATH"
# export PKG_CONFIG_PATH="\$INSTALL_DIR/lib/pkgconfig:\$PKG_CONFIG_PATH"
# export CPLUS_INCLUDE_PATH="\$INSTALL_DIR/include:\$CPLUS_INCLUDE_PATH"
# [ -z "\${PINOCCHIO_SETUP_ENV:-}" ] && export PINOCCHIO_SETUP_ENV=1 && echo "Pinocchio 环境变量已设置: \$INSTALL_DIR"
# EOF
#     chmod +x "$THIRDPARTY_DIR/setup_env.sh"
#     log "✓ 环境设置脚本创建成功"
# }

create_setup_env(){
    log "=== 创建环境设置脚本 ==="
    cat > "$THIRDPARTY_DIR/setup_env.sh" << EOF
#!/bin/bash
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="\$SCRIPT_DIR/install"

# 添加 CMake 和第三方工具的 bin 路径
CMAKE_BIN_DIR="\$INSTALL_DIR/bin"
[ -d "\$CMAKE_BIN_DIR" ] && export PATH="\$CMAKE_BIN_DIR:\$PATH"

# 设置 Pinocchio 根目录
export PINOCCHIO_ROOT="\$INSTALL_DIR"
export CMAKE_PREFIX_PATH="\$INSTALL_DIR:\$CMAKE_PREFIX_PATH"

# 添加第三方库路径到 LD_LIBRARY_PATH
THIRDPARTY_LIB_DIR="\$INSTALL_DIR/lib"
[ -d "\$THIRDPARTY_LIB_DIR" ] && export LD_LIBRARY_PATH="\$THIRDPARTY_LIB_DIR:\$LD_LIBRARY_PATH"

# 添加 MuJoCo 的库路径和头文件路径
MUJOCO_LIB_DIR="\$INSTALL_DIR/mujoco/lib"
MUJOCO_INCLUDE_DIR="\$INSTALL_DIR/mujoco/include"
[ -d "\$MUJOCO_LIB_DIR" ] && export LD_LIBRARY_PATH="\$MUJOCO_LIB_DIR:\$LD_LIBRARY_PATH"
[ -d "\$MUJOCO_INCLUDE_DIR" ] && export CPLUS_INCLUDE_PATH="\$MUJOCO_INCLUDE_DIR:\$CPLUS_INCLUDE_PATH"

# 添加 Drake 相关环境变量
DRAKE_DIR="\$INSTALL_DIR/drake"
if [ -d "\$DRAKE_DIR" ]; then
    export DRAKE_DIR="\$DRAKE_DIR"
    [ -d "\$DRAKE_DIR/bin" ] && export PATH="\$DRAKE_DIR/bin:\$PATH"
    [ -d "\$DRAKE_DIR/lib" ] && export LD_LIBRARY_PATH="\$DRAKE_DIR/lib:\$LD_LIBRARY_PATH"
    [ -d "\$DRAKE_DIR/lib/python3.10/site-packages" ] && export PYTHONPATH="\$DRAKE_DIR/lib/python3.10/site-packages:\$PYTHONPATH"
    [ -d "\$DRAKE_DIR/lib/cmake" ] && export CMAKE_PREFIX_PATH="\$DRAKE_DIR/lib/cmake:\$CMAKE_PREFIX_PATH"
fi

# 添加 Python 包路径
export PYTHONPATH="\$INSTALL_DIR/lib/python3/dist-packages:\$INSTALL_DIR/lib/python3/site-packages:\$PYTHONPATH"

# 添加 pkg-config 和 C++ 头文件路径
export PKG_CONFIG_PATH="\$INSTALL_DIR/lib/pkgconfig:\$PKG_CONFIG_PATH"
export CPLUS_INCLUDE_PATH="\$INSTALL_DIR/include:\$CPLUS_INCLUDE_PATH"

# 提示环境已设置
[ -z "\${PINOCCHIO_SETUP_ENV:-}" ] && export PINOCCHIO_SETUP_ENV=1 && echo "Drake+MuJoCo 环境变量已设置: \$INSTALL_DIR"
EOF
    chmod +x "$THIRDPARTY_DIR/setup_env.sh"
    log "✓ 环境设置脚本创建成功"
}

main(){
    log "=== 自动构建流程启动 ==="
    # unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
    setup_dirs
    ensure_cmake


    install_sys_deps
    # install_mujoco
    install_drake

    # download_eigen_latest
    # install_qpoases

    # install_eigenpy
    # install_hpp_fcl
    # install_pinocchio
    # verify_installation
    # install_json
    # install_imgui 
    # install_topp_ra
    create_setup_env
    log "=== 全部安装完成，请运行: source $THIRDPARTY_DIR/setup_env.sh"
    log "详细日志: $LOG_FILE"
}

main