#!/bin/bash
# Build script for Drake + MuJoCo co-simulation demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================"
echo "Building Drake + MuJoCo Co-Sim Demo"
echo "========================================"
echo ""

# Source environment
if [ -f "$PROJECT_DIR/thirdparty/setup_env.sh" ]; then
    echo "Sourcing environment variables..."
    source "$PROJECT_DIR/thirdparty/setup_env.sh"
else
    echo "Warning: setup_env.sh not found. Make sure thirdparty libraries are installed."
fi

# Create build directory
BUILD_DIR="$PROJECT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo ""
echo "Configuring CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Building demo_drake_mujoco_cosim..."
make -j$(nproc) demo_drake_mujoco_cosim

echo ""
echo "========================================"
echo "Build Complete!"
echo "========================================"
echo ""
echo "Executable: $BUILD_DIR/demo_drake_mujoco_cosim"
echo ""
echo "Usage:"
echo "  $BUILD_DIR/demo_drake_mujoco_cosim [duration] [timestep]"
echo ""
echo "Example:"
echo "  $BUILD_DIR/demo_drake_mujoco_cosim 1.0 0.001"
echo "    - Run for 1.0 second with 1ms timestep"
