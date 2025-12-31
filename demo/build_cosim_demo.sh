#!/bin/bash
# Build script for Drake + MuJoCo co-simulation demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

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

# Handle build directory
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
else
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"/*
fi

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
echo "To run the demo, use: ./run_cosim_demo.sh"
