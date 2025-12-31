#!/bin/bash
# Run script for Drake + MuJoCo co-simulation demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
EXECUTABLE="$BUILD_DIR/demo_drake_mujoco_cosim"

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable not found at $EXECUTABLE"
    echo "Please run ./build_cosim_demo.sh first to build the demo."
    exit 1
fi

# Show usage if --help or -h
if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "========================================"
    echo "Drake + MuJoCo Co-Sim Demo"
    echo "========================================"
    echo ""
    echo "Usage: $0 [trajectory_type] [duration] [timestep] [options]"
    echo ""
    echo "All parameters are OPTIONAL (defaults shown in parentheses)"
    echo ""
    echo "Trajectory Types:"
    echo "  circular   - Circular trajectory (default)"
    echo "  line       - Linear trajectory"
    echo "  waypoint   - Waypoint trajectory (A → B) with collision checking"
    echo "  avoid      - Obstacle avoidance with automatic path finding"
    echo ""
    echo "Parameters:"
    echo "  duration   - Simulation duration in seconds (default: 5.0)"
    echo "  timestep   - Time step in seconds (default: 0.001)"
    echo ""
    echo "Options:"
    echo "  --drake        - Use Drake + MuJoCo co-simulation (default: MuJoCo only)"
    echo "  --no-visual    - Disable visualization window (headless mode)"
    echo ""
    echo "Examples:"
    echo "  $0"
    echo "    - Run with defaults: circular trajectory, 5.0s, 1ms timestep"
    echo ""
    echo "  $0 line"
    echo "    - Linear trajectory with default duration and timestep"
    echo ""
    echo "  $0 circular 10.0"
    echo "    - Circular trajectory for 10 seconds"
    echo ""
    echo "  $0 waypoint 4.0 0.001 --no-visual"
    echo "    - Waypoint trajectory without visualization"
    echo ""
    exit 0
fi

echo "========================================"
echo "Running Drake + MuJoCo Co-Sim Demo"
echo "========================================"
echo ""

# Source environment
if [ -f "$PROJECT_DIR/thirdparty/setup_env.sh" ]; then
    echo "Sourcing environment variables..."
    source "$PROJECT_DIR/thirdparty/setup_env.sh"
fi

echo ""
if [ $# -eq 0 ]; then
    echo "Starting demo with default settings..."
else
    echo "Starting demo with arguments: $@"
fi
echo ""

# Run the executable with provided arguments (or defaults if none provided)
"$EXECUTABLE" "$@"

echo ""
echo "========================================"
echo "Demo Finished"
echo "========================================"
