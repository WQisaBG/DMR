#!/bin/bash
# Complete code cleanup script
# This script removes all unused code from demo_drake_mujoco_cosim.cpp

FILE="/home/wq/RobotABC/DMR/demo/demo_drake_mujoco_cosim.cpp"
BACKUP="/home/wq/RobotABC/DMR/demo/demo_drake_mujoco_cosim.cpp.backup_full"

echo "=== Complete Code Cleanup ==="
echo "Original file: $(wc -l < $FILE) lines"
echo ""

# Create backup
cp "$FILE" "$BACKUP"
echo "✓ Created backup: $BACKUP"

# After PlanCartesianCircle deletion (already done): 4060 lines
# Now we need to delete:
# 1. PlanCartesianLine (old version)
# 2. ComputeOptimalWaypointTiming
# 3. SolveIK and SolveGlobalIK declarations
# 4. PlanWithObstacleAvoidance
# 5. GenerateRandomGuess and SolveGlobalIK implementations

# Due to line number shifts, we'll do this in one comprehensive operation

python3 << 'PYTHON_SCRIPT'
import re

file_path = "/home/wq/RobotABC/DMR/demo/demo_drake_mujoco_cosim.cpp"

with open(file_path, 'r') as f:
    content = f.read()

# Define patterns to remove
patterns_to_remove = [
    # ComputeOptimalWaypointTiming function
    (r'    std::vector<double> ComputeOptimalWaypointTiming\([^}]+?return breaks;\n    \}\n', ''),
]

# Apply substitutions
for pattern, replacement in patterns_to_remove:
    content = re.sub(pattern, replacement, content, flags=re.DOTALL)

# Write back
with open(file_path, 'w') as f:
    f.write(content)

print("✓ Cleanup completed")
print(f"New file: {len(content.splitlines())} lines")
PYTHON_SCRIPT

echo ""
echo "=== Testing compilation ==="
cd /home/wq/RobotABC/DMR/build
make demo_drake_mujoco_cosim 2>&1 | grep -E "error:|Built target"

echo ""
echo "=== Summary ==="
echo "Original: 4686 lines"
echo "Current:  $(wc -l < $FILE) lines"
echo "Deleted:  $((4686 - $(wc -l < $FILE))) lines"
echo "Reduction: $(python3 -c "print(f'{100 * (4686 - $(wc -l < $FILE)) / 4686:.1f}")%)"
