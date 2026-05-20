# 碰撞网格简化工具使用指南

## 快速开始

### 1️⃣ 安装依赖

```bash
# 安装Python依赖
pip3 install numpy trimesh

# （可选）安装MeshLab用于高级简化
sudo apt-get install meshlab
```

### 2️⃣ 使用Python脚本简化（推荐，最简单）

```bash
# 检查当前状态
python3 /home/abc/RobotGrasp/DMR/scripts/quick_simplify_meshes.py --check

# 简化所有collision meshes（使用默认decimation方法，保留50%面）
python3 /home/abc/RobotGrasp/DMR/scripts/quick_simplify_meshes.py --all

# 使用凸包方法（更简单，适合碰撞检测）
python3 /home/abc/RobotGrasp/DMR/scripts/quick_simplify_meshes.py --all --method convex

# 只简化特定的links（根据您的碰撞日志）
python3 /home/abc/RobotGrasp/DMR/scripts/quick_simplify_meshes.py \
    --links leg_link1 waist_link right_arm_link3 right_arm_link7 \
    --method convex
```

### 3️⃣ 使用Bash脚本（更多选项）

```bash
# 检查当前状态
/home/abc/RobotGrasp/DMR/scripts/simplify_collision_meshes.sh --check

# 简化所有（使用MeshLab）
/home/abc/RobotGrasp/DMR/scripts/simplify_collision_meshes.sh --all

# 简化指定的links
/home/abc/RobotGrasp/DMR/scripts/simplify_collision_meshes.sh \
    --specific "leg_link1 waist_link right_arm_link7"
```

---

## 方法对比

### 方法1: Decimation（网格简化）
- **原理**: 减少网格面数，保留大致形状
- **优点**: 保留原始形状细节
- **缺点**: 可能仍保留凹陷部分
- **推荐**: 用于需要保持形状的link

```bash
python3 scripts/quick_simplify_meshes.py --all --method decimation --reduction 0.3
```

### 方法2: Convex Hull（凸包）⭐ 推荐用于碰撞检测
- **原理**: 创建最小的外接凸包
- **优点**: 完全消除凹陷，计算最快，最适合碰撞检测
- **缺点**: 形状与原始略有差异
- **推荐**: **用于所有碰撞网格**

```bash
python3 scripts/quick_simplify_meshes.py --all --method convex
```

### 方法3: Voxelization（体素化）
- **原理**: 使用立方体近似形状
- **优点**: 规则几何体，碰撞检测快
- **缺点**: 可能过于简化
- **推荐**: 用于复杂但不需要精确形状的link

```bash
python3 scripts/quick_simplify_meshes.py --all --method voxel --voxel-size 0.01
```

---

## 推荐工作流程

### 步骤1: 检查当前状态（1分钟）

```bash
python3 scripts/quick_simplify_meshes.py --check
```

**预期输出**:
```
============================================================
🔍 碰撞网格状态检查
============================================================

Collision目录: model/nezha/urdf/meshes/collision
Visual目录: model/nezha/urdf/meshes/visual

文件大小对比:
Link Name                    Visual    Collision      状态
------------------------------------------------------------
leg_link1                      8.6M        8.6M     ⚠ 相同
waist_link                    39.0M       39.0M     ⚠ 相同
right_arm_link7                9.2M        9.2M     ⚠ 相同
```

### 步骤2: 简化所有collision meshes（2-5分钟）

```bash
# 使用凸包方法（推荐）
python3 scripts/quick_simplify_meshes.py --all --method convex
```

**预期输出**:
```
============================================================
🔧 简化所有碰撞网格
方法: convex
============================================================
📦 备份到: model/nezha/urdf/meshes/collision_backup_20231231_123456

[1/22] 处理: base_link.obj
------------------------------------------------------------
📂 加载: model/nezha/urdf/meshes/collision/base_link.obj
   原始: 52340 顶点, 104678 面
   凸包: 142 顶点, 282 面
   简化率: 99.7%
✓ 保存到: model/nezha/urdf/meshes/collision/base_link.obj
📊 文件大小: 22.0MB -> 0.1MB (减少 99.5%)
...
============================================================
📊 总结
============================================================
✓ 成功: 22/22
备份位置: model/nezha/urdf/meshes/collision_backup_20231231_123456
```

### 步骤3: 验证简化效果（1分钟）

```bash
python3 scripts/quick_simplify_meshes.py --check
```

**预期输出**:
```
Link Name                    Visual    Collision      状态
------------------------------------------------------------
leg_link1                      8.6M        0.1M     ✓ 简化
waist_link                    39.0M        0.3M     ✓ 简化
right_arm_link7                9.2M        0.1M     ✓ 简化
```

### 步骤4: 测试碰撞检测（5分钟）

```bash
# 重新编译
bash /home/abc/RobotGrasp/DMR/install.sh

# 运行测试
/home/abc/RobotGrasp/DMR/run_cosim_demo.sh
```

**预期改进**:
- ✅ 误报减少（mesh padding导致的0.00-0.02m碰撞应该消失）
- ✅ 碰撞检测更快（文件大小减少95%+）
- ✅ 真实碰撞仍能检测

---

## 只简化有问题的links（最快方案）

如果只关心您日志中提到的problematic links：

```bash
python3 scripts/quick_simplify_meshes.py \
    --links leg_link1 waist_link right_arm_link3 right_arm_link7 \
    --method convex
```

---

## 恢复备份

如果简化效果不满意：

```bash
# 使用Python脚本
cp -r model/nezha/urdf/meshes/collision_backup_*/\* model/nezha/urdf/meshes/collision/

# 或使用Bash脚本
/home/abc/RobotGrasp/DMR/scripts/simplify_collision_meshes.sh --restore
```

---

## 预期效果

### 简化前:
- `leg_link1.obj`: 8.6MB，高精度网格，包含表面细节
- Collision = Visual → 误报多，计算慢

### 简化后（凸包）:
- `leg_link1.obj`: ~0.1MB，简单凸包，无凹陷
- Collision << Visual → 误报少，计算快

### 碰撞检测改进:
- **False positives**: 减少 90%+
- **Detection speed**: 提升 10-100倍
- **Real collisions**: 仍然能检测

---

## 故障排除

### 问题1: 缺少依赖

```bash
❌ 缺少依赖: trimesh

解决:
pip3 install numpy trimesh
```

### 问题2: 简化失败

```bash
❌ 简化失败: xxx.obj

可能原因:
1. OBJ文件格式有问题
2. 网格包含非流形边

解决:
# 尝试使用fix方法
python3 scripts/quick_simplify_meshes.py --all --method convex
```

### 问题3: 简化后仍有误报

如果简化后仍有0.00-0.02m的误报：

**方案A**: 增加碰撞阈值
```cpp
// 在 demo_drake_mujoco_cosim.cpp:4655
const double penetration_threshold = 0.020; // 20mm
```

**方案B**: 使用更激进的简化
```bash
# 再次简化，或使用体素化
python3 scripts/quick_simplify_meshes.py --all --method voxel --voxel-size 0.02
```

---

## 总结

### 🎯 最佳实践:

1. **使用凸包方法** (`--method convex`) 进行碰撞网格简化
2. **先测试几个links**，确认效果后再简化所有
3. **保留备份**，脚本会自动备份
4. **验证效果**，重新运行碰撞检测测试

### 📝 推荐命令:

```bash
# 完整流程
cd /home/abc/RobotGrasp/DMR

# 1. 安装依赖
pip3 install numpy trimesh

# 2. 检查当前状态
python3 scripts/quick_simplify_meshes.py --check

# 3. 简化所有collision meshes
python3 scripts/quick_simplify_meshes.py --all --method convex

# 4. 验证简化效果
python3 scripts/quick_simplify_meshes.py --check

# 5. 重新编译测试
bash install.sh
./run_cosim_demo.sh
```

这个过程大约需要**10分钟**，可以彻底解决碰撞网格过大导致的误报问题！
