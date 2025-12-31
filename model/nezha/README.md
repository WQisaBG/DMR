# Nezha Robot MuJoCo 配置说明

## 文件说明

### URDF 文件
- **robot_arm.urdf** - 完整的 Nezha 机器人 URDF 模型
  - 20个旋转关节
  - 关节顺序：leg(3) + waist(1) + left_arm(7) + right_arm(7) + head(2)

### MuJoCo 文件
- **nezha.xml** - 完整的机器人 MuJoCo 模型（使用四元数表示旋转）
  - 包含腿部、腰部、双臂和头部
  - 腰部坐标系已可视化（X=红色, Y=绿色, Z=蓝色轴线）
  - 所有关节轴在各自局部坐标系中为 (0, 0, 1)

- **scene.xml** - 场景文件
  - 包含地面、桌子、被抓取的圆柱体
  - 通过 include 引用 nezha.xml

### 关节索引（在 URDF/MuJoCo 中的顺序）
```
Index | Joint Name              | 描述
------|------------------------|------
0     | leg_joint1            | 左腿关节1
1     | leg_joint2            | 左腿关节2
2     | leg_joint3            | 左腿关节3
3     | waist_joint           | 腰部旋转（参考坐标系）
4-10  | left_arm_joint1-7    | 左臂关节1-7
11-17 | right_arm_joint1-7   | 右臂关节1-7（主要控制）
18-19 | head_joint1-2        | 头部关节1-2
```

## Demo 使用说明

### 编译
```bash
cd /home/abc/RobotGrasp/DMR/build
make demo_drake_mujoco_cosim
```

### 运行
```bash
# 圆形轨迹（默认）
./demo_drake_mujoco_cosim circular 5.0 0.001

# 直线轨迹
./demo_drake_mujoco_cosim line 3.0 0.001
```

### 初始配置
- **腿部**：中性站立位置（全部为0）
- **腰部**：0（面向前方）
- **左臂**：收缩位置，避免干涉
- **右臂**：准备抓取姿态
  - joint1 (shoulder pan): 0.0
  - joint2 (shoulder lift): 0.3
  - joint3 (elbow): 1.0
  - joint4 (wrist rotation): -0.8
  - joint5-7: 0.0
- **头部**：中性位置

### 参考坐标系
- **腰部坐标系 (waist_link)** 作为主要参考坐标系
- 末端执行器：right_tool_frame（固定在 right_arm_link7 上，偏移 0.31m）
- 圆形轨迹在世界坐标系中定义，但初始姿态基于腰部坐标系

## 注意事项

1. **关节轴约定**：MuJoCo 中所有关节轴在局部坐标系中为 (0, 0, 1)
2. **旋转表示**：使用四元数 (quat) 而非欧拉角，避免万向锁
3. **坐标系可视化**：腰部坐标系已用RGB三色轴线可视化（红X、绿Y、蓝Z）
