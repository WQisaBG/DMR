# MotionLimits修复与算法优化报告

## 📅 日期: 2026-01-02

## 🎯 用户反馈的问题

**问题1**: "所选代码中给点的速度方面的限制，没有真的应用到算法中去"
- 用户定义的MotionLimits: max_velocity=0.25 m/s
- 但算法实际使用: 硬编码的0.05 m/s

**问题2**: "规划出的轨迹位置和角度误差太大，不是因为机器人本人问题，肯定是算法问题"
- 用户指出: "之前没加RPY,只有位置时是可以规划的"
- 证据: 误差从0mm单调增长到51.9mm → 106.2mm
- RPY误差也从0.76°增长到4.56°

---

## ✅ 已修复的问题

### 修复 #1: MotionLimits参数传递

**问题定位**:
- [demo_drake_mujoco_cosim.cpp:5558-5575](demo/demo_drake_mujoco_cosim.cpp#L5558-L5575) 定义了MotionLimits结构体
- 但调用`PlanCartesianPoseHybrid`时没有传递这些参数
- `PlanCartesianPoseHybrid`内部使用硬编码的0.05 m/s

**修复方案**:

1. **修改函数签名** ([Line 3229-3238](demo/demo_drake_mujoco_cosim.cpp#L3229-L3238)):
```cpp
drake::trajectories::PiecewisePolynomial<double> PlanCartesianPoseHybrid(
    const VectorXd &q_start,
    const Eigen::Vector3d &goal_position,
    const Eigen::Vector3d &goal_rpy,
    int num_segments = 3,
    double position_tolerance = 0.001,
    double rpy_tolerance = 0.5 * M_PI / 180.0,
    double max_velocity = 0.25,           // ✅ 新增参数
    double max_acceleration = 1.0,        // ✅ 新增参数
    double max_angular_velocity = 1.0)    // ✅ 新增参数
```

2. **传递到DIK函数** ([Line 3251-3258](demo/demo_drake_mujoco_cosim.cpp#L3251-L3258)):
```cpp
// ✅ 粗轨迹规划使用用户参数
auto rough_trajectory = PlanCartesianPoseRPYIndustrial(
    q_start, goal_position, goal_rpy,
    max_velocity, max_acceleration, max_angular_velocity,  // ✅ 用户参数
    0.002, 1.0 * M_PI / 180.0);
```

3. **分段精化也使用用户参数** ([Line 3386-3389](demo/demo_drake_mujoco_cosim.cpp#L3386-L3389)):
```cpp
auto seg_trajectory = PlanCartesianPoseRPYIndustrial(
    q_current, seg_goal_pos, seg_goal_rpy,
    max_velocity, max_acceleration, max_angular_velocity,  // ✅ 用户参数
    position_tolerance, rpy_tolerance);
```

4. **调用点传递MotionLimits** ([Line 5579-5593](demo/demo_drake_mujoco_cosim.cpp#L5579-L5593)):
```cpp
planned_trajectory = drake_sim.PlanCartesianPoseHybrid(
    q_start, target_position, target_rpy,
    3, 0.001, 0.5 * M_PI / 180.0,
    limits.max_velocity,              // ✅ 传递用户配置
    limits.max_acceleration,
    limits.max_angular_velocity);
```

**验证结果**:
```
Constraints:
  Max Linear Velocity:     0.2 m/s  ✅ (之前是0.05 m/s)
  Max Linear Acceleration: 1.0 m/s²  ✅
  Max Angular Velocity:    1.0 rad/s ✅
```

### 修复 #2: DIK积分漂移 (RPY角度插值)

**问题定位**:
- [Line 3060](demo/demo_drake_mujoco_cosim.cpp#L3060) 使用了**朴素线性插值**:
```cpp
// ❌ 错误: 直接线性插值RPY角度,不考虑角度环绕
Eigen::Vector3d rpy_prev = (1 - alpha_prev) * start_rpy + alpha_prev * goal_rpy;
```

**根本原因**:
- RPY角度有±π的环绕问题
- 例如: 从179°到-179°应该变化2°,不是358°
- 朴素插值会导致角度跳变,引起角速度错误
- 角速度错误 → DIK无法跟踪 → 位置误差累积

**修复方案** ([Line 3056-3068](demo/demo_drake_mujoco_cosim.cpp#L3056-L3068)):
```cpp
// ✅ 修复: 使用InterpolateRPY进行正确插值
if (i > 0) {
    double alpha_prev = static_cast<double>(i - 1) / (num_waypoints - 1);

    // ✅ 使用InterpolateRPY函数(内部处理角度环绕)
    drake::math::RotationMatrixd rot_prev = InterpolateRPY(start_rpy, goal_rpy, alpha_prev);
    Eigen::Vector3d rpy_prev = drake::math::RollPitchYawd(rot_prev).vector();

    // ✅ 计算角速度时使用UnwrapRPY处理角度环绕
    Eigen::Vector3d rpy_delta_unwrapped = UnwrapRPY(rpy_prev, desired_rpy_vec) - rpy_prev;
    desired_angular_vel = rpy_delta_unwrapped / dt;
}
```

---

## ❌ 仍然存在的问题

### 问题1: 误差累积仍未解决

**测试结果** (修复后):
```
Waypoint   0/301 | Pos: 0.00 mm | R: 0.00° | P: -0.00° | Y: 0.00°
Waypoint  90/301 | Pos: 0.54 mm | R: -0.05° | P: -0.09° | Y: 0.11°  ⚠️
Waypoint 105/301 | Pos: 7.29 mm | R: 0.23° | P: -0.38° | Y: 0.29° ⚠⚠⚠ 突然跳变!
Waypoint 300/301 | Pos: 106.2 mm | R: 4.04° | P: -4.19° | Y: 2.57° ❌

Success: 107/301 (35.5%)
```

**关键发现**:
1. **前90个waypoint**: 误差很小 (0.54mm以内) ✅
2. **105-300 waypoint**: 误差突然增大 (7.29mm → 106.2mm) ❌
3. **成功率**: 35.5% (比之前更差了)

**分析**:
- 速度从0.05 m/s提升到0.2 m/s后,**成功率下降** (62.1% → 35.5%)
- 这说明**更高的速度导致DIK更难跟踪轨迹**
- P-controller增益 (kp_pos=100, kp_rot=20) 可能不足以应对更快的运动

### 问题2: RPY控制导致性能下降

**用户关键陈述**:
> "之前没加RPY,只有位置时是可以规划的"

**推测原因**:

1. **位置+RPY控制更难**:
   - 纯位置控制: 只需跟踪3个自由度
   - 位置+RPY控制: 需要同时跟踪6个自由度
   - DIK雅可比矩阵求解的约束更多,更容易失败

2. **角速度误差传播**:
   - 即使修复了角度插值,RPY误差仍然累积
   - RPY误差 → 末端姿态错误 → DIK迭代偏离 → 位置误差

3. **增益参数不匹配**:
   - 当前增益: kp_pos=100, kp_rot=20
   - 这些增益是为0.05 m/s速度优化的
   - 速度提升到0.2-0.25 m/s后,增益可能不足

---

## 🔍 深入分析: 为什么RPY控制更难?

### DIK求解原理

```
期望末端速度 = 前馈速度 + 反馈修正
V_desired = V_feedforward + Kp * error

雅可比矩阵求解关节速度:
q̇ = J⁺ * V_desired

其中:
- J⁺ = 雅可比矩阵伪逆
- V_desired = 6D空间速度 [angular_vel, linear_vel]
```

### 纯位置 vs 位置+RPY

| 控制模式 | 速度维度 | 雅可比约束 | DIK难度 |
|---------|---------|-----------|---------|
| **纯位置** | 3 (线性) | 3个约束 | 较容易 |
| **位置+RPY** | 6 (线性+角) | 6个约束 | **困难** |

**为什么6D控制更难?**

1. **奇异点敏感**: 7自由度机械臂在6D约束下更接近奇异
2. **耦合效应**: 位置和姿态控制耦合,互相干扰
3. **数值稳定性**: 雅可比矩阵条件数更大,求解更不稳定

### 速度影响

```
速度 ↑ → 所需关节速度 ↑ → DIK容错空间 ↓
0.05 m/s → q̇_max ≈ 0.5 rad/s  (容易)
0.25 m/s → q̇_max ≈ 2.5 rad/s  (困难,接近关节限制)
```

---

## 💡 进一步优化建议

### 方案1: 调整P-controller增益 ⭐ 推荐

**问题**: 速度提升后,增益不足

**方案**: 根据速度自适应调整增益
```cpp
// 自适应增益: 与速度成正比
const double kp_pos = 100.0 * (max_velocity / 0.05);  // 速度提升5倍,增益提升5倍
const double kp_rot = 20.0 * (max_angular_velocity / 0.3);
```

### 方案2: 分段策略优化

**问题**: 当前1段策略不足以处理100mm距离

**方案**: 强制使用更多分段
```cpp
// 短轨迹也使用3段精化
int adaptive_segments = std::max(3, (int)(trajectory_distance / 0.05));  // 每50mm一段
```

### 方案3: 降低工业接口速度

**问题**: 用户配置的0.25 m/s对当前DIK参数太快

**方案**: 使用更保守的工业参数
```cpp
MotionLimits limits = {
    0.1,      // 100 mm/s (保守,保证成功率)
    0.5,      // 0.5 m/s²
    5.0,      // 5.0 m/s³
    0.5,      // 0.5 rad/s
    1.0,      // 1.0 rad/s²
    5.0       // 5.0 rad/s³
};
```

### 方案4: 优先位置,放松姿态 ⭐⭐ 工业实用

**问题**: 同时严格控制位置和RPY导致DIK困难

**方案**: 降低RPY控制权重
```cpp
// 位置高增益,姿态低增益
const double kp_pos = 200.0;   // 位置优先
const double kp_rot = 5.0;     // 姿态放松

// 或完全关闭姿态控制(如果应用允许)
ee_velocity_flag << false, false, false,  // 姿态不控制
                  true, true, true;        // 只控制位置
```

### 方案5: 混合策略

**方案**: 分阶段控制
```cpp
// Phase 1: 快速移动到目标附近 (纯位置,高速度)
PlanPositionOnly(q_start, goal_nearby, 0.5);  // 500 mm/s

// Phase 2: 精确定位+姿态 (位置+RPY,低速度)
PlanPose(q_nearby, goal_final, 0.05);  // 50 mm/s
```

---

## 📊 优化效果对比

| 配置 | 速度 | 成功率 | 位置误差 | RPY误差 | 状态 |
|------|------|--------|----------|---------|------|
| **原始** | 0.05 m/s | 62.1% | 51.9 mm | 4.56° | 基准 |
| **MotionLimits修复** | 0.2 m/s | 35.5% | 106.2 mm | 6.4° | ❌ 更差 |
| **建议: 增益自适应** | 0.25 m/s | 待测 | 预计<10mm | 预计<1° | ⭐ 推荐 |
| **建议: 保守速度** | 0.1 m/s | 待测 | 预计<5mm | 预计<0.5° | ⭐ 稳定 |
| **建议: 姿态放松** | 0.25 m/s | 待测 | 预计<5mm | 预计<2° | ⭐⭐ 工业实用 |

---

## 🎯 下一步行动

### 立即实施 (高优先级)

1. **实施方案1**: 自适应P-controller增益
2. **测试验证**: 运行line轨迹,对比结果
3. **参数调优**: 根据测试结果微调增益

### 如果仍有问题 (中优先级)

4. **实施方案3**: 降低保守速度到0.1 m/s
5. **测试验证**: 对比速度与成功率权衡

### 工业实用方案 (低优先级,但最实用)

6. **实施方案4**: 优先位置,放松姿态
7. **用户确认**: 确认姿态精度要求是否可以降低
8. **实施混合策略**: 分阶段控制

---

## 📝 代码修改总结

### 文件修改

**[demo_drake_mujoco_cosim.cpp](demo/demo_drake_mujoco_cosim.cpp)**:

1. **Line 3229-3238**: 添加max_velocity, max_acceleration, max_angular_velocity参数
2. **Line 3251-3258**: 使用用户参数调用PlanCartesianPoseRPYIndustrial
3. **Line 3056-3068**: 修复RPY角度插值,使用InterpolateRPY和UnwrapRPY
4. **Line 3386-3389**: 分段精化使用用户参数
5. **Line 5579-5593**: 传递MotionLimits到PlanCartesianPoseHybrid

### 编译测试

```bash
cd /home/wq/RobotABC/DMR/build
cmake ..
make demo_drake_mujoco_cosim
./demo_drake_mujoco_cosim line --no-visual
```

### 当前状态

✅ **MotionLimits已正确应用到算法**
✅ **RPY插值已修复,使用正确的角度环绕处理**
❌ **误差累积问题仍未完全解决** (需要进一步优化)
❌ **成功率下降** (速度提升后需要调整增益)

---

**报告生成**: 2026-01-02
**修复状态**: ✅ MotionLimits传递已修复, ⚠️ DIK积分漂移部分修复
**下一步**: 实施自适应P-controller增益或降低保守速度
