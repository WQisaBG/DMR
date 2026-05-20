# Drake碰撞检测API深入理解与改进实现

## 概述

本文档深入解析Drake库的碰撞检测API，并提供了工业级的机械臂碰撞检测实现方案，正确处理：

1. **自碰撞检测** - 所有连杆不发生自碰撞
2. **障碍物碰撞检测** - 所有连杆不与环境障碍物碰撞
3. **相邻连杆过滤** - 利用Drake的拓扑结构正确排除相邻机械臂连接

## Drake碰撞检测API架构

### 1. QueryObject API (核心碰撞检测接口)

**文件位置**: `thirdparty/install/drake/include/drake/geometry/query_object.h`

**关键方法**:

```cpp
// 获取SceneGraph检查器（用于拓扑查询）
const SceneGraphInspector<T>& inspector() const;

// 碰撞检测查询
std::vector<PenetrationAsPointPair<T>> ComputePointPairHasCollisions() const;
std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
    const double max_distance = infinity) const;

// 快速检查
bool HasCollisions() const;

// 姿势查询
const math::RigidTransform<T>& GetPoseInWorld(FrameId frame_id) const;
```

### 2. SceneGraphInspector API (拓扑结构查询)

**文件位置**: `thirdparty/install/drake/include/drake/geometry/scene_graph_inspector.h`

**关键方法**:

```cpp
// 碰撞过滤检查
bool CollisionFiltered(GeometryId geometry_id1,
                       GeometryId geometry_id2) const;

// 获取几何体所属的Frame
FrameId GetFrameId(GeometryId geometry_id) const;

// 获取Frame的名称
const std::string& GetName(FrameId frame_id) const;
const std::string& GetName(GeometryId geometry_id) const;

// 获取Frame的父级
FrameId GetParentFrame(FrameId frame_id) const;

// 获取Frame的几何体
std::vector<GeometryId> GetGeometries(
    FrameId frame_id,
    const std::optional<Role>& role = std::nullopt) const;

// 获取碰撞候选对（已考虑过滤）
std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates() const;
```

### 3. MultibodyPlant API (刚体拓扑结构)

**文件位置**: `thirdparty/install/drake/include/drake/multibody/plant/multibody_plant.h`

**关键方法**:

```cpp
// 从FrameId获取Body
const RigidBody<T>* GetBodyFromFrameId(FrameId frame_id) const;

// 获取Body的碰撞几何体
std::vector<GeometryId> GetCollisionGeometriesForBody(
    const RigidBody<T>& body) const;

// 相邻连杆过滤配置
void set_adjacent_bodies_collision_filters(bool value);
bool get_adjacent_bodies_collision_filters() const;
```

### 4. LinkJointGraph API (连接关系查询)

**文件位置**: `thirdparty/install/drake/include/drake/multibody/topology/`

**关键类和方法**:

```cpp
// Joint类
class Joint {
    LinkIndex parent_link_index() const;  // 父连杆索引
    LinkIndex child_link_index() const;   // 子连杆索引
    bool is_weld() const;                // 是否焊接关节
    bool connects(LinkIndex link) const; // 是否连接指定连杆
    bool connects(LinkIndex link1, LinkIndex link2) const;  // 是否连接两连杆
};

// LinkJointGraph类
class LinkJointGraph {
    const std::vector<Joint>& joints() const;  // 所有关节
};
```

## Drake的自动碰撞过滤机制

### 内置过滤规则

Drake的SceneGraph在`Finalize()`时**自动应用**以下过滤规则：

1. **同一Frame的几何体** - 同一Frame上的多个几何体之间不进行碰撞检测
2. **相邻Body（通过Joint连接）** - 当`adjacent_bodies_collision_filters=true`时自动过滤
3. **例外情况**:
   - 6自由度关节连接的Body不过滤
   - 父Body为World的关节不过滤

### 检查和配置

```cpp
// 检查Drake的相邻过滤是否启用
bool is_enabled = plant->get_adjacent_bodies_collision_filters();

// 禁用Drake的自动过滤（如果需要自定义过滤逻辑）
// 注意：这必须在Finalize()之前调用
plant->set_adjacent_bodies_collision_filters(false);
```

## 改进的碰撞检测实现

### 核心设计原则

1. **充分利用Drake内置过滤** - 先检查`inspector().CollisionFiltered()`
2. **使用正确的拓扑查询** - 使用`AreBodiesAdjacent()`而非启发式方法
3. **分离自碰撞和障碍物碰撞** - 清晰区分两种碰撞类型
4. **提供详细诊断信息** - 便于调试和理解碰撞状态

### 实现代码结构

```
┌─────────────────────────────────────────────────────────────┐
│                  CollisionChecker 类                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  构造函数:                                                   │
│  - plant: MultibodyPlant指针                                │
│  - robot_model_instances: 机器人的ModelInstance列表            │
│  - penetration_threshold: 穿透深度阈值                      │
│                                                              │
│  主要方法:                                                   │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ CheckCollision(q, context)                              │   │
│  │   - 设置关节位置                                          │   │
│  │   - 获取QueryObject和Inspector                          │   │
│  │   - 检查碰撞对                                           │   │
│  │   - 应用Drake内置过滤                                     │   │
│  │   - 检查相邻连杆                                           │   │
│  │   - 分类碰撞类型                                         │   │
│  │   - 计算最小间隙                                         │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                              │
│  辅助方法:                                                   │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ AreBothRobotGeometries(id1, id2)                         │   │
│  │   - 检查两个几何体是否都属于机器人                        │   │
│  │                                                         │   │
│  │ AreBodiesAdjacent(body1, body2)                          │   │
│  │   - 使用LinkJointGraph检查Body连接关系                   │   │
│  │   - 检查是否有Joint直接连接两个Body                     │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                              │
│  工具方法:                                                   │
│  - GenerateDiagnosticReport(result) - 生成诊断报告            │
│  - set_verbose(bool) - 启用详细输出                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 碰撞检测流程图

```
输入: 关节配置 q
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 1: 设置Plant状态                                        │
│   plant_->SetPositions(&plant_context, q);                  │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 2: 获取QueryObject和Inspector                          │
│   query_object = geometry_query_port.Eval<QueryObject>();   │
│   inspector = query_object.inspector();                    │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 3: 获取所有穿透对                                        │
│   penetrations = query_object.ComputePointPairPenetration()  │
│   注意: Drake已经自动过滤了相同Frame的几何体               │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 4: 处理每个穿透对                                       │
│   ┌────────────────────────────────────────────────────┐   │
│   │ 4.1: 检查Drake是否已过滤此对                       │   │
│   │   if (inspector.CollisionFiltered(id_A, id_B))       │   │
│   │       → 跳过 (Drake已处理)                          │   │
│   └────────────────────────────────────────────────────┘   │
│   ┌────────────────────────────────────────────────────┐   │
│   │ 4.2: 检查是否为机器人几何体对                     │   │
│   │   if (AreBothRobotGeometries(id_A, id_B))           │   │
│   │       → 自碰撞候选                                   │   │
│   │   else                                               │   │
│   │       → 障碍物碰撞候选                               │   │   │
│   └────────────────────────────────────────────────────┘   │
│   ┌────────────────────────────────────────────────────┐   │
│   │ 4.3: 自碰撞对 - 检查Body相邻性                     │   │
│   │   if (AreBodiesAdjacent(body1, body2))             │   │
│ │ │       → 相邻Body，跳过（接触正常）                │   │
│   └────────────────────────────────────────────────────┘   │
│   ┌────────────────────────────────────────────────────┐   │
│   │ 4.4: 检查穿透深度阈值                               │   │
│   │   if (penetration.depth > threshold)                │   │
│   │       → 记录为碰撞                                   │   │
│   │   else                                               │   │
│   │       → 记录为低于阈值（可能接触）                    │   │
│   └────────────────────────────────────────────────────┘   │
│   └────────────────────────────────────────────────────┘   │
│        │
│        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 5: 计算最小间隙距离                                    │
│   distances = query_object.ComputeSignedDistance...()     │
│   跳过Drake过滤的对和相邻Body对                           │
│   更新 result.min_distance                                │
└───────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│ Step 6: 生成诊断报告                                         │
│   - 碰撞统计                                              │
│   - 距离信息                                              │
│   - 碰撞对列表                                            │
│   - 过滤信息                                              │
└───────────────────────────────────────────────────────────┘
        │
        ▼
    返回: CollisionCheckResult
```

## 集成到现有代码

### 步骤1: 包含头文件

在`demo_drake_mujoco_cosim.cpp`中添加：

```cpp
#include "collision_detection_improved.h"
using namespace drake_collision;
```

### 步骤2: 在DrakeSimulator类中添加CollisionChecker成员

```cpp
class DrakeSimulator {
private:
    // ... 现有成员 ...

    // Improved collision checker
    std::unique_ptr<CollisionChecker> collision_checker_;
    std::vector<drake::multibody::ModelInstanceIndex> robot_model_instances_;

public:
    void InitializeCollisionChecker()
    {
        // 获取机器人的ModelInstance
        robot_model_instances_.clear();
        for (drake::multibody::ModelInstanceIndex idx(0);
             idx < plant_->num_model_instances(); ++idx)
        {
            std::string name = plant_->GetModelInstanceName(idx);
            if (name.find("nezha") != std::string::npos ||
                name == "default_model_instance")
            {
                robot_model_instances_.push_back(idx);
            }
        }

        // 创建碰撞检测器
        collision_checker_ = std::make_unique<CollisionChecker>(
            plant_,
            robot_model_instances_,
            0.001  // 1mm penetration threshold
        );
    }

    CollisionCheckResult CheckCollisionImproved(const VectorXd& q)
    {
        if (!collision_checker_)
        {
            InitializeCollisionChecker();
        }
        return collision_checker_->CheckCollision(q);
    }
};
```

### 步骤3: 使用改进的碰撞检测

```cpp
// 原来的用法
// bool collision = drake_sim.CheckCollision(q);

// 新的用法
auto result = drake_sim.CheckCollisionImproved(q);

// 生成诊断报告
std::cout << collision_checker->GenerateDiagnosticReport(result) << std::endl;

// 检查是否安全
if (!result.IsSafe())
{
    std::cout << "Configuration is NOT safe!" << std::endl;
}
```

## 关键API使用示例

### 示例1: 基础碰撞检测

```cpp
// 获取QueryObject
const auto& query_object =
    plant->get_geometry_query_input_port()
        .Eval<drake::geometry::QueryObject<double>>(context);

// 获取Inspector
const auto& inspector = query_object.inspector();

// 检查碰撞
const auto& penetrations = query_object.ComputePointPairPenetration();

// 处理每个碰撞对
for (const auto& penetration : penetrations)
{
    GeometryId id1 = penetration.id_A;
    GeometryId id2 = penetration.id_B;

    // 检查Drake是否已过滤此对
    if (inspector.CollisionFiltered(id1, id2))
    {
        std::cout << "Drake filtered: "
                  << inspector.GetName(id1) << " <-> "
                  << inspector.GetName(id2) << std::endl;
        continue;
    }

    // 获取Frame和Body信息
    FrameId frame1 = inspector.GetFrameId(id1);
    FrameId frame2 = inspector.GetFrameId(id2);

    const auto* body1 = plant->GetBodyFromFrameId(frame1);
    const auto* body2 = plant->GetBodyFromFrameId(frame2);

    std::cout << "Collision: "
              << body1->name() << " <-> " << body2->name()
              << " (depth=" << (penetration.depth * 1000) << " mm)"
              << std::endl;
}
```

### 示例2: 检查Body相邻性

```cpp
bool AreBodiesAdjacent(
    const drake::multibody::MultibodyPlant<double>* plant,
    drake::multibody::BodyIndex body1,
    drake::multibody::BodyIndex body2)
{
    // 访问LinkJointGraph
    const auto& forest = plant->tree_topology();
    auto& graph = forest.get_graph_mutable();

    // 检查是否有Joint直接连接
    for (const auto& joint : graph.joints())
    {
        if (joint.connects(body1, body2))
        {
            // 找到直接连接
            std::cout << "Bodies " << body1 << " and " << body2
                      << " connected by joint: " << joint.name()
                      << std::endl;
            return true;
        }
    }

    return false;
}
```

### 示例3: 计算最小间隙距离

```cpp
const auto& signed_distances =
    query_object.ComputeSignedDistancePairwiseClosestPoints(100.0);  // 100mm内

double min_distance = std::numeric_limits<double>::infinity();

for (const auto& dist : signed_distances)
{
    if (dist.distance < min_distance)
    {
        min_distance = dist.distance;

        FrameId frame1 = query_object.inspector().GetFrameId(dist.id_A);
        FrameId frame2 = query_object.inspector().GetFrameId(dist.id_B);

        std::cout << "Closest pair: "
                  << query_object.inspector().GetName(frame1)
                  << " <-> "
                  << query_object.inspector().GetName(frame2)
                  << " distance=" << (dist.distance * 1000) << " mm"
                  << std::endl;
    }
}
```

## Drake的自动过滤 vs 手动过滤

### Drake自动过滤的优点

1. **Finalize时自动应用** - 不需要运行时检查
2. **基于拓扑结构** - 准确识别相邻Body
3. **性能优化** - 过滤在broadphase阶段完成
4. **正确性保证** - 使用官方维护的逻辑

### 何时需要手动过滤

1. **自定义相邻关系** - 当Drake的拓扑定义不完全符合需求时
2. **运行时动态过滤** - 某些情况需要在运行时调整
3. **特殊情况处理** - 例如某些相邻连杆实际上需要碰撞检测

### 推荐做法

```cpp
// 方案1: 完全依赖Drake自动过滤（推荐）
// 在Plant Finalize()之前配置
plant->set_adjacent_bodies_collision_filters(true);

// 方案2: Drake自动过滤 + 补充检查
// Drake处理大部分情况，运行时处理特殊情况
if (inspector.CollisionFiltered(id1, id2))
{
    // Drake已过滤，可能是相邻Body
    // 但仍需要检查是否真的应该被过滤
    // 可以添加额外的逻辑
}
```

## 调试和诊断

### 启用详细输出

```cpp
collision_checker->set_verbose(true);
auto result = collision_checker->CheckCollision(q);
std::cout << collision_checker->GenerateDiagnosticReport(result) << std::endl;
```

### 常见问题排查

1. **所有碰撞都被过滤**
   - 检查`plant->get_adjacent_bodies_collision_filters()`
   - 检查collision_candidates

2. **相邻连杆碰撞未过滤**
   - 验证Joint连接关系
   - 检查是否需要手动配置过滤

3. **性能问题**
   - 减少`max_distance`参数
   - 使用`HasCollisions()`快速检查

## 参考资料

- Drake Geometry Tutorial: https://drake.mit.edu/doxygen_cxx/group__geometry.html
- Drake Multibody Plant: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody__plant.html
- SceneGraph Collision Filtering: `drake/geometry/collision_filter_declaration.h`
