# 路径配置说明

## 工业级路径解析策略

本系统采用**多级路径解析策略**，确保代码在不同环境下都能正常工作，符合工业级软件部署标准。

---

## 自动路径解析（推荐）

系统会按以下优先级自动查找项目根目录：

### 1. 环境变量（最高优先级）

```bash
# 设置项目根目录环境变量
export DMR_PROJECT_ROOT=/path/to/DMR

# 可选：设置CSV导出目录
export DMR_CSV_DIR=/path/to/DMR/CSV
```

### 2. 相对于可执行文件（自动检测）

系统会自动检测可执行文件位置，并向上查找项目根目录。

**假设目录结构**：
```
DMR/
├── build/
│   └── demo_drake_mujoco_cosim  # 可执行文件
├── demo/
│   └── demo_drake_mujoco_cosim.cpp
└── model/
    └── nezha/
```

从 `build/` 目录运行时，系统会自动找到 `DMR/` 作为项目根目录。

### 3. 相对于当前工作目录（备选）

如果前两种方法都失败，系统会使用 `../` 作为项目根目录。

**使用方法**：
```bash
# 从 build/ 目录运行
cd /home/wq/RobotABC/DMR/build
./demo_drake_mujoco_cosim

# 或从 demo/ 目录运行
cd /home/wq/RobotABC/DMR/demo
./demo_drake_mujoco_cosim
```

---

## 路径验证

程序启动时会自动检查所有必需文件：

```
[PATH] Checking model files:
  URDF:   ../model/nezha/urdf/robot_arm.urdf
  Scene:  ../model/nezha/scene/scene.xml
  [OK] All model files found
```

如果文件缺失，会给出明确的错误提示：
```
[ERROR] URDF file not found: ../model/nezha/urdf/robot_arm.urdf
[INFO] Set DMR_PROJECT_ROOT environment variable or run from project directory
```

---

## 使用示例

### 示例1：从项目根目录运行

```bash
cd /home/wq/RobotABC/DMR
export DMR_PROJECT_ROOT=$(pwd)
./build/demo_drake_mujoco_cosim circular
```

### 示例2：从 build 目录运行（推荐）

```bash
cd /home/wq/RobotABC/DMR/build
./demo_drake_mujoco_cosim circular 5.0 0.001
```

### 示例3：从任意位置运行

```bash
# 设置环境变量
export DMR_PROJECT_ROOT=/home/wq/RobotABC/DMR
export DMR_CSV_DIR=/home/wq/RobotABC/DMR/CSV

# 从任意位置运行
~/DMR/build/demo_drake_mujoco_cosim line
```

---

## CSV 导出路径

CSV轨迹文件默认保存到 `./CSV/` 目录（相对于当前工作目录）。

**自定义CSV目录**：
```bash
export DMR_CSV_DIR=/custom/path/to/csv
./demo_drake_mujoco_cosim circular
```

**自动创建目录**：
系统会自动创建CSV目录（如果不存在）。

---

## 工业级优势

### ✅ 跨平台兼容
- Linux：使用 `/proc/self/exe`
- macOS：使用 `_NSGetExecutablePath`
- 自动适配不同操作系统

### ✅ 部署灵活
- 开发环境：自动检测路径
- 生产环境：使用环境变量
- 容器化：支持任意目录挂载

### ✅ 错误处理
- 启动时验证所有文件
- 清晰的错误提示
- 避免运行时崩溃

### ✅ 可移植性
- 无需修改代码
- 无硬编码路径
- 支持多用户部署

---

## 故障排查

### 问题1：找不到 URDF 文件

**错误**：
```
[ERROR] URDF file not found
```

**解决方法**：
```bash
# 方法1：设置环境变量
export DMR_PROJECT_ROOT=/home/wq/RobotABC/DMR

# 方法2：从正确的目录运行
cd /home/wq/RobotABC/DMR/build
./demo_drake_mujoco_cosim

# 方法3：检查文件是否存在
ls -l /home/wq/RobotABC/DMR/model/nezha/urdf/robot_arm.urdf
```

### 问题2：CSV 导出失败

**错误**：
```
Failed to open CSV file for writing
```

**解决方法**：
```bash
# 创建CSV目录
mkdir -p CSV

# 或设置自定义CSV目录
export DMR_CSV_DIR=/tmp/dmr_csv
mkdir -p $DMR_CSV_DIR
```

### 问题3：路径显示不正确

**调试方法**：
```bash
# 启用路径调试输出
./demo_drake_mujoco_cosim circular 2>&1 | grep "\[PATH\]"
```

---

## 环境变量汇总

| 变量 | 用途 | 默认值 | 示例 |
|------|------|--------|------|
| `DMR_PROJECT_ROOT` | 项目根目录 | 自动检测 | `/home/wq/RobotABC/DMR` |
| `DMR_CSV_DIR` | CSV导出目录 | `./CSV` | `/home/wq/RobotABC/DMR/CSV` |

---

## 与旧版本对比

### ❌ 旧版本（硬编码）
```cpp
std::string project_dir = "/home/abc/RobotGrasp/DMR";  // 硬编码！
```

**问题**：
- 换机器需要修改代码
- 不同用户路径不同
- 无法部署到生产环境

### ✅ 新版本（灵活解析）
```cpp
// 自动检测 + 环境变量 + 相对路径
if (const char* env_root = std::getenv("DMR_PROJECT_ROOT")) {
    project_dir = env_root;
} else {
    // 自动检测可执行文件位置...
}
```

**优势**：
- 无需修改代码
- 支持多用户
- 生产就绪

---

## 总结

这套路径解析系统确保：
1. ✅ 开发环境零配置（自动检测）
2. ✅ 生产环境灵活部署（环境变量）
3. ✅ 跨平台兼容（Linux/macOS）
4. ✅ 清晰的错误提示
5. ✅ 符合工业级软件标准

**建议**：生产环境使用环境变量，开发环境依赖自动检测。
