# 路径配置快速参考

## 🚀 三种使用方式

### 方式1：自动检测（最简单）
```bash
cd /home/wq/RobotABC/DMR/build
./demo_drake_mujoco_cosim circular
```

### 方式2：环境变量（生产推荐）
```bash
export DMR_PROJECT_ROOT=/home/wq/RobotABC/DMR
./demo_drake_mujoco_cosim circular
```

### 方式3：自定义CSV目录
```bash
export DMR_CSV_DIR=/tmp/trajectory_data
./demo_drake_mujoco_cosim circular
```

---

## 📂 文件位置

```
DMR/
├── model/nezha/
│   ├── urdf/robot_arm.urdf      ← URDF模型
│   └── scene/scene.xml           ← MuJoCo场景
├── CSV/                          ← CSV导出目录（自动创建）
└── build/demo_drake_mujoco_cosim ← 可执行文件
```

---

## ✅ 验证安装

运行时会看到：
```
[PATH] Checking model files:
  URDF:   ../model/nezha/urdf/robot_arm.urdf
  Scene:  ../model/nezha/scene/scene.xml
  [OK] All model files found
```

---

## 🔧 故障排查

| 问题 | 解决方法 |
|------|---------|
| URDF not found | `export DMR_PROJECT_ROOT=/path/to/DMR` |
| Permission denied | `chmod +x demo_drake_mujoco_cosim` |
| CSV directory error | `mkdir -p CSV` |

详细说明：见 [PATH_SETUP.md](PATH_SETUP.md)
