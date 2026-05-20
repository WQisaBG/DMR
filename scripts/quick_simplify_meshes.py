#!/usr/bin/env python3
"""
快速简化碰撞网格的Python脚本
不需要MeshLab，只使用Python标准库和基础包

用法:
    # 简化单个文件
    python3 scripts/quick_simplify_meshes.py --input model/nezha/urdf/meshes/collision/leg_link1.obj

    # 简化所有collision meshes
    python3 scripts/quick_simplify_meshes.py --all

    # 简化指定的links
    python3 scripts/quick_simplify_meshes.py --links leg_link1 waist_link right_arm_link7

    # 使用凸包方法
    python3 scripts/quick_simplify_meshes.py --all --method convex
"""

import argparse
import os
import sys
import shutil
from pathlib import Path
from datetime import datetime

def check_dependencies():
    """检查依赖"""
    missing = []

    try:
        import numpy as np
    except ImportError:
        missing.append("numpy")

    try:
        import trimesh
    except ImportError:
        missing.append("trimesh")

    if missing:
        print(f"❌ 缺少依赖: {', '.join(missing)}")
        print("\n请安装:")
        print("  pip3 install numpy trimesh")
        return False

    return True

def simplify_mesh_decimation(input_file, output_file, reduction_factor=0.5):
    """
    使用网格简化算法减少面数

    Args:
        input_file: 输入OBJ文件
        output_file: 输出OBJ文件
        reduction_factor: 简化比例 (0.1-0.9)，0.5表示保留50%的面
    """
    import trimesh

    print(f"📂 加载: {input_file}")
    mesh = trimesh.load_mesh(input_file)

    original_faces = len(mesh.faces)
    original_vertices = len(mesh.vertices)
    print(f"   原始: {original_vertices} 顶点, {original_faces} 面")

    # 方法1: 简化网格
    try:
        # Trimesh的简化接口
        simplified = mesh.simplify_quadric_decimation(
            face_count=int(original_faces * reduction_factor)
        )

        simplified_faces = len(simplified.faces)
        simplified_vertices = len(simplified.vertices)

        print(f"   简化后: {simplified_vertices} 顶点, {simplified_faces} 面")
        print(f"   简化率: {(1 - reduction_factor) * 100:.1f}%")

        # 保存
        simplified.export(output_file)
        print(f"✓ 保存到: {output_file}")
        return True

    except Exception as e:
        print(f"⚠ 简化失败: {e}")
        return False

def create_convex_hull(input_file, output_file):
    """
    创建凸包（最简单的方法）

    Args:
        input_file: 输入OBJ文件
        output_file: 输出OBJ文件
    """
    import trimesh

    print(f"📂 加载: {input_file}")
    mesh = trimesh.load_mesh(input_file)

    original_faces = len(mesh.faces)
    original_vertices = len(mesh.vertices)
    print(f"   原始: {original_vertices} 顶点, {original_faces} 面")

    # 创建凸包
    convex = mesh.convex_hull

    convex_faces = len(convex.faces)
    convex_vertices = len(convex.vertices)

    print(f"   凸包: {convex_vertices} 顶点, {convex_faces} 面")
    print(f"   简化率: {(1 - convex_faces / original_faces) * 100:.1f}%")

    # 保存
    convex.export(output_file)
    print(f"✓ 保存到: {output_file}")
    return True

def create_voxelized_mesh(input_file, output_file, voxel_size=0.01):
    """
    使用体素化简化网格

    Args:
        input_file: 输入OBJ文件
        output_file: 输出OBJ文件
        voxel_size: 体素大小（米）
    """
    import trimesh

    print(f"📂 加载: {input_file}")
    mesh = trimesh.load_mesh(input_file)

    original_faces = len(mesh.faces)
    original_vertices = len(mesh.vertices)
    print(f"   原始: {original_vertices} 顶点, {original_faces} 面")

    # 体素化
    voxelized = mesh.voxelized(pitch=voxel_size)

    # 转回mesh
    simplified = voxelized.as_boxes()

    simplified_faces = len(simplified.faces)
    simplified_vertices = len(simplified.vertices)

    print(f"   体素化: {simplified_vertices} 顶点, {simplified_faces} 面")

    # 保存
    simplified.export(output_file)
    print(f"✓ 保存到: {output_file}")
    return True

def backup_collision_directory(collision_dir):
    """备份整个collision目录"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = f"{collision_dir}_backup_{timestamp}"

    print(f"📦 备份到: {backup_dir}")
    shutil.copytree(collision_dir, backup_dir)
    print(f"✓ 备份完成")
    return backup_dir

def get_file_size_mb(file_path):
    """获取文件大小（MB）"""
    return os.path.getsize(file_path) / (1024 * 1024)

def simplify_single_file(input_file, output_file, method, **kwargs):
    """简化单个文件"""

    input_size = get_file_size_mb(input_file)

    success = False
    if method == 'decimation':
        success = simplify_mesh_decimation(input_file, output_file, kwargs.get('reduction', 0.5))
    elif method == 'convex':
        success = create_convex_hull(input_file, output_file)
    elif method == 'voxel':
        success = create_voxelized_mesh(input_file, output_file, kwargs.get('voxel_size', 0.01))
    else:
        print(f"❌ 未知方法: {method}")
        return False

    if success and os.path.exists(output_file):
        output_size = get_file_size_mb(output_file)
        reduction = (1 - output_size / input_size) * 100
        print(f"📊 文件大小: {input_size:.1f}MB -> {output_size:.1f}MB (减少 {reduction:.1f}%)")
        return True
    else:
        print(f"❌ 简化失败")
        return False

def process_all_collision_meshes(collision_dir, method, **kwargs):
    """处理所有collision meshes"""
    print("=" * 60)
    print("🔧 简化所有碰撞网格")
    print(f"方法: {method}")
    print("=" * 60)

    # 备份
    backup_dir = backup_collision_directory(collision_dir)

    # 找到所有OBJ文件
    obj_files = list(Path(collision_dir).glob("*.obj"))

    print(f"\n找到 {len(obj_files)} 个文件\n")

    success_count = 0
    failed = []

    for i, obj_file in enumerate(obj_files, 1):
        print(f"\n[{i}/{len(obj_files)}] 处理: {obj_file.name}")
        print("-" * 60)

        # 创建临时输出文件
        temp_output = obj_file.parent / f"{obj_file.stem}_temp.obj"

        if simplify_single_file(str(obj_file), str(temp_output), method, **kwargs):
            # 替换原文件
            shutil.move(str(temp_output), str(obj_file))
            success_count += 1
        else:
            failed.append(obj_file.name)

    # 总结
    print("\n" + "=" * 60)
    print("📊 总结")
    print("=" * 60)
    print(f"✓ 成功: {success_count}/{len(obj_files)}")

    if failed:
        print(f"❌ 失败: {len(failed)}")
        for f in failed:
            print(f"   - {f}")

    print(f"\n备份位置: {backup_dir}")

def main():
    parser = argparse.ArgumentParser(
        description="简化机器人碰撞网格",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 简化单个文件
  python3 %(prog)s --input model/nezha/urdf/meshes/collision/leg_link1.obj

  # 简化所有collision meshes（使用默认decimation方法）
  python3 %(prog)s --all

  # 使用凸包方法简化所有
  python3 %(prog)s --all --method convex

  # 简化指定的links
  python3 %(prog)s --links leg_link1 waist_link right_arm_link7

  # 只检查当前状态
  python3 %(prog)s --check
        """
    )

    parser.add_argument('--input', help='输入OBJ文件')
    parser.add_argument('--output', help='输出OBJ文件（默认覆盖输入）')
    parser.add_argument('--all', action='store_true', help='处理所有collision meshes')
    parser.add_argument('--links', nargs='+', help='指定要处理的links（如: leg_link1 waist_link）')
    parser.add_argument('--method', choices=['decimation', 'convex', 'voxel'],
                        default='decimation', help='简化方法（默认: decimation）')
    parser.add_argument('--reduction', type=float, default=0.5,
                        help='简化比例（0.1-0.9，默认: 0.5表示保留50%%面）')
    parser.add_argument('--voxel-size', type=float, default=0.01,
                        help='体素大小/米（默认: 0.01）')
    parser.add_argument('--collision-dir',
                        default='model/nezha/urdf/meshes/collision',
                        help='Collision目录（默认: model/nezha/urdf/meshes/collision）')
    parser.add_argument('--check', action='store_true', help='检查当前状态')

    args = parser.parse_args()

    # 检查依赖
    if not args.check and not check_dependencies():
        sys.exit(1)

    # 检查状态
    if args.check:
        collision_dir = Path(args.collision_dir)
        visual_dir = collision_dir.parent / 'visual'

        print("=" * 60)
        print("🔍 碰撞网格状态检查")
        print("=" * 60)

        if not collision_dir.exists():
            print(f"❌ Collision目录不存在: {collision_dir}")
            return

        print(f"\nCollision目录: {collision_dir}")
        print(f"Visual目录: {visual_dir}")

        # 比较文件大小
        print("\n文件大小对比:")
        print(f"{'Link Name':<25} {'Visual':>12} {'Collision':>12} {'状态':>10}")
        print("-" * 60)

        for visual_file in sorted(visual_dir.glob("*.obj")):
            collision_file = collision_dir / visual_file.name

            if collision_file.exists():
                v_size = get_file_size_mb(visual_file)
                c_size = get_file_size_mb(collision_file)

                if abs(v_size - c_size) < 0.01:  # 几乎相同
                    status = "⚠ 相同"
                elif c_size < v_size:
                    status = "✓ 简化"
                else:
                    status = "❌ 更大"

                print(f"{visual_file.stem:<25} {v_size:>10.1f}M {c_size:>10.1f}M {status:>10}")
            else:
                print(f"{visual_file.stem:<25} {'MISSING':>12}")

        # 检查备份
        print("\n备份:")
        parent_dir = collision_dir.parent
        backups = sorted(parent_dir.glob("collision_backup_*"))
        if backups:
            print(f"  找到 {len(backups)} 个备份")
            for b in backups[-3:]:  # 显示最近3个
                print(f"    - {b.name}")
        else:
            print("  ⚠ 没有找到备份")

        return

    # 处理所有文件
    if args.all:
        collision_dir = Path(args.collision_dir)
        if not collision_dir.exists():
            print(f"❌ 目录不存在: {collision_dir}")
            sys.exit(1)

        process_all_collision_meshes(
            str(collision_dir),
            args.method,
            reduction=args.reduction,
            voxel_size=args.voxel_size
        )
        return

    # 处理指定的links
    if args.links:
        collision_dir = Path(args.collision_dir)
        if not collision_dir.exists():
            print(f"❌ 目录不存在: {collision_dir}")
            sys.exit(1)

        print("=" * 60)
        print("🔧 简化指定的碰撞网格")
        print("=" * 60)

        backup_dir = backup_collision_directory(str(collision_dir))

        success_count = 0
        for link_name in args.links:
            obj_file = collision_dir / f"{link_name}.obj"
            if not obj_file.exists():
                print(f"⚠ 跳过（不存在）: {link_name}.obj")
                continue

            print(f"\n处理: {link_name}.obj")
            print("-" * 60)

            temp_output = obj_file.parent / f"{link_name}_temp.obj"
            if simplify_single_file(str(obj_file), str(temp_output), args.method,
                                   reduction=args.reduction, voxel_size=args.voxel_size):
                shutil.move(str(temp_output), str(obj_file))
                success_count += 1

        print(f"\n✓ 成功: {success_count}/{len(args.links)}")
        print(f"备份位置: {backup_dir}")
        return

    # 处理单个文件
    if args.input:
        output_file = args.output or args.input
        print("=" * 60)
        print("🔧 简化单个碰撞网格")
        print("=" * 60)

        if not os.path.exists(args.input):
            print(f"❌ 文件不存在: {args.input}")
            sys.exit(1)

        success = simplify_single_file(
            args.input,
            output_file,
            args.method,
            reduction=args.reduction,
            voxel_size=args.voxel_size
        )

        if success:
            print("\n✓ 简化完成！")
        else:
            print("\n❌ 简化失败")
            sys.exit(1)
        return

    # 默认显示帮助
    parser.print_help()

if __name__ == '__main__':
    main()
