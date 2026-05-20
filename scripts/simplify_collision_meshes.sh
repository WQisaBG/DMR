#!/bin/bash
# ============================================================
# 碰撞网格简化脚本
# 用法: ./scripts/simplify_collision_meshes.sh [--all|--specific LINK_NAMES]
# ============================================================

set -e

MESH_DIR="/home/abc/RobotGrasp/DMR/model/nezha/urdf/meshes"
COLLISION_DIR="${MESH_DIR}/collision"
VISUAL_DIR="${MESH_DIR}/visual"
BACKUP_DIR="${MESH_DIR}/collision_backup_$(date +%Y%m%d_%H%M%S)"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查依赖
check_dependencies() {
    log_info "检查依赖工具..."

    local missing_deps=()

    # 检查MeshLab
    if ! command -v meshlabserver &> /dev/null; then
        missing_deps+=("meshlabserver")
    fi

    # 检查Python3和PyMeshFix
    if ! command -v python3 &> /dev/null; then
        missing_deps+=("python3")
    elif ! python3 -c "import pymeshfix" &> /dev/null; then
        missing_deps+=("python3-pymeshfix")
    fi

    # 检查Blender (可选)
    if ! command -v blender &> /dev/null; then
        log_warn "Blender未安装（可选，用于高级简化）"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        log_error "缺少依赖: ${missing_deps[*]}"
        echo ""
        echo "请安装缺少的依赖："
        echo "  sudo apt-get install meshlab python3-pip"
        echo "  pip3 install pymeshfix trimesh"
        exit 1
    fi

    log_info "所有依赖已满足"
}

# 备份原始文件
backup_original_files() {
    log_info "备份原始碰撞网格到: ${BACKUP_DIR}"

    mkdir -p "${BACKUP_DIR}"

    if [ -d "${COLLISION_DIR}" ]; then
        cp -r "${COLLISION_DIR}"/* "${BACKUP_DIR}/" 2>/dev/null || true
        log_info "备份完成"
    else
        log_warn "碰撞网格目录不存在: ${COLLISION_DIR}"
    fi
}

# 方法1: 使用MeshLab简化（推荐）
simplify_with_meshlab() {
    local input_file=$1
    local output_file=$2
    local reduction_ratio=${3:-0.5}  # 默认简化到50%

    log_info "使用MeshLab简化: $(basename ${input_file})"

    # 创建MeshLab过滤器脚本
    local mlx_script=$(mktemp)
    cat > "${mlx_script}" <<EOF
<!DOCTYPE FilterScript>
<FilterScript>
    <filter name="Simplification: Quadric Edge Collapse Decimation">
        <Param type="RichFloat" value="100000" name="TargetFaceNum" tooltip="Desired number of faces"/>
        <Param type="RichFloat" value="${reduction_ratio}" name="TargetPerc" tooltip="Percentage of reduction (0..1)"/>
        <Param type="RichFloat" value="0.3" name="QualityThr" tooltip="Quality threshold for boundary extraction"/>
        <Param type="RichBool" value="false" name="PreserveBoundary"/>
        <Param type="RichBool" value="false" name="BoundaryValue"/>
        <Param type="RichBool" value="true" name="PreserveNormal"/>
        <Param type="RichBool" value="true" name="PreserveTopology"/>
        <Param type="RichBool" value="false" name="PlanarSimp"/>
        <Param type="RichBool" value="false" name="PlanarQuadric"/>
    </filter>
</FilterScript>
EOF

    # 执行简化
    meshlabserver -i "${input_file}" \
                  -o "${output_file}" \
                  -s "${mlx_script}" \
                  -om vn fn 2>&1 | grep -E "Simplification|faces" || true

    rm "${mlx_script}"

    log_info "简化完成: $(basename ${output_file})"
}

# 方法2: 使用Python + PyMeshFix创建凸包
create_convex_hull_python() {
    local input_file=$1
    local output_file=$2

    log_info "使用Python创建凸包: $(basename ${input_file})"

    python3 <<EOF
import trimesh
import numpy as np

# 加载原始网格
mesh = trimesh.load_mesh('${input_file}')

# 创建凸包
convex = mesh.convex_hull

# 保存
convex.export('${output_file}')

print(f"原始面数: {len(mesh.faces)}")
print(f"凸包面数: {len(convex.faces)}")
print(f"简化率: {len(convex.faces) / len(mesh.faces) * 100:.1f}%")
EOF

    log_info "凸包创建完成: $(basename ${output_file})"
}

# 方法3: 使用PyMeshFix修复和简化
fix_and_simplify_mesh() {
    local input_file=$1
    local output_file=$2

    log_info "使用PyMeshFix修复网格: $(basename ${input_file})"

    python3 <<EOF
import pymeshfix
import trimesh

# 加载网格
mesh = trimesh.load_mesh('${input_file}')

# 修复网格（填充洞，修复法向量等）
v, f = pymeshfix.clean_from_arrays(mesh.vertices, mesh.faces)

# 保存修复后的网格
fixed_mesh = trimesh.Trimesh(vertices=v, faces=f)
fixed_mesh.export('${output_file}')

print(f"原始顶点: {len(mesh.vertices)}, 面数: {len(mesh.faces)}")
print(f"修复后顶点: {len(v)}, 面数: {len(f)}")
EOF

    log_info "网格修复完成: $(basename ${output_file})"
}

# 方法4: 使用Blender批量简化（高级）
simplify_with_blender() {
    local input_file=$1
    local output_file=$2
    local reduction_ratio=${3:-0.5}

    if ! command -v blender &> /dev/null; then
        log_warn "Blender未安装，跳过此方法"
        return 1
    fi

    log_info "使用Blender简化: $(basename ${input_file})"

    blender --background --python-exit-code 3 - <<EOF
import bpy
import bmesh
import math

# 清除场景
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# 导入模型
bpy.ops.import_scene.obj(filepath='${input_file}')
obj = bpy.context.selected_objects[0]

# 进入编辑模式
bpy.context.view_layer.objects.active = obj
bpy.ops.object.mode_set(mode='EDIT')

# 修约修改器
modifier = obj.modifiers.new(name="Decimate", type='DECIMATE')
modifier.ratio = ${reduction_ratio}

# 应用修改器
bpy.ops.object.modifier_apply(modifier="Decimate")

# 导出
bpy.ops.export_scene.obj(filepath='${output_file}')
EOF

    log_info "Blender简化完成"
}

# 比较文件大小
compare_mesh_sizes() {
    local original=$1
    local simplified=$2

    local orig_size=$(du -h "${original}" | cut -f1)
    local new_size=$(du -h "${simplified}" | cut -f1)
    local orig_bytes=$(stat -f%z "${original}" 2>/dev/null || stat -c%s "${original}")
    local new_bytes=$(stat -f%z "${simplified}" 2>/dev/null || stat -c%s "${simplified}")
    local reduction=$(echo "scale=1; (1 - ${new_bytes}/${orig_bytes}) * 100" | bc -l 2>/dev/null || echo "N/A")

    log_info "文件大小: ${orig_size} -> ${new_size} (减少 ${reduction}%)"
}

# 简化单个文件
simplify_mesh() {
    local link_name=$1
    local method=${2:-"meshlab"}  # meshlab, convex, fix, blender

    local input_file="${COLLISION_DIR}/${link_name}.obj"
    local output_file="${COLLISION_DIR}/${link_name}_simple.obj"

    if [ ! -f "${input_file}" ]; then
        log_error "文件不存在: ${input_file}"
        return 1
    fi

    log_info "================================================"
    log_info "简化: ${link_name}"
    log_info "================================================"

    case "${method}" in
        meshlab)
            simplify_with_meshlab "${input_file}" "${output_file}" 0.5
            ;;
        convex)
            create_convex_hull_python "${input_file}" "${output_file}"
            ;;
        fix)
            fix_and_simplify_mesh "${input_file}" "${output_file}"
            ;;
        blender)
            simplify_with_blender "${input_file}" "${output_file}" 0.5
            ;;
        *)
            log_error "未知方法: ${method}"
            return 1
            ;;
    esac

    if [ -f "${output_file}" ]; then
        compare_mesh_sizes "${input_file}" "${output_file}"

        # 备份原文件并替换
        mv "${input_file}" "${input_file}.bak"
        mv "${output_file}" "${input_file}"

        log_info "✓ 已替换: ${link_name}.obj"
        return 0
    else
        log_error "简化失败: ${output_file} 未生成"
        return 1
    fi
}

# 简化所有碰撞网格
simplify_all_collision_meshes() {
    local method=${1:-"meshlab"}

    log_info "================================================"
    log_info "简化所有碰撞网格"
    log_info "方法: ${method}"
    log_info "================================================"

    local count=0
    for mesh_file in "${COLLISION_DIR}"/*.obj; do
        if [ -f "${mesh_file}" ]; then
            local link_name=$(basename "${mesh_file}" .obj)
            simplify_mesh "${link_name}" "${method}"
            ((count++))
        fi
    done

    log_info "总计简化了 ${count} 个文件"
}

# 简化特定的link
simplify_specific_links() {
    local links=$1
    local method=${2:-"meshlab"}

    log_info "================================================"
    log_info "简化特定的碰撞网格: ${links}"
    log_info "方法: ${method}"
    log_info "================================================"

    for link_name in ${links}; do
        if [ -f "${COLLISION_DIR}/${link_name}.obj" ]; then
            simplify_mesh "${link_name}" "${method}"
        else
            log_warn "文件不存在: ${link_name}.obj，跳过"
        fi
    done
}

# 更新URDF文件（可选）
update_urdf_scale() {
    local urdf_file="/home/abc/RobotGrasp/DMR/model/nezha/urdf/robot_arm.urdf"

    log_info "================================================"
    log_info "更新URDF文件（添加scale参数）"
    log_info "================================================"

    # 备份URDF
    cp "${urdf_file}" "${urdf_file}.bak"

    # 为所有collision mesh添加scale="0.95 0.95 0.95"
    # 使用sed进行替换
    sed -i 's|<mesh filename="\./meshes/collision/\([^"]*\)" />|<mesh filename="./meshes/collision/\1" scale="0.95 0.95 0.95" />|g' "${urdf_file}"

    log_info "URDF已更新: 所有collision mesh添加了scale=\"0.95 0.95 0.95\""
    log_info "备份: ${urdf_file}.bak"
}

# 显示使用说明
show_usage() {
    cat <<EOF
用法: $0 [OPTIONS]

简化机器人的碰撞网格，提高碰撞检测性能和准确性。

OPTIONS:
    --all [METHOD]            简化所有碰撞网格
    --specific "LINK1 LINK2"  简化指定的links（用空格分隔）
    --update-urdf             更新URDF文件，为collision meshes添加scale参数
    --check                   检查依赖和当前网格状态
    --restore                 从备份恢复原始文件
    --help                    显示此帮助信息

METHODS:
    meshlab    使用MeshLab简化（推荐，需要安装meshlab）
    convex     使用Python创建凸包（需要pymeshfix）
    fix        使用PyMeshFix修复和简化
    blender    使用Blender简化（需要安装blender）

示例:
    # 简化所有碰撞网格（使用默认方法meshlab）
    $0 --all

    # 简化指定的links
    $0 --specific "leg_link1 waist_link right_arm_link7"

    # 使用凸包方法简化
    $0 --all convex

    # 更新URDF添加scale参数
    $0 --update-urdf

    # 检查当前状态
    $0 --check

EOF
}

# 检查当前状态
check_status() {
    log_info "================================================"
    log_info "检查当前网格状态"
    log_info "================================================"

    echo ""
    echo "碰撞网格目录: ${COLLISION_DIR}"
    echo "视觉网格目录: ${VISUAL_DIR}"
    echo ""

    if [ ! -d "${COLLISION_DIR}" ]; then
        log_error "碰撞网格目录不存在"
        return 1
    fi

    echo "文件大小对比:"
    printf "%-25s %15s %15s %10s\n" "Link Name" "Visual" "Collision" "Ratio"
    printf "%-25s %15s %15s %10s\n" "---------" "-----" "---------" "-----"

    for visual_file in "${VISUAL_DIR}"/*.obj; do
        if [ -f "${visual_file}" ]; then
            local link_name=$(basename "${visual_file}" .obj)
            local collision_file="${COLLISION_DIR}/${link_name}.obj"

            if [ -f "${collision_file}" ]; then
                local v_size=$(du -h "${visual_file}" | cut -f1)
                local c_size=$(du -h "${collision_file}" | cut -f1)
                local v_bytes=$(stat -f%z "${visual_file}" 2>/dev/null || stat -c%s "${visual_file}")
                local c_bytes=$(stat -f%z "${collision_file}" 2>/dev/null || stat -c%s "${collision_file}")

                if [ "${c_bytes}" -eq "${v_bytes}" ]; then
                    printf "%-25s %15s %15s %10s\n" "${link_name}" "${v_size}" "${c_size}" "IDENTICAL"
                else
                    local ratio=$(echo "scale=1; ${c_bytes}/${v_bytes}" | bc -l 2>/dev/null || echo "1.0")
                    printf "%-25s %15s %15s %10s\n" "${link_name}" "${v_size}" "${c_size}" "${ratio}x"
                fi
            fi
        fi
    done

    echo ""
    log_info "检查备份:"
    if ls -d ${MESH_DIR}/collision_backup_* 1> /dev/null 2>&1; then
        ls -lh ${MESH_DIR}/collision_backup_* | tail -1
    else
        log_warn "没有找到备份"
    fi
}

# 恢复备份
restore_backup() {
    log_info "================================================"
    log_info "恢复原始碰撞网格"
    log_info "================================================"

    # 查找最新的备份
    local latest_backup=$(ls -td ${MESH_DIR}/collision_backup_* 2>/dev/null | head -1)

    if [ -z "${latest_backup}" ]; then
        log_error "没有找到备份"
        return 1
    fi

    log_info "从备份恢复: ${latest_backup}"

    cp -r "${latest_backup}"/* "${COLLISION_DIR}/"

    log_info "恢复完成"
}

# ============================================================
# 主函数
# ============================================================

main() {
    if [ $# -eq 0 ]; then
        show_usage
        exit 0
    fi

    case "$1" in
        --all)
            check_dependencies
            backup_original_files
            simplify_all_collision_meshes "${2:-meshlab}"
            ;;
        --specific)
            check_dependencies
            backup_original_files
            if [ -z "$2" ]; then
                log_error "请指定要简化的links"
                exit 1
            fi
            simplify_specific_links "$2" "${3:-meshlab}"
            ;;
        --update-urdf)
            update_urdf_scale
            ;;
        --check)
            check_status
            ;;
        --restore)
            restore_backup
            ;;
        --help|-h)
            show_usage
            ;;
        *)
            log_error "未知选项: $1"
            show_usage
            exit 1
            ;;
    esac

    log_info "================================================"
    log_info "完成！"
    log_info "================================================"
}

main "$@"
