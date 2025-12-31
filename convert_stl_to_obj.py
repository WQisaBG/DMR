#!/usr/bin/env python3
"""
Convert STL mesh files to OBJ format with coordinate system correction.

Usage:
    python3 convert_stl_to_obj.py

This script:
1. Finds all .STL files in the meshes directory
2. Converts them to .obj format
3. Corrects coordinate system if needed (swap/flip axes)
"""

import os
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    print("Error: numpy library not installed.")
    print("Install with: pip install numpy")
    sys.exit(1)


def parse_stl_file(stl_path):
    """
    Parse STL file (both ASCII and binary formats) and extract vertices and faces.

    Returns:
        vertices: numpy array of shape (N, 3)
        faces: numpy array of shape (M, 3) indices into vertices
    """
    with open(stl_path, 'rb') as f:
        # Read first 80 bytes to check if it's binary STL
        header = f.read(80)
        f.seek(0)

        # Check for ASCII format
        if b'solid' in header[:5]:
            return parse_ascii_stl(stl_path)
        else:
            return parse_binary_stl(stl_path)


def parse_ascii_stl(stl_path):
    """Parse ASCII STL format."""
    vertices = []
    faces = []
    vertex_map = {}

    def get_vertex_index(v):
        """Get or create vertex index (deduplicate vertices)."""
        v_tuple = tuple(v)
        if v_tuple not in vertex_map:
            vertex_map[v_tuple] = len(vertices)
            vertices.append(v)
        return vertex_map[v_tuple]

    with open(stl_path, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith('facet normal'):
            # Next line should be "outer loop"
            i += 2  # Skip "outer loop"
            # Read three vertices
            v1 = parse_vertex_line(lines[i])
            v2 = parse_vertex_line(lines[i+1])
            v3 = parse_vertex_line(lines[i+2])

            # Add face indices
            faces.append([get_vertex_index(v1), get_vertex_index(v2), get_vertex_index(v3)])

            i += 5  # Skip "endloop" and "endfacet"
        i += 1

    return np.array(vertices, dtype=np.float32), np.array(faces, dtype=np.uint32)


def parse_vertex_line(line):
    """Parse a vertex line from STL: 'vertex x y z'."""
    parts = line.strip().split()
    return [float(parts[1]), float(parts[2]), float(parts[3])]


def parse_binary_stl(stl_path):
    """Parse binary STL format."""
    with open(stl_path, 'rb') as f:
        # Skip 80-byte header
        f.read(80)

        # Read number of triangles
        num_triangles = np.frombuffer(f.read(4), dtype=np.uint32)[0]

        vertices = []
        faces = []
        vertex_map = {}

        def get_vertex_index(v):
            """Get or create vertex index (deduplicate vertices)."""
            v_tuple = tuple(v)
            if v_tuple not in vertex_map:
                vertex_map[v_tuple] = len(vertices)
                vertices.append(v)
            return vertex_map[v_tuple]

        for _ in range(num_triangles):
            # Skip normal vector (3 floats)
            f.read(12)

            # Read three vertices
            v1 = np.frombuffer(f.read(12), dtype=np.float32)
            v2 = np.frombuffer(f.read(12), dtype=np.float32)
            v3 = np.frombuffer(f.read(12), dtype=np.float32)

            # Add face indices
            faces.append([get_vertex_index(v1), get_vertex_index(v2), get_vertex_index(v3)])

            # Skip attribute byte count
            f.read(2)

        return np.array(vertices, dtype=np.float32), np.array(faces, dtype=np.uint32)


def write_obj_file(vertices, faces, obj_path):
    """Write vertices and faces to OBJ file format."""
    with open(obj_path, 'w') as f:
        # Write vertices (OBJ uses 1-based indexing)
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        # Write faces
        for face in faces:
            # OBJ indices start at 1, not 0
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")


def convert_stl_to_obj(stl_path, obj_path, coordinate_transform=None):
    """
    Convert STL file to OBJ format with optional coordinate transformation.

    Args:
        stl_path: Path to input STL file
        obj_path: Path to output OBJ file
        coordinate_transform: Function to transform vertices, or None
    """
    print(f"Converting {stl_path.name} -> {obj_path.name}")

    # Parse STL file
    vertices, faces = parse_stl_file(stl_path)
    print(f"  Loaded {len(vertices)} vertices, {len(faces)} faces")

    # Apply coordinate transformation if specified
    if coordinate_transform:
        vertices = coordinate_transform(vertices)
        print(f"  Applied coordinate transformation")

    # Write OBJ file
    write_obj_file(vertices, faces, obj_path)
    print(f"  ✓ Saved {obj_path.name}")


def transform_coordinates_for_drake(vertices):
    """
    Transform coordinates from robot-specific system to Drake's expected system.

    Drake uses: X-forward, Y-left, Z-up (right-handed)
    Some robots use different coordinate systems.

    Common transformations:
    - Swap Y and Z axes (Z-up to Y-up)
    - Flip axis directions

    Adjust this function based on your specific coordinate system mismatch.
    """
    # By default, no transformation
    # Modify if you need to correct coordinate system
    return vertices


def main():
    # Find the meshes directory
    script_dir = Path(__file__).parent
    meshes_base = script_dir / "model/nezha/urdf/meshes"

    if not meshes_base.exists():
        print(f"Error: Meshes directory not found: {meshes_base}")
        sys.exit(1)

    # Find all STL files in both visual and collision subdirectories
    stl_files = []
    for subdir in ["visual", "collision"]:
        subdir_path = meshes_base / subdir
        if subdir_path.exists():
            stl_files.extend(subdir_path.glob("*.STL"))
            stl_files.extend(subdir_path.glob("*.stl"))

    if not stl_files:
        print("No STL files found in meshes directory")
        return

    print(f"Found {len(stl_files)} STL files to convert\n")

    # Convert each STL file
    for stl_path in sorted(stl_files):
        # Generate output path
        obj_path = stl_path.with_suffix('.obj')

        # Skip if OBJ already exists and is newer
        if obj_path.exists():
            stl_mtime = stl_path.stat().st_mtime
            obj_mtime = obj_path.stat().st_mtime
            if obj_mtime > stl_mtime:
                print(f"Skipping {stl_path.name} (OBJ already up to date)")
                print()
                continue

        # Convert with coordinate transformation
        # You can modify transform_coordinates_for_drake() above if needed
        convert_stl_to_obj(stl_path, obj_path, transform_coordinates_for_drake)
        print()

    print(f"✓ Conversion complete!")
    print(f"OBJ files saved to: {meshes_base}")


if __name__ == "__main__":
    main()
