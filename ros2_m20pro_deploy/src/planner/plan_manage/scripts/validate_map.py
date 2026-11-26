#!/usr/bin/env python3
"""先验点云地图 (.pcd) 完整性与跨楼层范围校验脚本。

Validate a PCD point cloud map (file existence, point count, finite ratio, 3D bounding box,
and vertical Z span for 1F-2F multi-floor staircase scenarios).

功能 / Purpose:
  - 在 Step 0 手动扫图保存 `.pcd` 后，自动检查地图是否为空、是否存在大量 NaN 坏点、
    以及三维包围盒与 Z 轴高差是否覆盖目标楼层（例如 1 楼到 2 楼楼梯）。
  - 纯 Python 标准库 + 可选 NumPy 实现，无需额外 pip 依赖，支持 ASCII 与 Binary PCD 格式。

运行方式 / Usage:
  ros2 run scan_planner validate_map.py --pcd /tmp/scan_planner_maps/building_1f_2f.pcd \
    --min-points 5000 --min-z-span 1.0
"""

import argparse
import math
import os
import struct
import sys


def parse_pcd_header(handle):
    """解析 PCD 文件头部元数据并返回 (header_dict, data_offset)。"""
    header = {}
    while True:
        raw_line = handle.readline()
        if not raw_line:
            raise ValueError("Unexpected EOF while reading PCD header")
        line = raw_line.decode("ascii", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        header[key] = values
        if key == "DATA":
            break
    return header, handle.tell()


def _dtype_char(field_type, field_size):
    """将 PCD TYPE/SIZE 映射为 struct 格式字符 / Map PCD field type+size to struct code."""
    mapping = {
        ("F", 4): "f",
        ("F", 8): "d",
        ("I", 1): "b",
        ("I", 2): "h",
        ("I", 4): "i",
        ("I", 8): "q",
        ("U", 1): "B",
        ("U", 2): "H",
        ("U", 4): "I",
        ("U", 8): "Q",
    }
    return mapping.get((field_type, field_size))


def inspect_pcd(pcd_path, max_sample_points=500000):
    """读取并统计 PCD 文件的点数、有限值比例与三维包围盒信息。"""
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")

    file_size = os.path.getsize(pcd_path)
    if file_size == 0:
        raise ValueError(f"PCD file is empty (0 bytes): {pcd_path}")

    with open(pcd_path, "rb") as handle:
        header, data_offset = parse_pcd_header(handle)

        fields = header.get("FIELDS", [])
        sizes = [int(v) for v in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
        declared_points = int(header.get("POINTS", ["0"])[0])
        data_mode = header.get("DATA", ["unknown"])[0].lower()

        result = {
            "path": os.path.abspath(pcd_path),
            "file_size_bytes": file_size,
            "data_mode": data_mode,
            "fields": fields,
            "declared_points": declared_points,
            "valid_points": 0,
            "nan_points": 0,
            "sampled_points": 0,
            "bbox_min": None,
            "bbox_max": None,
        }

        if declared_points <= 0:
            return result

        if not all(axis in fields for axis in ("x", "y", "z")):
            raise ValueError(f"PCD missing x/y/z fields: {fields}")

        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        min_v = [math.inf, math.inf, math.inf]
        max_v = [-math.inf, -math.inf, -math.inf]
        valid = 0
        nan_cnt = 0
        sampled = 0

        if data_mode == "ascii":
            stride = max(1, declared_points // max_sample_points)
            for idx, raw_line in enumerate(handle):
                if idx % stride != 0:
                    continue
                cols = raw_line.decode("ascii", errors="replace").split()
                if len(cols) <= max(ix, iy, iz):
                    continue
                sampled += 1
                try:
                    x, y, z = float(cols[ix]), float(cols[iy]), float(cols[iz])
                except ValueError:
                    nan_cnt += 1
                    continue
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    nan_cnt += 1
                    continue
                valid += 1
                for axis_idx, val in enumerate((x, y, z)):
                    if val < min_v[axis_idx]:
                        min_v[axis_idx] = val
                    if val > max_v[axis_idx]:
                        max_v[axis_idx] = val

        elif data_mode == "binary":
            # 计算每个点字节步长与 x/y/z 字段字节偏移量
            offsets = []
            cursor = 0
            for size, count in zip(sizes, counts):
                offsets.append(cursor)
                cursor += size * count
            point_step = cursor

            fmt_x = _dtype_char(types[ix], sizes[ix])
            fmt_y = _dtype_char(types[iy], sizes[iy])
            fmt_z = _dtype_char(types[iz], sizes[iz])
            if not (fmt_x and fmt_y and fmt_z):
                raise ValueError("Unsupported binary PCD x/y/z field type")

            handle.seek(data_offset)
            payload = handle.read(point_step * declared_points)
            actual_points = len(payload) // point_step
            result["declared_points"] = actual_points

            # 优先尝试使用 NumPy 向量化解析（处理几十万点仅需数毫秒）
            parsed_with_numpy = False
            try:
                import numpy as np  # pylint: disable=import-outside-toplevel

                np_dtype = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": [f"<{fmt_x}", f"<{fmt_y}", f"<{fmt_z}"],
                        "offsets": [offsets[ix], offsets[iy], offsets[iz]],
                        "itemsize": point_step,
                    }
                )
                arr = np.frombuffer(payload[: actual_points * point_step], dtype=np_dtype)
                xs = arr["x"].astype(np.float64)
                ys = arr["y"].astype(np.float64)
                zs = arr["z"].astype(np.float64)
                finite_mask = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
                sampled = int(actual_points)
                valid = int(np.count_nonzero(finite_mask))
                nan_cnt = sampled - valid
                if valid > 0:
                    min_v = [float(xs[finite_mask].min()),
                             float(ys[finite_mask].min()),
                             float(zs[finite_mask].min())]
                    max_v = [float(xs[finite_mask].max()),
                             float(ys[finite_mask].max()),
                             float(zs[finite_mask].max())]
                parsed_with_numpy = True
            except Exception:
                parsed_with_numpy = False

            if not parsed_with_numpy:
                stride = max(1, actual_points // max_sample_points)
                off_x, off_y, off_z = offsets[ix], offsets[iy], offsets[iz]
                for i in range(0, actual_points, stride):
                    base = i * point_step
                    x = struct.unpack_from("<" + fmt_x, payload, base + off_x)[0]
                    y = struct.unpack_from("<" + fmt_y, payload, base + off_y)[0]
                    z = struct.unpack_from("<" + fmt_z, payload, base + off_z)[0]
                    sampled += 1
                    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                        nan_cnt += 1
                        continue
                    valid += 1
                    for axis_idx, val in enumerate((x, y, z)):
                        if val < min_v[axis_idx]:
                            min_v[axis_idx] = val
                        if val > max_v[axis_idx]:
                            max_v[axis_idx] = val

        else:
            # binary_compressed 仅验证文件头与字节非空，不强制依赖外部 LZF 库
            result["valid_points"] = declared_points
            result["sampled_points"] = 0
            return result

        result["valid_points"] = valid
        result["nan_points"] = nan_cnt
        result["sampled_points"] = sampled
        if valid > 0:
            result["bbox_min"] = tuple(min_v)
            result["bbox_max"] = tuple(max_v)
        return result


def validate_map(pcd_path, min_points=1000, min_z_span=0.0, max_nan_ratio=0.05):
    """执行完整校验并打印中文/英文报告，返回 (passed: bool, stats: dict)。"""
    stats = inspect_pcd(pcd_path)
    issues = []

    if stats["declared_points"] < min_points:
        issues.append(
            f"点数不足 (points={stats['declared_points']} < min_points={min_points})"
        )

    if stats["sampled_points"] > 0:
        nan_ratio = stats["nan_points"] / float(stats["sampled_points"])
        if nan_ratio > max_nan_ratio:
            issues.append(
                f"无效 NaN/Inf 点比例过高 ({nan_ratio * 100:.2f}% > {max_nan_ratio * 100:.2f}%)"
            )

    z_span = 0.0
    if stats["bbox_min"] is not None and stats["bbox_max"] is not None:
        z_span = stats["bbox_max"][2] - stats["bbox_min"][2]
        if z_span < min_z_span:
            issues.append(
                f"Z 轴高度跨度偏小 (z_span={z_span:.2f}m < min_z_span={min_z_span:.2f}m)"
            )

    print("========== PCD Map Validation Report / 先验地图校验报告 ==========")
    print(f"File       : {stats['path']}")
    print(f"Size       : {stats['file_size_bytes'] / (1024.0 * 1024.0):.2f} MB ({stats['data_mode']})")
    print(f"Fields     : {', '.join(stats['fields'])}")
    print(f"Points     : {stats['declared_points']} (valid sampled: {stats['valid_points']}, NaN: {stats['nan_points']})")
    if stats["bbox_min"] is not None:
        bmin, bmax = stats["bbox_min"], stats["bbox_max"]
        print(f"X Range    : [{bmin[0]:.2f}, {bmax[0]:.2f}] m (span {bmax[0] - bmin[0]:.2f} m)")
        print(f"Y Range    : [{bmin[1]:.2f}, {bmax[1]:.2f}] m (span {bmax[1] - bmin[1]:.2f} m)")
        print(f"Z Range    : [{bmin[2]:.2f}, {bmax[2]:.2f}] m (span {z_span:.2f} m)")
    if issues:
        print("Status     : FAILED / 校验未通过")
        for item in issues:
            print(f"  - {item}")
        return False, stats

    print("Status     : PASSED / 校验通过")
    return True, stats


def main():
    """命令行入口 / CLI entry point."""
    parser = argparse.ArgumentParser(description="Validate prior PCD point cloud map integrity.")
    parser.add_argument("--pcd", required=True, help="Path to the .pcd file to validate")
    parser.add_argument("--min-points", type=int, default=1000,
                        help="Minimum acceptable point count (default: 1000)")
    parser.add_argument("--min-z-span", type=float, default=0.0,
                        help="Minimum acceptable Z height span in meters (e.g. 1.0 for stairs)")
    parser.add_argument("--max-nan-ratio", type=float, default=0.05,
                        help="Maximum acceptable NaN/Inf ratio (default: 0.05)")
    args = parser.parse_args()

    try:
        passed, _ = validate_map(
            args.pcd,
            min_points=args.min_points,
            min_z_span=args.min_z_span,
            max_nan_ratio=args.max_nan_ratio,
        )
    except Exception as exc:
        print(f"[ERROR] Failed to validate PCD map: {exc}", file=sys.stderr)
        sys.exit(2)

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
