#!/usr/bin/env python3
"""
Continuity check for motion files.

Checks:
- Boundary jump between first/last frame (root xyz + joint angles)
- Boundary velocity continuity (start velocity vs end velocity)
- Basic velocity smoothness summary

Outputs (per run):
- report.json
- report.md
- charts/*.svg
"""

import argparse
import json
import math
from datetime import datetime
from pathlib import Path


def load_motion(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "Frames" not in data or not isinstance(data["Frames"], list) or not data["Frames"]:
        raise ValueError("Missing or invalid Frames")
    return data


def detect_joint_count(data, frames):
    if "Joints" in data and isinstance(data["Joints"], list) and data["Joints"]:
        return len(data["Joints"]), "from_joints_key", False

    frame_len = len(frames[0])
    candidates = []

    # No-contact format: 7 + n + 3 + 3 + 3 + 3 + n + 3 + 3 = 25 + 2n
    if (frame_len - 25) % 2 == 0:
        n = (frame_len - 25) // 2
        if n > 0:
            candidates.append((n, False))

    # Contact format: previous + 2 contacts => 27 + 2n
    if (frame_len - 27) % 2 == 0:
        n = (frame_len - 27) // 2
        if n > 0:
            candidates.append((n, True))

    if not candidates:
        raise ValueError(f"Unable to infer joint count from frame length {frame_len}")

    # Prefer candidate with plausible contact values if contacts assumed.
    if len(candidates) == 2:
        no_contact = next((c for c in candidates if not c[1]), None)
        with_contact = next((c for c in candidates if c[1]), None)
        if with_contact is not None:
            n = with_contact[0]
            looks_like_contacts = True
            sample_n = min(20, len(frames))
            for i in range(sample_n):
                frame = frames[i]
                c0 = frame[25 + 2 * n]
                c1 = frame[25 + 2 * n + 1]
                if not (abs(c0 - round(c0)) < 0.2 and abs(c1 - round(c1)) < 0.2 and -0.2 <= c0 <= 1.2 and -0.2 <= c1 <= 1.2):
                    looks_like_contacts = False
                    break
            if looks_like_contacts:
                return n, "inferred_with_contacts", True
        if no_contact is not None:
            return no_contact[0], "inferred_no_contacts", False

    n, has_contacts = candidates[0]
    src = "inferred_with_contacts" if has_contacts else "inferred_no_contacts"
    return n, src, has_contacts


def split_frame(frame, n_joints):
    root_xyz = frame[0:3]
    joints = frame[7:7 + n_joints]
    return root_xyz, joints


def vec_sub(a, b):
    return [x - y for x, y in zip(a, b)]


def vec_abs(v):
    return [abs(x) for x in v]


def vec_norm(v):
    return math.sqrt(sum(x * x for x in v))


def mean(values):
    return sum(values) / len(values) if values else 0.0


def max_abs(values):
    return max((abs(v) for v in values), default=0.0)


def compute_analysis(data, source_file):
    frames = data["Frames"]
    dt = float(data.get("FrameDuration", 0.02))
    if dt <= 0:
        raise ValueError("FrameDuration must be > 0")
    if len(frames) < 3:
        raise ValueError("Need at least 3 frames for continuity checks")

    n_joints, joint_source, has_contacts = detect_joint_count(data, frames)

    root = []
    joints = []
    for frame in frames:
        if len(frame) < 7 + n_joints:
            raise ValueError("Frame length smaller than expected")
        r, j = split_frame(frame, n_joints)
        root.append(r)
        joints.append(j)

    first_root = root[0]
    last_root = root[-1]
    root_jump = vec_sub(last_root, first_root)
    root_jump_abs = vec_abs(root_jump)

    first_j = joints[0]
    last_j = joints[-1]
    joint_jump = vec_sub(last_j, first_j)
    joint_jump_abs = vec_abs(joint_jump)

    root_vel = []
    joint_vel = []
    for i in range(1, len(frames)):
        rv = [(root[i][k] - root[i - 1][k]) / dt for k in range(3)]
        jv = [(joints[i][k] - joints[i - 1][k]) / dt for k in range(n_joints)]
        root_vel.append(rv)
        joint_vel.append(jv)

    start_root_vel = root_vel[0]
    end_root_vel = root_vel[-1]
    root_vel_mismatch = vec_sub(end_root_vel, start_root_vel)
    root_vel_mismatch_abs = vec_abs(root_vel_mismatch)

    start_joint_vel = joint_vel[0]
    end_joint_vel = joint_vel[-1]
    joint_vel_mismatch = vec_sub(end_joint_vel, start_joint_vel)
    joint_vel_mismatch_abs = vec_abs(joint_vel_mismatch)

    root_speed = [vec_norm(v) for v in root_vel]
    speed_delta = [root_speed[i] - root_speed[i - 1] for i in range(1, len(root_speed))]

    return {
        "meta": {
            "source_file": str(source_file),
            "frames": len(frames),
            "frame_duration": dt,
            "joint_count": n_joints,
            "joint_count_source": joint_source,
            "has_contacts_assumed": has_contacts,
        },
        "boundary": {
            "root_xyz_jump": root_jump,
            "root_xyz_jump_abs": root_jump_abs,
            "root_xyz_jump_norm": vec_norm(root_jump),
            "joint_jump_abs_max": max(joint_jump_abs),
            "joint_jump_abs_mean": mean(joint_jump_abs),
            "joint_jump_abs": joint_jump_abs,
        },
        "velocity_boundary": {
            "root_vel_start": start_root_vel,
            "root_vel_end": end_root_vel,
            "root_vel_mismatch": root_vel_mismatch,
            "root_vel_mismatch_abs": root_vel_mismatch_abs,
            "root_vel_mismatch_norm": vec_norm(root_vel_mismatch),
            "joint_vel_mismatch_abs_max": max(joint_vel_mismatch_abs),
            "joint_vel_mismatch_abs_mean": mean(joint_vel_mismatch_abs),
            "joint_vel_mismatch_abs": joint_vel_mismatch_abs,
        },
        "velocity_smoothness": {
            "root_speed_max": max(root_speed),
            "root_speed_mean": mean(root_speed),
            "root_speed_delta_abs_max": max_abs(speed_delta),
            "root_speed_delta_abs_mean": mean([abs(x) for x in speed_delta]),
        },
        "series": {
            "root_x": [r[0] for r in root],
            "root_y": [r[1] for r in root],
            "root_z": [r[2] for r in root],
            "root_speed": root_speed,
            "joint_jump_abs": joint_jump_abs,
            "joint_vel_mismatch_abs": joint_vel_mismatch_abs,
        },
    }


def scale_points(values, width, height, pad):
    if not values:
        return []
    vmin = min(values)
    vmax = max(values)
    if abs(vmax - vmin) < 1e-12:
        vmax = vmin + 1.0
    n = len(values)
    pts = []
    for i, v in enumerate(values):
        x = pad + (width - 2 * pad) * (i / (n - 1 if n > 1 else 1))
        y = height - pad - (height - 2 * pad) * ((v - vmin) / (vmax - vmin))
        pts.append((x, y))
    return pts, vmin, vmax


def line_svg(values_dict, title, out_path, width=1200, height=360):
    pad = 40
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    all_values = []
    for vals in values_dict.values():
        all_values.extend(vals)
    if not all_values:
        return
    y_min = min(all_values)
    y_max = max(all_values)
    if abs(y_max - y_min) < 1e-12:
        y_max = y_min + 1.0

    def project(vals):
        n = len(vals)
        pts = []
        for i, v in enumerate(vals):
            x = pad + (width - 2 * pad) * (i / (n - 1 if n > 1 else 1))
            y = height - pad - (height - 2 * pad) * ((v - y_min) / (y_max - y_min))
            pts.append(f"{x:.2f},{y:.2f}")
        return " ".join(pts)

    legend_y = 18
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="white" />',
        f'<text x="{pad}" y="20" font-size="16" font-family="monospace">{title}</text>',
        f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#888" />',
        f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#888" />',
        f'<text x="{pad}" y="{height-8}" font-size="12" font-family="monospace">0</text>',
    ]
    lines.append(
        f'<text x="{width-pad-80}" y="{height-8}" font-size="12" font-family="monospace">frame</text>'
    )
    lines.append(
        f'<text x="{pad+6}" y="{pad+12}" font-size="12" font-family="monospace">y_max={y_max:.5f}</text>'
    )
    lines.append(
        f'<text x="{pad+6}" y="{height-pad-4}" font-size="12" font-family="monospace">y_min={y_min:.5f}</text>'
    )

    for idx, (name, vals) in enumerate(values_dict.items()):
        color = colors[idx % len(colors)]
        lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{project(vals)}" />')
        lx = width - 220
        ly = legend_y + idx * 16
        lines.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+20}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
        lines.append(f'<text x="{lx+25}" y="{ly+4}" font-size="12" font-family="monospace">{name}</text>')
    lines.append("</svg>")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def bar_svg(values, title, out_path, width=1200, height=360):
    if not values:
        return
    pad = 40
    vmax = max(values)
    if vmax < 1e-12:
        vmax = 1.0
    n = len(values)
    bar_w = max(1.0, (width - 2 * pad) / max(1, n))
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="white" />',
        f'<text x="{pad}" y="20" font-size="16" font-family="monospace">{title}</text>',
        f'<line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#888" />',
        f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#888" />',
        f'<text x="{pad+6}" y="{pad+12}" font-size="12" font-family="monospace">max={max(values):.5f}</text>',
    ]
    for i, v in enumerate(values):
        x = pad + i * bar_w
        h = (height - 2 * pad) * (v / vmax)
        y = height - pad - h
        lines.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{max(1.0, bar_w-1):.2f}" height="{h:.2f}" fill="#1f77b4" />'
        )
    lines.append("</svg>")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def write_reports(analysis, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    chart_dir = out_dir / "charts"
    chart_dir.mkdir(parents=True, exist_ok=True)

    line_svg(
        {
            "root_x": analysis["series"]["root_x"],
            "root_y": analysis["series"]["root_y"],
            "root_z": analysis["series"]["root_z"],
        },
        "Root XYZ Over Frames",
        chart_dir / "root_xyz.svg",
    )
    line_svg(
        {"root_speed": analysis["series"]["root_speed"]},
        "Root Speed Over Frames",
        chart_dir / "root_speed.svg",
    )
    bar_svg(
        analysis["series"]["joint_jump_abs"],
        "Boundary Joint Angle Jump |last-first|",
        chart_dir / "joint_jump_abs.svg",
    )
    bar_svg(
        analysis["series"]["joint_vel_mismatch_abs"],
        "Boundary Joint Velocity Mismatch |v_end-v_start|",
        chart_dir / "joint_vel_mismatch_abs.svg",
    )

    report_json = out_dir / "report.json"
    report_json.write_text(json.dumps(analysis, indent=2), encoding="utf-8")

    b = analysis["boundary"]
    v = analysis["velocity_boundary"]
    s = analysis["velocity_smoothness"]
    m = analysis["meta"]
    report_md = out_dir / "report.md"
    report_md.write_text(
        "\n".join(
            [
                "# Continuity Check Report",
                "",
                f"- source: `{m['source_file']}`",
                f"- frames: `{m['frames']}`",
                f"- frame_duration: `{m['frame_duration']}`",
                f"- joint_count: `{m['joint_count']}` ({m['joint_count_source']})",
                "",
                "## Boundary Jump (last vs first)",
                f"- root_xyz_jump: `{[round(x, 5) for x in b['root_xyz_jump']]}`",
                f"- root_xyz_jump_norm: `{b['root_xyz_jump_norm']:.5f}`",
                f"- joint_jump_abs_max: `{b['joint_jump_abs_max']:.5f}`",
                f"- joint_jump_abs_mean: `{b['joint_jump_abs_mean']:.5f}`",
                "",
                "## Velocity Continuity (end vs start)",
                f"- root_vel_mismatch: `{[round(x, 5) for x in v['root_vel_mismatch']]}`",
                f"- root_vel_mismatch_norm: `{v['root_vel_mismatch_norm']:.5f}`",
                f"- joint_vel_mismatch_abs_max: `{v['joint_vel_mismatch_abs_max']:.5f}`",
                f"- joint_vel_mismatch_abs_mean: `{v['joint_vel_mismatch_abs_mean']:.5f}`",
                "",
                "## Smoothness Summary",
                f"- root_speed_max: `{s['root_speed_max']:.5f}`",
                f"- root_speed_mean: `{s['root_speed_mean']:.5f}`",
                f"- root_speed_delta_abs_max: `{s['root_speed_delta_abs_max']:.5f}`",
                f"- root_speed_delta_abs_mean: `{s['root_speed_delta_abs_mean']:.5f}`",
                "",
                "## Charts",
                "- `charts/root_xyz.svg`",
                "- `charts/root_speed.svg`",
                "- `charts/joint_jump_abs.svg`",
                "- `charts/joint_vel_mismatch_abs.svg`",
            ]
        ),
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser(description="Motion continuity checker")
    parser.add_argument("--input_file", type=str, required=True, help="Path to motion txt/json")
    parser.add_argument(
        "--output_dir",
        type=str,
        default="reference_motion_generator/reference_motion_generator/recordings/continuity_reports",
        help="Base output directory",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="",
        help="Optional run tag appended to output folder name",
    )
    args = parser.parse_args()

    input_path = Path(args.input_file)
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    data = load_motion(input_path)
    analysis = compute_analysis(data, input_path)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = input_path.stem
    suffix = f"_{args.tag}" if args.tag else ""
    out_dir = Path(args.output_dir) / f"{name}_{ts}{suffix}"
    write_reports(analysis, out_dir)

    print(f"Report generated: {out_dir}")
    print(f"- {out_dir / 'report.md'}")
    print(f"- {out_dir / 'report.json'}")
    print(f"- {out_dir / 'charts'}")


if __name__ == "__main__":
    main()
