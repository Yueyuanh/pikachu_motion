#!/usr/bin/env python3
"""
Normalize motion files by subtracting the first frame root translation.

Supported inputs:
- Single file: --input_file <path>
- Directory batch: --input_dir <dir>

Output:
- Written to --output directory with the same filename(s)
"""

import argparse
import json
from pathlib import Path

DECIMALS = 5


def validate_frames(data, path):
    if "Frames" not in data or not isinstance(data["Frames"], list):
        raise ValueError(f"{path}: missing or invalid 'Frames'")
    if not data["Frames"]:
        raise ValueError(f"{path}: 'Frames' is empty")
    first = data["Frames"][0]
    if not isinstance(first, list) or len(first) < 3:
        raise ValueError(f"{path}: frame format invalid, expected at least 3 values")


def normalize_frames(data, xy_only=True):
    frames = data["Frames"]
    x0, y0, z0 = frames[0][0], frames[0][1], frames[0][2]

    for frame in frames:
        if not isinstance(frame, list) or len(frame) < 3:
            raise ValueError("Encountered invalid frame while normalizing")
        frame[0] -= x0
        frame[1] -= y0
        if not xy_only:
            frame[2] -= z0

    data["Normalization"] = {
        "type": "first_frame_translation",
        "xy_only": xy_only,
        "offset": [x0, y0, 0.0 if xy_only else z0],
    }
    return data

def round_floats(value, decimals=DECIMALS):
    if isinstance(value, float):
        return round(value, decimals)
    if isinstance(value, list):
        return [round_floats(v, decimals) for v in value]
    if isinstance(value, dict):
        return {k: round_floats(v, decimals) for k, v in value.items()}
    return value


def write_motion_file(path, data):
    # Keep the same style used by existing tools: one frame per line.
    rounded_data = round_floats(data, DECIMALS)
    with open(path, "w", encoding="utf-8") as f:
        f.write("{\n")
        keys = [k for k in rounded_data.keys() if k != "Frames"]
        for i, key in enumerate(keys):
            comma = "," if i < len(keys) - 1 or "Frames" in rounded_data else ""
            f.write(f'  "{key}": {json.dumps(rounded_data[key])}{comma}\n')

        if "Frames" in rounded_data:
            if keys:
                # Ensure preceding entry already ends with comma when Frames follows.
                pass
            f.write('  "Frames": [\n')
            frames = rounded_data["Frames"]
            for i, frame in enumerate(frames):
                line = json.dumps(frame)
                if i < len(frames) - 1:
                    f.write(f"    {line},\n")
                else:
                    f.write(f"    {line}\n")
            f.write("  ]\n")
        f.write("}\n")


def process_file(input_path, output_path, xy_only=True):
    with open(input_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    validate_frames(data, input_path)
    normalized = normalize_frames(data, xy_only=xy_only)
    write_motion_file(output_path, normalized)


def process_dir(input_dir, output_dir, xy_only=True):
    input_path = Path(input_dir)
    files = sorted(
        [p for p in input_path.iterdir() if p.is_file() and p.suffix.lower() in {".txt", ".json"}]
    )
    if not files:
        print(f"No .txt/.json files found in {input_dir}")
        return

    for file_path in files:
        out_path = Path(output_dir) / file_path.name
        process_file(str(file_path), str(out_path), xy_only=xy_only)
        print(f"Normalized: {file_path.name} -> {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Normalize first-frame translation for motion files"
    )
    parser.add_argument("--input_file", type=str, help="Path to a single input file")
    parser.add_argument("--input_dir", type=str, help="Path to input directory")
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output directory for normalized files",
    )
    parser.add_argument(
        "--xyz",
        action="store_true",
        help="Normalize x/y/z. Default behavior normalizes x/y only.",
    )
    args = parser.parse_args()

    if bool(args.input_file) == bool(args.input_dir):
        raise ValueError("Use exactly one of --input_file or --input_dir")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.input_file:
        input_path = Path(args.input_file)
        if not input_path.exists():
            raise FileNotFoundError(f"Input file not found: {input_path}")
        out_path = output_dir / input_path.name
        process_file(str(input_path), str(out_path), xy_only=not args.xyz)
        print(f"Normalized: {input_path} -> {out_path}")
    else:
        input_dir = Path(args.input_dir)
        if not input_dir.exists() or not input_dir.is_dir():
            raise FileNotFoundError(f"Input directory not found: {input_dir}")
        process_dir(str(input_dir), str(output_dir), xy_only=not args.xyz)


if __name__ == "__main__":
    main()
