# norn_xy.py

Normalize motion files by subtracting the first-frame root translation.

Default behavior:
- Normalize `x/y` only
- Keep `z` unchanged
- Output numeric values rounded to 5 decimal places

This is usually safer for AMP training because ground height is preserved.

## Input/Output

Input file format:
- JSON-like motion file with a top-level `Frames` list
- Each frame must have at least 3 values: `[x, y, z, ...]`

Output:
- Writes normalized files to `--output`
- Keeps original filename
- Adds a `Normalization` block:
  - `type`
  - `xy_only`
  - `offset`

## Usage

Single file:

```bash
python norn_xy.py \
  --input_file pikachu_walk_txt/pikachu_walk_0.185_0.0_0.0.txt \
  --output pikachu_walk_txt_norm
```

Directory batch:

```bash
python norn_xy.py \
  --input_dir pikachu_walk_txt \
  --output pikachu_walk_txt_norm
```

Normalize `x/y/z` (override default):

```bash
python norn_xy.py \
  --input_dir pikachu_walk_txt \
  --output pikachu_walk_txt_norm_xyz \
  --xyz
```

## Notes

- Use exactly one of `--input_file` or `--input_dir`.
- Supported file extensions for batch mode: `.txt`, `.json`.
- The script rewrites output as formatted JSON with one frame per line.
