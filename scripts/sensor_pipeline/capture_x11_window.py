"""Capture one X11 window by title with xwd and convert a 32-bit ZPixmap to PNG."""
import argparse
import re
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image


parser = argparse.ArgumentParser()
parser.add_argument("--title", default="Isaac Sim Python 6.2.0")
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--timeout", type=float, default=30.0)
parser.add_argument("--xwd-input", type=Path, help="Convert an existing XWD instead of finding a window")
args = parser.parse_args()


def convert_xwd(source: Path, output: Path):
    data = source.read_bytes()
    if len(data) < 100:
        raise RuntimeError("XWD is shorter than its fixed header")
    header = struct.unpack(">25I", data[:100])
    (
        header_size, version, pixmap_format, _depth, width, height, _xoffset,
        byte_order, _bitmap_unit, _bitmap_bit_order, _bitmap_pad,
        bits_per_pixel, bytes_per_line, _visual_class, _red_mask, _green_mask,
        _blue_mask, _bits_per_rgb, _colormap_entries, ncolors, window_width,
        window_height, _window_x, _window_y, _border_width,
    ) = header
    if (version, pixmap_format, bits_per_pixel) != (7, 2, 32):
        raise RuntimeError(
            f"Only XWD v7 ZPixmap 32-bpp is supported, got {(version, pixmap_format, bits_per_pixel)}"
        )
    if (width, height) != (window_width, window_height):
        raise RuntimeError("Pixmap and window dimensions differ")
    pixel_offset = header_size + ncolors * 12
    expected = pixel_offset + bytes_per_line * height
    if len(data) < expected:
        raise RuntimeError(f"Truncated XWD: {len(data)} < {expected}")
    rows = np.frombuffer(
        data, dtype=np.uint8, count=bytes_per_line * height, offset=pixel_offset
    ).reshape(height, bytes_per_line)
    pixels = rows[:, : width * 4].reshape(height, width, 4)
    rgb = pixels[:, :, 2::-1] if byte_order == 0 else pixels[:, :, :3]
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb.copy(), "RGB").save(output)
    return {"width": width, "height": height, "byte_order": byte_order}


if args.xwd_input:
    metadata = convert_xwd(args.xwd_input.resolve(), args.output.resolve())
    print({"output": str(args.output.resolve()), **metadata})
    raise SystemExit(0)

deadline = time.monotonic() + args.timeout
window_id = None
pattern = re.compile(r'^\s*(0x[0-9a-f]+)\s+"' + re.escape(args.title) + r'"')
while time.monotonic() < deadline:
    tree = subprocess.run(
        ["xwininfo", "-root", "-tree"], check=True, capture_output=True, text=True
    ).stdout
    match = next((pattern.search(line) for line in tree.splitlines() if pattern.search(line)), None)
    if match:
        window_id = match.group(1)
        break
    time.sleep(0.5)
if window_id is None:
    raise RuntimeError(f"No X11 window titled {args.title!r} within {args.timeout} seconds")

with tempfile.NamedTemporaryFile(prefix="robotdog_gui_", suffix=".xwd") as temporary:
    subprocess.run(
        ["xwd", "-silent", "-id", window_id, "-out", temporary.name], check=True
    )
    metadata = convert_xwd(Path(temporary.name), args.output.resolve())
print({"window_id": window_id, "output": str(args.output.resolve()), **metadata})
