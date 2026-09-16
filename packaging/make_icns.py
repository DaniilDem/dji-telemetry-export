#!/usr/bin/env python3
"""Generate AppIcon.icns for the macOS bundle without third-party dependencies.

Draws the same mark as the in-app icon (dark disc, green G-meter arc) as PNGs
and packs them with `iconutil`, which exists on every macOS runner.
"""
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib


def png(width, height, rows):
    raw = b"".join(b"\x00" + bytes(row) for row in rows)

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def render(size):
    c = (size - 1) / 2
    rows = []
    for y in range(size):
        row = []
        for x in range(size):
            dx, dy = x - c, y - c
            r = math.hypot(dx, dy)
            if r > c * 0.98:
                row += [0, 0, 0, 0]
                continue
            ring = abs(r - c * 0.62) < c * 0.12
            angle = math.atan2(dy, dx)
            gap = -0.5 < angle < 0.5
            dot = r < c * 0.18
            if (ring and not gap) or dot:
                row += [76, 217, 100, 255]
            else:
                row += [28, 30, 36, 255]
        rows.append(row)
    return png(size, size, rows)


def main(out):
    if shutil.which("iconutil") is None:
        sys.exit("iconutil not found (macOS only)")
    with tempfile.TemporaryDirectory() as tmp:
        iconset = os.path.join(tmp, "AppIcon.iconset")
        os.mkdir(iconset)
        for size in (16, 32, 128, 256, 512):
            with open(os.path.join(iconset, "icon_%dx%d.png" % (size, size)), "wb") as f:
                f.write(render(size))
            with open(os.path.join(iconset, "icon_%dx%d@2x.png" % (size, size)), "wb") as f:
                f.write(render(size * 2))
        subprocess.check_call(["iconutil", "-c", "icns", iconset, "-o", out])
    print("wrote %s" % out)


if __name__ == "__main__":
    main(sys.argv[1])
