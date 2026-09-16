#!/usr/bin/env python3
"""Cut the first N `djmd` samples out of a DJI clip into a small test fixture.

Output format: repeated  <u32 big-endian length><sample bytes>.
Only the moov atom and the selected samples are read, so this is fast even on
multi-gigabyte clips.

    python tools/extract_fixture.py DJI_0001.MP4 tests/fixtures/ac204_djmd_first300.bin 300
"""
import struct
import sys


def boxes(buf, start, end):
    pos = start
    while pos + 8 <= end:
        size = struct.unpack_from(">I", buf, pos)[0]
        kind = buf[pos + 4:pos + 8]
        header = 8
        if size == 1:
            size = struct.unpack_from(">Q", buf, pos + 8)[0]
            header = 16
        elif size == 0:
            size = end - pos
        if size < header or pos + size > end:
            return
        yield kind, pos + header, pos + size
        pos += size


def find(buf, path, start, end):
    head, rest = path[0], path[1:]
    for kind, s, e in boxes(buf, start, end):
        if kind != head:
            continue
        if not rest:
            yield s, e
        else:
            yield from find(buf, rest, s, e)


def main(video, out, count):
    f = open(video, "rb")
    total = f.seek(0, 2)
    pos = 0
    moov = None
    while pos < total:
        f.seek(pos)
        hdr = f.read(16)
        size = struct.unpack(">I", hdr[:4])[0]
        kind = hdr[4:8]
        if size == 1:
            size = struct.unpack(">Q", hdr[8:16])[0]
        elif size == 0:
            size = total - pos
        if kind == b"moov":
            f.seek(pos)
            moov = f.read(size)
            break
        pos += size
    if moov is None:
        sys.exit("no moov")

    for ts, te in find(moov, [b"moov", b"trak"], 0, len(moov)):
        for s, e in find(moov, [b"mdia", b"minf", b"stbl"], ts, te):
            fourcc = None
            sizes = []
            chunks = []
            stsc = []
            for kind, bs, be in boxes(moov, s, e):
                if kind == b"stsd":
                    fourcc = moov[bs + 12:bs + 16]
                elif kind == b"stsz":
                    fixed = struct.unpack_from(">I", moov, bs + 4)[0]
                    n = struct.unpack_from(">I", moov, bs + 8)[0]
                    sizes = [fixed] * n if fixed else list(struct.unpack_from(f">{n}I", moov, bs + 12))
                elif kind in (b"stco", b"co64"):
                    n = struct.unpack_from(">I", moov, bs + 4)[0]
                    fmt = ">%d%s" % (n, "I" if kind == b"stco" else "Q")
                    chunks = list(struct.unpack_from(fmt, moov, bs + 8))
                elif kind == b"stsc":
                    n = struct.unpack_from(">I", moov, bs + 4)[0]
                    stsc = [struct.unpack_from(">III", moov, bs + 8 + i * 12)[:2] for i in range(n)]
            if fourcc != b"djmd":
                continue
            offsets = []
            si = 0
            for i, (first, spc) in enumerate(stsc):
                last = stsc[i + 1][0] - 1 if i + 1 < len(stsc) else len(chunks)
                for c in range(first, last + 1):
                    p = chunks[c - 1]
                    for _ in range(spc):
                        if si >= len(sizes):
                            break
                        offsets.append(p)
                        p += sizes[si]
                        si += 1
            with open(out, "wb") as o:
                for i in range(min(count, len(offsets))):
                    f.seek(offsets[i])
                    data = f.read(sizes[i])
                    o.write(struct.pack(">I", len(data)))
                    o.write(data)
            print(f"wrote {min(count, len(offsets))} samples to {out}")
            return
    sys.exit("no djmd track")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 300)
