# -*- coding: utf-8 -*-
"""打印某一行的蓝色像素簇，看清样例图里到底有几条蓝线。"""
import os
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
H, W = 97, 160


def blue_cols(path, row):
    im = Image.open(path).convert("RGB")
    px = im.load()
    py = H - 1 - row
    cols = []
    for x in range(im.size[0]):
        r, g, b = px[x, py]
        if max(r, g, b) - min(r, g, b) <= 40:
            continue
        if b > r + 30 and b > g + 30:
            cols.append((x, (r, g, b)))
    return cols


def clusters(cols):
    out = []
    cur = []
    for x, c in cols:
        if cur and x - cur[-1][0] > 3:
            out.append(cur)
            cur = []
        cur.append((x, c))
    if cur:
        out.append(cur)
    return out


for name, idx, rows in [("44523fd1b320bc3e90ccaffd78657fb8", 88, range(30, 96, 6)),
                        ("fa97cc960762b5fd5e6eda5287b521a7", 421, range(30, 96, 6))]:
    print("=" * 70)
    print("样例 %s  帧 %d" % (name[:12], idx))
    rp = os.path.join(REF_DIR, name + ".jpg")
    op = os.path.join(OUT_DIR, "%d.jpg" % idx)
    for row in rows:
        cs = clusters(blue_cols(rp, row))
        desc = " | ".join("[%d..%d] n=%d 色%s" % (c[0][0], c[-1][0], len(c), c[len(c) // 2][1])
                          for c in cs)
        ocs = clusters(blue_cols(op, row))
        odesc = " | ".join("[%d..%d]" % (c[0][0], c[-1][0]) for c in ocs)
        print(" 行%3d REF: %-52s OURS: %s" % (row, desc, odesc if ocs else "-"))
