# -*- coding: utf-8 -*-
"""把样例图的左/右/中线列，和本版输出的左/右/中线列，逐行并排打印。"""
import os
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
H = 97
NAME = "44523fd1b320bc3e90ccaffd78657fb8.jpg"
IDX = 88


def cols_of(path, row):
    im = Image.open(path).convert("RGB")
    px = im.load()
    py = H - 1 - row
    res = {'R': [], 'G': [], 'B': [], 'C': []}
    for x in range(im.size[0]):
        r, g, b = px[x, py]
        if max(r, g, b) - min(r, g, b) <= 40:
            continue
        if g > r + 30 and b > r + 30 and abs(g - b) <= 40:
            res['C'].append(x)
        elif b > r + 30 and b > g + 30:
            res['B'].append(x)
        elif r > g + 30 and r > b + 30:
            res['R'].append(x)
        elif g > r + 30 and g > b + 30:
            res['G'].append(x)
    return {k: (min(v), max(v)) for k, v in res.items() if v}


print("样例 44523fd1b320（帧 88） vs 本版输出")
print("%4s | %-24s | %-24s" % ("行", "REF  左/右/中线/青", "OURS 左/右/中线"))
print("-" * 62)
rp = os.path.join(REF_DIR, NAME)
op = os.path.join(OUT_DIR, "%d.jpg" % IDX)
for row in range(24, 90, 3):
    a = cols_of(rp, row)
    b = cols_of(op, row)

    def fmt(d, keys):
        parts = []
        for k in keys:
            if k in d:
                lo, hi = d[k]
                parts.append("%s%d-%d" % (k, lo, hi) if lo != hi else "%s%d" % (k, lo))
        return " ".join(parts) if parts else "-"

    print("%4d | %-24s | %-24s" % (row, fmt(a, ['R', 'G', 'B', 'C']), fmt(b, ['R', 'G', 'B'])))
