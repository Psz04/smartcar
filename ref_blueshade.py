# -*- coding: utf-8 -*-
"""把样例图里的蓝色按深浅拆开：深蓝 vs 浅蓝，分别对照本版中线。
假设：深蓝=实际采用的中线，浅蓝=示意用的"延伸到弯道尽头"段。"""
import os
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
H = 97
NAME = "44523fd1b320bc3e90ccaffd78657fb8.jpg"
IDX = 88


def fam(path, row):
    im = Image.open(path).convert("RGB")
    px = im.load()
    py = H - 1 - row
    dark, light, other = [], [], []
    for x in range(im.size[0]):
        r, g, b = px[x, py]
        if max(r, g, b) - min(r, g, b) <= 40:
            continue
        if b > r + 30 and b > g + 30:
            if r < 90:
                dark.append(x)      # 深蓝：接近纯蓝
            else:
                light.append(x)     # 浅蓝：被白底冲淡
        else:
            other.append(x)
    def mid(a):
        return a[len(a) // 2] if a else None
    return mid(dark), mid(light)


print("行 | 样例深蓝 样例浅蓝 | 本版中线")
print("-" * 44)
for row in range(24, 78, 3):
    dk, lt = fam(os.path.join(REF_DIR, NAME), row)
    o = fam(os.path.join(OUT_DIR, "%d.jpg" % IDX), row)
    ov = o[0] if o[0] is not None else o[1]
    print("%3d | %8s %8s | %8s" %
          (row, dk if dk is not None else '-', lt if lt is not None else '-',
           ov if ov is not None else '-'))
