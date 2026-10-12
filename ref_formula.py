# -*- coding: utf-8 -*-
"""对若干样例，逐行比较：样例蓝线 vs (L+R)/2 vs L+半个标定宽度 vs 本版中线。"""
import os
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
H = 97
W_AT_0, W_SLOPE = 142.95, -1.2273

CASES = [("1a93d70250f3db71c64b859524186f10", 198),
         ("4c403d2a8ebcf437d7f12274f3e3059e", 73),
         ("fa97cc960762b5fd5e6eda5287b521a7", 421)]


def fam(path, row):
    im = Image.open(path).convert("RGB")
    px = im.load()
    py = H - 1 - row
    d = {'R': [], 'G': [], 'B': [], 'C': [], 'O': []}
    for x in range(im.size[0]):
        r, g, b = px[x, py]
        if max(r, g, b) - min(r, g, b) <= 40:
            continue
        if g > r + 30 and b > r + 30 and abs(g - b) <= 40:
            d['C'].append(x)
        elif b > r + 30 and b > g + 30:
            d['B'].append(x)
        elif r > g + 30 and r > b + 30:
            d['O' if (g - b) > 40 else 'R'].append(x)
        elif g > r + 30 and g > b + 30:
            d['G'].append(x)
    return {k: v[len(v) // 2] for k, v in d.items() if v}


def w_at(row):
    return max(12.0, min(160.0, W_AT_0 + W_SLOPE * row))


for name, idx in CASES:
    print("=" * 78)
    print("样例 %s  帧 %d" % (name[:12], idx))
    print("%4s | %4s %4s %4s %4s | %8s %8s %8s | %6s" %
          ("行", "RL", "RR", "RB", "RC", "(L+R)/2", "L+w/2", "R-w/2", "本版"))
    rp = os.path.join(REF_DIR, name + ".jpg")
    op = os.path.join(OUT_DIR, "%d.jpg" % idx)
    for row in range(20, 92, 4):
        a = fam(rp, row)
        b = fam(op, row)
        RL, RR, RB = a.get('R'), a.get('G'), a.get('B')
        if RL is None or RR is None or RB is None:
            continue
        w = w_at(row)
        print("%4d | %4s %4s %4s %4s | %8.1f %8.1f %8.1f | %6s" %
              (row, RL, RR, RB, a.get('C', '-'),
               (RL + RR) / 2.0, RL + w / 2, RR - w / 2,
               b.get('B', b.get('O', '-'))))
