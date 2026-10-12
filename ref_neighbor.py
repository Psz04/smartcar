# -*- coding: utf-8 -*-
"""对"对不上"的样例，在匹配帧附近 ±6 帧内搜索，看是否存在某一帧能对上样例中线。
若是，则说明样例取自相邻帧（车在动，画面平移了几列），算法本身没问题。"""
import os
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
H, W = 97, 160

CASE = [("18226adcdaa1bee8441617e9769366d6", 211),
        ("1e4acbcaab8b42feef2a39fea2dcd76f", 382),
        ("2a2440295f5d4adfeb4762a2405a3331", 103),
        ("d0148350f60c6d8ec88d4369f10e6597", 223),
        ("fa97cc960762b5fd5e6eda5287b521a7", 421)]


def classify(r, g, b):
    if r > 150 and g < 110 and b < 110:
        return 'R'
    if g > 150 and r < 110 and b < 110:
        return 'G'
    if b > 150 and r < 110 and g < 110:
        return 'B'
    if g > 150 and b > 150 and r < 110:
        return 'C'
    if r > 180 and 100 < g < 200 and b < 110:
        return 'O'
    return None


def extract(path):
    im = Image.open(path).convert("RGB")
    px = im.load()
    cols = {}
    for py in range(im.size[1]):
        row = H - 1 - py
        for pxx in range(im.size[0]):
            c = classify(*px[pxx, py])
            if c:
                cols.setdefault(row, {}).setdefault(c, []).append(pxx)
    return {r: {k: sorted(v)[len(v) // 2] for k, v in d.items()} for r, d in cols.items()}


def score(R, op):
    if not os.path.exists(op):
        return None
    O = extract(op)
    diffs = []
    for row in R:
        if 'B' not in R[row]:
            continue
        o = O.get(row, {})
        ov = o.get('B', o.get('O'))
        if ov is None:
            continue
        diffs.append(abs(ov - R[row]['B']))
    if not diffs:
        return (999, 0)
    return (sum(diffs) / len(diffs), len(diffs))


for name, idx in CASE:
    R = extract(os.path.join(REF_DIR, name + ".jpg"))
    nref = sum(1 for r in R if 'B' in R[r])
    cands = []
    for d in range(-6, 7):
        s = score(R, os.path.join(OUT_DIR, "%d.jpg" % (idx + d)))
        if s:
            cands.append((s[0], idx + d, s[1]))
    cands.sort()
    print("样例 %s  匹配帧 %d  样例有中线行 %d" % (name[:12], idx, nref))
    for m, f, n in cands[:4]:
        tag = '  <-- 当前匹配' if f == idx else ''
        print("    frame %3d : 平均差 %7.2f 列, 可比行 %d%s" % (f, m, n, tag))
    print()
