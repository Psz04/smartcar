# -*- coding: utf-8 -*-
"""逐行提取"样例图"与"本版输出"里的中线列坐标，做定量形状比对。

配色（RGB）：红=左边沿，绿=右边沿，蓝=中线；橙色=本版的外推/十字补线段。
样例图另有青色竖线（画面中心）。

对每一行取蓝色像素质心的列坐标，然后比较 REF 与 OURS 的中线差。
"""
import os
import glob
from PIL import Image

REF_DIR = "/home/smartcar/ref"
OUT_DIR = "/home/smartcar/output"
DST = "/home/smartcar/refmatch"
H, W = 97, 160


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
    """返回 {行号: {'B': 列中值, 'R': .., 'G': .., 'O': ..}}，行号=IMG 行(0 在下)"""
    im = Image.open(path).convert("RGB")
    px = im.load()
    cols = {}
    for py in range(im.size[1]):
        row = H - 1 - py
        for pxx in range(im.size[0]):
            c = classify(*px[pxx, py])
            if c:
                cols.setdefault(row, {}).setdefault(c, []).append(pxx)
    out = {}
    for row, d in cols.items():
        out[row] = {k: sorted(v)[len(v) // 2] for k, v in d.items()}
    return out


_idx = {"18226adcdaa1": 211, "1a93d70250f3": 198, "1e4acbcaab8b": 382,
        "2a2440295f5d": 103, "44523fd1b320": 88, "4c403d2a8ebc": 73,
        "51d6367feeae": 86, "d0148350f60c": 223, "fa97cc960762": 421}
_full = {}
for p in glob.glob(os.path.join(REF_DIR, "*.jpg")):
    b = os.path.splitext(os.path.basename(p))[0]
    if b[:12] in _idx:
        _full[b[:12]] = p
pairs = [(k, _idx[k], _full[k]) for k in sorted(_idx) if k in _full]
print("找到样例图:", len(pairs), "/", len(_idx))
print()

print('%-14s %5s %6s %7s %7s %8s %8s' %
      ('样例', '帧', '行数', '蓝差均值', '蓝差中位', '最大正偏', '最大负偏'))
print('-' * 62)
allabs = []
for name, idx, rp in pairs:
    op = os.path.join(OUT_DIR, "%d.jpg" % idx)
    if not (os.path.exists(rp) and os.path.exists(op)):
        print(name, idx, "缺文件")
        continue
    R = extract(rp)
    O = extract(op)
    diffs = []
    for row in sorted(R):
        if 'B' not in R[row]:
            continue
        o = O.get(row, {})
        ov = o.get('B', o.get('O'))     # 本版的中线；外推段画成橙色
        if ov is None:
            continue
        diffs.append(ov - R[row]['B'])
    if not diffs:
        print('%-14s %5d  无重叠行' % (name, idx))
        continue
    a = [abs(x) for x in diffs]
    allabs += a
    print('%-14s %5d %6d %7.2f %7d %8d %8d' %
          (name[:12], idx, len(diffs), sum(a) / len(a), sorted(a)[len(a) // 2],
           max(diffs), min(diffs)))

# 叠加可视化：把 REF 蓝线与本版蓝线画在同一张灰度底图上
try:
    from PIL import ImageDraw
    for name, idx, rp in pairs[:4]:
        op = os.path.join(OUT_DIR, "%d.jpg" % idx)
        if not (os.path.exists(rp) and os.path.exists(op)):
            continue
        base = Image.open(os.path.join("/home/smartcar/input", "%d.jpg" % idx)).convert("RGB")
        base = base.resize((W * 4, H * 4), Image.LANCZOS)
        dr = ImageDraw.Draw(base)
        R = extract(rp)
        O = extract(op)
        for row in R:
            if 'B' in R[row]:
                dr.point((R[row]['B'] * 4, (H - 1 - row) * 4), fill=(255, 60, 60))
        for row in O:
            ov = O[row].get('B', O[row].get('O'))
            if ov is not None:
                dr.point((ov * 4, (H - 1 - row) * 4 + 1), fill=(60, 200, 255))
        base.save(os.path.join(DST, "overlay_%s.png" % name[:12]))
    print("\n叠加图已生成: overlay_*.png  (红=样例中线, 青=本版中线)")
except Exception as e:
    print("叠加图失败:", e)

if allabs:
    print('\n全部样例合并: 行数 %d, 平均绝对差 %.2f 列, 中位 %d 列, 90 分位 %d 列'
          % (len(allabs), sum(allabs) / len(allabs),
             sorted(allabs)[len(allabs) // 2], sorted(allabs)[int(len(allabs) * 0.9)]))
