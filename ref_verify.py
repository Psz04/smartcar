# -*- coding: utf-8 -*-
"""样例图 -> 源帧 的稳健匹配 + 逐行中线比对。

匹配的改进点：样例图上叠了彩色标注线，直接缩略会污染灰度和。
这里先把彩色像素用"该行非彩色像素的中位灰度"填掉，再做缩略图匹配。
"""
import glob
import os
from PIL import Image, ImageChops, ImageStat, ImageDraw

REF_DIR = "/home/smartcar/ref"
IN_DIR = "/home/smartcar/input"
OUT_DIR = "/home/smartcar/output"
DST = "/home/smartcar/refmatch"
H, W = 97, 160
THUMB = (40, 24)
os.makedirs(DST, exist_ok=True)


def is_anno(r, g, b):
    mx, mn = max(r, g, b), min(r, g, b)
    return (mx - mn) > 55


def clean_gray(path):
    """把彩色标注像素填成该行中位灰度，返回 L 图。"""
    im = Image.open(path).convert("RGB")
    px = im.load()
    w, h = im.size
    out = Image.new("L", (w, h))
    op = out.load()
    for y in range(h):
        vals = []
        for x in range(w):
            r, g, b = px[x, y]
            if not is_anno(r, g, b):
                vals.append((r * 299 + g * 587 + b * 114) // 1000)
        med = sorted(vals)[len(vals) // 2] if vals else 128
        for x in range(w):
            r, g, b = px[x, y]
            if is_anno(r, g, b):
                op[x, y] = med
            else:
                op[x, y] = (r * 299 + g * 587 + b * 114) // 1000
    return out


def classify(r, g, b):
    """相对色相判据：样例图经过 JPEG 压缩，纯色被冲淡（例如中线是浅蓝 (160,160,224)），
    所以不能再用绝对阈值，只能看"哪个通道明显占优"。"""
    mx, mn = max(r, g, b), min(r, g, b)
    if mx - mn <= 40:
        return None
    if g > r + 30 and b > r + 30 and abs(g - b) <= 40:
        return 'C'                     # 青：绿蓝同时压过红，且绿≈蓝
    if b > r + 30 and b > g + 30:
        return 'B'                     # 蓝：中线（含浅蓝/深蓝两种）
    if r > g + 30 and r > b + 30:
        return 'O' if (g - b) > 40 else 'R'   # 橙(延伸段) vs 红(左边沿)
    if g > r + 30 and g > b + 30:
        return 'G'
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


inputs = {}
for i in range(0, 1008):
    p = os.path.join(IN_DIR, "%d.jpg" % i)
    if os.path.exists(p):
        inputs[i] = Image.open(p).convert("L").resize(THUMB, Image.LANCZOS)

print("输入帧:", len(inputs))

pairs = []
for rp in sorted(glob.glob(os.path.join(REF_DIR, "*.jpg"))):
    name = os.path.splitext(os.path.basename(rp))[0]
    rt = clean_gray(rp).resize(THUMB, Image.LANCZOS)
    best_i, best_d = None, 1e9
    for i, it in inputs.items():
        d = ImageStat.Stat(ImageChops.difference(rt, it)).mean[0]
        if d < best_d:
            best_d, best_i = d, i
    pairs.append((name, best_i, best_d, rp))
    print("%s -> frame %3d  (匹配差 %.2f)" % (name[:12], best_i, best_d))

print()
print('%-14s %5s %6s %8s %8s %7s %7s' %
      ('样例', '帧', '可比行', '差均值', '差中位', '正偏max', '负偏max'))
print('-' * 62)
allabs = []
for name, idx, d, rp in pairs:
    op = os.path.join(OUT_DIR, "%d.jpg" % idx)
    if not os.path.exists(op):
        print(name[:12], idx, "缺输出帧")
        continue
    R = extract(rp)
    O = extract(op)
    diffs = []
    for row in sorted(R):
        if 'B' not in R[row]:
            continue
        o = O.get(row, {})
        ov = o.get('B', o.get('O'))
        if ov is None:
            continue
        diffs.append(ov - R[row]['B'])
    if not diffs:
        print('%-14s %5d %6d  (无重叠行，样例中线与本版中线的行号不交叠)' % (name[:12], idx, 0))
        continue
    a = [abs(x) for x in diffs]
    allabs += a
    print('%-14s %5d %6d %8.2f %8d %7d %7d' %
          (name[:12], idx, len(diffs), sum(a) / len(a), sorted(a)[len(a) // 2],
           max(diffs), min(diffs)))

    # 叠加图：红点=样例中线，青点=本版中线
    base = Image.open(os.path.join(IN_DIR, "%d.jpg" % idx)).convert("RGB")
    base = base.resize((W * 4, H * 4), Image.LANCZOS)
    dr = ImageDraw.Draw(base)
    for row, dd in R.items():
        if 'B' in dd:
            dr.rectangle([dd['B'] * 4 - 1, (H - 1 - row) * 4 - 1,
                          dd['B'] * 4 + 1, (H - 1 - row) * 4 + 1], fill=(255, 40, 40))
    for row, dd in O.items():
        ov = dd.get('B', dd.get('O'))
        if ov is not None:
            dr.rectangle([ov * 4, (H - 1 - row) * 4 + 1,
                          ov * 4 + 2, (H - 1 - row) * 4 + 3], fill=(40, 220, 255))
    base.save(os.path.join(DST, "ov_%s.png" % name[:12]))

if allabs:
    print('\n合并: 可比行 %d，平均绝对差 %.2f 列，中位 %d 列，p90 %d 列，最大 %d 列'
          % (len(allabs), sum(allabs) / len(allabs),
             sorted(allabs)[len(allabs) // 2],
             sorted(allabs)[int(len(allabs) * 0.9)], max(allabs)))
