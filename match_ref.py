# -*- coding: utf-8 -*-
"""把"弯道补线样例图"按画面内容匹配回源视频帧，再与本版输出并排比对。

样例图的文件名是哈希，无法直接知道对应第几帧，所以用缩略图灰度差做匹配。
样例图上叠了红/绿/蓝细线（标注），缩到 32x20 之后影响很小，不影响匹配。
"""
import os
import glob
from PIL import Image, ImageChops, ImageStat, ImageDraw, ImageFont

REF_DIR = "/home/smartcar/ref"
IN_DIR = "/home/smartcar/input"
OUT_DIR = "/home/smartcar/output"
DST = "/home/smartcar/refmatch"
THUMB = (32, 20)

os.makedirs(DST, exist_ok=True)
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 13)
except OSError:
    FONT = ImageFont.load_default()


def thumb(path):
    im = Image.open(path).convert("L")
    return im.resize(THUMB, Image.LANCZOS)


def mean_abs_diff(a, b):
    return ImageStat.Stat(ImageChops.difference(a, b)).mean[0]


# 预先把所有输入帧压成缩略图，避免重复解码
inputs = {}
for i in range(0, 1008):
    p = os.path.join(IN_DIR, "%d.jpg" % i)
    if os.path.exists(p):
        inputs[i] = thumb(p)

print("可用输入帧:", len(inputs))

refs = sorted(glob.glob(os.path.join(REF_DIR, "*.jpg")))
print("样例图:", len(refs))

rows = []
for rp in refs:
    name = os.path.splitext(os.path.basename(rp))[0]
    rt = thumb(rp)
    best_i, best_d = None, 1e9
    for i, it in inputs.items():
        d = mean_abs_diff(rt, it)
        if d < best_d:
            best_d, best_i = d, i
    rows.append((name, best_i, best_d))
    print("%s  ->  frame %d   (缩略图灰度差 %.2f)" % (name[:12], best_i, best_d))

# 逐对生成并排图
SCALE = 4
PANEL = (160 * SCALE, 97 * SCALE)
BAR = 22
pairs = []
for name, idx, d in rows:
    ref = Image.open(os.path.join(REF_DIR, name + ".jpg")).convert("RGB")
    out_p = os.path.join(OUT_DIR, "%d.jpg" % idx)
    if not os.path.exists(out_p):
        print("缺少输出帧", out_p)
        continue
    out = Image.open(out_p).convert("RGB")
    ref = ref.resize(PANEL, Image.LANCZOS)
    out = out.resize(PANEL, Image.LANCZOS)

    canvas = Image.new("RGB", (PANEL[0] * 2, BAR + PANEL[1]), (18, 18, 20))
    canvas.paste(ref, (0, BAR))
    canvas.paste(out, (PANEL[0], BAR))
    dr = ImageDraw.Draw(canvas)
    lab = "frame %d" % idx
    dr.text((6, 4), "REF (teacher sample)", fill=(255, 220, 120), font=FONT)
    dr.text((PANEL[0] + 6, 4), "OURS  (v3d)", fill=(120, 255, 140), font=FONT)
    dr.text((canvas.size[0] - 8 * len(lab) - 8, 4), lab, fill=(230, 230, 230), font=FONT)
    canvas.save(os.path.join(DST, "%s_pair.png" % name[:12]))
    pairs.append((canvas, name, idx))

# 3x3 总览
if pairs:
    cols, rowsn = 3, (len(pairs) + 2) // 3
    cw, ch = pairs[0][0].size
    sw, sh = cw // 2, ch // 2
    sheet = Image.new("RGB", (sw * cols, sh * rowsn), (12, 12, 14))
    for k, (c, name, idx) in enumerate(pairs):
        sheet.paste(c.resize((sw, sh), Image.LANCZOS), ((k % cols) * sw, (k // cols) * sh))
    sheet.save(os.path.join(DST, "sheet.png"))
    print("总览图:", os.path.join(DST, "sheet.png"), sheet.size)
