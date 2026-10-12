# -*- coding: utf-8 -*-
"""合成识别效果视频帧：左=原始灰度，右=算法标注，顶部标题栏。

输出到 vidframes/ 目录，再由 ffmpeg 编码为 mp4。
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

IN_DIR = "/home/smartcar/input"
OUT_DIR = "/home/smartcar/output"
DST_DIR = "/home/smartcar/vidframes"
SCALE = 3
BAR = 23
PW, PH = 160 * SCALE, 97 * SCALE      # 单幅面板 480 x 291
CANVAS = (PW * 2, BAR + PH)            # 960 x 314

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

LEFT_TITLE = "20th A Group - Image Algorithm"
LEFT_SUB = "LEFT: raw camera"
RIGHT_SUB = "RIGHT: detect + centerline"


def main():
    lo = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    hi = int(sys.argv[2]) if len(sys.argv) > 2 else 1007
    os.makedirs(DST_DIR, exist_ok=True)
    try:
        font = ImageFont.truetype(FONT_PATH, 15)
    except OSError:
        font = ImageFont.load_default()

    n = 0
    for i in range(lo, hi + 1):
        a_path = f"{IN_DIR}/{i}.jpg"
        b_path = f"{OUT_DIR}/{i}.jpg"
        if not (os.path.exists(a_path) and os.path.exists(b_path)):
            continue
        raw = Image.open(a_path).convert("RGB").resize((PW, PH), Image.LANCZOS)
        det = Image.open(b_path).convert("RGB").resize((PW, PH), Image.LANCZOS)

        canvas = Image.new("RGB", CANVAS, (0, 0, 0))
        canvas.paste(raw, (0, BAR))
        canvas.paste(det, (PW, BAR))

        d = ImageDraw.Draw(canvas)

        def width_of(text):
            try:
                return d.textlength(text, font=font)
            except AttributeError:
                return d.textsize(text, font=font)[0]

        d.text((6, 4), LEFT_TITLE, fill=(255, 220, 120), font=font)
        d.text((6 + width_of(LEFT_TITLE) + 22, 4), LEFT_SUB,
               fill=(200, 200, 200), font=font)
        d.text((PW + 6, 4), RIGHT_SUB, fill=(120, 255, 140), font=font)
        label = f"frame {i}"
        d.text((CANVAS[0] - width_of(label) - 6, 4), label,
               fill=(255, 255, 255), font=font)

        canvas.save(f"{DST_DIR}/{n:05d}.png")
        n += 1
    print("frames written:", n)


if __name__ == "__main__":
    main()
