# -*- coding: utf-8 -*-
"""列出每张样例图里"非灰"像素的颜色分布，确认标注线的实际颜色。"""
import glob
import os
from collections import Counter
from PIL import Image

for p in sorted(glob.glob("/home/smartcar/ref/*.jpg")):
    im = Image.open(p).convert("RGB")
    px = im.load()
    w, h = im.size
    cnt = Counter()
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if max(r, g, b) - min(r, g, b) > 40:
                cnt[(r // 32 * 32, g // 32 * 32, b // 32 * 32)] += 1
    top = cnt.most_common(6)
    print("%s  非灰像素 %d" % (os.path.basename(p)[:12], sum(cnt.values())))
    for c, n in top:
        r, g, b = c
        if r > g and r > b:
            nm = "偏红"
        elif g > r and g > b:
            nm = "偏绿"
        elif b > r and b > g:
            nm = "偏蓝"
        else:
            nm = "?"
        print("    RGB~%-18s %5d  %s" % (str(c), n, nm))
