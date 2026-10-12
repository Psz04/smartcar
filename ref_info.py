# -*- coding: utf-8 -*-
import glob
import os
from PIL import Image

for p in sorted(glob.glob("/home/smartcar/ref/*.jpg")):
    im = Image.open(p).convert("RGB")
    px = im.load()
    w, h = im.size
    cols = {"R": set(), "G": set(), "B": set(), "C": set(), "O": set()}
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            if r > 150 and g < 110 and b < 110:
                cols["R"].add(x)
            elif g > 150 and r < 110 and b < 110:
                cols["G"].add(x)
            elif b > 150 and r < 110 and g < 110:
                cols["B"].add(x)
            elif g > 150 and b > 150 and r < 110:
                cols["C"].add(x)
            elif r > 180 and 100 < g < 200 and b < 110:
                cols["O"].add(x)
    print("%s size=%s  cyan=%s" %
          (os.path.basename(p)[:12], im.size,
           sorted(cols["C"])[:4] if cols["C"] else "-"))
