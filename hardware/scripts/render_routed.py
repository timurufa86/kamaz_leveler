# -*- coding: utf-8 -*-
"""Render board placement + tracks to PNG."""
import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import pcbnew
from PIL import Image, ImageDraw, ImageFont

PCB = r'C:\my\for cursor\kamaz_leveler\hardware\kicad\kamaz_leveler_hw.kicad_pcb'
OUT = r'C:\my\for cursor\kamaz_leveler\hardware\images\BOARD_routed.png'
b = pcbnew.LoadBoard(PCB)
W, H = 84.0, 59.0
S = 14.0
img = Image.new('RGB', (int(W * S), int(H * S)), (255, 255, 255))
d = ImageDraw.Draw(img)
X = lambda v: int((v + 0.5) * S)
Y = lambda v: int((v + 0.5) * S)
try:
    font = ImageFont.truetype(r'C:\Windows\Fonts\segoeui.ttf', 11)
    fs = ImageFont.truetype(r'C:\Windows\Fonts\segoeui.ttf', 9)
except Exception:
    font = fs = None

d.rectangle([X(0), Y(0), X(83), Y(58)], outline=(0, 0, 0), width=2)
COL = {'F': (200, 30, 30), 'B': (30, 80, 200)}


def crt(fp):
    layer = pcbnew.F_CrtYd if fp.GetLayer() == pcbnew.F_Cu else pcbnew.B_CrtYd
    try:
        bb = fp.GetCourtyard(layer).BBox()
        if bb.GetWidth() > 0:
            return (pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY()),
                    pcbnew.ToMM(bb.GetX() + bb.GetWidth()),
                    pcbnew.ToMM(bb.GetY() + bb.GetHeight()))
    except Exception:
        pass
    bb = fp.GetBoundingBox(False, False)
    return (pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY()),
            pcbnew.ToMM(bb.GetX() + bb.GetWidth()), pcbnew.ToMM(bb.GetY() + bb.GetHeight()))


# tracks first (under footprints)
for t in b.GetTracks():
    if t.Type() == pcbnew.PCB_TRACE_T:
        s, e = t.GetStart(), t.GetEnd()
        col = (255, 160, 160) if t.GetLayer() == pcbnew.F_Cu else (160, 190, 255)
        d.line([X(pcbnew.ToMM(s.x)), Y(pcbnew.ToMM(s.y)),
                X(pcbnew.ToMM(e.x)), Y(pcbnew.ToMM(e.y))], fill=col, width=1)
    elif t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        r = 2
        d.ellipse([X(pcbnew.ToMM(p.x)) - r, Y(pcbnew.ToMM(p.y)) - r,
                   X(pcbnew.ToMM(p.x)) + r, Y(pcbnew.ToMM(p.y)) + r], outline=(120, 120, 120))

d.rectangle([X(32), Y(13.7), X(82.6), Y(41.5)], outline=(150, 150, 150), width=2)
d.text((X(42), Y(27)), 'ESP32 DevKitC (модуль)', fill=(150, 150, 150), font=font)
d.rectangle([X(74), Y(8), X(83), Y(47)], outline=(230, 120, 0), width=2)
d.text((X(74.4), Y(27)), 'ANT', fill=(230, 120, 0), font=fs)

for fp in b.GetFootprints():
    ref = fp.GetReference()
    side = 'F' if fp.GetLayer() == pcbnew.F_Cu else 'B'
    x0, y0, x1, y1 = crt(fp)
    d.rectangle([X(x0), Y(y0), X(x1), Y(y1)], outline=COL[side], width=1)
    if ref in ('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8', 'H1', 'H2', 'H3', 'H4',
               'U9', 'U6', 'U5', 'U3', 'U4', 'U11', 'U12', 'U10', 'U1', 'L2', 'L3', 'L4',
               'F1', 'D1', 'Q1', 'JP1', 'D20'):
        d.text((X(x0) + 2, Y(y0) - 6), ref, fill=COL[side], font=fs)

d.text((X(1), Y(58.6)), 'Плата 83×58: 118 футпринтов, 1773 трассы/вии (2 слоя), 72 цепи не разведены.', fill=(60, 60, 60), font=font)
img.save(OUT)
print('saved', OUT, img.size)
