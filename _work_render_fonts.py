#!/usr/bin/env python3
"""Рендер образцов шрифтов устройства в PNG (U8g2, реальные глифы из u8g2_fonts.c)."""
import importlib.util, sys, os
from PIL import Image, ImageDraw

spec = importlib.util.spec_from_file_location("fp", os.path.join(os.path.dirname(__file__), "_work_fontpreview.py"))
fp = importlib.util.module_from_spec(spec); sys.argv = ["x"]
spec.loader.exec_module(fp)

SRC = open(r"C:/my/libraries/U8g2/src/clib/u8g2_fonts.c", encoding="utf-8", errors="replace").read()

FONTS = [
    ("Tiny",     "u8g2_font_5x7_t_cyrillic",     "подсказки, шкалы"),
    ("Small",    "u8g2_font_6x13_t_cyrillic",    "основной мелкий"),
    ("SmallB",   "u8g2_font_6x13B_t_cyrillic",   "акцент/заголовок"),
    ("Body",     "u8g2_font_7x13_t_cyrillic",    "основной текст"),
    ("BodyWide", "u8g2_font_8x13_t_cyrillic",    "текст пошире"),
    ("Uni16",    "u8g2_font_unifont_t_cyrillic", "широкая кириллица"),
    ("Med",      "u8g2_font_10x20_t_cyrillic",   "заголовки, значения"),
    ("Large",    "u8g2_font_inr24_t_cyrillic",   "крупные значения"),
    ("XLarge",   "u8g2_font_inr33_t_cyrillic",   "очень крупные"),
    ("Huge",     "u8g2_font_inr46_t_cyrillic",   "сплэш/акцент"),
]

TEXT1 = "Автодом КамАЗ Левелер 0123456789"
TEXT2 = "АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ абвгдеёжзийклмнопрстуфхцчшщъыьэюя"
TEXT3 = "ДАВЛЕНИЕ 4,5 бар  КРЕН -2,3  РУЧНОЙ"

SCALE = 3
PAD = 10
LABEL_W = 300


def render_font(name, fname, note, texts):
    f = fp.Font(fname, fp.parse_c_array(SRC, fname))
    mono = any(t in fname for t in ("5x7", "6x13", "7x13", "8x13", "10x20"))
    rows = []
    for t in texts:
        # рисуем каждый глиф с шагом max_w+1 и выравниванием по базовой линии
        glyphs = []
        pen = 0
        for ch in t:
            try:
                g = f.glyph(ord(ch))
            except Exception:
                g = None
            if g is None:
                pen += f.max_w if mono else max(3, f.max_w // 2)
                glyphs.append(None)
                continue
            glyphs.append((pen, g))
            pen += (f.max_w if mono else g["w"] + 2)
        W = max(1, pen)
        H = f.max_h + 2
        canvas = [[0] * W for _ in range(H)]
        for item in glyphs:
            if item is None:
                continue
            p0, g = item
            for yy in range(g["h"]):
                for xx in range(g["w"]):
                    if g["px"][yy][xx]:
                        gx = p0 + xx
                        gy = yy + (H - 1 - g["h"])
                        if 0 <= gx < W and 0 <= gy < H:
                            canvas[gy][gx] = 1
        rows.append((W, H, canvas))
    return f, rows


def main():
    margin = 16
    blocks = []
    for name, fname, note in FONTS:
        try:
            f, rows = render_font(name, fname, note, [TEXT1, TEXT3])
        except Exception as e:
            blocks.append((name, fname, note, None, str(e)))
            continue
        blocks.append((name, fname, note, (f, rows), None))

    # геометрия
    total_h = margin
    for name, fname, note, data, err in blocks:
        if data is None:
            total_h += 30
            continue
        f, rows = data
        for W, H, _ in rows:
            total_h += H * SCALE + 6
        total_h += 26
    total_w = LABEL_W + 1200 + 2 * margin

    img = Image.new("RGB", (total_w, total_h + margin), (16, 16, 20))
    dr = ImageDraw.Draw(img)
    y = margin
    for name, fname, note, data, err in blocks:
        dr.text((margin, y), f"{name}  —  {fname}", fill=(160, 220, 160))
        y += 14
        if data is None:
            dr.text((margin, y), f"ОШИБКА: {err}", fill=(255, 120, 120))
            y += 30
            continue
        dr.text((margin, y), note, fill=(140, 140, 150))
        y += 12
        f, rows = data
        for W, H, canvas in rows:
            for yy in range(H):
                for xx in range(W):
                    if canvas[yy][xx]:
                        dr.rectangle([LABEL_W + xx * SCALE, y + yy * SCALE,
                                      LABEL_W + (xx + 1) * SCALE - 1, y + (yy + 1) * SCALE - 1],
                                     fill=(235, 235, 235))
            y += H * SCALE + 6
        y += 18
    out = os.path.join(os.path.dirname(__file__), "_work_fonts_preview.png")
    img.save(out)
    print("saved:", out, img.size)


if __name__ == "__main__":
    main()
