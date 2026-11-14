#!/usr/bin/env python3
"""Декодер шрифтов U8g2 (формат 't', 8/16-битные кодировки) + рендер образцов.
Порт u8g2_font.c: read_font_info, get_glyph_data, decode_glyph, decode_len.
"""
import re
import sys

U8G2_FONTS_C = r"C:/my/libraries/U8g2/src/clib/u8g2_fonts.c"
DATA_STRUCT_SIZE = 23


def parse_c_array(src: str, name: str) -> bytes:
    m = re.search(r"const uint8_t\s+" + re.escape(name) + r"\s*\[\d+\][^=]*=\s*(.*?);\s*\n",
                  src, re.S)
    if not m:
        raise KeyError(name)
    body = m.group(1)
    out = bytearray()
    for lit in re.finditer(r'"((?:[^"\\]|\\.)*)"', body):
        s = lit.group(1)
        i = 0
        while i < len(s):
            c = s[i]
            if c != "\\":
                out.append(ord(c)); i += 1; continue
            i += 1
            c2 = s[i]
            if c2 in "01234567":
                d = c2; i += 1
                while i < len(s) and len(d) < 3 and s[i] in "01234567":
                    d += s[i]; i += 1
                out.append(int(d, 8)); continue
            if c2 == "x":
                i += 1; h = ""
                while i < len(s) and len(h) < 2 and s[i] in "0123456789abcdefABCDEF":
                    h += s[i]; i += 1
                out.append(int(h, 16)); continue
            out.append({"n": 10, "t": 9, "r": 13, "\\": 92, '"': 34, "'": 39, "0": 0}.get(c2, ord(c2)))
            i += 1
    return bytes(out)


class Font:
    def __init__(self, name: str, data: bytes):
        self.name = name
        self.d = data
        b = data
        self.glyph_cnt = b[0]
        self.bits_per_0 = b[2]
        self.bits_per_1 = b[3]
        self.bits_w = b[4]
        self.bits_h = b[5]
        self.bits_x = b[6]
        self.bits_y = b[7]
        self.bits_dx = b[8]
        self.max_w = b[9]
        self.max_h = b[10]
        self.y_off = b[12]
        self.ascent_A = b[13]
        self.descent_g = b[14]
        self.pos_upper = (b[17] << 8) | b[18]
        self.pos_lower = (b[19] << 8) | b[20]
        self.pos_unicode = (b[21] << 8) | b[22]
        self.base = DATA_STRUCT_SIZE

    # --- поиск глифа (u8g2_font_get_glyph_data) ---
    def glyph_data(self, enc: int):
        font = self.base
        if enc <= 255:
            if enc >= ord('a'):
                font += self.pos_lower
            elif enc >= ord('A'):
                font += self.pos_upper
            while True:
                if self.d[font + 1] == 0:
                    return None
                if self.d[font] == enc:
                    return font + 2
                font += self.d[font + 1]
        else:
            font += self.pos_unicode
            lut = font
            while True:
                font += (self.d[lut] << 8) | self.d[lut + 1]
                e = self.d[lut + 2] | (self.d[lut + 3] << 8)
                lut += 4
                if e >= enc:
                    break
            while True:
                e = (self.d[font] << 8) | self.d[font + 1]
                if e == 0:
                    return None
                if e == enc:
                    return font + 3
                font += self.d[font + 2]

    # --- битовый ридер ---
    class Bits:
        def __init__(self, d, ptr):
            self.d, self.ptr, self.pos = d, ptr, 0

        def u(self, cnt):
            if cnt == 0:
                return 0
            val = self.d[self.ptr] >> self.pos
            if self.pos + cnt >= 8:
                self.ptr += 1
                val |= self.d[self.ptr] << (8 - self.pos)
                self.pos += cnt - 8
            else:
                self.pos += cnt
            return val & ((1 << cnt) - 1)

        def s(self, cnt):
            v = self.u(cnt)
            if v >= (1 << (cnt - 1)):
                v -= (1 << cnt)
            return v

    def glyph(self, enc):
        gd = self.glyph_data(enc)
        if gd is None:
            return None
        r = Font.Bits(self.d, gd)
        w = r.u(self.bits_w)
        h = r.u(self.bits_h)
        x = r.s(self.bits_x)
        y = r.s(self.bits_y)
        d = r.s(self.bits_dx)
        px = [[0] * w for _ in range(h)]
        if w > 0 and h > 0:
            cx = cy = 0
            total = w * h
            drawn = 0
            for _ in range(10000):
                a = r.u(self.bits_per_0)
                b = r.u(self.bits_per_1)
                while True:
                    for _ in range(a):
                        if cy >= h: break
                        cx += 1
                        if cx >= w:
                            cx = 0; cy += 1
                        drawn += 1
                    for _ in range(b):
                        if cy >= h: break
                        px[cy][cx] = 1
                        cx += 1
                        if cx >= w:
                            cx = 0; cy += 1
                        drawn += 1
                    if r.u(1) == 0:
                        break
                if drawn >= total or cy >= h:
                    break
        return dict(w=w, h=h, x=x, y=y, d=d, px=px)

    def text_bitmap(self, text, scale=1, spacing=1):
        """Возвращает (ширина, высота, 2D массив) для строки."""
        glyphs = []
        pen = 0
        for ch in text:
            g = self.glyph(ord(ch))
            if g is None:
                pen += 4
                glyphs.append(None)
                continue
            glyphs.append((pen, g))
            pen += g["d"] + spacing
        W = max(1, pen)
        H = self.max_h + 2
        canvas = [[0] * W for _ in range(H)]
        for item in glyphs:
            if item is None:
                continue
            pen0, g = item
            for yy in range(g["h"]):
                for xx in range(g["w"]):
                    if g["px"][yy][xx]:
                        gx = pen0 + g["x"] + xx
                        gy = (H - 1) - (g["h"] + g["y"]) + yy
                        if 0 <= gx < W and 0 <= gy < H:
                            canvas[gy][gx] = 1
        return W, H, canvas


def ascii_art(font, text):
    W, H, c = font.text_bitmap(text)
    return "\n".join("".join("#" if v else "." for v in row) for row in c)


if __name__ == "__main__":
    src = open(U8G2_FONTS_C, encoding="utf-8", errors="replace").read()
    for name, sample in [("u8g2_font_5x7_t_cyrillic", "Авт"),
                         ("u8g2_font_6x13_t_cyrillic", "Авт")]:
        f = Font(name, parse_c_array(src, name))
        print(f"=== {name}: glyph_cnt={f.glyph_cnt} max={f.max_w}x{f.max_h} "
              f"bits(0/1/w/h/x/y/dx)={f.bits_per_0}/{f.bits_per_1}/{f.bits_w}/{f.bits_h}/"
              f"{f.bits_x}/{f.bits_y}/{f.bits_dx} ====")
        print(ascii_art(f, sample))
        print()
