import io, sys, base64
from PIL import Image

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

SRC = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work"
OUT = SRC


def board_bbox(im):
    """bbox of dark-green board pixels (the PCB body), ignoring white bg and grey shadow."""
    px = im.convert("RGB").load()
    W, H = im.size
    x0, y0, x1, y1 = W, H, -1, -1
    for y in range(0, H, 2):
        for x in range(0, W, 2):
            r, g, b = px[x, y]
            if g > r + 8 and g > b + 8 and 30 < g < 160:      # green PCB
                if x < x0: x0 = x
                if x > x1: x1 = x
                if y < y0: y0 = y
                if y > y1: y1 = y
    return x0, y0, x1, y1


for name in ("render_top", "render_bot"):
    im = Image.open(f"{OUT}\\{name}.png")
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        bg = Image.new("RGB", im.size, (255, 255, 255))
        bg.paste(im, mask=im.split()[-1])
        im = bg
    else:
        im = im.convert("RGB")
    x0, y0, x1, y1 = board_bbox(im)
    m = int(0.045 * max(x1 - x0, y1 - y0))
    box = (max(0, x0 - m), max(0, y0 - m), min(im.width, x1 + m), min(im.height, y1 + m))
    cropped = im.crop(box)
    scale = min(1.0, 820 / cropped.width)
    cropped = cropped.resize((int(cropped.width * scale), int(cropped.height * scale)), Image.LANCZOS)
    path = f"{OUT}\\{name}_card.jpg"
    cropped.save(path, "JPEG", quality=80, optimize=True)
    size = len(open(path, "rb").read())
    print(f"{name}: crop={box} -> {cropped.size}, jpeg={size} B, base64={int(size*4/3)} B")
