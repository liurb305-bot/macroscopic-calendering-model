"""Create a PDF-style schematic preview for the visible roll/electrode model."""

from __future__ import print_function

from PIL import Image, ImageDraw, ImageFont


OUT = "visible_roll_electrode_schematic.png"


def font(size):
    for name in ("arial.ttf", "calibri.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()


def main():
    img = Image.new("RGB", (1200, 720), "white")
    draw = ImageDraw.Draw(img)
    black = (20, 20, 20)
    roll = (70, 115, 170)
    coating = (104, 184, 128)
    collector = (132, 160, 152)
    steel = (170, 170, 170)

    draw.text((55, 35), "Visible roll + electrode assembly", fill=black, font=font(28))
    draw.text((55, 72), "True Abaqus model uses a 900 mm analytical rigid roll and a 0.165 mm electrode half-model.", fill=(70, 70, 70), font=font(16))

    # Main 3D-style overview, not to exact visual scale.
    cx, cy, r = 260, 250, 130
    draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=roll, outline=(20, 45, 95), width=3)
    for x in range(cx - r + 18, cx + r, 20):
        draw.line((x, cy - r + 8, x, cy + r - 8), fill=(30, 70, 130), width=2)
    draw.rectangle((cx, cy - 28, 930, cy + 28), fill=steel, outline=(80, 80, 80))
    draw.rectangle((cx + 35, cy - 26, 885, cy + 26), fill=coating, outline=(60, 120, 80))
    draw.rectangle((cx + 35, cy + 9, 930, cy + 18), fill=collector, outline=(70, 90, 85))
    draw.line((70, cy, 1040, cy), fill=(230, 190, 20), width=2)
    for x in range(cx + 35, 885, 16):
        draw.line((x, cy - 26, x, cy + 26), fill=(50, 120, 80), width=1)

    draw.text((115, 410), "analytical rigid roll, RP at axis", fill=black, font=font(16))
    draw.text((610, 410), "coating / current collector / 80 mm local strip", fill=black, font=font(16))

    # Enlarged section inset.
    left, top, right, bottom = 120, 500, 1040, 640
    draw.rectangle((left, top, right, bottom), outline=(160, 40, 40), width=2)
    draw.text((left, top - 30), "enlarged electrode section near roll gap", fill=(120, 30, 30), font=font(18))
    y0 = bottom - 35
    draw.rectangle((left + 60, y0 - 8, right - 60, y0), fill=collector, outline=(70, 90, 85))
    draw.rectangle((left + 60, y0 - 68, right - 90, y0 - 8), fill=coating, outline=(60, 120, 80))
    draw.arc((left + 260, y0 - 910, left + 1160, y0 - 10), start=215, end=325, fill=roll, width=10)
    draw.text((left + 8, y0 - 56), "roll", fill=black, font=font(15))
    draw.text((left + 8, y0 - 35), "coating", fill=black, font=font(15))
    draw.text((left + 8, y0 - 12), "collector", fill=black, font=font(15))
    draw.line((left + 80, y0 - 56, left + 52, y0 - 56), fill=black, width=2)
    draw.line((left + 80, y0 - 35, left + 52, y0 - 35), fill=black, width=2)
    draw.line((left + 80, y0 - 8, left + 52, y0 - 8), fill=black, width=2)

    draw.text((55, 675), "Generated preview is schematic; open visible_roll_electrode.cae for the real Abaqus geometry.", fill=(80, 80, 80), font=font(14))
    img.save(OUT)
    print("Wrote %s" % OUT)


if __name__ == "__main__":
    main()
