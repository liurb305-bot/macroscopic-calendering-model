"""Draw a readable geometry section showing where the roll is in the model."""

from __future__ import print_function

import math

from PIL import Image, ImageDraw, ImageFont


ROLL_RADIUS = 450.0
HALF_STACK_THICKNESS = 0.0825
MODEL_Z_LENGTH = 8.0
ROLL_LAYER_THICKNESS = 2.0
ROLL_CENTER_FLEXURE_GAP = 0.004


def font(size=14):
    for name in ("arial.ttf", "calibri.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()


def roll_arc(z):
    return ROLL_RADIUS - math.sqrt(max(0.0, ROLL_RADIUS * ROLL_RADIUS - z * z))


def y_bottom(z):
    return HALF_STACK_THICKNESS + ROLL_CENTER_FLEXURE_GAP + roll_arc(z)


def main():
    width, height = 1200, 650
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)

    draw.text((40, 25), "Where the roll is in the 1/8 equivalent model", fill=(20, 20, 20), font=font(24))
    draw.text((40, 58), "Section at x = centerline; vertical scale is enlarged so the 165 um electrode is visible.", fill=(60, 60, 60), font=font(15))

    left, top, right, bottom = 80, 105, 1120, 555
    zmin, zmax = 0.0, MODEL_Z_LENGTH
    ymin, ymax = 0.0, 0.18

    def px(z):
        return left + (z - zmin) / (zmax - zmin) * (right - left)

    def py(y):
        return bottom - (y - ymin) / (ymax - ymin) * (bottom - top)

    draw.rectangle((left, top, right, bottom), outline=(20, 20, 20), width=1)

    # Electrode layers.
    y_collector = 0.0075
    y_electrode_top = HALF_STACK_THICKNESS
    draw.rectangle((px(0), py(y_collector), px(MODEL_Z_LENGTH), py(0)), fill=(120, 160, 150), outline=(80, 110, 105))
    draw.rectangle((px(0), py(y_electrode_top), px(MODEL_Z_LENGTH), py(y_collector)), fill=(110, 185, 135), outline=(70, 130, 90))

    # Roll local contact layer.  The real layer is 2 mm thick; only a cropped
    # contact-side band is shown in this zoom.
    samples = 160
    bottom_curve = []
    top_curve = []
    shown_roll_band = 0.055
    for i in range(samples + 1):
        z = MODEL_Z_LENGTH * i / float(samples)
        yb = y_bottom(z)
        yt = min(ymax, yb + shown_roll_band)
        bottom_curve.append((px(z), py(yb)))
        top_curve.append((px(z), py(yt)))
    poly = bottom_curve + list(reversed(top_curve))
    draw.polygon(poly, fill=(210, 165, 95), outline=(135, 95, 40))
    draw.line(bottom_curve, fill=(120, 70, 20), width=3)

    # Nominal current top after imposed displacement at final step.
    draw.line((px(0), py(y_electrode_top - 0.0165), px(MODEL_Z_LENGTH), py(y_electrode_top - 0.0165)), fill=(180, 45, 45), width=2)

    for i in range(5):
        z = MODEL_Z_LENGTH * i / 4.0
        draw.line((px(z), bottom, px(z), bottom + 5), fill=(0, 0, 0))
        draw.text((px(z) - 12, bottom + 10), "%.1f" % z, fill=(0, 0, 0), font=font(12))
    for i in range(7):
        y = ymax * i / 6.0
        draw.line((left - 5, py(y), left, py(y)), fill=(0, 0, 0))
        draw.text((25, py(y) - 7), "%.3f" % y, fill=(0, 0, 0), font=font(12))

    draw.text(((left + right) // 2 - 40, bottom + 38), "z from roll-gap symmetry plane (mm)", fill=(0, 0, 0), font=font(14))
    draw.text((20, top - 25), "y (mm)", fill=(0, 0, 0), font=font(14))

    draw.rectangle((830, 118, 850, 138), fill=(210, 165, 95), outline=(135, 95, 40))
    draw.text((858, 116), "local deformable roll contact layer (E_ROLL)", fill=(30, 30, 30), font=font(14))
    draw.rectangle((830, 148, 850, 168), fill=(110, 185, 135), outline=(70, 130, 90))
    draw.text((858, 146), "coating", fill=(30, 30, 30), font=font(14))
    draw.rectangle((830, 178, 850, 198), fill=(120, 160, 150), outline=(80, 110, 105))
    draw.text((858, 176), "half current collector", fill=(30, 30, 30), font=font(14))
    draw.line((830, 216, 850, 216), fill=(180, 45, 45), width=2)
    draw.text((858, 207), "20% reduction displacement reference", fill=(30, 30, 30), font=font(14))

    img.save("wide_roll_1_8_roll_geometry_section.png")
    print("Wrote wide_roll_1_8_roll_geometry_section.png")


if __name__ == "__main__":
    main()
