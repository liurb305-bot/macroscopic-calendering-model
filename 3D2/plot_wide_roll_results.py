"""Create lightweight PNG plots from the extracted CSV files.

This uses Pillow rather than matplotlib so it works with the bundled runtime.
"""

from __future__ import print_function

import csv
import math
import os

from PIL import Image, ImageDraw, ImageFont


JOB_NAME = "wide_roll_1_8"
THICKNESS_CSV = JOB_NAME + "_thickness.csv"
CONTACT_CSV = JOB_NAME + "_contact_pressure.csv"


def font(size=14):
    for name in ("arial.ttf", "calibri.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()


def color_ramp(t):
    t = max(0.0, min(1.0, t))
    stops = [
        (0.00, (35, 76, 180)),
        (0.25, (70, 170, 220)),
        (0.50, (100, 200, 95)),
        (0.75, (245, 210, 70)),
        (1.00, (200, 45, 45)),
    ]
    for idx in range(len(stops) - 1):
        p0, c0 = stops[idx]
        p1, c1 = stops[idx + 1]
        if t <= p1:
            local = (t - p0) / (p1 - p0)
            return tuple(int(c0[i] + local * (c1[i] - c0[i])) for i in range(3))
    return stops[-1][1]


def read_thickness():
    rows = []
    with open(THICKNESS_CSV, "r") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append((float(row["x_mm"]), float(row["full_thickness_um"])))
    rows.sort()
    return rows


def read_contact():
    grid = {}
    with open(CONTACT_CSV, "r") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            x = round(float(row["x_mm"]), 6)
            z = round(float(row["z_mm"]), 6)
            p = float(row["cpress_mpa"])
            key = (x, z)
            if p > grid.get(key, -1.0e30):
                grid[key] = p
    return grid


def draw_axes(draw, left, top, right, bottom, title, xlabel, ylabel):
    black = (20, 20, 20)
    draw.rectangle((left, top, right, bottom), outline=black, width=1)
    draw.text((left, 12), title, fill=black, font=font(20))
    draw.text(((left + right) // 2 - 40, bottom + 38), xlabel, fill=black, font=font(14))
    draw.text((left - 78, top - 26), ylabel, fill=black, font=font(14))


def plot_thickness(rows):
    width, height = 1400, 520
    left, top, right, bottom = 85, 55, 1320, 430
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    draw_axes(draw, left, top, right, bottom, "Wide roll 1/8 model - transverse thickness profile", "x from centerline (mm)", "thickness (um)")

    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    pad = max(1.0, (ymax - ymin) * 0.15)
    ymin -= pad
    ymax += pad

    for i in range(6):
        x = left + (right - left) * i / 5.0
        val = xmin + (xmax - xmin) * i / 5.0
        draw.line((x, bottom, x, bottom + 5), fill=(0, 0, 0))
        draw.text((x - 22, bottom + 10), "%.0f" % val, fill=(0, 0, 0), font=font(12))
    for i in range(6):
        y = bottom - (bottom - top) * i / 5.0
        val = ymin + (ymax - ymin) * i / 5.0
        draw.line((left - 5, y, left, y), fill=(0, 0, 0))
        draw.text((18, y - 7), "%.1f" % val, fill=(0, 0, 0), font=font(12))

    points = []
    for x_val, y_val in rows:
        px = left + (x_val - xmin) / (xmax - xmin) * (right - left)
        py = bottom - (y_val - ymin) / (ymax - ymin) * (bottom - top)
        points.append((px, py))
    draw.line(points, fill=(210, 45, 45), width=3)
    draw.text((980, 70), "min %.2f um   max %.2f um" % (min(ys), max(ys)), fill=(20, 20, 20), font=font(14))
    img.save(JOB_NAME + "_thickness_profile.png")
    print("Wrote %s_thickness_profile.png" % JOB_NAME)


def plot_contact(grid):
    width, height = 1400, 560
    left, top, right, bottom = 85, 55, 1320, 410
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    draw_axes(draw, left, top, right, bottom, "Wide roll 1/8 model - contact pressure map", "x from centerline (mm)", "z (mm)")

    xs = sorted(set(k[0] for k in grid))
    zs = sorted(set(k[1] for k in grid))
    xmin, xmax = min(xs), max(xs)
    zmin, zmax = min(zs), max(zs)
    pmax = max(grid.values())
    pmin = min(grid.values())
    dx = (right - left) / float(len(xs))
    dz = (bottom - top) / float(len(zs))

    for ix, x in enumerate(xs):
        for iz, z in enumerate(zs):
            p = grid.get((x, z), 0.0)
            color = color_ramp((p - pmin) / (pmax - pmin if pmax > pmin else 1.0))
            x0 = left + ix * dx
            x1 = left + (ix + 1) * dx + 1
            y0 = bottom - (iz + 1) * dz
            y1 = bottom - iz * dz + 1
            draw.rectangle((x0, y0, x1, y1), fill=color)

    for i in range(6):
        x = left + (right - left) * i / 5.0
        val = xmin + (xmax - xmin) * i / 5.0
        draw.line((x, bottom, x, bottom + 5), fill=(0, 0, 0))
        draw.text((x - 22, bottom + 10), "%.0f" % val, fill=(0, 0, 0), font=font(12))
    for i in range(5):
        y = bottom - (bottom - top) * i / 4.0
        val = zmin + (zmax - zmin) * i / 4.0
        draw.line((left - 5, y, left, y), fill=(0, 0, 0))
        draw.text((25, y - 7), "%.1f" % val, fill=(0, 0, 0), font=font(12))

    bar_left, bar_top, bar_right, bar_bottom = 1180, 445, 1320, 465
    for i in range(bar_right - bar_left):
        t = i / float(bar_right - bar_left - 1)
        draw.line((bar_left + i, bar_top, bar_left + i, bar_bottom), fill=color_ramp(t))
    draw.rectangle((bar_left, bar_top, bar_right, bar_bottom), outline=(0, 0, 0))
    draw.text((bar_left, bar_bottom + 8), "%.1f MPa" % pmin, fill=(0, 0, 0), font=font(12))
    draw.text((bar_right - 65, bar_bottom + 8), "%.1f MPa" % pmax, fill=(0, 0, 0), font=font(12))
    draw.text((985, 445), "CPRESS", fill=(0, 0, 0), font=font(14))
    img.save(JOB_NAME + "_contact_pressure.png")
    print("Wrote %s_contact_pressure.png" % JOB_NAME)


def main():
    if not os.path.exists(THICKNESS_CSV):
        raise RuntimeError("Missing %s" % THICKNESS_CSV)
    if not os.path.exists(CONTACT_CSV):
        raise RuntimeError("Missing %s" % CONTACT_CSV)
    plot_thickness(read_thickness())
    plot_contact(read_contact())


if __name__ == "__main__":
    main()
