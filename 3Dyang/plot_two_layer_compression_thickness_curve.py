# -*- coding: utf-8 -*-
"""绘制简洁的两层极片压缩—卸载厚度曲线。"""

import csv
import os

from PIL import Image, ImageDraw, ImageFont


WORK_DIR = r"E:\abaqus\3Dyang"
CSV_PATH = os.path.join(
    WORK_DIR, "Yang_Macro_TwoLayerElectrode_Compression_thickness_curve.csv"
)
PNG_PATH = os.path.join(
    WORK_DIR, "Yang_Macro_TwoLayerElectrode_Compression_thickness_curve.png"
)
FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"

with open(CSV_PATH, "r", encoding="utf-8-sig", newline="") as source:
    rows = list(csv.DictReader(source))

compression = [row for row in rows if row["stage"] == "COMPRESSION"]
unload = [row for row in rows if row["stage"] == "UNLOAD"]


def numbers(items, key):
    return [float(item[key]) for item in items]


xc = numbers(compression, "process_coordinate")
yc = numbers(compression, "total_thickness_mm")
xu = numbers(unload, "process_coordinate")
yu = numbers(unload, "total_thickness_mm")

width, height = 1900, 1120
image = Image.new("RGB", (width, height), "white")
draw = ImageDraw.Draw(image)


def font(size):
    return ImageFont.truetype(FONT_PATH, size=size)


title_font = font(54)
axis_font = font(31)
tick_font = font(27)
legend_font = font(28)

left, top, right, bottom = 190, 145, 1770, 920
y_min, y_max = 0.133, 0.152


def px(x):
    return left + x / 2.0 * (right - left)


def py(y):
    return bottom - (y - y_min) / (y_max - y_min) * (bottom - top)


# 白色背景与网格
for value in (0.134, 0.138, 0.142, 0.146, 0.150):
    y = py(value)
    draw.line((left, y, right, y), fill="#C8D0D8", width=2)
    label = "{:.3f}".format(value)
    box = draw.textbbox((0, 0), label, font=tick_font)
    draw.text((left - 20 - (box[2] - box[0]), y - 17), label,
              fill="#444444", font=tick_font)

for value in (0.0, 0.5, 1.0, 1.5, 2.0):
    x = px(value)
    draw.line((x, bottom, x, bottom + 9), fill="#333333", width=3)
    label = "{:.1f}".format(value)
    box = draw.textbbox((0, 0), label, font=tick_font)
    draw.text((x - (box[2] - box[0]) / 2, bottom + 18), label,
              fill="#333333", font=tick_font)

draw.line((left, top, left, bottom), fill="#333333", width=3)
draw.line((left, bottom, right, bottom), fill="#333333", width=3)
draw.line((px(1.0), top, px(1.0), bottom), fill="#777777", width=3)


def draw_series(x_values, y_values, color):
    points = [(px(x), py(y)) for x, y in zip(x_values, y_values)]
    draw.line(points, fill=color, width=8, joint="curve")
    for x, y in points:
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)


blue = "#C00000"
orange = "#C00000"
draw_series(xc, yc, blue)
draw_series(xu, yu, orange)

title = "两层极片压缩—卸载厚度曲线"
title_box = draw.textbbox((0, 0), title, font=title_font)
draw.text(((width - (title_box[2] - title_box[0])) / 2, 42), title,
          fill="#17365D", font=title_font)
draw.text((32, 104), "极片平均总厚度（mm）", fill="#333333", font=axis_font)

x_label = "累计分析步伪时间"
x_box = draw.textbbox((0, 0), x_label, font=axis_font)
draw.text(((width - (x_box[2] - x_box[0])) / 2, 1020), x_label,
          fill="#333333", font=axis_font)

# 简洁图例
legend_y = 105
draw.line((1320, legend_y, 1385, legend_y), fill=blue, width=8)
draw.text((1400, legend_y - 20), "压缩", fill="#333333", font=legend_font)
draw.line((1515, legend_y, 1580, legend_y), fill=orange, width=8)
draw.text((1595, legend_y - 20), "卸载", fill="#333333", font=legend_font)

image.save(PNG_PATH, quality=95)
print(PNG_PATH)
