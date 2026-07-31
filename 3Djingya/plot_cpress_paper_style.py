from pathlib import Path
import csv
import numpy as np
from PIL import Image, ImageDraw, ImageFont

WORK = Path(r'E:\abaqus\3Djingya')
CSV_PATH = WORK / 'electrode_cpress_lastframe.csv'
OUT_PATH = WORK / 'electrode_cpress_paper_style.png'


def jet01(t):
    t = max(0.0, min(1.0, float(t)))
    r = max(0.0, min(1.0, 1.5 - abs(4 * t - 3)))
    g = max(0.0, min(1.0, 1.5 - abs(4 * t - 2)))
    b = max(0.0, min(1.0, 1.5 - abs(4 * t - 1)))
    return int(r * 255), int(g * 255), int(b * 255)


def load_font(size, bold=False):
    paths = []
    if bold:
        paths.append(r'C:\Windows\Fonts\arialbd.ttf')
    paths.extend([
        r'C:\Windows\Fonts\arial.ttf',
        r'C:\Windows\Fonts\msyh.ttc',
        r'C:\Windows\Fonts\simhei.ttf',
    ])
    for path in paths:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


def center_text(draw, xy, text, font):
    box = draw.textbbox((0, 0), text, font=font)
    draw.text((xy[0] - (box[2] - box[0]) / 2, xy[1] - (box[3] - box[1]) / 2),
              text, fill='black', font=font)


def rotated_label(img, center, text, font):
    tmp = Image.new('RGBA', (560, 60), (255, 255, 255, 0))
    draw = ImageDraw.Draw(tmp)
    draw.text((0, 8), text, fill='black', font=font)
    rot = tmp.rotate(90, expand=True)
    img.paste(rot, (int(center[0] - rot.size[0] / 2), int(center[1] - rot.size[1] / 2)), rot)


rows = []
with CSV_PATH.open('r', newline='') as f:
    for row in csv.DictReader(f):
        rows.append({
            'x': float(row['x_mm']),
            'y': float(row['y_mm']),
            'z': float(row['z_mm']),
            'cpress': float(row['CPRESS_MPa']),
        })

# Combine top and bottom contact nodes by taking the maximum CPRESS at the same X-Z location.
columns = {}
for row in rows:
    key = (round(row['x'], 6), round(row['z'], 6))
    columns[key] = max(columns.get(key, 0.0), row['cpress'])

xs = np.array(sorted({key[0] for key in columns}))
zs = np.array(sorted({key[1] for key in columns}))
xi = {value: idx for idx, value in enumerate(xs)}
zi = {value: idx for idx, value in enumerate(zs)}

grid = np.zeros((len(zs), len(xs)), dtype=float)
for (x, z), value in columns.items():
    grid[zi[z], xi[x]] = value

vmin = 0.0
vmax = float(np.max(grid))
if vmax <= 0.0:
    vmax = 1.0

rgb = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
norm = np.clip((grid - vmin) / (vmax - vmin), 0, 1)
for iz in range(grid.shape[0]):
    for ix in range(grid.shape[1]):
        rgb[iz, ix, :] = jet01(norm[iz, ix])

heat = Image.fromarray(np.flipud(rgb), 'RGB')

W, H = 1500, 900
left, top, plot_w, plot_h = 195, 130, 975, 560
img = Image.new('RGB', (W, H), 'white')
draw = ImageDraw.Draw(img)

font_title = load_font(34, True)
font_label = load_font(24)
font_tick = load_font(20)
font_note = load_font(18)

img.paste(heat.resize((plot_w, plot_h), Image.Resampling.BICUBIC), (left, top))
draw.rectangle([left, top, left + plot_w, top + plot_h], outline='black', width=2)
draw.text((left, 35), 'Electrode Contact Pressure Contour - CPRESS', fill='black', font=font_title)

for value in np.linspace(float(xs.min()), float(xs.max()), 6):
    px = left + (value - xs.min()) / (xs.max() - xs.min()) * plot_w
    draw.line([(px, top + plot_h), (px, top + plot_h + 8)], fill='black', width=2)
    center_text(draw, (px, top + plot_h + 28), f'{value:.1f}', font_tick)

for value in np.linspace(float(zs.min()), float(zs.max()), 6):
    py = top + plot_h - (value - zs.min()) / (zs.max() - zs.min()) * plot_h
    draw.line([(left - 8, py), (left, py)], fill='black', width=2)
    text = f'{value:.1f}'
    box = draw.textbbox((0, 0), text, font=font_tick)
    draw.text((left - 18 - (box[2] - box[0]), py - (box[3] - box[1]) / 2),
              text, fill='black', font=font_tick)

center_text(draw, (left + plot_w / 2, top + plot_h + 76),
            'X / mm  (roller axial direction)', font_label)
rotated_label(img, (60, top + plot_h / 2),
              'Z / mm  (rolling/local length direction)', font_label)

cb_left, cb_top, cb_w, cb_h = left + plot_w + 85, top, 55, plot_h
for j in range(cb_h):
    draw.line([(cb_left, cb_top + j), (cb_left + cb_w, cb_top + j)],
              fill=jet01(1 - j / (cb_h - 1)), width=1)
draw.rectangle([cb_left, cb_top, cb_left + cb_w, cb_top + cb_h], outline='black', width=2)
for value in np.linspace(vmin, vmax, 7):
    py = cb_top + cb_h - (value - vmin) / (vmax - vmin) * cb_h
    draw.line([(cb_left + cb_w, py), (cb_left + cb_w + 8, py)], fill='black', width=2)
    draw.text((cb_left + cb_w + 16, py - 11), f'{value:.3e}', fill='black', font=font_tick)
draw.text((cb_left - 5, cb_top - 42), 'CPRESS / MPa', fill='black', font=font_label)

max_idx = np.unravel_index(np.argmax(grid), grid.shape)
note = ('Final frame: Press_Down, t=1.0e-3 s; electrode-side contact pressure. '
        'Max=%0.3e MPa at X=%0.2f, Z=%0.2f.' % (vmax, xs[max_idx[1]], zs[max_idx[0]]))
draw.text((left, H - 68), note, fill='black', font=font_note)

img.save(OUT_PATH, quality=95)
print('%s max=%g rows=%d grid=%s' % (OUT_PATH, vmax, len(rows), grid.shape))
