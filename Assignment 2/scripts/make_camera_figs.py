#!/usr/bin/env python3
"""Figures of the real camera view of Tag 3 and the phone photos of the run.
Reads figures/camera/flat_tags_0p6m.png (robot camera, 640x360) and figures/photos/*.jpg (phone photos).
Writes figures/fig_tag3_camera.png and figures/fig_run_photos.png."""
import sys
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 9})
sys.path.insert(0, 'ws/src/rm_flatness_nav')
from rm_flatness_nav.tag_detector import camera_rotation, ground_range_bearing

img = cv2.imread('figures/camera/flat_tags_0p6m.png')
rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
K = np.array([[317.0, 0, 320.0], [0, 317.0, 180.0], [0, 0, 1.0]])
R = camera_rotation(np.radians(20.0))
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
p = cv2.aruco.DetectorParameters_create()
corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
found = {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.flatten())}
h = 0.15 / 2
obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
ok, rvec, tvec = cv2.solvePnP(obj, found[3].astype(np.float32), K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
rng, brg = ground_range_bearing(tvec.reshape(3), R)

fig = plt.figure(figsize=(7.4, 5.6), constrained_layout=True)
gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.15])
a = fig.add_subplot(gs[0, 0]); a.imshow(rgb); a.set_title('(a) raw frame, robot camera', fontsize=9); a.axis('off')
b = fig.add_subplot(gs[0, 1]); b.imshow(rgb); b.axis('off'); b.set_title('(b) detections of the project detector', fontsize=9)
for i, c in found.items():
    tgt = (i == 3)
    b.add_patch(plt.Polygon(c, closed=True, fill=False, ec='#16a34a' if tgt else '#f59e0b', lw=2.2 if tgt else 1.4))
    b.text(c[:, 0].mean(), c[:, 1].min() - 6, f'Tag {i}' + (' (target)' if tgt else ' (ignored)'), color='#16a34a' if tgt else '#b45309',
           ha='center', va='bottom', fontsize=7.5, fontweight='bold', bbox=dict(fc='white', ec='none', alpha=0.8, pad=1))
c3 = found[3]; x0, x1 = int(c3[:, 0].min()) - 18, int(c3[:, 0].max()) + 18; y0, y1 = int(c3[:, 1].min()) - 18, int(c3[:, 1].max()) + 18
crop = cv2.resize(rgb[y0:y1, x0:x1], None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)
cc = fig.add_subplot(gs[1, :]); cc.imshow(crop); cc.axis('off')
cc.set_title(f'(c) Tag 3 enlarged 6x: {np.ptp(c3[:, 0]):.0f} x {np.ptp(c3[:, 1]):.0f} px in the 640x360 frame; '
             f'estimated ground range {rng:.2f} m, bearing {np.degrees(brg):+.1f} deg (camera tilt 20 deg, tag 0.15 m)', fontsize=8.5)
fig.savefig('figures/fig_tag3_camera.png', dpi=200); plt.close(fig)
print('tag3 px', np.ptp(c3[:, 0]), np.ptp(c3[:, 1]), 'range %.2f bearing %.1f' % (rng, np.degrees(brg)))

ph = [('figures/photos/run_start_1p5m.jpg', '(a) start layout,\nphone measure: 1.5 m'),
      ('figures/photos/run_69cm.jpg', '(b) phone measure:\n69 cm to the tags'),
      ('figures/photos/run_end_b.jpg', '(c) after the stop\n(top view)'),
      ('figures/photos/run_end_a.jpg', '(d) after the stop,\ncloser view')]
imgs = [plt.imread(f) for f, _ in ph]
ratios = [i.shape[1] / i.shape[0] for i in imgs]
fig, ax = plt.subplots(1, 4, figsize=(7.4, 4.3), gridspec_kw={'width_ratios': ratios}, constrained_layout=True)
for a, im, (f, t) in zip(ax, imgs, ph):
    a.imshow(im); a.axis('off'); a.set_title(t, fontsize=7.5)
fig.savefig('figures/fig_run_photos.png', dpi=200); plt.close(fig); print('ok')
