#!/usr/bin/env python3
"""Generate (1) printable AprilTags T1, T2, T3 (family tag36h11, IDs 1, 2, 3, black square 150 mm, A4 at 1:1),
(2) a preview sheet, (3) the white-background mission figure used in the reports and READMEs.
Tags are drawn as VECTOR squares from the 8x8 module grid (so the PDF prints crisp and exactly to size)."""
import cv2, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Polygon, Arc

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False})
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
SIDE_MM = 150.0

def grid(i):
    """8x8 boolean array, True = black module (the outer ring is the black border)."""
    m = cv2.aruco.drawMarker(D, i, 800)
    return m[50::100, 50::100] < 128

def draw_tag(ax, cx, cy, side, i, quiet=0.25, z=3):
    """Vector AprilTag centred at (cx, cy); `side` = black square side; white quiet zone = quiet*side each side."""
    q = side * (1 + 2 * quiet)
    ax.add_patch(Rectangle((cx - q / 2, cy - q / 2), q, q, fc='white', ec='none', zorder=z))
    c = side / 8.0; g = grid(i)
    for r in range(8):
        for k in range(8):
            if g[r, k]:
                ax.add_patch(Rectangle((cx - side / 2 + k * c, cy + side / 2 - (r + 1) * c), c, c, fc='black', ec='black', lw=0.2, zorder=z + 1))

# ---------------- printable A4 sheets, 1:1 scale ----------------
for i in (1, 2, 3):
    fig = plt.figure(figsize=(210 / 25.4, 297 / 25.4)); ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 210); ax.set_ylim(0, 297); ax.set_aspect('equal'); ax.axis('off')
    cx, cy = 105, 180
    draw_tag(ax, cx, cy, SIDE_MM, i, quiet=0.13)
    ax.add_patch(Rectangle((cx - SIDE_MM * 1.13 / 2 - 6, cy - SIDE_MM * 1.13 / 2 - 6), SIDE_MM * 1.13 + 12, SIDE_MM * 1.13 + 12, fill=False, ec='0.6', lw=0.6, ls=(0, (4, 3))))
    ax.text(105, 276, f'TAG {i}', ha='center', fontsize=40, fontweight='bold')
    ax.text(105, 262, f'AprilTag family tag36h11, ID {i}', ha='center', fontsize=15)
    ax.text(105, 78, 'Print at 100 % (no "fit to page"). The black square must measure 150 mm.', ha='center', fontsize=11)
    x0 = 105 - 50
    ax.plot([x0, x0 + 100], [66, 66], 'k-', lw=2); ax.plot([x0, x0], [64, 68], 'k-', lw=2); ax.plot([x0 + 100, x0 + 100], [64, 68], 'k-', lw=2)
    ax.text(105, 58, '100 mm check', ha='center', fontsize=10)
    ax.text(105, 20, 'SCE 672 Advanced Robotics, KFUPM. Tag 3 is the target of the flatness pursuit; Tags 1 and 2 are decoys.', ha='center', fontsize=8, color='0.35')
    fig.savefig(f'Tag{i}_A4_print.pdf'); fig.savefig(f'Tag{i}.png', dpi=200); plt.close(fig)

# ---------------- preview sheet (white background) ----------------
fig, axs = plt.subplots(1, 3, figsize=(8.6, 3.3))
for a, i in zip(axs, (1, 2, 3)):
    a.set_xlim(-1, 1); a.set_ylim(-1, 1); a.set_aspect('equal'); a.axis('off')
    draw_tag(a, 0, 0, 1.55, i, quiet=0.13); a.set_title(f'Tag {i}  (tag36h11, ID {i})', fontsize=11)
fig.suptitle('Printable AprilTags used as targets (black square 150 mm)', fontsize=11)
fig.savefig('tags_preview.png', dpi=200, bbox_inches='tight', facecolor='white'); plt.close(fig)

# ---------------- mission figure: white background ----------------
fig, ax = plt.subplots(figsize=(9.4, 4.4)); ax.set_aspect('equal'); ax.axis('off')
ax.set_xlim(-0.5, 2.15); ax.set_ylim(-0.85, 0.82)
TX = 1.85                                         # tag line x
ax.plot([TX, TX], [-0.72, 0.74], color='0.78', lw=1.0, ls=(0, (6, 4)))
ax.add_patch(Polygon([[TX, 0.70], [TX + 0.11, 0.49], [TX - 0.11, 0.49]], closed=True, fc='#ffd9a8', ec='#c76a00', lw=1.5)); ax.text(TX, 0.40, 'T1', ha='center', fontsize=11, fontweight='bold')
ax.add_patch(Circle((TX, -0.52), 0.105, fc='#cfe6ff', ec='#1c6fd1', lw=1.5)); ax.text(TX, -0.69, 'T2', ha='center', fontsize=11, fontweight='bold')
draw_tag(ax, TX, 0.0, 0.26, 3, quiet=0.15)
ax.text(TX, 0.27, 'T3 (target)', ha='center', fontsize=11, fontweight='bold', color='#1a7f37')
def robot(cx, cy, alpha=1.0, ls='-', z=4):
    ax.add_patch(FancyBboxPatch((cx - 0.13, cy - 0.10), 0.26, 0.20, boxstyle='round,pad=0,rounding_size=0.03', fc='#e8eefc', ec='#1f3b82', lw=1.6, alpha=alpha, ls=ls, zorder=z))
    for sx in (-1, 1):
        for sy in (-1, 1):
            ax.add_patch(Rectangle((cx + sx * 0.085 - 0.035, cy + sy * 0.115 - 0.018), 0.07, 0.036, fc='#333', ec='k', alpha=alpha, zorder=z))
    ax.add_patch(Rectangle((cx + 0.13, cy - 0.035), 0.06, 0.07, fc='#8aa0d6', ec='#1f3b82', alpha=alpha, zorder=z))
robot(0.0, 0.0)
ax.text(0.0, -0.40, 'RoboMaster EP\nstart $(0,0,0)$', ha='center', fontsize=8.5, color='0.3')
ax.add_patch(Arc((0, 0), 0.62, 0.62, theta1=-60, theta2=235, color='#7048e8', lw=1.6, ls=(0, (3, 2))))
ax.annotate('', xy=(-0.06, 0.312), xytext=(0.04, 0.311), arrowprops=dict(arrowstyle='-|>', color='#7048e8', lw=1.6))
ax.text(0.0, 0.43, '1  SEARCH: spin in place', fontsize=9, color='#7048e8', ha='center')
xs = TX - 0.13 - 0.20 - 0.19                      # robot centre when the gripper front is 20 cm from the tag face
robot(xs, 0.0, alpha=0.4, ls='--')
t = np.linspace(0, 1, 80); xx = 0.35 + (xs - 0.13 - 0.35) * (3 * t ** 2 - 2 * t ** 3)
ax.plot(xx, 0 * xx, color='#d6336c', lw=2.2, ls=(0, (5, 3)), zorder=3)
ax.annotate('', xy=(xs - 0.14, 0.0), xytext=(xs - 0.25, 0.0), arrowprops=dict(arrowstyle='-|>', color='#d6336c', lw=2.0), zorder=5)
ax.text(0.98, -0.28, '3  CHASE: cubic flat-output path $z(t)=[x(t),\\,y(t)]^T$', fontsize=9, color='#d6336c', ha='center')
ax.text(0.98, -0.38, 'speed rises, peaks at 0.137 m/s, and falls to zero', fontsize=8.2, color='#d6336c', ha='center')
ax.text(0.80, 0.18, '2  ALIGN: turn in place until the tag is\ncentred ($|\\theta|\\leq 10^\\circ$), then drive', fontsize=9, color='#e8590c', ha='center')
ax.annotate('', xy=(TX - 0.13, -0.17), xytext=(xs + 0.19, -0.17), arrowprops=dict(arrowstyle='<->', color='#1a7f37', lw=1.5))
ax.text(1.46, -0.66, '4  STOP: 20 cm standoff\nfrom the tag face', fontsize=9, color='#1a7f37', ha='center')
ax.plot([1.62, 1.62], [-0.56, -0.20], color='#1a7f37', lw=0.8)
ax.annotate('', xy=(-0.14, -0.74), xytext=(-0.42, -0.74), arrowprops=dict(arrowstyle='-|>', lw=1.5)); ax.text(-0.28, -0.84, 'body $+X$ (forward)', fontsize=8.5, ha='center')
ax.set_title('Mission: search, align, follow a flat-output trajectory, stop 20 cm from Tag 3', fontsize=10.5)
fig.savefig('fig_mission_chase.png', dpi=220, bbox_inches='tight', facecolor='white'); fig.savefig('fig_mission_chase.pdf', bbox_inches='tight', facecolor='white'); plt.close(fig)
print('ok')
