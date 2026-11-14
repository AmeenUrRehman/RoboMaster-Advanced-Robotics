"""AprilTag detection and the camera distance measurement used for the correction step.

The tag (tag36h11, ID 3, 15 cm black square) stands upright.  solvePnP gives the tag pose in the
camera optical frame (x right, y down, z forward): rotation R_ct and centre t_ct.  The distance used
is the perpendicular distance from the camera centre to the tag plane:

        z_plane = | n · t_ct |,     n = R_ct e_3  (tag normal)

It does not depend on the camera pitch (the EP camera looks about 20° down) or on where the tag
centre is in the image.  A linear calibration maps it to the robot reference point:

        z = a z_plane + b          (a: scale error of the intrinsics/tag size, b: camera-to-front offset)
        σ_z(z) = c0 + c1 z²        (camera noise grows with the square of the distance)
"""
import json
import math
import os
from typing import Optional

import cv2
import numpy as np

FAMILIES = {'tag16h5': 'DICT_APRILTAG_16h5', 'tag25h9': 'DICT_APRILTAG_25h9',
            'tag36h10': 'DICT_APRILTAG_36h10', 'tag36h11': 'DICT_APRILTAG_36h11'}


class TagDetection:
    def __init__(self, tag_id, corners, tvec, rvec):
        self.tag_id, self.corners, self.tvec, self.rvec = tag_id, corners, tvec, rvec
        R, _ = cv2.Rodrigues(rvec)
        self.normal = R[:, 2]

    @property
    def plane_distance(self):
        return abs(float(self.normal @ self.tvec))

    @property
    def range(self):
        return float(np.linalg.norm(self.tvec))

    @property
    def bearing_deg(self):
        """Horizontal angle of the tag centre, positive = tag to the RIGHT of the optical axis."""
        return math.degrees(math.atan2(self.tvec[0], self.tvec[2]))

    @property
    def side_px(self):
        c = self.corners
        return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


class TagDetector:
    def __init__(self, family='tag36h11', tag_id=3, tag_size=0.15):
        self.tag_id, self.size = tag_id, tag_size
        self._dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, FAMILIES[family]))
        # OpenCV 4.6 segfaults when a constructor-built DetectorParameters is modified; _create() is safe.
        self._params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
                        else cv2.aruco.DetectorParameters())
        # Fast path (about 6 ms per 720p frame): contour detection + sub-pixel corners.
        self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Fallback (about 80 ms): the AprilTag quad detector on a 2x decimated image, used only when the
        # fast path misses the tag (blur, glare).
        self._fallback = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
                          else cv2.aruco.DetectorParameters())
        self._fallback.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self._fallback.aprilTagQuadDecimate = 2.0
        self._new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self._new_api:
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)
            self._detector_fb = cv2.aruco.ArucoDetector(self._dict, self._fallback)
        self.n_fallback = 0
        s = tag_size / 2.0
        self._obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def _markers(self, gray, fallback):
        if self._new_api:
            return (self._detector_fb if fallback else self._detector).detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self._dict, parameters=self._fallback if fallback else self._params)

    def detect(self, gray, K, dist=None, allow_fallback=True) -> Optional[TagDetection]:
        corners, ids, _ = self._markers(gray, False)
        if (ids is None or self.tag_id not in ids.flatten()) and allow_fallback:
            corners, ids, _ = self._markers(gray, True)
            if ids is not None and self.tag_id in ids.flatten():
                self.n_fallback += 1
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if int(i) != self.tag_id:
                continue
            img = c.reshape(4, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(self._obj, img, K, np.zeros(5) if dist is None else dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                return TagDetection(int(i), img, tvec.reshape(3), rvec.reshape(3))
        return None


class CameraModel:
    """Calibrated camera distance z = a z_plane + b and its noise σ_z = c0 + c1 z²."""

    def __init__(self, a=1.0, b=0.0, c0=0.004, c1=0.006, latency_s=0.12, calibrated=False, latency_measured=False):
        self.a, self.b, self.c0, self.c1 = a, b, c0, c1
        self.latency_s, self.calibrated, self.latency_measured = latency_s, calibrated, latency_measured

    def distance(self, z_plane):
        return self.a * z_plane + self.b

    def sigma(self, z):
        return self.c0 + self.c1 * z * z

    @classmethod
    def load(cls, config_dir):
        m = cls()
        p = os.path.join(config_dir, 'calib.json')
        if os.path.exists(p):
            c = json.load(open(p))
            m.a, m.b, m.c0, m.c1, m.calibrated = c['a'], c['b'], c['c0'], c['c1'], True
        p = os.path.join(config_dir, 'latency.json')
        if os.path.exists(p):
            m.latency_s, m.latency_measured = json.load(open(p))['latency_s'], True
        return m


def make_tag_image(tag_id=3, family='tag36h11', px_per_cell=20, quiet_cells=3):
    """Printed tag: 8x8-cell black-bordered marker plus a white paper margin of `quiet_cells` cells.
    Returns the image; the black square spans 8 of its (8 + 2 quiet_cells) cells.  Simulator and tests only."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, FAMILIES[family]))
    side = 8 * px_per_cell
    m = (cv2.aruco.drawMarker(d, tag_id, side) if hasattr(cv2.aruco, 'drawMarker')
         else cv2.aruco.generateImageMarker(d, tag_id, side))
    b = quiet_cells * px_per_cell
    return cv2.copyMakeBorder(m, b, b, b, b, cv2.BORDER_CONSTANT, value=255)


def render_tag(K, size_wh, tag_img, tag_size, R_ct, t_ct, background=150, quiet_cells=3):
    """Project the printed tag (black square side tag_size) with pose (R_ct, t_ct) into a grey image."""
    w, h = size_wh
    px = tag_img.shape[0]
    s = tag_size * (8 + 2 * quiet_cells) / 8 / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
    cam = (R_ct @ obj.T).T + t_ct
    uv = (K @ cam.T).T
    uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float32)
    src = np.array([[0, 0], [px, 0], [px, px], [0, px]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, uv)
    out = np.full((h, w), background, np.uint8)
    warped = cv2.warpPerspective(tag_img, H, (w, h), flags=cv2.INTER_AREA, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(tag_img, 255), H, (w, h), flags=cv2.INTER_NEAREST, borderValue=0)
    out[mask > 0] = warped[mask > 0]
    return out
