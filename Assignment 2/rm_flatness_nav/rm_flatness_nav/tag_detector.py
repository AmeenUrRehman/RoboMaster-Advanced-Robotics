"""AprilTag detection (OpenCV aruco AprilTag dictionaries) and floor-plane geometry."""
import math
from typing import Optional

import cv2
import numpy as np

FAMILIES = {
    'tag16h5': 'DICT_APRILTAG_16h5', 'tag25h9': 'DICT_APRILTAG_25h9',
    'tag36h10': 'DICT_APRILTAG_36h10', 'tag36h11': 'DICT_APRILTAG_36h11',
}

# optical frame (x right, y down, z forward) -> base frame (x forward, y left, z up)
R_OPTICAL_TO_BASE = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])


def camera_rotation(pitch_down):
    """Rotation optical -> base for a camera tilted down by `pitch_down` rad."""
    c, s = math.cos(pitch_down), math.sin(pitch_down)
    ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])  # +y rotation tilts +x down
    return ry @ R_OPTICAL_TO_BASE


def ground_range_bearing(p_cam, r_base_cam):
    """Horizontal distance and bearing of a point given in the camera optical frame.

    r_base_cam rotates optical vectors into the base frame.  Bearing > 0 = target on the LEFT.
    Range is measured from the camera, projected onto the floor plane.
    """
    p = r_base_cam @ np.asarray(p_cam, dtype=float)
    return math.hypot(p[0], p[1]), math.atan2(p[1], p[0])


class TagDetection:
    def __init__(self, tag_id, corners, tvec, rvec):
        self.tag_id, self.corners, self.tvec, self.rvec = tag_id, corners, tvec, rvec


class TagDetector:
    def __init__(self, family='tag36h11', tag_id=3, tag_size=0.10):
        name = FAMILIES[family]
        self.tag_id, self.size = tag_id, tag_size
        self._dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        # OpenCV 4.6 (Ubuntu 24.04 / ROS Jazzy) segfaults when a DetectorParameters() built with
        # the constructor is modified; its _create() factory is safe.  4.7+ only has the constructor.
        self._params = (cv2.aruco.DetectorParameters_create()
                        if hasattr(cv2.aruco, 'DetectorParameters_create')
                        else cv2.aruco.DetectorParameters())
        self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self._new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self._new_api:
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)
        s = tag_size / 2.0
        self._obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def detect(self, gray, camera_matrix, dist_coeffs=None, scales=(1.0, 2.0)) -> Optional[TagDetection]:
        """Pose of the configured tag id, or None if it is not in the image.

        A small tag (about 30 px per side or less, i.e. a 15 cm tag beyond ~1.3 m at 360p) is often missed
        at native size but found after enlarging the image, so each scale is tried in order and only the
        first success is used.  Corners are mapped back to native pixels before the pose is solved."""
        for scale in scales:
            img = gray if scale == 1.0 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            det = self._detect_once(img, camera_matrix, dist_coeffs, scale)
            if det is not None:
                return det
        return None

    def _detect_once(self, gray, camera_matrix, dist_coeffs, scale):
        if self._new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if self.tag_id is not None and int(i) != self.tag_id:
                continue
            img = (c.reshape(4, 2) / scale).astype(np.float32)
            dist = np.zeros(5) if dist_coeffs is None else dist_coeffs
            ok, rvec, tvec = cv2.solvePnP(self._obj, img, camera_matrix, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                return TagDetection(int(i), img, tvec.reshape(3), rvec.reshape(3))
        return None
