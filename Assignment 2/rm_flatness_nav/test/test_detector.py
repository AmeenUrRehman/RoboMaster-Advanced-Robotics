import math

import cv2
import numpy as np
import pytest

from rm_flatness_nav.tag_detector import TagDetector, camera_rotation, ground_range_bearing

# RoboMaster EP 640x360 stream (robomaster_ros rm_camera_calibration_360.yaml)
K = np.array([[317.03, 0.0, 317.26], [0.0, 316.26, 182.58], [0.0, 0.0, 1.0]])
SIZE, TAG_ID, PITCH = 0.30, 3, math.atan2(0.20, 0.70)   # large tag, centred at 0.7 m


def render(forward, left, height=0.20, tag_id=TAG_ID, family=cv2.aruco.DICT_APRILTAG_36h11):
    """Image of a flat floor tag at (forward, left) metres, seen by a camera `height` m
    above the floor and tilted down by PITCH (base frame: x forward, y left, z up)."""
    r_bc = camera_rotation(PITCH)                       # optical -> base
    s = SIZE / 2.0
    # tag corners on the floor (z = -height relative to the camera), TL, TR, BR, BL as seen from above
    corners_base = np.array([[forward + s, left + s, -height], [forward + s, left - s, -height],
                             [forward - s, left - s, -height], [forward - s, left + s, -height]])
    corners_cam = (r_bc.T @ corners_base.T).T
    px = (K @ corners_cam.T).T
    px = (px[:, :2] / px[:, 2:3]).astype(np.float32)
    d = cv2.aruco.getPredefinedDictionary(family)
    marker = cv2.aruco.generateImageMarker(d, tag_id, 200) if hasattr(cv2.aruco, 'generateImageMarker') \
        else cv2.aruco.drawMarker(d, tag_id, 200)
    marker = cv2.copyMakeBorder(marker, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    src = np.array([[20, 20], [219, 20], [219, 219], [20, 219]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, px)
    img = cv2.warpPerspective(marker, H, (640, 360), flags=cv2.INTER_LINEAR, borderValue=128)
    return img


@pytest.mark.parametrize('forward,left', [(0.7, 0.0), (0.7, 0.10), (0.7, -0.10)])
def test_detects_and_recovers_ground_range_and_bearing(forward, left):
    img = render(forward, left)
    det = TagDetector('tag36h11', TAG_ID, SIZE).detect(img, K)
    assert det is not None, 'tag not found'
    d, b = ground_range_bearing(det.tvec, camera_rotation(PITCH))
    assert d == pytest.approx(math.hypot(forward, left), abs=0.05)
    assert b == pytest.approx(math.atan2(left, forward), abs=0.05)


def test_ignores_other_tag_ids():
    img = render(0.7, 0.0, tag_id=7)
    assert TagDetector('tag36h11', TAG_ID, SIZE).detect(img, K) is None
