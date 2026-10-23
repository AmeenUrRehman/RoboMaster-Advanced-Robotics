"""Predict how large a floor tag looks in the EP camera (no robot needed).

    ros2 run rm_flatness_nav camera_pose --ros-args -p tag_size:=0.30 -p camera_height:=0.25 -p camera_pitch_deg:=20.0

The camera is fixed to the chassis (checked with images: it did not move when the arm moved), so the
TF tree, which hangs the camera on the arm, is not used.  The height above the floor is an estimate: measure it.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node

from .tag_detector import camera_rotation

FX, FY, CX, CY = 317.03, 316.26, 317.26, 182.58     # EP 640x360 stream calibration


def main(args=None):
    rclpy.init(args=args)
    node = Node('camera_pose')
    size = node.declare_parameter('tag_size', 0.30).value
    h = node.declare_parameter('camera_height', 0.25).value
    pitch = math.radians(node.declare_parameter('camera_pitch_deg', 20.0).value)
    scale = node.declare_parameter('resolution_scale', 1.0).value     # 2.0 for the 720p stream
    R = camera_rotation(pitch)
    print(f'assumed camera: {h:.2f} m above the floor, {math.degrees(pitch):.0f} deg below horizontal; tag side {size:.2f} m')
    print('dist(m)  rows_visible_px  width_px  centre_row   (rows < ~15 px usually undetectable)')
    for d in (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 1.8):
        pts = np.array([[d + size / 2, 0, -h], [d - size / 2, 0, -h]])
        cam = (R.T @ pts.T).T
        v = FY * scale * cam[:, 1] / cam[:, 2] + CY * scale
        width = FX * scale * size / np.mean(cam[:, 2])
        print(f'{d:6.1f}   {abs(v[0] - v[1]):10.1f}   {width:8.1f}   {np.mean(v):9.0f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
