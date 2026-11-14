"""RoboMaster EP Phase 3: odometry prediction + AprilTag camera correction of the distance to a fixed tag."""
import json
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(ROOT, 'config')


def load_config(path=None):
    cfg = json.load(open(path or os.path.join(CONFIG_DIR, 'setup.json')))
    cfg['K'] = np.array(cfg['camera_matrix_720p'], dtype=float)
    return cfg
