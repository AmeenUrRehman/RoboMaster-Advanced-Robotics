"""Supervisory state machine: SEARCHING -> ALIGNING -> CHASING -> ARRIVED.

Pure logic (no ROS), driven by `update()` once per control tick, so it can be unit
tested and simulated.  Sign convention: bearing > 0 means the tag is to the LEFT of the
camera axis (ROS REP-103, +w = counter-clockwise), so aligning is w = +K * bearing.
"""
from dataclasses import dataclass
from typing import Optional

from .flatness import FlatTrajectory

SEARCHING, ALIGNING, CHASING, ARRIVED = 'SEARCHING', 'ALIGNING', 'CHASING', 'ARRIVED'


@dataclass
class Params:
    lost_timeout: float = 0.45       # s without a tag -> SEARCHING
    search_rate: float = 0.30        # rad/s, in-place sweep
    align_tol: float = 0.18          # rad (about 10 deg) to start chasing
    realign_tol: float = 0.30        # rad, hysteresis: leave CHASING only beyond this
    settle_time: float = 0.30        # s the bearing must stay inside align_tol
    align_gain: float = 0.8          # 1/s
    align_min_rate: float = 0.12     # rad/s, overcome wheel dead-band
    align_max_rate: float = 0.40     # rad/s
    chase_bearing_gain: float = 0.8  # 1/s, small correction added to the flat-output w
    chase_max_w: float = 0.40        # rad/s
    brake_distance: float = 0.20     # m, camera distance or ToF threshold
    v_max: float = 0.16              # m/s
    blind_chase: bool = True         # keep following the trajectory if the tag leaves the image
    blind_max_range: float = 1.8     # m, only if the tag was last seen at most this far away


@dataclass
class Observation:
    now: float
    visible: bool = False                 # tag detected in the latest frame
    bearing: float = 0.0                  # rad, valid when visible
    distance: float = float('inf')        # m, ground-plane camera distance, valid when visible
    tof: Optional[float] = None           # m, front ToF reading (None if unavailable)


def _clip(x, lo, hi):
    return max(lo, min(hi, x))


class NavStateMachine:
    def __init__(self, params: Params = None):
        self.p = params or Params()
        self.state = SEARCHING
        self.last_seen = None            # time of the last sighting
        self.last_dir = 1.0              # +1 CCW (tag was left), -1 CW
        self.centered_since = None
        self.traj = None
        self.traj_t0 = None
        self.arrival_reason = ''
        self.last_distance = float('inf')

    # -- helpers -------------------------------------------------------------------
    def _brake_hit(self, obs):
        if obs.visible and obs.distance <= self.p.brake_distance:
            return 'camera'
        if obs.tof is not None and obs.tof <= self.p.brake_distance:
            return 'tof'
        return ''

    def _enter_search(self):
        self.state = SEARCHING
        self.traj = None
        self.centered_since = None

    # -- main step -----------------------------------------------------------------
    def update(self, obs: Observation):
        """Returns (v, w) to command this tick; self.state holds the current state."""
        p = self.p
        if self.state == ARRIVED:
            return 0.0, 0.0

        if obs.visible:
            self.last_seen = obs.now
            self.last_distance = obs.distance
            if abs(obs.bearing) > 1e-3:
                self.last_dir = 1.0 if obs.bearing > 0 else -1.0

        # active brake: engages immediately from any state
        reason = self._brake_hit(obs)
        if reason:
            self.state, self.arrival_reason, self.traj = ARRIVED, reason, None
            return 0.0, 0.0

        lost = self.last_seen is None or (obs.now - self.last_seen) > p.lost_timeout
        # A floor tag leaves the image when the robot is close (the camera cannot look straight down
        # at its own feet).  While chasing, keep following the planned trajectory without the tag:
        # it already ends at the 20 cm standoff.
        if (lost and p.blind_chase and self.state == CHASING and self.traj is not None
                and self.last_distance <= p.blind_max_range):
            return self._chase(obs)
        if lost:
            if self.state != SEARCHING:
                self._enter_search()
            return 0.0, p.search_rate * self.last_dir

        # the tag is (recently) visible
        if self.state == SEARCHING:
            self.state = ALIGNING
            self.centered_since = None

        if self.state == ALIGNING:
            if not obs.visible:
                return 0.0, 0.0            # between frames: hold still, do not guess
            if abs(obs.bearing) <= p.align_tol:
                if self.centered_since is None:
                    self.centered_since = obs.now
                if obs.now - self.centered_since >= p.settle_time:
                    self.traj = FlatTrajectory(obs.distance, v_max=p.v_max)
                    self.traj_t0 = obs.now
                    self.state = CHASING
                    return self._chase(obs)
                return 0.0, 0.0            # settling, in place
            self.centered_since = None
            w = _clip(p.align_gain * obs.bearing, -p.align_max_rate, p.align_max_rate)
            if abs(w) < p.align_min_rate:
                w = p.align_min_rate * (1.0 if w >= 0 else -1.0)
            return 0.0, w

        # CHASING
        if obs.visible and abs(obs.bearing) > p.realign_tol:
            self.state, self.traj, self.centered_since = ALIGNING, None, None
            return 0.0, 0.0
        return self._chase(obs)

    def _chase(self, obs):
        t = obs.now - self.traj_t0
        if self.traj.finished(t):
            self.state, self.traj = ARRIVED, None
            self.arrival_reason = 'trajectory end' if obs.visible else 'trajectory end, tag out of view'
            return 0.0, 0.0
        v, w_ff, _, _ = self.traj.sample(t)
        w = w_ff
        if obs.visible:
            w += self.p.chase_bearing_gain * obs.bearing
        return min(v, self.p.v_max), _clip(w, -self.p.chase_max_w, self.p.chase_max_w)
