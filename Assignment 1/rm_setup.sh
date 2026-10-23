#!/usr/bin/env bash
# Workspace environment loader for the RoboMaster EP assignment.
# Usage:  source ~/rm_setup.sh
#
# Order matters: ROS 2 and the workspace overlay first (they set PYTHONPATH),
# then the venv, then pin the interpreter's search path so ROS binary bindings
# (rclpy, cv_bridge) are found but user-site packages cannot shadow them.

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# venv was created with --system-site-packages, so it sees /opt/ros/jazzy/... via PYTHONPATH
source "$HOME/rm_venv/bin/activate"

# Make sure the venv's site-packages (numpy<2, robomaster SDK) win over ROS's
# dist-packages, and that ~/.local packages (possibly numpy 2.x) are ignored.
VENV_SITE="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="$VENV_SITE:/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH}"
export PYTHONNOUSERSITE=1

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
