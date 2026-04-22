#!/bin/bash
set -e

# Add ros2 source, ws source and cd to ws to bashrc so the docker starts there
if ! grep -q "smip_uav_ws" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "[ -f /smip_uav_ws/install/setup.bash ] && source /smip_uav_ws/install/setup.bash" >> ~/.bashrc
    echo "cd /smip_uav_ws" >> ~/.bashrc
fi

exec "$@"
