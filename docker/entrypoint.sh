#!/bin/bash
set -e

# Rebuild shared library cache. Required on first start of the runtime image
# because it is assembled via dpkg-deb extraction (post-install scripts were
# not run during the QEMU-free image build). Runs natively on the drone.
ldconfig 2>/dev/null || true

if ! grep -q "smip_uav_ws" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source /opt/px4_ws/install/setup.bash" >> ~/.bashrc
    echo "[ -f /smip_uav_ws/install/setup.bash ] && source /smip_uav_ws/install/setup.bash" >> ~/.bashrc
    echo "cd /smip_uav_ws" >> ~/.bashrc
fi

exec "$@"
