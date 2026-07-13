#!/usr/bin/env bash
set -euo pipefail

# Install robot_localization without modifying /opt/ros or requiring sudo.
DEB_DIR="${HOME}/robot_localization_debs"
OVERLAY_ROOT="${HOME}/ros_overlay"

rm -rf "${DEB_DIR}"
mkdir -p "${DEB_DIR}" "${OVERLAY_ROOT}"
cd "${DEB_DIR}"
apt download \
  ros-humble-robot-localization \
  ros-humble-geographic-msgs \
  libgeographic19
find "${DEB_DIR}" -maxdepth 1 -name '*.deb' \
  -exec dpkg-deb -x '{}' "${OVERLAY_ROOT}" ';'

echo "robot_localization extracted under ${OVERLAY_ROOT}"
