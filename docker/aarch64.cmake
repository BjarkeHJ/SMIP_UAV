set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# arm64 sysroot: all arm64 .deb packages are extracted here.
# ament cmake configs compute IMPORT_PREFIX relative to their file location,
# so configs in /opt/arm64-sysroot/opt/ros/humble/share/ correctly resolve
# IMPORTED_LOCATION to arm64 libraries inside the sysroot.
set(CMAKE_SYSROOT /opt/arm64-sysroot)
set(CMAKE_FIND_ROOT_PATH /opt/arm64-sysroot /opt/px4_ws/install)

# Build tools (python, cmake scripts) run natively on the amd64 host
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Link against arm64 libraries only (in sysroot or px4_ws/install)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# Headers are arch-independent; search both sysroot and host include paths
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
# cmake configs: search sysroot paths first (arm64), then host (amd64) as fallback
# for packages like px4_msgs whose install prefix sits outside the sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
