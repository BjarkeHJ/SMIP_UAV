set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /sysroot)

set(CMAKE_FIND_ROOT_PATH /sysroot)
# Generators (rosidl, ament) are x86 host tools — never search sysroot for executables
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Libraries and headers must come from the arm64 sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# BOTH: search sysroot first (for arm64 ROS2), then normal prefix paths.
# This lets cmake find workspace-installed packages (e.g. px4_msgs built earlier
# by colcon and installed to /smip_uav_ws/install/) which live outside the sysroot.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# Explicit Python hints so FindPythonLibs doesn't search and fail.
# rosidl_generator_py compiles Python C-extension .so files that link against
# the target (arm64) libpython — these must point into the sysroot.
set(PYTHON_LIBRARY     /sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so CACHE FILEPATH "")
set(PYTHON_INCLUDE_DIR /sysroot/usr/include/python3.10                      CACHE PATH "")
