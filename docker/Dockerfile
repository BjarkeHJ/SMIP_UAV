
# ==================================================
# Dependencies Image
# ==================================================
FROM ubuntu:22.04 AS smip-deps

ENV DEBIAN_FRONTEND=noninteractive

# Setup locale
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS2 Humble Sources
RUN apt-get update && apt-get install -y \
  software-properties-common \
  curl \
  && add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  ros-humble-ros-base \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-launch-testing \
  ros-humble-ament-cmake \
  && rm -rf /var/lib/apt/lists/*

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  cmake \
  git \
  iproute2 \
  pkg-config \
  python3-dev \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro humble

# Third-party apt dependencies (dev packages: headers + libs)
RUN echo "===== Building Third-Party apt dependencies ====="
COPY dependencies/apt-deps.txt /tmp/apt-deps.txt
RUN apt-get update \
  && grep -vE '^\s*#|^\s*$' /tmp/apt-deps.txt | xargs -r apt-get install -y --no-install-recommends \
  && rm -rf /var/lib/apt/lists/*

# Third-party Python dependencies
RUN echo "===== Building Third-Party Python dependencies ====="
COPY dependencies/pip-deps.txt /tmp/pip-deps.txt
RUN pip3 install --no-cache-dir -r /tmp/pip-deps.txt

RUN ldconfig

# ==================================================
# Development Image
# ==================================================
FROM smip-deps AS smip-dev

RUN apt-get update && apt-get install -y --no-install-recommends \
  gdb \
  vim \
  ssh \
  sudo \
  && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ==================================================
# ARM64 Sysroot (QEMU-built, file source only)
# Must use a registry base image (ubuntu:22.04) — NOT a local stage.
# BuildKit does not reliably rebuild local stages for a different platform;
# it reuses the cached x86_64 layer. Starting from ubuntu:22.04 forces
# BuildKit to pull the real arm64 variant via QEMU.
# ==================================================
FROM --platform=linux/arm64 ubuntu:22.04 AS smip-sysroot

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
  software-properties-common \
  curl \
  && add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  ros-humble-ros-base \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-launch-testing \
  ros-humble-ament-cmake \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  cmake \
  python3-dev \
  python3-pip \
  pkg-config \
  && rm -rf /var/lib/apt/lists/*

COPY dependencies/apt-deps.txt /tmp/apt-deps.txt
RUN apt-get update \
  && grep -vE '^\s*#|^\s*$' /tmp/apt-deps.txt | xargs -r apt-get install -y --no-install-recommends \
  && rm -rf /var/lib/apt/lists/*

RUN ldconfig

# ==================================================
# x86 Cross-Compilation Image
# smip-deps (x86) + aarch64 toolchain + arm64 sysroot.
# colcon build runs natively on x86 and outputs aarch64 binaries.
# ==================================================
FROM smip-deps AS smip-crosscompile

# aarch64 cross-compiler: native x86 binary that emits arm64 machine code
RUN apt-get update && apt-get install -y --no-install-recommends \
  gcc-aarch64-linux-gnu \
  g++-aarch64-linux-gnu \
  binutils-aarch64-linux-gnu \
  && rm -rf /var/lib/apt/lists/*

# Pull the ARM64 filesystem from the QEMU-built sysroot stage.
# /usr holds all arm64 headers and libraries (including aarch64-linux-gnu/).
# /opt/ros/humble holds the arm64 ROS2 installation for cmake config discovery.
COPY --from=smip-sysroot /usr            /sysroot/usr
COPY --from=smip-sysroot /opt/ros/humble /sysroot/opt/ros/humble

# Ubuntu 22.04 uses /lib -> usr/lib (relative symlink).
# Recreate it so the cross-linker resolves /sysroot/lib/aarch64-linux-gnu correctly.
RUN ln -s usr/lib /sysroot/lib

# The arm64 cmake config files inside the sysroot have hardcoded IMPORTED_LOCATION
# paths like /opt/ros/humble/lib/libfoo.so. GNU ld opens these as literal absolute
# paths — --sysroot does NOT rewrite them. Without this copy, the linker would open
# the x86_64 libs from smip-deps and fail with "file in wrong format".
# Overwrite the x86_64 ROS2 libs with arm64 ones so the absolute paths resolve correctly.
# The rosidl/ament generators are pure Python and keep working after this overwrite.
COPY --from=smip-sysroot /opt/ros/humble/lib /opt/ros/humble/lib

COPY aarch64-toolchain.cmake /toolchain.cmake

# cmake strips the sysroot prefix when generating Makefile dependency rules,
# so Make checks canonical paths like /usr/lib/aarch64-linux-gnu/libpython3.10.so
# rather than /sysroot/usr/... The linker (with --sysroot) finds the file correctly,
# but Make fails before reaching the linker. Symlink the canonical path to the
# sysroot location so the dependency check passes.
RUN mkdir -p /usr/lib/aarch64-linux-gnu && \
    ln -sf /sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so \
           /usr/lib/aarch64-linux-gnu/libpython3.10.so

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


# ==================================================
# Onboard Runtime Image
# ==================================================
FROM ubuntu:22.04 AS smip-runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV RC_ALL=LC_ALL

# Setup ROS2 Humble Sources
RUN apt-get update && apt-get install -y \
  software-properties-common \
  curl \
  && add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages (base + build tools) 
RUN apt-get update && apt-get install -y \
  ros-humble-ros-base \
  ros-humble-rmw-fastrtps-cpp \
  && rm -rf /var/lib/apt/lists/*

# Minimal system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-pip \
  sudo \ 
  && rm -rf /var/lib/apt/lists/*

# Third-party apt dependencies (dev packages: headers + libs)
COPY dependencies/apt-deps.txt /tmp/apt-deps.txt
RUN apt-get update \
  && grep -vE '^\s*#|^\s*$' /tmp/apt-deps.txt \
    | xargs -r apt-get install -y --no-install-recommends \
  && rm -rf /var/lib/apt/lists/*

# Third-party Python dependencies
COPY dependencies/pip-deps.txt /tmp/pip-deps.txt
RUN pip3 install --no-cache-dir -r /tmp/pip-deps.txt

# Copy source-built libraries from deps stage (Only the installed artifacts - no source code)
COPY --from=smip-deps /usr/local/lib /usr/local/lib
COPY --from=smip-deps /usr/local/share/ /usr/local/share
COPY --from=smip-deps /usr/local/bin /usr/local/bin

CMD ["/bin/bash"]
