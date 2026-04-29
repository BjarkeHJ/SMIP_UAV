
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
