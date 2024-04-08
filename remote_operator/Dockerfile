FROM ros:humble-ros-core
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*


# install ros2 packages
RUN apt update && apt install -y \
        ros-$ROS_DISTRO-demo-nodes-cpp \
	ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# network config
RUN apt-get update && apt-get install -y \
    iputils-ping \
    iproute2 \
    vim

WORKDIR /bb_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
                  colcon build"
RUN /bin/bash -c "source /bb_ws/install/setup.bash"



# Create a directory to store APT repository keys
RUN install -d -m 0755 /etc/apt/keyrings

# Install wget if not available in the base image
RUN apt-get update && apt-get install -y wget gnupg

# Import the Mozilla APT repository signing key
RUN wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O- | tee /etc/apt/keyrings/packages.mozilla.org.asc > /dev/null

# Add the Mozilla APT repository to your sources list
RUN echo "deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main" > /etc/apt/sources.list.d/mozilla.list

# Configure APT to prioritize packages from the Mozilla repository
RUN echo "Package: *\nPin: origin packages.mozilla.org\nPin-Priority: 1000" > /etc/apt/preferences.d/mozilla

# Update your package list and install the Firefox .deb package
RUN apt-get update && apt-get install -y firefox


# setup entrypoint
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
