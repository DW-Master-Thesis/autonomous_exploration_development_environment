FROM ros:humble

# Install requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils dialog 2>&1 && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        curl \
        libsm6 \
        libxext6 \
        libusb-dev \
        openssh-client \
        sudo && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-desktop-full \
        ros-humble-xacro \
        ros-humble-gazebo-msgs \
        ros-humble-gazebo-plugins \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-ros2-control \
        ros-humble-gazebo-ros-pkgs \
        ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Set remote user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID="$USER_UID"
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> "/etc/sudoers.d/${USERNAME}" && \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

