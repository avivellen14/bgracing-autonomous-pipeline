FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-geometry-msgs \
    python3-numpy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY publisher.py .

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 publisher.py"]