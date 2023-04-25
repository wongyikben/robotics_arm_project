FROM dorowu/ubuntu-desktop-lxde-vnc:latest

SHELL ["/bin/bash", "-c"]

WORKDIR /root
ENV HOME=/root

COPY packages.txt /tmp/packages.txt

RUN wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | apt-key add - \
    && apt-get update \
    && apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && xargs apt-get install -y < /tmp/packages.txt

ENV RESOLUTION=1920x1080x24

COPY . /root/src/cr_ros_pkg
RUN git clone -b foxy-devel https://github.com/Box-Robotics/ros2_numpy.git src/ros2_numpy

ENV ROS_DISTRO=foxy

RUN rosdep init \
    && rosdep update \
    && rosdep install --from-paths /root/src -y --ignore-src

RUN /bin/bash -c ". /opt/ros/foxy/setup.bash && colcon build --symlink-install"
RUN echo ". /root/install/setup.bash" >> /root/.bashrc
