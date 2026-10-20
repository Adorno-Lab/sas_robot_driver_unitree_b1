FROM murilomarinho/sas:jazzy
ARG ROS_DOMAIN_ID=1
ENV ROS_DOMAIN_ID=$ROS_DOMAIN_ID
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

# The base image already has ROS 2 Jazzy, the SAS packages, DQ Robotics, and marinholab::sas::core.

# Update the SAS packages of the base image. Only those already installed. The debug symbols are removed first,
# as they pin the old versions.
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Clone sas_tools and the driver
RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws/src && \
    git clone https://github.com/Adorno-Lab/sas_tools.git && \
    git clone -b new_generation https://github.com/Adorno-Lab/sas_robot_driver_unitree_b1 --recursive

# Build the driver. sas_legged_client_window (sas_tools) is not built, as it needs Qt.
RUN source /opt/ros/jazzy/setup.bash && \
    cd ~/ros2_ws && \
    colcon build --packages-up-to sas_robot_driver_unitree_b1

# ALIASES
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env
RUN echo "source ~/ros2_ws/install/setup.bash" >> /etc/bash_env
RUN echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /etc/bash_env
RUN echo "alias cdros2='cd ~/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build --packages-up-to sas_robot_driver_unitree_b1 && source install/setup.bash' " >> /etc/bash_env
RUN echo "alias launch_ROS2_drivers_white='ros2 launch sas_robot_driver_unitree_b1 real_b1_white_robot_launch.py' " >> /etc/bash_env
RUN echo "alias launch_ROS2_drivers_black='ros2 launch sas_robot_driver_unitree_b1 real_b1_black_robot_launch.py' " >> /etc/bash_env
RUN echo "source /etc/bash_env" >> ~/.bashrc
