
# Docker file for a ROS Melodic environment with kinova-ros package for running Kinova Jaco 2 Arm

FROM osrf/ros:melodic-desktop

# nvidia-container-runtime
# For running Rviz
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Replace 1000 with your user / group id
RUN export uid=4500 gid=1800 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

EXPOSE 9030
EXPOSE 9031

RUN apt-get update && apt-get install -y \
    python \
    python-tk \
    git \
    python-pip \
    vim \
    build-essential \
    unzip \
    wget \
    libglfw3 \
    libglew2.0 \
    libgl1-mesa-glx \
    libosmesa6 \
    libblas-dev \
    liblapack-dev \
    screen \
    man \
    net-tools \
    openssh-client \
    ssh \ 
    netcat \ 
    iputils-ping \
    rsync  \
    usbutils \
    software-properties-common

RUN apt-get install -y \
    ros-melodic-moveit \
    ros-melodic-tf-conversions \
    ros-melodic-trac-ik \
    ros-melodic-eigen-conversions \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-robot-state-publisher

RUN pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages scipy
RUN pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages ipython
RUN pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages pid
RUN pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages numpy
RUN pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages matplotlib
RUN sudo pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages -U rosdep
RUN sudo pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages rospkg
RUN sudo pip install --target=/opt/ros/melodic/lib/python2.7/dist-packages python-error
RUN pip uninstall -y enum34
RUN pip install scipy --upgrade
#RUN CP /usr/lib/python2.7/dist-packages/ge


# Source ROS setup.bash
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash" 

# Make and initialize the catkin_ws
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN /bin/bash -c "source ~/catkin_ws/devel/setup.bash"

# Clone and make the kinova-ros package
RUN cd ~/catkin_ws/src/ \
    && git clone -b melodic-devel https://github.com/Kinovarobotics/kinova-ros.git \
    && /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'

# Clone and make the ros_interface package
RUN cd ~/catkin_ws/src/ \
    && git clone https://github.com/melfm/ros_interface.git \
    && /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN  cd ~/catkin_ws/src/ \ 
    && git clone https://github.com/SoulGateW/std_msgs_stamped.git \
    && cd .. \
    && /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make --pkg std_msgs_stamped;catkin_make install' 

RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc
RUN echo 'source ~/catkin_ws/devel/setup.bash' >> /root/.bashrc


RUN  /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws/src/; catkin_create_pkg custom_driver rospy std_msgs geometry_msgs kinova_msgs actionlib'


RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release; catkin_make install'
RUN bin/bash -c '. /opt/ros/melodic/setup.bash && cd ~/catkin_ws && rosdep update'
RUN bin/bash -c '. /opt/ros/melodic/setup.bash && cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y'

WORKDIR /root/catkin_ws/src/custom_driver/src/ 
COPY ./UI_robot_driver.py . 
COPY ./Input_logging_node.py . 
RUN chmod +x UI_robot_driver.py 
RUN chmod +x Input_logging_node.py 

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release; catkin_make install'
RUN rm -rf UI_robot_driver.py 
RUN rm -rf Input_logging_node.py 
COPY true_main/ .

RUN chmod +x UI_robot_driver.py 
RUN chmod +x Input_logging_node.py 
WORKDIR /root/catkin_ws/src
RUN mkdir ./logs/