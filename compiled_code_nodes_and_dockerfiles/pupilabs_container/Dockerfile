FROM osrf/ros:noetic-desktop
RUN apt-get update
RUN apt-get install -y pip
RUN apt-get install -y git
RUN apt install -y udev
RUN sudo apt-get install -y python3-catkin-tools
RUN git clone https://github.com/pupil-labs/pupil.git && cd pupil && git checkout develop && python3 -m pip install -r requirements.txt
#COPY ./10-libuvc.rules /e11tc/udev/rules.d/
RUN sudo echo 'SUBSYSTEM=="usb",  ENV{DEVTYPE}=="usb_device", GROUP="plugdev", MODE="0664"' | sudo tee /etc/udev/rules.d/10-libuvc.rules > /dev/null && sudo udevadm trigger
RUN sudo apt install -y libportaudio2 
RUN sudo pip install zmq msgpack
RUN sudo pip install msgpack 
RUN sudo pip install pygame 
RUN mkdir -p ./catkin_ws/src/

WORKDIR /catkin_ws/src/

RUN  /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws/src/; catkin_create_pkg pupil_custom rospy std_msgs  '
WORKDIR /catkin_ws/src/pupil_custom/src/
COPY ./coppy_to_src .
RUN /bin/bash -c 'cd /catkin_ws/src/pupil_custom/src/ && chmod +x pupiltrackernode.py '

RUN cd / && /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ./catkin_ws; catkin_make'

RUN mkdir /pupil/capture_settings
COPY ./pupil_settings/capture_settings/ /pupil/capture_settings/
COPY ./pupil_settings/root1/ /root/
COPY ./pupil_settings/tmp/ /tmp/
COPY ./pupil_settings/var/ /var/

CMD /bin/bash 'source /ros_entrypoint.sh && source /catkin_ws/devel/setup.bash '
