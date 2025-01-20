#!\bin\bash

docker stop pupil_dock
docker stop jaco_dock
docker stop itongue_dock

docker remove pupil_dock
docker remove jaco_dock
docker remove itongue_dock

xhost local:root

XAUTH=/tmp/.docker.xauth

D_IMG=itongue_ros:v2


docker run -dit \
        --name jaco_dock \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/etc/localtime:/etc/locltime:ro" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --privileged \
        --rm \
        jaco_2:v6 \
        /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300 feedback_publish_rate:=0.1'

sleep 8

docker run -dit \
        --name pupil_dock \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/etc/localtime:/etc/locltime:ro" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --privileged \
        --rm \
        eyetrack_ros:v1 \
        /bin/bash -c 'cd /pupil/pupil_src/ && python3 main.py capture' 

docker exec -dit pupil_dock /bin/bash -c 'source /ros_entrypoint.sh && source /catkin_ws/devel/setup.bash && rosrun pupil_custom pupiltrackernode.py'

sleep 2
docker exec -dit jaco_dock    /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun custom_driver UI_robot_driver.py'

#docker exec -it jaco_dock    /bin/bash -c 'source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun custom_driver Input_logging_node.py'
docker run -it \
        --name itongue_dock \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/etc/localtime:/etc/locltime:ro" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --privileged \
        --rm \
        itongue_ros:v4 \
        /bin/bash ric_.bash

docker cp jaco_dock:/root/catkin_ws/src/logs/ ./logs/
