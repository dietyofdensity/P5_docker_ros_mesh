# syntax=docker/dockerfile:1

# Comments are provided throughout this file to help you get started.
# If you need more help, visit the Dockerfile reference guide at
# https://docs.docker.com/go/dockerfile-reference/

# Want to help us make this template better? Share your feedback here: https://forms.gle/ybq9Krt8jtBL3iCk7

ARG PYTHON_VERSION=3.12.3
#FROM python:${PYTHON_VERSION}-slim as base
FROM osrf/ros:noetic-desktop



# Prevents Python from writing pyc files.
ENV PYTHONDONTWRITEBYTECODE=1

# Keeps Python from buffering stdout and stderr to avoid situations where
# the application crashes without emitting any logs due to buffering.
ENV PYTHONUNBUFFERED=1

WORKDIR /
RUN apt-get update
RUN apt-get install -y pip 
RUN apt-get install -y git
RUN apt install -y udev
RUN sudo apt-get install -y python3-catkin-tools
RUN sudo apt install -y libportaudio2 
RUN sudo pip install zmq msgpack
RUN sudo pip install msgpack 
RUN sudo pip install pygame 

# Create a non-privileged user that the app will run under.
# See https://docs.docker.com/go/dockerfile-user-best-practices/
ARG UID=10001
RUN adduser \
    --disabled-password \
    --gecos "" \
    --home "/nonexistent" \
    --shell "/sbin/nologin" \
    --no-create-home \
    --uid "${UID}" \
    appuser

# Download dependencies as a separate step to take advantage of Docker's caching.
# Leverage a cache mount to /root/.cache/pip to speed up subsequent builds.
# Leverage a bind mount to requirements.txt to avoid having to copy them into
# into this layer.
RUN --mount=type=cache,target=/root/.cache/pip \
    --mount=type=bind,source=requirements.txt,target=requirements.txt \
      pip install -r requirements.txt

# Switch to the non-privileged user to run the application.
COPY ./crerob-pytongue.zip /app/
RUN pip install /app/crerob-pytongue.zip 

RUN mkdir -p /catkin_ws/src/
RUN /bin/bash -c ' cd /catkin_ws/ && source /ros_entrypoint.sh && catkin_make '

RUN  /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws/src/; catkin_create_pkg itongue_custom rospy std_msgs  '
COPY ./scripts/ /catkin_ws/src/itongue_custom/src/

RUN /bin/bash -c 'cd /catkin_ws/src/itongue_custom/src && chmod +x *'

WORKDIR /catkin_ws/src/ 
RUN git clone https://github.com/SoulGateW/std_msgs_stamped.git 
WORKDIR /catkin_ws/
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make --pkg std_msgs_stamped;catkin_make install' 
RUN /bin/bash -c ' cd /catkin_ws/ && source /ros_entrypoint.sh && catkin_make ' 

WORKDIR /catkin_ws
RUN /bin/bash -c '. /ros_entrypoint.sh && catkin_make'





#USER appuser

# Copy the source code into the container.
COPY . .

# Expose the port that the application listens on.
EXPOSE 8001

# Run the application.
CMD python3 -m tongue  app:app --host=0.0.0.0 --port=8000
