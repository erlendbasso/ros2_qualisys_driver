FROM  osrf/ros:humble-desktop

ENV ROS_DISTRO=humble


RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && echo Sourced ROS-${ROS_DISTRO}" >> /root/.bashrc;

# RUN apt-get update && apt-get install 

# INSTALL OPENCV
RUN apt-get update && apt-get install -y cmake g++ wget unzip 


WORKDIR /tmp

# BUILD AND INSTALL
# RUN mkdir opencv && \
#     wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip && \
#     wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip && \
#     unzip opencv.zip && \
#     unzip opencv_contrib.zip &&\
#     mkdir -p build && cd build &&\
#     cmake -D CMAKE_BUILD_TYPE=RELEASE -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.5/modules -D WITH_GSTREAMER=ON ../opencv-4.5.5 && \
#     cmake --build . --target install -j`nproc`

# RUN rm -rf /tmp

# RUN wget -O mavsdk.deb https://github.com/mavlink/MAVSDK/releases/download/v1.0.8/libmavsdk-dev_1.0.8_ubuntu20.04_amd64.deb \
    # && dpkg -i mavsdk.deb

# RUN git clone https://github.com/mavlink/MAVSDK.git \
#     && cd MAVSDK \
#     && git checkout v1.0.8 \
#     && git submodule update --init --recursive \
#     && cmake -Bbuild/default -DCMAKE_BUILD_TYPE=Release -H. \
#     && cmake --build build/default -j`nproc` \
#     && cmake --build build/default --target install

# RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus\
#     && cd /tmp/Sophus \
#     && mkdir build && cd build \
#     && cmake .. -DCMAKE_BUILD_TYPE=Release \
#     && make install

# RUN rosdep update

# RUN apt-get install ros-foxy-realtime-tools pip libfmt-dev -y
# RUN pip install conan

# COPY ./startup.sh /startup.sh
# COPY ./run.sh /run.sh

# RUN chmod a+x /startup.sh
# RUN chmod a+x /run.sh

# ENV FASTRTPS_DEFAULT_PROFILES_FILE="/workspaces/ros2_ws/src/ros2_qualisys_driver/DEFAULT_FASTRTPS_PROFILES.xml"

# ENTRYPOINT ["/bin/bash", "-c", "/startup.sh  && $@", ""]

CMD ["/bin/bash"]
