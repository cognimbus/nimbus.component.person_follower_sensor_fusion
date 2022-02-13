FROM ros:noetic

COPY ./leg_target_detector /person_follower_sensor_fusion_ws/src/leg_target_detector
COPY ./object_msgs /person_follower_sensor_fusion_ws/src/object_msgs
COPY ./person_follower_sensor_fusion /person_follower_sensor_fusion_ws/src/person_follower_sensor_fusion
COPY ./bbox_convertor /person_follower_sensor_fusion_ws/src/bbox_convertor

WORKDIR /person_follower_sensor_fusion_ws



    

RUN apt update && apt-get install ros-noetic-vision-msgs && \
    apt-get install ros-noetic-image-transport-plugins -y && \ 
    apt-get install ros-noetic-image-transport -y && \
    apt-get install ros-noetic-cv-bridge -y && \
    apt-get install nano -y && \
    rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/noetic/setup.sh && catkin_make  



COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


