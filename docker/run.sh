xhost +

sudo docker run \
  --restart=no \
  -it \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --env ROS_IP=localhost \
  --env ROS_HOSTNAME=localhost \
  --env="DISPLAY" \
  --volume "$(pwd)/../linked_folder/chord_detection:/home/catkin_ws/src/chord_detection" \
  --device /dev/snd \
  --net=host \
  ros:chord_detection \
  -c "source /home/init_build_and_start"
  # -c "/bin/bash"
