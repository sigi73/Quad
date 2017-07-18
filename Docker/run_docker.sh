docker run --rm -it -p 4000:11311 \
    --name="rosquad" \
    --volume="$PWD/../ROS_Navigation/:/ROS_Navigation" \
    --privileged \
    --device=/dev/video0 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    rosquad
