# Make Let the docker container access the display
# run $ sh display.sh

# xhost +local:docker
# XSOCK=/tmp/.X11-unix
# XAUTH=/tmp/.docker.xauth
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
# docker run -m 8GB -it --rm -e DISPLAY=$DISPLAY -v $XSOCK:$XSOCK -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH -v ${PWD}:/src  -it jerinka/opencv:1
# xhost -local:docker

xhost +local:docker
docker run -it --device /dev/video0:/dev/video0 -v $(pwd):/home -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -p 5000:5000 -p 8888:8888 opencvcourses/opencv-docker

