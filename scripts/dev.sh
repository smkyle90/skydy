CONTAINER_NAME=pydy

xhost +local:

sudo docker run -it --privileged --rm \
  --network host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -v $(pwd):/app:rw \
  --name=${CONTAINER_NAME} \
  ${CONTAINER_NAME}:latest
