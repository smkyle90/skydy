CONTAINER_NAME=holdings

sudo docker run -it --privileged --rm \
  --network host \
  -v $(pwd):/app:rw \
  --name=${CONTAINER_NAME} \
  ${CONTAINER_NAME}:latest \
