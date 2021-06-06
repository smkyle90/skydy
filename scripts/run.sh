CONTAINER_NAME=pydy

sudo docker run -it --privileged --rm \
  --network host \
  -v $(pwd):/skydy:rw \
  --name=${CONTAINER_NAME} \
  ${CONTAINER_NAME}:latest \
  /bin/bash
