: ${IMAGE_NAME:=asssaf/vl53l4cd:latest}
docker run --rm -it --privileged --device /dev/i2c-1 "$IMAGE_NAME" $*
