: ${IMAGE_NAME:=asssaf/vl53l4cd:latest}
BASE="$(dirname $0)/.."
docker build -t $IMAGE_NAME -f $BASE/docker/Dockerfile $BASE
