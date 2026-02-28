SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
PROJECT_DIR=$SCRIPTPATH/..

DOCKER_IMAGE=autoware_universe_4a3de49
DOCKER_FILE=$SCRIPTPATH/../docker/Dockerfile

if [ ! "$(docker images -q ${DOCKER_IMAGE})" ]; then
    echo "${DOCKER_IMAGE} does not exist. Creating..."
    docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -f ${DOCKER_FILE} -t ${DOCKER_IMAGE} .
fi

rocker --nvidia \
       --privileged \
       --x11 \
       --user \
       --network host \
       --volume $PROJECT_DIR/autoware:$HOME/autoware \
       --volume $PROJECT_DIR/autoware_data:$HOME/autoware_data \
       --volume $SCRIPTPATH:$HOME/scripts \
       --volume $PROJECT_DIR/data/carla_map:/$HOME/carla_map \
       --volume $PROJECT_DIR/shared:/$HOME/shared \
       --volume $PROJECT_DIR/data:$HOME/data \
       -- ${DOCKER_IMAGE}