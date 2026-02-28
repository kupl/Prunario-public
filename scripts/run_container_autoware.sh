SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
PROJECT_DIR=$SCRIPTPATH/..

# Set the number of GPUs to use. Default is 1.
if [[ $1 == '' ]] ; then
	NUM_GPU=1
else
        NUM_GPU=$1
fi


for i in $(seq 0 $((NUM_GPU-1)))
do
        NAME_CONTAINER="autoware_universe_4a3de49_${i}"
        DIR_SHARED=$PROJECT_DIR/shared/autoware_universe_${i}
        NAME_BRIDGE="autoware_carla_interface"
        cp -r $PROJECT_DIR/data/shared_base $DIR_SHARED
        sudo chown -R $(id -u):$(id -g) $DIR_SHARED
        docker run \
                -d \
                -it \
                --gpus all \
                --privileged \
                --name $NAME_CONTAINER \
                -v $PROJECT_DIR/autoware:$HOME/autoware  \
                -v $PROJECT_DIR/autoware_data:$HOME/autoware_data \
                -v $PROJECT_DIR/data/carla_map:$HOME/carla_map \
                -v $SCRIPTPATH:$HOME/scripts \
                -v $PROJECT_DIR/data:$HOME/data \
                -v $DIR_SHARED:/$HOME/shared \
                -v $DIR_SHARED/src:$HOME/autoware/src/universe/autoware.universe/simulator/$NAME_BRIDGE/src/$NAME_BRIDGE/src \
                -v $DIR_SHARED/autoware_launch:$HOME/autoware/install/autoware_launch/share/autoware_launch/tmp \
                -v $DIR_SHARED/carla_autoware:$HOME/autoware/install/$NAME_BRIDGE/share/$NAME_BRIDGE/tmp \
                -e DISPLAY \
                -e TERM  \
                -e QT_X11_NO_MITSHM=1   \
                -e XAUTHORITY=/tmp/.dockervzalb41d.xauth \
                -v /tmp/.dockervzalb41d.xauth:/tmp/.dockervzalb41d.xauth   \
                -v /tmp/.X11-unix:/tmp/.X11-unix   \
                -v /etc/localtime:/etc/localtime:ro  \
                ae71e90c4c9d
done
