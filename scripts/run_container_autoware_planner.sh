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
        NAME_CONTAINER="autoware_universe_planner_${i}"
        docker run \
                -d \
                -it \
                --gpus all \
                --privileged \
                --name $NAME_CONTAINER \
                -v $PROJECT_DIR/autoware_planner:$HOME/autoware  \
                -v $PROJECT_DIR/data/carla_map:$HOME/carla_map \
                -v $SCRIPTPATH:$HOME/scripts \
                -e DISPLAY \
                -e TERM  \
                -e QT_X11_NO_MITSHM=1   \
                -e XAUTHORITY=/tmp/.dockervzalb41d.xauth \
                -v /tmp/.dockervzalb41d.xauth:/tmp/.dockervzalb41d.xauth   \
                -v /tmp/.X11-unix:/tmp/.X11-unix   \
                -v /etc/localtime:/etc/localtime:ro  \
                ae71e90c4c9d
done
