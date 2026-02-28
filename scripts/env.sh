source autoware/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PYTHONPATH:$HOME/data/PythonAPI/carla
sudo ip link set lo multicast on