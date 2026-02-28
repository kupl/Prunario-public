CMD="source scripts/env.sh && \
    ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/carla_map/Town01 \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit \
    simulator_type:=carla \
    carla_map:=Town01"

docker exec -it autoware_universe_1 bash -c "$CMD"