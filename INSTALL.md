# How to set the running environment

This file contains the environmental settings to run **Prunario** to test an ADS **Autoware** in the simulation environment **CARLA**.
- Autoware(v0.39.1): https://github.com/autowarefoundation/autoware/tree/0.39.1
- CARLA(v0.9.15): https://github.com/carla-simulator/carla/tree/0.9.15

## Requirements

We tested running Prunario with the environmental settings as below:
- Ubuntu 20.04 (Autoware v0.39.1 strictly requires ubuntu 22.04 which supports ROS2 humble, but we run it on a docker container.)
- Python 3.8 (with Anaconda 22.9.0)
- 1 GPU (GeForce RTX 2070 SUPER with 8GB of VRAM)
- Cuda 12.3 (following [here](https://github.com/autowarefoundation/autoware/blob/5c01fb0991ea70c3ccb9112ba9739511bb38a491/amd64.env#L6))
- Nvidia Driver 545.23.08


Please refer to hardware requirements for the simulator(CARLA[[link](https://carla.readthedocs.io/en/latest/start_quickstart/#before-you-begin)]) and the ADS to test(Autoware[[link](https://autowarefoundation.github.io/autoware-documentation/main/installation/#minimum-hardware-requirements)]).


## 0. Download

Download all files from [figshare](https://figshare.com/s/9167d245ffbc1385f90c) and locate each files as below:
```bash
# At the project root
tar -xzf autoware.tar.gz -C  ./
tar -xzf autoware_planner.tar.gz -C  ./
tar -xzf autoware_data.tar.gz -C  ./
tar -xzf carla_map.tar.gz -C  ./data/
mv carla-0.9.15-cp38-cp38-linux_x86_64.whl ./data/
mv carla-0.9.15-cp310-cp310-linux_x86_64.whl ./data/
```


## 1. Install Dependencies

```bash
pip install -r requirements.txt
pip install data/carla-0.9.15-cp38-cp38-linux_x86_64.whl
```

## 2. Install CARLA

Pull CARLA v0.9.15 image (This is the only required step, as Prunario automatically turns on the simulator before run testing)
```bash
# This might takes several minutes.
docker pull carlasim/carla:0.9.15
```

## 3. Setting Autoware and its Planner

### Setting Autoware

- Pull Autoware v0.39.1 image
```bash
docker pull ghcr.io/autowarefoundation/autoware:universe-devel-cuda-0.39.1-amd64
```

- Create Autoware image and run into the temporary container
```bash
bash scripts/create_container_autoware.sh
```

- After running the command above, copy the id of the created image from the log and paste it to `scripts/run_container_autoware.sh`
  - In my case, **e76b873900b3** is the id of the image to copy.
```bash
# log from the command bash scripts/create_container_autoware.sh
...
building >  ---> Using cache
building >  ---> e76b873900b3
building > Successfully built e76b873900b3
Executing command:
docker run --rm -it --network host   --gpus all --privileged -v /ssd4tb/autoware:/home/mins/autoware -v /home/mins/autoware_data:/home/mins/autoware_data -v /ssd1tb/the-fuzzer/autoware_carla_launch/carla_map://home/mins/carla_map  -e DISPLAY -e TERM   -e QT_X11_NO_MITSHM=1   -e XAUTHORITY=/tmp/.docker40efsz2g.xauth -v /tmp/.docker40efsz2g.xauth:/tmp/.docker40efsz2g.xauth   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /etc/localtime:/etc/localtime:ro  e76b873900b3
```

```bash
# scripts/run_container_autoware.sh
docker run \
        -d \
        ...
        -v /etc/localtime:/etc/localtime:ro  \
        e76b873900b3 <- here
```

- Build Autoware inside the temporary container (all the commands below are based on the commands from [autoware documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-workspace))
```bash
# Install dependencies
cd autoware
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r

# Build Autoware (may take 20-30 minutes)
sudo chown -R $(id -u):$(id -g) install
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- After exiting from the temporary container, run Autoware container. If you want to run Prunario in parallel, you can specify the number of GPUs to use (default: 1).
```bash
bash scripts/run_container_autoware.sh <number of GPU>
```

- After executing the command above, make sure that the container `autoware_universe_4a3de49_X` is running, where `X` is the id for each GPU.
  - If you set the `<number of GPU>` as 2, for example, two containers `autoware_universe_4a3de49_0` and `autoware_universe_4a3de49_1` should be generated as below.
```
CONTAINER ID   IMAGE              COMMAND                  CREATED        STATUS             PORTS        NAMES
xxxxxxxxxxxx   e76b873900b3       "/ros_entrypoint.sh …"   2 months ago   Up 56 minutes                   autoware_universe_4a3de49_1
xxxxxxxxxxxx   e76b873900b3       "/ros_entrypoint.sh …"   2 months ago   Up 57 minutes                   autoware_universe_4a3de49_0
```

- Install the dependencies. If multiple containers are used, ensure that the dependencies are installed in each container.
```bash
# Get in to the running container
docker exec -it autoware_universe_4a3de49_0 bash
# Install dependencies
bash scripts/setup_autoware.sh
```

- Install the Autoware perception stack. This step needs to be performed only once across all containers. The built perception stack will then be shared among them.

```bash
# Get in to the running container
docker exec -it autoware_universe_4a3de49_0 bash

# Build models
source /opt/ros/humble/setup.bash
source autoware/install/setup.bash
bash scripts/build_models.sh
```

### Setting Autoware Planner

- Create Autoware Planner container
```
bash scripts/create_container_autoware_planner.sh
```
- (Same process for the Autoware container) After running the command above, copy the id of the created image from the log and paste it to `scripts/run_container_autoware_planner.sh`
  - In my case, **aedb5b7af314** is the id of the image to copy.
```bash
# log from the command bash scripts/create_container_autoware.sh
...
building >  ---> Using cache
building >  ---> aedb5b7af314
building > Successfully built aedb5b7af314
Executing command:
docker run --rm -it --network host   --gpus all --privileged -v /ssd4tb/autoware:/home/mins/autoware -v /home/mins/autoware_data:/home/mins/autoware_data -v /ssd1tb/the-fuzzer/autoware_carla_launch/carla_map://home/mins/carla_map  -e DISPLAY -e TERM   -e QT_X11_NO_MITSHM=1   -e XAUTHORITY=/tmp/.docker40efsz2g.xauth -v /tmp/.docker40efsz2g.xauth:/tmp/.docker40efsz2g.xauth   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /etc/localtime:/etc/localtime:ro  aedb5b7af314
```

```bash
# scripts/run_container_autoware_planner.sh
docker run \
        -d \
        ...
        -v /etc/localtime:/etc/localtime:ro  \
        aedb5b7af314 <- here
```

- Build Autoware Planner
```bash
# Install dependencies
cd autoware
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r

# Build Autoware (may take 20-30 minutes)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- Create Autoware Planner container. Please specify the same number of GPU used for creating Autoware container.
```bash
bash scripts/run_container_autoware_planner.sh <number of GPU>
```

- Finally, make sure the permission of the `shared` directory.
```bash
# At the project root (host machine, not the container)
sudo chown -R $(id -u):$(id -g) shared
```
