# mullet-lio

Development and container scaffolding for ROS 2 LiDAR-inertial odometry on the
Mullet Jetson platform.

No LIO implementation or implementation-specific dependency is installed in
the image yet. The first step is to build and validate a selected package in a
running base-derived container. Once that works, move the reproducible install
and build commands into `images/lio/Dockerfile` and pin the upstream source
revision.

## Architecture

The Livox driver and LIO implementation run as separate containers:

```text
MID-360S -> Livox container -> ROS 2 point-cloud/IMU topics -> LIO container
```

The LIO image inherits `mullet-ros2-base`, not the Livox image. This assumes the
selected implementation consumes standard ROS messages such as
`sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Imu`. If it requires
`livox_ros_driver2/msg/CustomMsg`, its build will also need that interface
dependency.

## Layout

```text
images/lio/          Empty base-derived development image
config/lio/          LIO runtime configuration mount point
ws/src/              Host workspace for the LIO source repository
compose.yaml         Livox and LIO development services
scripts/validate.sh  Structural validation
```

## Start the development container

Create the local environment file:

```bash
cp .env.example .env
```

`LIVOX_CONFIG_FILE` defaults to the configuration in the sibling
`mullet-drivers` checkout. Set it in `.env` to a different absolute or
relative host path when using a robot-specific configuration. The file is
mounted inside the Livox container as `/config/MID360s_config.json`; it is not
copied into this repository.

Build the empty development image and start the stack:

```bash
docker compose build lio
docker compose up -d
docker compose ps
```

The default `lio` profile starts both the Livox and LIO services. The Livox
image defaults to the locally built `ghcr.io/skymule-ai/mullet-drivers-livox:local`
image and can be overridden through `LIVOX_IMAGE`.

## Build a LIO package interactively

Clone the selected LIO repository into the mounted workspace on the host:

```bash
git clone <lio-repository> ws/src/<package-directory>
```

Enter the container and build it:

```bash
docker compose exec lio bash -l
cd /ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

The workspace is bind-mounted, so source and build results remain available
after the container is recreated. The container currently runs as root; files
created during the build may therefore be owned by root on the host.

## Check sensor input

From the login shell in the LIO container:

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

After the selected package builds and produces odometry from these topics,
transfer its apt dependencies, pinned source checkout, colcon build, entrypoint
overlay, configuration, and launch command into the Dockerfile.

## Validate the scaffold

```bash
./scripts/validate.sh
```
