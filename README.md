# mullet-lio

Container and deployment configuration for ROS 2 LiDAR-inertial odometry on the Mullet Jetson
platform. The current image builds FAST-LIO2 for ROS 2 Jazzy on the Livox MID-360 / MID-360S;
other implementations (FAST-LIVO2) get their own image.

## Architecture

At build time, each LIO image inherits the shared ROS base and copies the Livox messages from the
driver image:

```text
ghcr.io/skymule-ai/mullet-base:v0.5.1
└── ghcr.io/skymule-ai/mullet-lio-fast-lio   (+ livox_ros_driver2 copied from mullet-drivers-livox)
```

At runtime, Compose runs the Livox driver, its PTP master and the LIO as separate containers:

```text
MID-360 -> livox (CustomMsg /livox/lidar, /livox/imu) -> fast-lio -> /uav0/fast_lio/odometry
        <- livox-ptp (PTP master on the Orin's LiDAR interface)
```

FAST-LIO reads `livox_ros_driver2/msg/CustomMsg`, not PointCloud2: only CustomMsg carries the
per-point times it needs to deskew a scan. The message package is copied from the driver image
(`LIVOX_IMAGE`), not rebuilt, so its type hash matches the driver's.

## Layout

```text
images/fast-lio/          FAST-LIO2 source build, patches, runtime entrypoint
config/fast-lio/          Example runtime configuration (mid360.yaml)
config/livox/             Example MID-360 driver configuration
docs/fast-lio.md          How FAST-LIO2 works, our settings, failure modes
compose.yaml              Livox + PTP + FAST-LIO services
scripts/validate.sh       Structural checks that do not build images
scripts/smoke-fast-lio.sh Sensor-free smoke test, incl. scripts/synthetic-odometry.py
.github/workflows/        ARM64 image build and publishing
```

[docs/fast-lio.md](docs/fast-lio.md) explains the estimator, what it publishes and the settings
we changed. Read it before tuning.

## Build FAST-LIO2

```bash
cp .env.example .env
./scripts/validate.sh
docker compose --profile fast-lio build fast-lio
./scripts/smoke-fast-lio.sh
```

The image builds upstream `hku-mars/FAST_LIO` (`ROS2` branch), pinned by build arguments:

```dotenv
FAST_LIO_REPOSITORY=https://github.com/hku-mars/FAST_LIO.git
FAST_LIO_REF=a4743b095409588842a5b30ddfa27e29d2f99164
LIVOX_IMAGE=ghcr.io/skymule-ai/mullet-drivers-livox:sha-8d0563c
```

Three changes are applied to upstream at build time:

- C++14 → C++17, which Jazzy's `rclcpp` requires.
- [`images/fast-lio/patches/gravity-align.patch`](images/fast-lio/patches/gravity-align.patch). The
  world frame starts gravity-aligned (z up, heading = the IMU's at start-up) instead of being the
  IMU frame at start-up, so a tilted mount does not tilt the world.
- [`images/fast-lio/patches/odometry.patch`](images/fast-lio/patches/odometry.patch). It makes
  `/uav0/fast_lio/odometry` standard odometry, which is what mullet-localization's bridge expects:
  the covariance is filled before publishing and in ROS order, and the twist is set in the body
  frame. Upstream leaves the twist at zero.

The smoke test checks package discovery, the executable, the Livox message type, the mounted
configuration, and that FAST-LIO stays alive waiting for sensor topics. It then replays a synthetic
MID-360 in a box room (`scripts/synthetic-odometry.py`, ~16 s in real time), level and then mounted
at 5° roll / 20° pitch. It checks that there is no height drift, the body's "up", the body-frame twist
and the covariances. That validates the patches, not the odometry quality on real
data.

## Run

```bash
docker compose --profile fast-lio up
```

This starts `livox`, `livox-ptp` and `fast-lio`. Set `LIVOX_IFACE` in `.env` to the interface
wired to the LiDAR. All services use host networking, the same ROS domain and the same RMW. Keep
the drone still for the first seconds: FAST-LIO initialises gravity and the IMU biases from the
first IMU samples.

| Topic | Type | What |
| --- | --- | --- |
| `/uav0/fast_lio/odometry` | `nav_msgs/Odometry` | IMU pose in `camera_init` (z up, heading of the IMU at start), 10 Hz |
| `/uav0/fast_lio/path` | `nav_msgs/Path` | trajectory, for viewing |
| `/uav0/fast_lio/cloud_registered_body` | `sensor_msgs/PointCloud2` | deskewed scan, IMU frame |
| `/uav0/fast_lio/cloud_registered` | `sensor_msgs/PointCloud2` | deskewed scan, world frame |

## Dependency ownership

Each LIO implementation declares its dependencies in its own image. Livox software comes from
[mullet-drivers](https://github.com/skymule-ai/mullet-drivers) only. Do not add LIO-specific
dependencies to `mullet-base`.

## Releases

CI builds for ARM64 and smoke-tests the image before publishing it. Branch pushes publish branch
and SHA tags; release tags such as `v0.1.0` publish matching image tags. `BASE_IMAGE` and
`LIVOX_IMAGE` are read from `.env.example`. **Actions → Build and Push LIO Images → Run workflow**
builds another FAST-LIO repository or revision.

Do not create a release tag until a hardware test on the Orin shows `/livox/lidar` and
`/livox/imu` stamped in ROS time and `/uav0/fast_lio/odometry` publishing.
