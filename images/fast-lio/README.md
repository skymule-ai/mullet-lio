# FAST-LIO2 image

The Dockerfile builds the pinned upstream FAST-LIO2 (`ROS2` branch, with its ikd-Tree submodule) in
`/opt/fast_lio_ws`. It applies `patches/` and runs `fastlio_mapping` with `/config/mid360.yaml`,
remapping its absolute topics under `/uav0/fast_lio/`.

Preserve these integration contracts:

- Inherit an immutable `mullet-base` release through `BASE_IMAGE`.
- Copy `livox_ros_driver2` from `LIVOX_IMAGE` (mullet-drivers, same base release); never rebuild it.
- Pin every upstream source revision. Keep patches small and in `patches/`.
- Publish standard `nav_msgs/Odometry` (REP 103, twist in the child frame) for mullet-localization.
- Keep calibration and drone-specific values out of the image; mount them read-only at `/config`.
- Run `scripts/smoke-fast-lio.sh` before publishing an image.
