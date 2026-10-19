#!/usr/bin/env bash
set -euo pipefail

image="${FAST_LIO_IMAGE:-ghcr.io/skymule-ai/mullet-lio-fast-lio:local}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_dir="${repo_root}/config/fast-lio"
container_name="mullet-fast-lio-smoke-${RANDOM}-$$"

cleanup() {
  docker rm --force "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker image inspect "${image}" >/dev/null

docker run --rm "${image}" ros2 pkg prefix fast_lio >/dev/null
docker run --rm "${image}" bash -c '
  ros2 pkg executables fast_lio | grep -q "fast_lio fastlio_mapping"
  ros2 interface show livox_ros_driver2/msg/CustomMsg | grep -q offset_time
'

docker run --rm \
  --volume "${config_dir}:/config:ro" \
  "${image}" \
  test -r /config/mid360.yaml

# Own ROS domain: the check publishes /livox/* and must not meet a real driver on the network.
docker run --detach \
  --name "${container_name}" \
  --network host \
  --env ROS_DOMAIN_ID=77 --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --volume "${config_dir}:/config:ro" \
  "${image}" >/dev/null

sleep 5

if [[ "$(docker inspect --format '{{.State.Running}}' "${container_name}")" != "true" ]]; then
  docker logs "${container_name}" >&2
  echo "FAST-LIO exited instead of waiting for sensor topics." >&2
  exit 1
fi

docker run --rm \
  --network host \
  --env ROS_DOMAIN_ID=77 --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --volume "${repo_root}/scripts:/scripts:ro" \
  "${image}" \
  python3 /scripts/synthetic-odometry.py \
  || { docker logs "${container_name}" >&2; exit 1; }

echo "FAST-LIO image smoke test passed."
