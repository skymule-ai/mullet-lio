#!/usr/bin/env bash
set -euo pipefail

docker compose \
  --env-file .env.example \
  --profile fast-lio \
  config \
  --quiet

bash -n \
  images/fast-lio/entrypoint.sh \
  images/fast-lio/profile.sh \
  scripts/smoke-fast-lio.sh

test -f config/fast-lio/mid360.yaml
test -f config/livox/MID360s_config.json
