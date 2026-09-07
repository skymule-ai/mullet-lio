#!/usr/bin/env bash
set -euo pipefail

docker compose \
  --env-file .env.example \
  --profile lio \
  config \
  --quiet

bash -n \
  images/lio/entrypoint.sh \
  images/lio/profile.sh
