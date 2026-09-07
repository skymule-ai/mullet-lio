ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.4.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

LABEL org.opencontainers.image.title="mullet ROS LIO development image"
LABEL org.opencontainers.image.description="Empty runtime container for developing a ROS 2 LIO implementation"

# LIO dependencies and source installation intentionally remain out of this
# image until the package has been built and validated interactively in /ws.

COPY images/lio/entrypoint.sh /usr/local/bin/mullet-lio-entrypoint
COPY images/lio/profile.sh /etc/profile.d/10-mullet-lio.sh

RUN chmod 0755 /usr/local/bin/mullet-lio-entrypoint && \
    chmod 0644 /etc/profile.d/10-mullet-lio.sh

ENTRYPOINT ["/usr/local/bin/mullet-lio-entrypoint"]
CMD ["bash", "-lc", "exec sleep infinity"]
