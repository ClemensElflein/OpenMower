#!/bin/sh
# Runs inside the same fakeroot session as image generation, AFTER
# buildroot's own blanket `chown -R 0:0` over the whole target dir (see
# buildroot/fs/common.mk). The vendored ROS tree under
# /opt/openmower-root/home/openmower is built by a different pipeline
# (docker buildx, see external/package/openmower-ros) that runs as the
# invoking host user, not fakeroot, so its real ownership (openmower:openmower,
# uid/gid 1000 from the Dockerfile's useradd) never survives the package
# install step on its own -- restore it explicitly here so the openmower
# service user can write its own $HOME at runtime.
set -eu
chown -R 1000:1000 "$1/opt/openmower-root/home/openmower"
