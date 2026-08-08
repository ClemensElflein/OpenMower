FROM debian:bookworm-slim

# Buildroot mandatory + convenience host dependencies.
# See https://buildroot.org/downloads/manual/manual.html#requirement
RUN apt-get update && apt-get install -y --no-install-recommends \
        bc \
        binutils \
        build-essential \
        bzip2 \
        ca-certificates \
        cpio \
        file \
        findutils \
        git \
        libncurses-dev \
        libssl-dev \
        locales \
        openssl \
        patch \
        perl \
        python3 \
        rsync \
        unzip \
        wget \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/^# *\(en_US.UTF-8\)/\1/' /etc/locale.gen && locale-gen
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Buildroot refuses to run as root; build.sh passes the invoking UID/GID.
# HOME must be writable for that arbitrary UID.
ENV HOME=/tmp
WORKDIR /work
