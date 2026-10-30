# Matches CI (.github/workflows): Ubuntu 24.04, ARM GCC 13, hosted GCC 14.
FROM ubuntu:24.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates git openssh-client usbutils \
        gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
        gdb-multiarch openocd \
        gcc-14 g++-14 libgtest-dev \
        python3 python3-pip pipenv \
    && rm -rf /var/lib/apt/lists/*

# Volume mounts under ~/.vscode-server would otherwise create it root-owned.
RUN install -d -o ubuntu -g ubuntu /home/ubuntu/.vscode-server \
        /home/ubuntu/.vscode-server/extensions
