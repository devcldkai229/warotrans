#!/usr/bin/env bash
# Build Raspberry Pi's libcamera (PiSP for Pi 5) + rpicam-apps on Ubuntu 24.04 (noble ships libcamera 0.2,
# which has no Pi 5 support). Build runs as the normal user; installing needs sudo and is a separate step.
#   1) sudo apt-get install -y git meson ninja-build cmake pkg-config python3-yaml python3-ply python3-jinja2 \
#        libyaml-dev libgnutls28-dev libdrm-dev libexif-dev libjpeg-dev libtiff-dev libpng-dev \
#        libboost-program-options-dev libepoxy-dev libevent-dev libssl-dev openssl
#   2) tools/build_camera_stack.sh build-libcamera      (no sudo)
#   3) sudo ninja -C ~/camera_build/libcamera/build install && sudo ldconfig
#   4) tools/build_camera_stack.sh build-rpicam          (no sudo, needs libcamera installed)
#   5) sudo ninja -C ~/camera_build/rpicam-apps/build install && sudo ldconfig
set -euo pipefail
WORK=${WORK:-$HOME/camera_build}
mkdir -p "$WORK"
cd "$WORK"

case "${1:-}" in
  build-libcamera)
    [ -d libcamera ] || git clone --depth 1 https://github.com/raspberrypi/libcamera.git
    cd libcamera
    echo "libcamera $(git rev-parse --short HEAD)"
    meson setup build --buildtype=release --prefix=/usr/local --wipe \
      -Dpipelines=rpi/pisp -Dipas=rpi/pisp -Dv4l2=enabled -Dgstreamer=disabled -Dtest=false \
      -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
    ninja -C build -j3
    echo "LIBCAMERA BUILD DONE"
    ;;
  build-rpicam)
    [ -d rpicam-apps ] || git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git
    cd rpicam-apps
    echo "rpicam-apps $(git rev-parse --short HEAD)"
    PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig:/usr/local/lib/pkgconfig \
    meson setup build --buildtype=release --prefix=/usr/local --wipe \
      -Denable_libav=disabled -Denable_drm=disabled -Denable_egl=disabled -Denable_qt=disabled \
      -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled -Denable_imx500=false
    ninja -C build -j3
    echo "RPICAM BUILD DONE"
    ;;
  *)
    echo "usage: $0 build-libcamera | build-rpicam"; exit 2 ;;
esac
