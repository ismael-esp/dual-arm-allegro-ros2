#!/usr/bin/env bash
# Build and load the PEAK pcan kernel module on the HOST (needed once per kernel update).
# The Allegro hands talk over PCAN-USB in chardev mode (/dev/pcanusb32, /dev/pcanusb33).
set -euo pipefail
VER="${PEAK_VER:-8.20.0}"
SHA="efecadba9294edc38bbe189387708d0c34b25d7621fbfd4208d8b12811fc2580"
TMP="$(mktemp -d)"; cd "$TMP"
wget -q "https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-${VER}.tar.gz"
[ "$VER" != 8.20.0 ] || echo "$SHA  peak-linux-driver-${VER}.tar.gz" | sha256sum -c -
tar xzf "peak-linux-driver-${VER}.tar.gz" && cd "peak-linux-driver-${VER}"
sudo apt-get install -y build-essential "linux-headers-$(uname -r)" libpopt-dev
make clean && make NET=NO_NETDEV_SUPPORT     # chardev interface, as used by the Allegro driver
sudo make install
sudo modprobe pcan
ls -l /dev/pcan* && echo "pcan module loaded"
