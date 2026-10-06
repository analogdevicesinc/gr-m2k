#!/bin/bash

set -e

version=$1
source_code=$(basename "$PWD")

# Use sudo only if not running as root
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

###############################################################################
# Install general build for gr-m2k
###############################################################################
echo "Installing gr-m2k dependencies"

$SUDO apt-get update
$SUDO apt-get install -y \
    build-essential cmake devscripts debhelper \
    gnuradio-dev python3-dev pybind11-dev dh-python \
    git libiio-dev libgoogle-glog-dev libserialport-dev \
    swig python3-setuptools mono-mcs cli-common-dev

###############################################################################
# Install gr-m2k build dependencies and update list of packages
###############################################################################
echo "Installing libm2k"

if [[ "$architecture" == "amd64" ]]; then
    cd /tmp
    git clone https://github.com/analogdevicesinc/libm2k.git
    cd libm2k
    libm2k_version=${VERSION}
    echo "Building libm2k version: $libm2k_version"
    .github/scripts/create_debian.sh "$libm2k_version"
    $SUDO dpkg -i ../libm2k_*.deb ../libm2k-dev_*.deb
else
    echo "==> Adding ADI package repository..."
    curl -1sLf 'https://packages.analog.com/public/setup.deb.sh' | ${SUDO:+sudo -E} bash
    curl -1sLf 'https://packages.analog.com/kuiper/setup.deb.sh' | ${SUDO:+sudo -E} bash
    $SUDO apt-get update -qq
    $SUDO apt-get install -y libm2k-dev
fi
