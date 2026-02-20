#!/bin/bash

version=$1
architecture=$(dpkg --print-architecture)

source_code=$(basename "$PWD")

# Use sudo only if not running as root
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

###############################################################################
# 1. Install gr-m2k build dependencies
###############################################################################
$SUDO apt-get update
$SUDO apt-get install -y \
    build-essential cmake devscripts debhelper \
    gnuradio-dev python3-dev pybind11-dev dh-python \
    git

###############################################################################
# 2. Build and install libm2k from source (not in upstream repos)
###############################################################################
$SUDO apt-get install -y \
    libiio-dev libgoogle-glog-dev libserialport-dev \
    swig python3-setuptools mono-mcs cli-common-dev

pushd /tmp
git clone https://github.com/analogdevicesinc/libm2k.git
cd libm2k
# TODO: Remove this checkout before merge — libm2k create_debian.sh is only on this branch for now
git checkout feature/build_debian13

# Extract libm2k version from its CMakeLists.txt
libm2k_version=$(grep -oP 'set\s*\(\s*LIBM2K_VERSION_(MAJOR|MINOR|PATCH)\s+\K[0-9]+' CMakeLists.txt | paste -sd '.')
echo "Building libm2k version: $libm2k_version"

.github/scripts/create_debian.sh "$libm2k_version"
$SUDO dpkg -i ../libm2k_*.deb ../libm2k-dev_*.deb
popd

###############################################################################
# 3. Build gr-m2k .deb packages
###############################################################################

# Replace placeholders inside the debian template files
sed -i "s/@VERSION@/$version-1/" packaging/debian/changelog
sed -i "s/@DATE@/$(date -R)/" packaging/debian/changelog
sed -i "s/@ARCHITECTURE@/$architecture/" packaging/debian/control

cp -r packaging/debian .

rm -rf packaging

pushd ..
tar czf ${source_code}_${version}.orig.tar.gz \
    --exclude='.git' \
    --exclude='debian' \
    $source_code
popd

debuild
