#!/bin/bash

set -e

version=$1
source_code=$(basename "$PWD")

###############################################################################
# Build gr-m2k .deb packages
###############################################################################

# Replace placeholders inside the debian template files
sed -i "s/@VERSION@/$version-1/" packaging/debian/changelog
sed -i "s/@DATE@/$(date -R)/" packaging/debian/changelog

cp -r packaging/debian .

rm -rf packaging

pushd ..
tar czf ${source_code}_${version}.orig.tar.gz \
    --exclude='.git' \
    --exclude='debian' \
    $source_code
popd

debuild
