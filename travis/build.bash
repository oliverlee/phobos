#!/usr/bin/env bash

# Stop this script when a command fails.
set -e

# Print every line after resolving variables.
PS4="[${BASH_SOURCE}] $ "
set -x

# Make sure build dir exists.
mkdir --parents build

pushd build
# Configure build.
cmake $BUILD_TYPE -DTOOLCHAIN_DIR=$TRAVIS_BUILD_DIR/toolchain/gcc-arm-none-eabi-5_4-2016q2 -DPROTOBUF_PROTOC_EXECUTABLE=$HOME/protobuf/bin/protoc -DCMAKE_TOOLCHAIN_FILE=../toolchain-gcc-arm-none-eabi-5_4-2016q2.cmake ..

# Build project.
cmake --build .

if [ -d "tests" ]; then
    pushd tests
    ctest --verbose --output-on-failure
    popd #tests
fi
popd #build
