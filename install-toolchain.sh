#!/bin/sh
set -e
if [ ! -d "$HOME/toolchain/gcc-arm-none-eabi-5_3-2016q1" ]; then
    mkdir -p $HOME/toolchain;
    cd $HOME/toolchain;
    wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q1-update/+download/gcc-arm-none-eabi-5_3-2016q1-20160330-linux.tar.bz2;
    tar -xf gcc-arm-none-eabi-5_3-2016q1-20160330-linux.tar.bz2;
    cd $HOME;
else
    echo 'Using cached directory.';
fi
