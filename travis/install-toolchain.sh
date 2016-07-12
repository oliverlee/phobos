#!/bin/sh
set -e
if [ ! -d "$HOME/toolchain/gcc-arm-none-eabi-5_4-2016q2" ]; then
    mkdir -p $HOME/toolchain;
    cd $HOME/toolchain;
    wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2;

    tar -xf gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2;
    cd $HOME;
else
    echo 'Using cached toolchain directory.';
fi
