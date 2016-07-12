#!/bin/sh
set -e
if [ ! -d "$HOME/boost/boost_1_61_0" ]; then
    mkdir -p $HOME/boost;
    cd $HOME/boost;
    wget -O boost_1_61_0.tar.bz2 https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2/download;
    tar -xf boost_1_61_0.tar.bz2;
    cd $HOME;
else
    echo 'Using cached boost directory.';
fi
