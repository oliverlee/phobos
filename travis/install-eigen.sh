#!/bin/sh
set -e
if [ ! -d "$HOME/eigen/eigen-eigen-07105f7124f9" ]; then
    mkdir -p $HOME/eigen;
    cd $HOME/eigen;
    wget https://bitbucket.org/eigen/eigen/get/3.2.8.tar.bz2;
    tar -xf 3.2.8.tar.bz2;
    cd $HOME;
else
    echo 'Using cached eigen directory.';
fi
