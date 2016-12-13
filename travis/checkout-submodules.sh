#!/bin/sh
set -e

# FIXME: check gitsha1 of submodules instead of requiring Travis cache to be
# cleared when any submodule gets updated.

if [ ! -d "$HOME/submodules/bicycle" ]; then # only check for existence of one submodule
    mkdir -p $HOME/submodules;
    cd $TRAVIS_BUILD_DIR; # repository path
    git submodule update --init --recursive --recommend-shallow;
    cp -r $TRAVIS_BUILD_DIR/external/* $HOME/submodules;
    cd $HOME;
else
    echo 'Copying cached git submodules.';
    cp -r $HOME/submodules/* $TRAVIS_BUILD_DIR/external
fi
