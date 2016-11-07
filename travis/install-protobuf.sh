#!/bin/sh
set -e
if [ ! -d "$HOME/protobuf/lib" ] || ! python -c "import google.protobuf.text_format"; then
    mkdir -p $HOME/protobuf;
    cd $HOME/protobuf;
    wget https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz;
    tar -xf protobuf-2.6.1.tar.gz;
    cd protobuf-2.6.1 && ./configure --prefix=$HOME/protobuf && make && make install;
    cd python && python setup.py build --cpp_implementation && python setup.py install --cpp_implementation --user;
    cd $HOME;
else
    echo 'Using cached protobuf directory.';
fi
