#!/bin/sh
set -e
if [ ! -d "$HOME/protobuf/lib" ]; then
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

if ! python -c "import google.protobuf.text_format"; then
    cd $HOME/protobuf/protobuf-2.6.1/python;
    if [ -f "dist/protobuf-2.6.1-py2.7-linux-x86_64.egg" ]; then
        python -m easy_install --user dist/protobuf-2.6.1-py2.7-linux-x86_64.egg
    else
        python setup.py install --cpp_implementation --user;
    fi
    cd $HOME;
else
    echo 'Using cached protobuf-python install.';
fi
