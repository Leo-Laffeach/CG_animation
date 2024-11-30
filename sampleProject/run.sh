#!/usr/bin/env bash

SCRIPTS_REALPATH=$(realpath -s $0)
SCRIPT_PATH=$(dirname $SCRIPTS_REALPATH)

do_cmake()
{
    mkdir -p $SCRIPT_PATH/../sfmlGraphicsPipeline/build
    cd $SCRIPT_PATH/../sfmlGraphicsPipeline/build
    cmake ../

    mkdir -p $SCRIPT_PATH/build
    cd $SCRIPT_PATH/build
    cmake ../

    cd $SCRIPT_PATH
}

do_make()
{
    mkdir -p $SCRIPT_PATH/../sfmlGraphicsPipeline/build
    cd $SCRIPT_PATH/../sfmlGraphicsPipeline/build
    make -j6

    mkdir -p $SCRIPT_PATH/build
    cd $SCRIPT_PATH/build
    make $1 -j6

    cd $SCRIPT_PATH
}

do_run()
{
    cd $SCRIPT_PATH/build
    ./"$1"
    cd $SCRIPT_PATH
}

if [ $2 ];then
    if [ $2 == "ccr" ]; then
        do_cmake
        do_make $1
    elif [ $2 == "cr" ]; then
        do_make $1
    fi
fi
do_run $1