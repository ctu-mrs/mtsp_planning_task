#!/usr/bin/env bash

# Download and compile LKH Solver

#VERSION=2.0.7
VERSION=2.0.9
ARCHIVE=LKH-$VERSION.tgz
DIR=LKH-$VERSION
THIS_SCRIPT_DIR=`dirname "$0"`
CURRENT_DIR=`pwd`

cd $THIS_SCRIPT_DIR
if [ ! -f $ARCHIVE ] ; then
    wget http://www.akira.ruc.dk/~keld/research/LKH/$ARCHIVE
fi

if [ ! -f $DIR ] ; then
    tar xvfz $ARCHIVE
fi

cd $DIR
make
cd $CURRENT_DIR
