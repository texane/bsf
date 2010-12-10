#!/usr/bin/env sh
XKAAPIDIR=$HOME/install/xkaapi_release
g++ -Wall -O3 -march=native -I$XKAAPIDIR/include ../src/main.cc -L$XKAAPIDIR/lib -lkaapi
