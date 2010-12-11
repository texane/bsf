#!/usr/bin/env sh
XKAAPIDIR=$HOME/install/xkaapi_release
LD_LIBRARY_PATH=$XKAAPIDIR/lib KAAPI_CPUSET=0,1 ../build/bfs_par
../build/bfs_seq
