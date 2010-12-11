#!/usr/bin/env sh
XKAAPIDIR=$HOME/install/xkaapi_release
PROC_MAX=`getconf _NPROCESSORS_ONLN`
../build/bfs_seq
#LD_LIBRARY_PATH=$XKAAPIDIR/lib KAAPI_CPUSET=0:$((MAX_PROC-1)) ../build/bfs_par
LD_LIBRARY_PATH=$XKAAPIDIR/lib KAAPI_CPUSET=0:2 ../build/bfs_par
