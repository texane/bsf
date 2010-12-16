#!/usr/bin/env sh

XKAAPIDIR=$HOME/install/xkaapi_release

PROC_MAX=`getconf _NPROCESSORS_ONLN`
COUNT=10000
DEGREE=50
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
FROM=0
TO=5001

taskset -c 0 ../build/bfs_seq $GRAPH $FROM $TO ;

for i in `seq 0 $((PROC_MAX - 1))`; do
    LD_LIBRARY_PATH=$XKAAPIDIR/lib \
    KAAPI_CPUSET=0:$i \
    ../build/bfs_par $GRAPH $FROM $TO ;
done
