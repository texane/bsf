#!/usr/bin/env sh

XKAAPIDIR=$HOME/install/xkaapi_release
PROC=`getconf _NPROCESSORS_ONLN`
COUNT=500000 ;
DEGREE=5 ;
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
FROM=0
TO=1

../build/bfs_seq $GRAPH $FROM $TO ;

for i in `seq 0 $((PROC - 1))`; do
    LD_LIBRARY_PATH=$XKAAPIDIR/lib KAAPI_CPUSET=0:$i ../build/bfs_par $GRAPH $FROM $TO ;
done
