#!/usr/bin/env sh

XKAAPIDIR=$HOME/install/xkaapi_release

PROC_MAX=`getconf _NPROCESSORS_ONLN`

# FROM=0
# TO=501
# COUNT=10000
# DEGREE=50

FROM=10
TO=21
COUNT=1000000
DEGREE=3

GRAPH=../dat/$COUNT\_$DEGREE.dat ;

taskset -c 0 ../build/bfs_seq $GRAPH $FROM $TO ;
echo ;

for i in `seq 0 $((PROC_MAX - 1))`; do
    echo -n $i ' ' ;
    LD_LIBRARY_PATH=$XKAAPIDIR/lib \
    KAAPI_CPUSET=0:$i \
    ../build/bfs_par $GRAPH $FROM $TO ;
    echo ;
done
