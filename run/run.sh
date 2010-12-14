#!/usr/bin/env sh

XKAAPIDIR=$HOME/install/xkaapi_release

PROC=`getconf _NPROCESSORS_ONLN`
COUNT=100000
DEGREE=40
#COUNT=150000
#DEGREE=50
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
FROM=0
TO=50001

# KAAPI_CPUSET=0:7 \
# LD_LIBRARY_PATH=$XKAAPIDIR/lib \
# numactl --interleave=all \
# ../build/bfs_par $GRAPH $FROM $TO ;

# KAAPI_CPUSET=0:15 \
# LD_LIBRARY_PATH=$XKAAPIDIR/lib \
# numactl --interleave=all \
# ../build/bfs_par $GRAPH $FROM $TO ;

# KAAPI_CPUSET=0:23 \
# LD_LIBRARY_PATH=$XKAAPIDIR/lib \
# numactl --interleave=all \
# ../build/bfs_par $GRAPH $FROM $TO ;

# KAAPI_CPUSET=0:31 \
# LD_LIBRARY_PATH=$XKAAPIDIR/lib \
# numactl --interleave=all \
# ../build/bfs_par $GRAPH $FROM $TO ;

taskset -c 0 ../build/bfs_seq $GRAPH $FROM $TO ;

for i in `seq 0 $((PROC - 1))`; do
    LD_LIBRARY_PATH=$XKAAPIDIR/lib \
    KAAPI_CPUSET=0:$i \
    ../build/bfs_par $GRAPH $FROM $TO ;
done
