#!/usr/bin/env sh
COUNT=500000 ;
DEGREE=2 ;
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
../build/bfs_gen $COUNT $DEGREE $GRAPH ;
chmod 700 $GRAPH ;
