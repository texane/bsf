#!/usr/bin/env sh
COUNT=100000 ;
DEGREE=100 ;
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
../build/bfs_gen $COUNT $DEGREE $GRAPH ;
chmod 700 $GRAPH ;
