#!/usr/bin/env sh
COUNT=1000000 ;
DEGREE=3 ;
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
../build/bfs_gen $COUNT $DEGREE $GRAPH ;
chmod 700 $GRAPH ;
