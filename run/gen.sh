#!/usr/bin/env sh
COUNT=250000 ;
DEGREE=4 ;
GRAPH=../dat/$COUNT\_$DEGREE.dat ;
../build/bfs_gen $COUNT $DEGREE $GRAPH ;
chmod 700 $GRAPH ;
