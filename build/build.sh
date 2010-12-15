#!/usr/bin/env sh
XKAAPIDIR=$HOME/install/xkaapi_release
g++ -Wall -O3 -march=native -DCONFIG_NODEID=1 -o bfs_gen ../src/main.cc
g++ -Wall -O3 -march=native -DCONFIG_PARALLEL=1 -I$XKAAPIDIR/include -o bfs_par ../src/main.cc -L$XKAAPIDIR/lib -lkaapi
g++ -Wall -O3 -march=native -DCONFIG_SEQUENTIAL=1 -o bfs_seq ../src/main.cc
