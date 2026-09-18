#!/bin/bash

g++ bridge.cpp -fPIC -shared -o libbridge.so
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH::`pwd`
g++ demo.cpp -L. -lbridge -o demo
./demo
