1 编译为动态库
    g++ bridge.cpp -fPIC -shared -o libbridge.so

2 动态库加入环境
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH::`pwd` (调试使用)
    或 cp libbridge.so /usr/local/lib/

3 编译demo
    g++ demo.cpp -L. -lbridge -o demo
