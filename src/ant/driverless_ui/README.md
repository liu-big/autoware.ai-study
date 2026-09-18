cd ~/Downloads/ && git clone https://github.com/jbeder/yaml-cpp.git

cd yaml-cpp
mkdir build
cd build/
##编译为动态链接库
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make
sudo make install
## 可以检查一下build文件夹下是否生成了libyaml-cpp.so.0.6.3,如果是libyaml-cpp.a则不是动态链接库
## 也可以到 /usr/local/lib是否有libyam-cpp.so