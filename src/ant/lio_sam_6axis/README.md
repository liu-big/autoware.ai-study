# LIO_SAM_6AXIS  使用说明

# 安装gtsam4.0.2
    wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
    cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
    cd ~/Downloads/gtsam-4.0.2/
    mkdir build && cd build
    编译前在CMakeList.txt中299行后if(GTSAM_USE_SYSTEM_EIGEN)前加上set(GTSAM_USE_SYSTEM_EIGEN on)
    //在上一步新建的biuld文件夹内打开终端进行编译
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
  make -j1
  sudo make install

#  conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’解决

  终端输入：
sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

