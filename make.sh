#!/bin/bash 

if [ $# -eq 0 ];then
	AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
	rm -rf ./install/Project/share/
else
	for arg in "$@"
	do
		echo "正在编译: $arg"
		rm -rf install/$arg
		AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select $arg
	done

fi

