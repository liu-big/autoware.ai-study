#!/bin/bash

map_name=`ls $(rospack find startingup_ros)/data/Vehicle/`

array=(${map_name// / })  
 echo "There are the following vehicles:"
let count=0
for var in ${array[@]}
do
   echo "    $count: [ $var ]"
   let count+=1
done

while true;do
    echo 
    read -p "please input num: " num

    if [ "$num" -gt -1 -a "$num" -lt $count  ];then
        break
    fi
    echo -e "\e[31m[ERROR] The input is wrong, please re-enter...\e[0m"
done

Vehicle_Path=$(rospack find startingup_ros)/data/Vehicle/${array[$num]}
Common_Path=$(rospack find startingup_ros)/data/Common

cp -rf $Vehicle_Path/* $Common_Path/

sed -i "/AUTOCAR_VEHICLE_TYPE/c export AUTOCAR_VEHICLE_TYPE=\"${array[$num]}\"" ~/.bashrc