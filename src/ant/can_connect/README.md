OLD_PACKAGE_NAME="can_connect"
NEW_PACKAGE_NAME="vehicle_can"
OLD_FILE_NAME="can_connect"
NEW_FILE_NAME="vehicle_can"
OLD_CLASS_NAME="CanConnect"
NEW_CLASS_NAME="CanSocket"

# 出错多次执行
find . -name "*$OLD_PACKAGE_NAME*" | while read id; do mv $id ${id/$OLD_PACKAGE_NAME/$NEW_PACKAGE_NAME}; done
sed -i "s/$OLD_PACKAGE_NAME/$NEW_PACKAGE_NAME/g" `grep $OLD_PACKAGE_NAME -rl $NEW_PACKAGE_NAME`

find $NEW_PACKAGE_NAME -name "*$OLD_FILE_NAME*" | while read id; do mv $id ${id/$OLD_FILE_NAME/$NEW_FILE_NAME}; done
sed -i "s/$OLD_FILE_NAME/$NEW_FILE_NAME/g" `grep $OLD_FILE_NAME -rl $NEW_PACKAGE_NAME`

sed -i "s/$OLD_CLASS_NAME/$NEW_CLASS_NAME/g" `grep $OLD_CLASS_NAME -rl $NEW_PACKAGE_NAME`

# 存放不同的can盒,可以为socket和物理can
src/can_box/
    can_socket.cpp      #socket can盒
# can设备 , 底盘 超声波 毫米波
src/can_client/
    vehicle_can.cpp       # 底盘