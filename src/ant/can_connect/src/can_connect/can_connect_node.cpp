
#include "can_connect/can_connect.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_connect");
  CanConnect node;
  node.run();
  ros::spin();
  return 0;
}
