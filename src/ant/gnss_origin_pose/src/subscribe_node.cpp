#include "subscribe_node.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>

int sub_flag=false;
double x_,y_,z_;
double roll, pitch, yaw;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  static int cout = 0;
  if(cout++ == 100){
    x_ = msg->pose.position.x;
    y_ = msg->pose.position.y;
    z_ = msg->pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    sub_flag = true;
  }
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "gnss_origin_pose");

  ros::NodeHandle nh;
  ros::NodeHandle np("~");
  ros::Subscriber chatter_sub = nh.subscribe("/gnss_pose", 1,chatterCallback);
  
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
    if(sub_flag == true)
    {
      np.setParam("init_pose_x",x_);
      np.setParam("init_pose_y",y_);
      np.setParam("init_pose_z",z_);
      np.setParam("init_pose_yaw",yaw*180/3.14159);
      ROS_ERROR("[pose] x: %f, y: %f, z: %f, yaw: %f\n",x_,y_,z_,yaw*180/3.14159);
      // system("cp $(rospack find startingup_ros)/data/$/param/gnss_init_pose.yaml $(rospack find startingup_ros)/data/Common/param/gnss_init_pose_old.txt");
      system("rosparam dump $(rospack find startingup_ros)/data/Common/param/gnss_init_pose.yaml /subscribe_node");
      ROS_INFO("gnss_origin_pose save OK");
      return 0;
    }
    //loop_rate.sleep();
    ros::spinOnce();

  }
  ros::spin();

  return 0;
}
