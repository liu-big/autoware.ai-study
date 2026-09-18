/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "fix2tfpose.h"

static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;

static int _plane, _useUwb = 0;

double init_yaw = 0, init_x = 0,init_y = 0, init_z = 0;


Fix2tfPose::Fix2tfPose(): private_nh("~")
{

}
  
Fix2tfPose::~Fix2tfPose()
{
  
}

void Fix2tfPose::imu_callback(const sensor_msgs::Imu::Ptr& input)
{
    sensor_msgs::Imu ImuMsg = *input;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(ImuMsg.orientation, quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, imu_yaw);//进行转换
    // ROS_ERROR("---roll %f     pitch %f    yaw %f  %f",roll*180/3.1415926,pitch*180/3.1415926,yaw*180/3.1415926,imu_yaw);
}


void Fix2tfPose::uwb_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, uwb_yaw);//进行转换
}

void Fix2tfPose::GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  geo_pos_conv geo;

  geo.set_err(init_x,init_y,init_z, 0/*init_yaw*/);

  geo.set_plane(_plane);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.x();
  pose.pose.position.y = geo.y();
  pose.pose.position.z = geo.z();

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  std::cout << "distance : " << distance << std::endl;

  if(imu_yaw!=0)
    yaw = imu_yaw + init_yaw;
  else
  {
    if (distance > 0.2)
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
  }

  //是否使用UWB位置
  if(_useUwb)
  {
    yaw = uwb_yaw + init_yaw;
  }
    
  // ROS_WARN("%f  ",yaw);
  _quat = tf::createQuaternionMsgFromYaw(yaw);
  _prev_pose = pose;
  _orientation_ready = true;

  if (_orientation_ready)
  {
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    gnss_publisher.publish(gnss_stat_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
  }

  static bool save_flag=false;
  // ROS_ERROR("%d    ", get_gnss_init_pose);
  if(get_gnss_init_pose == 1 && save_flag==false){
    save_flag = true;

    private_nh.setParam("init_pose/lat", msg->latitude);
    private_nh.setParam("init_pose/lon", msg->longitude);
    private_nh.setParam("init_pose/h", msg->altitude);
    // private_nh.setParam("init_pose/yaw", yaw);
    system("rosparam dump $(rospack find startingup_ros)/data/Common/param/gnss_init_pose.yaml /fix2tfpose/init_pose");

    // ROS_WARN("m_lat: %f,m_lon: %f,h: %f,yaw_: %f", msg->latitude, msg->longitude, msg->altitude, yaw);
  }
}

void Fix2tfPose::run()
{
   // ros::NodeHandle nh;
  // ros::NodeHandle private_nh("~");

  private_nh.param("/fix2tfpose/get_gnss_init_pose", get_gnss_init_pose,int(0));

  private_nh.getParam("plane", _plane);
  private_nh.getParam("useUwb", _useUwb);


  if(get_gnss_init_pose == 0){
    private_nh.getParam("yaw", init_yaw);
    private_nh.getParam("lat", init_x);
    private_nh.getParam("lon", init_y);
    private_nh.getParam("h", init_z);
    init_yaw  *= 3.1415926 / 180.0;
    
    ROS_WARN("[init param] lat: %f, lon: %f, h: %f, yaw: %f",init_x,init_y,init_z,init_yaw);
  }

  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  gnss_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/fix", 100, &Fix2tfPose::GNSSCallback,this);
  ros::Subscriber imu_sub = nh.subscribe("imu_raw", 10, &Fix2tfPose::imu_callback,this);
  ros::Subscriber uwb_sub = nh.subscribe("uwb_pose", 10, &Fix2tfPose::uwb_callback,this);


  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose");
  Fix2tfPose fix;
  fix.run();
  return 0;
 
}
