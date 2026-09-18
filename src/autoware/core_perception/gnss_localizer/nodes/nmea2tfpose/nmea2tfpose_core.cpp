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

#include "nmea2tfpose_core.h"
#include <math.h> 

namespace gnss_localizer
{
// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(-std::numeric_limits<double>::infinity())
  , position_time_(-std::numeric_limits<double>::infinity())
  , current_time_(0)
  , orientation_stamp_(0)
  , orientation_ready_(false)
  , init_yaw(0)
  , init_x(0)
  , init_y(0)
  , init_z(0)
{
  initForROS();
  geo_.set_plane(plane_number_);
  m_dGpsHight = 0;
  gps_hight_flag = false;
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

void Nmea2TFPoseNode::initForROS()
{

  nh_.param("/nmea2tfpose/get_gnss_init_pose", get_gnss_init_pose, false);
  if(get_gnss_init_pose == false){

    private_nh_.getParam("yaw", init_yaw);
    private_nh_.getParam("lat", init_x);
    private_nh_.getParam("lon", init_y);
    private_nh_.getParam("h", init_z);
    init_yaw  *= M_PI / 180.;
    geo_.set_err(init_x,init_y,init_z,init_yaw);
    ROS_WARN("[init param] lat: %f, lon: %f, h: %f, yaw: %f",init_x,init_y,init_z,init_yaw * 180./3.14159);

  }
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 10, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);

  sub2_ = nh_.subscribe("map_curbs_rviz", 1, &Nmea2TFPoseNode::callbackMapCurbsRviz, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.x();
  pose.pose.position.y = geo_.y();

  pose.pose.position.z = geo_.z();
  if(gps_hight_flag)
      pose.pose.position.z = m_dGpsHight;
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  pub1_.publish(pose);


  // tf::Quaternion quat;
  // double roll, pitch, yaw;
  // tf::quaternionMsgToTF(pose.pose.position.orientation, quat);
  // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  // ROS_ERROR("x: %f , y: %f, z: %f,  yaw: %f",geo_.x(),geo_.y(),geo_.z(),yaw_*180/3.14159);
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  if(gps_hight_flag){
      transform.setOrigin(tf::Vector3(geo_.x(), geo_.y(), m_dGpsHight));
  }else
  {
      transform.setOrigin(tf::Vector3(geo_.x(), geo_.y(), geo_.z()));
  }
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  //yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  // yaw_ = init_yaw;
  // roll_ = 0;
  // pitch_ = 0;
}

void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)   // 设置 geo_
{
  try
  {
    static bool sim_flag = false;
    if (nmea.at(0) == "$QQ02C")
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      yaw_ -= init_yaw;

      orientation_stamp_ = current_stamp;
      orientation_ready_ = true;
      ROS_INFO("QQ is subscribed.");
    }else if (nmea.at(0) == "QQ02C" )
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(6)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(4)) * M_PI / 180. + M_PI / 2;
      yaw_ -= init_yaw;

      orientation_stamp_ = current_stamp;
      orientation_ready_ = true;
      ROS_INFO("QQ is subscribed.");

      sim_flag = true;
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      orientation_ready_ = true;
      ROS_INFO("PASHR is subscribed.");
    }
    else if (nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));

      if (nmea.at(3) == "S")
        lat = -lat;

      if (nmea.at(5) == "W")
        lon = -lon;

      static bool save_flag=false;
      if(get_gnss_init_pose == true && save_flag == false && orientation_ready_ == true){
        save_flag = true;

        // ddmm转为dd
        double latm_, latd_, lond_, lonm_, m_lat,m_lon;
        latd_ = floor(lat / 100.);         //得到度
        lond_ = floor(lon / 100.);

        latm_ = lat - latd_ * 100.; //得到分
        lonm_ = lon - lond_ * 100.; //得到分

        m_lat = (latd_ + latm_ / 60.0);
        m_lon = (lond_ + lonm_ / 60.0);

        private_nh_.setParam("init_pose/lat", m_lat);
        private_nh_.setParam("init_pose/lon", m_lon);
        private_nh_.setParam("init_pose/h", h);
        private_nh_.setParam("init_pose/yaw", yaw_ * 180 / M_PI);
        system("rosparam dump $(rospack find startingup_ros)/data/Common/param/gnss_init_pose.yaml /nmea2tfpose/init_pose");

        ROS_WARN("m_lat: %f,m_lon: %f,h: %f,yaw_: %f", m_lat, m_lon, h, yaw_ * 180 / M_PI);
      }

      geo_.set_llh_nmea_degrees(lat, lon, h);
      ROS_INFO("x: %lf, y: %lf, z: %lf, yaw: %lf", geo_.x(), geo_.y(), geo_.z(), yaw_ * 180./M_PI);
      if(sim_flag != true)
      {
          geo_.gnss_to_map();
      }

      ROS_INFO("GGA is subscribed.");
    }
    else if (nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;

      if (nmea.at(4) == "S")
        lat = -lat;

      if (nmea.at(6) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GPRMC is subscribed.");
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}
void Nmea2TFPoseNode::callbackMapCurbsRviz(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    gps_hight_flag = true;
    double d = 0;
    double min_dis = 100;
    for(int i=0;i < msg->markers.size();i++ ){
        if (msg->markers[i].type == 7){
            for (int j = 0; j < msg->markers[i].points.size(); j++){
                d = hypot(msg->markers[i].points[j].y - geo_.y(), msg->markers[i].points[j].x - geo_.x());
                if (d < min_dis)
                {
                    min_dis = d;
                    m_dGpsHight = msg->markers[i].points[j].z + 0.22 ;
                }
            }
        }
    }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;
  // if orientation_stamp_ is 0 then no "QQ" sentence was ever received,
  // so orientation should be computed from offsets
  if (true)// orientation_stamp_.isZero() || fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
    // if (dt > threshold)
    // {
      /* If orientation data is not available it is generated based on translation
         from the previous position. For the first message the previous position is
         simply the origin, which gives a wildly incorrect orientation. Some nodes
         (e.g. ndt_matching) rely on that first message to initialise their pose guess,
         and cannot recover from such incorrect orientation.
         Therefore the first message is not published, ensuring that orientation is
         only calculated from sensible positions.
      */
      if (orientation_ready_)
      {
        ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
        createOrientation();
        publishPoseStamped();
        publishTF();
      }
      else
      {
        orientation_ready_ = true;
      }
      last_geo_ = geo_;
    //}
    return;
  }

  double e = 1e-2;
  if ((fabs(orientation_time_ - position_time_) < e) && orientation_ready_)
  {
    publishPoseStamped();
    publishTF();
    return;
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // namespace gnss_localizer
