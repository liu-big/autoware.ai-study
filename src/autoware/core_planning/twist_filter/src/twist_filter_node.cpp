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

#include "twist_filter/twist_filter_node.h"

namespace twist_filter_node
{
TwistFilterNode::TwistFilterNode() : nh_(), private_nh_("~"), health_checker_(nh_, private_nh_)
{
  gnss_is_ok = false;
  m_topic_status = false;
  ndt_flag = false;

  // Subscribe
  twist_sub_ = nh_.subscribe("twist_raw", 1, &TwistFilterNode::twistCmdCallback, this);
  ctrl_sub_ = nh_.subscribe("ctrl_raw", 1, &TwistFilterNode::ctrlCmdCallback, this);
  config_sub_ = nh_.subscribe("config/twist_filter", 10, &TwistFilterNode::configCallback, this);
  gpsfix_sub_ = nh_.subscribe("fix", 10, &TwistFilterNode::gpsfix_callback, this);
  node_status_sub_ = nh_.subscribe("node_status", 10, &TwistFilterNode::nodeStatusCallback, this);
  sub_ndtStat_sub_ = nh_.subscribe("ndt_stat", 10,&TwistFilterNode::callbackFromNdtStat, this);
  sub_Trajectory_Cost = nh_.subscribe("/local_trajectory_cost", 1, &TwistFilterNode::callbackGetLocalTrajectoryCost, this);
  spd_ratio_sub = nh_.subscribe("safe_range_ctrl", 2, &TwistFilterNode::callbackSpeedRatio, this);  // 定义队列为2,及时响应控制

  // Publish
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmd", 5);
  ctrl_pub_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 5);
  twist_lacc_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>("limitation_debug/twist/lateral_accel", 5);
  twist_ljerk_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>("limitation_debug/twist/lateral_jerk", 5);
  ctrl_lacc_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>("limitation_debug/ctrl/lateral_accel", 5);
  ctrl_ljerk_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>("limitation_debug/ctrl/lateral_jerk", 5);
  twist_lacc_result_pub_ = private_nh_.advertise<std_msgs::Float32>("result/twist/lateral_accel", 5);
  twist_ljerk_result_pub_ = private_nh_.advertise<std_msgs::Float32>("result/twist/lateral_jerk", 5);
  ctrl_lacc_result_pub_ = private_nh_.advertise<std_msgs::Float32>("result/ctrl/lateral_accel", 5);
  ctrl_ljerk_result_pub_ = private_nh_.advertise<std_msgs::Float32>("result/ctrl/lateral_jerk", 5);

  brake_mode_pub_ = nh_.advertise<std_msgs::Float32>("vehicle_brake_mode", 1);

  // Parameters
  twist_filter::Configuration twist_filter_config;
  nh_.param("vehicle_info/wheel_base", twist_filter_config.wheel_base, 2.7);
  nh_.param("twist_filter/lateral_accel_limit", twist_filter_config.lateral_accel_limit, 5.0);
  nh_.param("twist_filter/lateral_jerk_limit", twist_filter_config.lateral_jerk_limit, 5.0);
  nh_.param("twist_filter/lowpass_gain_linear_x", twist_filter_config.lowpass_gain_linear_x, 0.0);
  nh_.param("twist_filter/lowpass_gain_angular_z", twist_filter_config.lowpass_gain_angular_z, 0.0);
  nh_.param("twist_filter/lowpass_gain_steering_angle", twist_filter_config.lowpass_gain_steering_angle, 0.0);
  nh_.param("/ndt_matching/use_gnss", _use_gnss, 0);
  nh_.param("/ndt_matching/gnss_reinit_fitness", gnss_reinit_fitness, 5.0);

  nh_.param<bool>("twist_filter/use_sensor_data_filter", use_sensor_data_filter_, true);    // 使用传感器数据进行过滤， 仿真设置为false

  nh_.param<double>("/op_common_params/rollOutsNumber", m_rollOuts_number, 1);
  nh_.param<double>("/op_common_params/maxSteerAngle", maxsteerangle, 0.488);
  nh_.param<double>("/op_common_params/maxDistanceToAvoid", maxdistancetoavoid, 7);
  nh_.param<double>("/op_common_params/minDistanceToAvoid", minDistanceToAvoid, 10);
  nh_.param<double>("/op_common_params/maxVelocity", maxvelocity, 5.0);
  nh_.param<double>("/op_common_params/minVelocity", minVelocity, 0.1);
  
  twist_filter_ptr_ = std::make_shared<twist_filter::TwistFilter>(twist_filter_config);

  // Enable health checker
  health_checker_.ENABLE();
}



void TwistFilterNode::nodeStatusCallback(const autoware_system_msgs::NodeStatus::ConstPtr& msg)
{
  if(msg->node_name == "/op_motion_predictor"){
    m_topic_status = msg->node_activated;
  }
}

void TwistFilterNode::callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg)
{
  speed_ratio = msg->safe_ctrl;
  m_aSafeDir[0] = msg->safe_from;
  m_aSafeDir[1] = msg->safe_lear;
}


void TwistFilterNode::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
  b_obstacle_status = false;
  if(msg->closest_object_distance == 0)
  {
    b_obstacle_status = true;
  }
  back_distance = msg->closest_object_distance;
  is_blocked = msg->is_blocked;

  lane_id = msg->lane_index;
}


void TwistFilterNode::callbackFromNdtStat(const autoware_msgs::NDTStatConstPtr &msg)
{
    // ROS_ERROR("score:   %f    %f",msg->score,goal_score_);
    if(msg->score > gnss_reinit_fitness){
        ndt_flag = false;
        ROS_ERROR("Ndt incorrect positioning!!!!!");
    }
    else
        ndt_flag = true;
}

void TwistFilterNode::gpsfix_callback(const sensor_msgs::NavSatFix::Ptr& input)
{
  gnss_is_ok = false;
  if(input->status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX){

    gnss_is_ok = true;
  }
}

void TwistFilterNode::configCallback(const autoware_config_msgs::ConfigTwistFilterConstPtr& config_msg)
{
  twist_filter::Configuration twist_filter_config;
  twist_filter_config.lateral_accel_limit = config_msg->lateral_accel_limit;
  twist_filter_config.lateral_jerk_limit = config_msg->lateral_jerk_limit;
  twist_filter_config.lowpass_gain_linear_x = config_msg->lowpass_gain_linear_x;
  twist_filter_config.lowpass_gain_angular_z = config_msg->lowpass_gain_angular_z;
  twist_filter_config.lowpass_gain_steering_angle = config_msg->lowpass_gain_steering_angle;
  twist_filter_ptr_->setConfiguration(twist_filter_config);
}

twist_filter::Twist TwistFilterNode::mysmoothTwist(const twist_filter::Twist& twist)
{
  twist_filter::Twist twist_out;
  twist_out = twist;
  std_msgs::Float32 brake_mode;
  // 不刹车
  brake_mode.data = 0;


  // 得到剩下速度的占比
  double m_object_speed;
  double vel_ratio = 1;
  double ang_ratio=1;
  double trans_ratio=1;
  double obstacle_ratio=1;
  double diff_min=1;

  m_object_speed = twist_out.lx;

  ang_ratio = fabs(ctrl_save.sa / maxsteerangle);
  ang_ratio = (1 - pow(ang_ratio,0.5) * 0.5);
  if(ang_ratio < 0)
    ang_ratio = 0;

  trans_ratio = 1 - (fabs((floor(m_rollOuts_number / 2) - lane_id)) / floor(m_rollOuts_number / 2)) * 0.5;

  if(speed_ratio >= 0 && speed_ratio <= 1){
      obstacle_ratio = speed_ratio *(1 - ang_ratio/4.0);
      if(obstacle_ratio < 0.2){
        obstacle_ratio = 0.2;
      }
  }else if(speed_ratio == 0){
    obstacle_ratio = 0.1;
  }

  // ROS_ERROR("%f,%f,%f,%f",ang_ratio,obstacle_ratio,trans_ratio,avoid_ratio);
  diff_min = std::min(diff_min,obstacle_ratio);
  diff_min = std::min(diff_min,trans_ratio);

  // if(is_blocked && diff_min > 0.5){
  //   diff_min = 0.5;
  // }

  m_object_speed *= diff_min;                     // 得到比例后的速度

  if(is_blocked==1 && back_distance < minDistanceToAvoid)
  {
    m_object_speed = minVelocity * 1.5;
  }

  if ((m_object_speed < minVelocity && m_object_speed > 0.1)){
    m_object_speed = minVelocity;
  }

  if(is_blocked){
    // if((back_distance < maxdistancetoavoid && is_blocked) && b_obstacle_status)
    if(back_distance < maxdistancetoavoid)
    {
      m_object_speed = 0;
      // 急刹车
      brake_mode.data = 2;
    }else if(speed_ratio == 0){
      // -acc + brake
      brake_mode.data = 1;
    }
  }
  
  twist_out.lx = m_object_speed;           // 得到最终输出的速度
  // ROS_ERROR(" ---m_object_speed: %f,ang_ratio: %f,trans_ratio: %f,obstacle_ratio: %f",m_object_speed ,ang_ratio ,trans_ratio ,obstacle_ratio);

  if ((m_topic_status == false || (gnss_is_ok == false && _use_gnss == 1 && ndt_flag == false)) && brake_mode.data != 2) // 如果没有感知数据刹停。车辆定位不准,进行刹停
  { 
    twist_out.lx = 0;
    twist_out.az = 0;
    // 平缓刹车
    brake_mode.data = 1;
  }

  brake_mode_pub_.publish(brake_mode);
  return twist_out;
}

void TwistFilterNode::twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  const twist_filter::Twist twist = { msg->twist.linear.x, msg->twist.angular.z };
  ros::Time current_time = ros::Time::now();

  static ros::Time last_callback_time = current_time;
  static twist_filter::Twist twist_prev = twist;

  double time_elapsed = (current_time - last_callback_time).toSec();

  health_checker_.NODE_ACTIVATE();
  checkTwist(twist, twist_prev, time_elapsed);

  twist_filter::Twist twist_out = twist;

  // Apply lateral limit
  auto twist_limit_result = twist_filter_ptr_->lateralLimitTwist(twist, twist_prev, time_elapsed);
  if (twist_limit_result)
  {
    twist_out = twist_limit_result.get();
  }

  // Publish lateral accel and jerk before smoothing
  auto lacc_no_smoothed_result = twist_filter_ptr_->calcLaccWithAngularZ(twist);
  if (lacc_no_smoothed_result)
  {
    std_msgs::Float32 lacc_msg_debug;
    lacc_msg_debug.data = lacc_no_smoothed_result.get();
    twist_lacc_limit_debug_pub_.publish(lacc_msg_debug);
  }
  auto ljerk_no_smoothed_result = twist_filter_ptr_->calcLjerkWithAngularZ(twist, twist_prev, time_elapsed);
  if (ljerk_no_smoothed_result)
  {
    std_msgs::Float32 ljerk_msg_debug;
    ljerk_msg_debug.data = ljerk_no_smoothed_result.get();
    twist_ljerk_limit_debug_pub_.publish(ljerk_msg_debug);
  }

  // Smoothing
  twist_out = twist_filter_ptr_->smoothTwist(twist_out);

  if (use_sensor_data_filter_ == true)
    twist_out = mysmoothTwist(twist_out);

  // Smoothed value publish
  geometry_msgs::TwistStamped out_msg = *msg;
  out_msg.twist.linear.x = twist_out.lx;
  out_msg.twist.angular.z = twist_out.az;

  twist_pub_.publish(out_msg);

  // Publish lateral accel and jerk after smoothing
  auto lacc_smoothed_result = twist_filter_ptr_->calcLaccWithAngularZ(twist_out);
  if (lacc_smoothed_result)
  {
    std_msgs::Float32 lacc_msg;
    lacc_msg.data = lacc_smoothed_result.get();
    twist_lacc_result_pub_.publish(lacc_msg);
  }
  auto ljerk_smoothed_result = twist_filter_ptr_->calcLjerkWithAngularZ(twist_out, twist_prev, time_elapsed);
  if (ljerk_smoothed_result)
  {
    std_msgs::Float32 ljerk_msg;
    ljerk_msg.data = ljerk_smoothed_result.get();
    twist_ljerk_result_pub_.publish(ljerk_msg);
  }

  // Preserve value and time
  twist_prev = twist_out;
  last_callback_time = current_time;
}

void TwistFilterNode::ctrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg)
{
  const twist_filter::Ctrl ctrl = { msg->cmd.linear_velocity, msg->cmd.steering_angle };
  ros::Time current_time = ros::Time::now();

  static ros::Time last_callback_time = current_time;
  static twist_filter::Ctrl ctrl_prev = ctrl;
  ctrl_save = ctrl;

  double time_elapsed = (current_time - last_callback_time).toSec();

  health_checker_.NODE_ACTIVATE();
  checkCtrl(ctrl, ctrl_prev, time_elapsed);

  twist_filter::Ctrl ctrl_out = ctrl;

  // Apply lateral limit
  auto ctrl_limit_result = twist_filter_ptr_->lateralLimitCtrl(ctrl, ctrl_prev, time_elapsed);
  if (ctrl_limit_result)
  {
    ctrl_out = ctrl_limit_result.get();
  }

  // Publish lateral accel and jerk before smoothing
  auto lacc_no_smoothed_result = twist_filter_ptr_->calcLaccWithSteeringAngle(ctrl);
  if (lacc_no_smoothed_result)
  {
    std_msgs::Float32 lacc_msg_debug;
    lacc_msg_debug.data = lacc_no_smoothed_result.get();
    ctrl_lacc_limit_debug_pub_.publish(lacc_msg_debug);
  }
  auto ljerk_no_smoothed_result = twist_filter_ptr_->calcLjerkWithSteeringAngle(ctrl, ctrl_prev, time_elapsed);
  if (ljerk_no_smoothed_result)
  {
    std_msgs::Float32 ljerk_msg_debug;
    ljerk_msg_debug.data = ljerk_no_smoothed_result.get();
    ctrl_ljerk_limit_debug_pub_.publish(ljerk_msg_debug);
  }

  // Smoothing
  ctrl_out = twist_filter_ptr_->smoothCtrl(ctrl_out);

  // Smoothed value publish
  autoware_msgs::ControlCommandStamped out_msg = *msg;
  out_msg.cmd.linear_velocity = ctrl_out.lv;
  out_msg.cmd.steering_angle = ctrl_out.sa;
  
  //角度不进行过滤
  out_msg.cmd.steering_angle = msg->cmd.steering_angle ;
  ctrl_pub_.publish(out_msg);

  // Publish lateral accel and jerk after smoothing
  auto lacc_smoothed_result = twist_filter_ptr_->calcLaccWithSteeringAngle(ctrl_out);
  if (lacc_smoothed_result)
  {
    std_msgs::Float32 lacc_msg;
    lacc_msg.data = lacc_smoothed_result.get();
    ctrl_lacc_result_pub_.publish(lacc_msg);
  }
  auto ljerk_smoothed_result = twist_filter_ptr_->calcLjerkWithSteeringAngle(ctrl_out, ctrl_prev, time_elapsed);
  if (ljerk_smoothed_result)
  {
    std_msgs::Float32 ljerk_msg;
    ljerk_msg.data = ljerk_smoothed_result.get();
    ctrl_ljerk_result_pub_.publish(ljerk_msg);
  }

  // Preserve value and time
  ctrl_prev = ctrl_out;
  last_callback_time = current_time;
}

void TwistFilterNode::checkTwist(const twist_filter::Twist twist, const twist_filter::Twist twist_prev,
                                 const double& dt)
{
  const auto lacc = twist_filter_ptr_->calcLaccWithAngularZ(twist);
  const auto ljerk = twist_filter_ptr_->calcLjerkWithAngularZ(twist, twist_prev, dt);

  const twist_filter::Configuration& config = twist_filter_ptr_->getConfiguration();

  if (lacc)
  {
    health_checker_.CHECK_MAX_VALUE("twist_lateral_accel_high", lacc.get(), config.lateral_accel_limit,
                                    2 * config.lateral_accel_limit, DBL_MAX,
                                    "lateral_accel is too high in twist filtering");
  }
  if (ljerk)
  {
    health_checker_.CHECK_MAX_VALUE("twist_lateral_jerk_high", lacc.get(), config.lateral_jerk_limit,
                                    2 * config.lateral_jerk_limit, DBL_MAX,
                                    "lateral_jerk is too high in twist filtering");
  }
}

void TwistFilterNode::checkCtrl(const twist_filter::Ctrl ctrl, const twist_filter::Ctrl ctrl_prev, const double& dt)
{
  const auto lacc = twist_filter_ptr_->calcLaccWithSteeringAngle(ctrl);
  const auto ljerk = twist_filter_ptr_->calcLjerkWithSteeringAngle(ctrl, ctrl_prev, dt);

  const twist_filter::Configuration& config = twist_filter_ptr_->getConfiguration();

  if (lacc)
  {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_accel_high", lacc.get(), config.lateral_accel_limit,
                                    3 * config.lateral_accel_limit, DBL_MAX,
                                    "lateral_accel is too high in ctrl filtering");
  }
  if (ljerk)
  {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_jerk_high", lacc.get(), config.lateral_jerk_limit,
                                    3 * config.lateral_jerk_limit, DBL_MAX,
                                    "lateral_jerk is too high in ctrl filtering");
  }
}

}  // namespace twist_filter_node
