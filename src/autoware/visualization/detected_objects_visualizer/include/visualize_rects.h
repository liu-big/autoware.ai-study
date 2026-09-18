/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 *
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#ifndef _VISUALIZERECTS_H
#define _VISUALIZERECTS_H

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <autoware_msgs/TrafficLight.h>

#define __APP_NAME__ "visualize_rects"

using namespace cv;
using namespace std;
class VisualizeRects
{
private:
  std::string input_topic_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;
  image_transport::Subscriber subscriber_image_;

  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

  ros::Publisher publisher_image_;
  ros::Publisher signalState_pub;

  cv::Mat image_;
  std_msgs::Header image_header_;



  bool is_sim_;
  int pose_type_;

  typedef
  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *detections_synchronizer_;

  void
  SyncedDetectionsCallback(
    const sensor_msgs::Image::ConstPtr &in_image_msg,
    const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections);

  bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

  cv::Mat ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects);

  int brightnessDetect(const cv::Mat& frame,Rect rect);
  int processImgR(Mat);
  int processImgG(Mat);
  bool isIntersected(Rect, Rect);

  bool isFirstDetectedR;
  bool isFirstDetectedG;
  Rect* lastTrackBoxR;
  Rect* lastTrackBoxG;
  int lastTrackNumR;
  int lastTrackNumG;

  // int cr_red_min, cr_red_max, cr_green_min, cr_green_max, cb_value;

public:
  VisualizeRects();
};

#endif  // _VISUALIZERECTS_H
