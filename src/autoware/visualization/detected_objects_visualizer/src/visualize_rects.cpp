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

#include "visualize_rects.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

VisualizeRects::VisualizeRects()
{
  ros::NodeHandle private_nh_("~");

  ros::NodeHandle nh;

  std::string image_src_topic;
  std::string object_src_topic;
  std::string image_out_topic;

  private_nh_.param<std::string>("image_src", image_src_topic, "/image_raw");
  private_nh_.param<std::string>("object_src", object_src_topic, "/detection/image_detector/objects");
  private_nh_.param<std::string>("image_out", image_out_topic, "/image_rects");
  nh.param<bool>("/vehicle_is_sim", is_sim_, false);   // 是否为仿真车辆
  nh.param<int>("/traffic_light_pose", pose_type_, 0); // 红绿灯位置类型

  // get namespace from topic
  std::string ros_namespace = image_src_topic;
  std::size_t found_pos = ros_namespace.rfind("/"); // find last / from topic name to extract namespace
  std::cout << ros_namespace << std::endl;
  if (found_pos != std::string::npos)
    ros_namespace.erase(found_pos, ros_namespace.length() - found_pos);
  std::cout << ros_namespace << std::endl;
  image_out_topic = ros_namespace + image_out_topic;

  image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(private_nh_,
                                                                                 image_src_topic,
                                                                                 1);
  ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_src_topic.c_str());
  detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(private_nh_,
                                                                                                     object_src_topic,
                                                                                                     1);
  ROS_INFO("[%s] object_src: %s", __APP_NAME__, object_src_topic.c_str());

  detections_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                     *image_filter_subscriber_,
                                                     *detection_filter_subscriber_);
  detections_synchronizer_->registerCallback(
      boost::bind(&VisualizeRects::SyncedDetectionsCallback, this, _1, _2));

  publisher_image_ = node_handle_.advertise<sensor_msgs::Image>(
      image_out_topic, 1);
  ROS_INFO("[%s] image_out: %s", __APP_NAME__, image_out_topic.c_str());

  signalState_pub = node_handle_.advertise<autoware_msgs::TrafficLight>("/camera_light_color", 10, true);
  isFirstDetectedR = true;
  isFirstDetectedG = true;
}

void VisualizeRects::SyncedDetectionsCallback(
    const sensor_msgs::Image::ConstPtr &in_image_msg,
    const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  try
  {
    image_ = cv_bridge::toCvShare(in_image_msg, "bgr8")->image;
    cv::Mat drawn_image;
    drawn_image = ObjectsToRects(image_, in_objects);
    sensor_msgs::ImagePtr drawn_msg = cv_bridge::CvImage(in_image_msg->header, "bgr8", drawn_image).toImageMsg();
    publisher_image_.publish(drawn_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("[%s] Could not convert from '%s' to 'bgr8'.", __APP_NAME__, in_image_msg->encoding.c_str());
  }
}

// 将 CvSeq* 转换为 std::vector<cv::Point> 的函数
std::vector<cv::Point> convertCvSeqToVector(CvSeq *seq)
{
  std::vector<cv::Point> points;
  for (int i = 0; i < seq->total; ++i)
  {
    CvPoint *pt = CV_GET_SEQ_ELEM(CvPoint, seq, i);
    points.push_back(cv::Point(pt->x, pt->y));
  }
  return points;
}

int VisualizeRects::brightnessDetect(const cv::Mat &frame, Rect rect)
{
  Mat img;
  Mat imgGreen, imgRed;
  int redCount;
  int greenCount;

  Mat image_roi = frame(rect); // 剪切
  // 调整亮度,减少噪点
  double a = 0.3;
  double b = (1 - a) * 125;
  image_roi.convertTo(img, img.type(), a, b);
  // img = image_roi;

  if (is_sim_ == false)
  { // YCrCb
    Mat imgYCrCb;

    // 转换为YCrCb颜色空间
    cvtColor(img, imgYCrCb, CV_BGR2YCrCb);

    // 分解YCrCb的三个成分
    vector<Mat> planes;
    split(imgYCrCb, planes);

    imgRed.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);
    imgGreen.create(imgYCrCb.rows, imgYCrCb.cols, CV_8UC1);

    // 遍历以根据Cr分量拆分红色和绿色
    MatIterator_<uchar> it_Cr = planes[1].begin<uchar>(),
                        it_Cr_end = planes[1].end<uchar>();
    MatIterator_<uchar> it_Cb = planes[2].begin<uchar>();
    MatIterator_<uchar> it_Red = imgRed.begin<uchar>();
    MatIterator_<uchar> it_Green = imgGreen.begin<uchar>();

    for (; it_Cr != it_Cr_end; ++it_Cr, ++it_Red, ++it_Green)
    {
      // RED, 145<Cr<470 红色
      if (*it_Cr > 145 && *it_Cr < 470)
        // if (*it_Cr > cr_red_min && *it_Cr < cr_red_max && *it_Cb > cb_value)
        *it_Red = 255;
      else
        *it_Red = 0;

      // GREEN 95<Cr<110 绿色
      if (*it_Cr > 85 && *it_Cr < 100)
        // if (*it_Cr > cr_green_min && *it_Cr < cr_green_max && *it_Cb > cb_value)
        *it_Green = 255;
      else
        *it_Green = 0;
    }
  }
  else
  { // HSV, sim

    Mat imgHsv;

    cvtColor(img, imgHsv, CV_BGR2HSV);

    inRange(imgHsv, Scalar(2, 45, 45), Scalar(10, 255, 255), imgRed);
    inRange(imgHsv, Scalar(35, 10, 20), Scalar(80, 255, 255), imgGreen);
  }

  // 膨胀和腐蚀
  //  dilate(imgRed, imgRed, Mat(15, 15, CV_8UC1), Point(-1, -1));
  //  erode(imgRed, imgRed, Mat(1, 1, CV_8UC1), Point(-1, -1));
  //  dilate(imgGreen, imgGreen, Mat(15, 15, CV_8UC1), Point(-1, -1));
  //  erode(imgGreen, imgGreen, Mat(1, 1, CV_8UC1), Point(-1, -1));

  // redCount = processImgR(imgRed);
  // greenCount = processImgR(imgGreen);
  // 形态学处理：膨胀和腐蚀以移除噪声
  int morph_size = 3;
  Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

  dilate(imgRed, imgRed, element);
  erode(imgRed, imgRed, element);
  dilate(imgGreen, imgGreen, element);
  erode(imgGreen, imgGreen, element);
  redCount = processImgR(imgRed);
  greenCount = processImgR(imgGreen);
  // printf(" 1--- red: %d, green: %d\n", redCount, greenCount);

  int light_color;
  if (redCount < 2 && greenCount < 2)
  {
    // cv::putText(frame, "lights out", Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 8, 8, 0);
    light_color = 2;
  }
  else if (redCount > greenCount)
  {
    // cv::putText(frame, "red light", Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 8, 8, 0);
    light_color = 0;
  }
  else
  {
    // cv::putText(frame, "green light", Point(40, 150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 8, 8, 0);
    light_color = 1;
  }

  static std::vector<float_t> steer_val;
  steer_val.push_back(light_color);
  int max_num = 10;
  static int light_static = 0;
  if (steer_val.size() >= max_num)
  {
    std::sort(steer_val.begin(), steer_val.end());
    light_static = steer_val.at(int(max_num / 2));
    steer_val.clear();
  }
  light_color = light_static;
  return light_color;
}
int VisualizeRects::processImgR(Mat src)
{
#if 0
    Mat tmp;
 
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point> hull;
 
    CvPoint2D32f tempNode;
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* pointSeq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage);
 
    Rect* trackBox;
    Rect* result;
    int resultNum = 0;
 
    int area = 0;
    src.copyTo(tmp);
 
    //提取轮廓
    findContours(tmp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
 
    if (contours.size() > 0)
    {
        trackBox = new Rect[contours.size()];
        result = new Rect[contours.size()];
 
        //确定要跟踪的区域
        for (int i = 0; i < contours.size(); i++)
        {
            cvClearSeq(pointSeq);
            // 获取凸包的点集
            convexHull(Mat(contours[i]), hull, true);
            int hullcount = (int)hull.size();
            // 凸包的保存点
            for (int j = 0; j < hullcount - 1; j++)
            {
                tempNode.x = hull[j].x;
                tempNode.y = hull[j].y;
                cvSeqPush(pointSeq, &tempNode);
            }
 
            trackBox[i] = cv::boundingRect(pointSeq);
        }
 
        if (isFirstDetectedR)
        {
            lastTrackBoxR = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
                lastTrackBoxR[i] = trackBox[i];
            lastTrackNumR = contours.size();
            isFirstDetectedR = false;
        }
        else
        {
            for (int i = 0; i < contours.size(); i++)
            {
                for (int j = 0; j < lastTrackNumR; j++)
                {
                    if (isIntersected(trackBox[i], lastTrackBoxR[j]))
                    {
                        result[resultNum] = trackBox[i];
                        break;
                    }
                }
                resultNum++;
            }
            delete[] lastTrackBoxR;
            lastTrackBoxR = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++)
            {
                lastTrackBoxR[i] = trackBox[i];
            }
            lastTrackNumR = contours.size();
        }
 
        delete[] trackBox;
    }
    else
    {
        isFirstDetectedR = true;
        result = NULL;
    }
    cvReleaseMemStorage(&storage);
 
    if (result != NULL)
    {
        for (int i = 0; i < resultNum; i++)
        {
            area += result[i].area();
        }
    }
    delete[] result;
 
    return area;
#else
  int percentage;
  MatIterator_<uchar> it_ = src.begin<uchar>(),
                      it__end = src.end<uchar>();

  int num = 0; // 记录颜色的像素点

  for (; it_ != it__end; ++it_)
  {
    // RED, 145<Cr<470 红色
    if (*it_ > 100) // 127
    {
      num++;
    }
  }

  percentage = (int)((num / (float)(src.rows * src.cols)) * 100.0);
  // printf("----- %d,%d,%d\n", num, percentage,(src.rows * src.cols));
  return percentage;
#endif
}

int VisualizeRects::processImgG(Mat src)
{
  Mat tmp;

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  vector<Point> hull;

  CvPoint2D32f tempNode;
  CvMemStorage *storage = cvCreateMemStorage();
  CvSeq *pointSeq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage);

  Rect *trackBox;
  Rect *result;
  int resultNum = 0;

  int area = 0;

  src.copyTo(tmp);
  // 提取轮廓
  findContours(tmp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  if (contours.size() > 0)
  {
    trackBox = new Rect[contours.size()];
    result = new Rect[contours.size()];

    // 确定要跟踪的区域
    for (int i = 0; i < contours.size(); i++)
    {
      cvClearSeq(pointSeq);
      // 获取凸包的点集
      convexHull(Mat(contours[i]), hull, true);
      int hullcount = (int)hull.size();
      // 保存凸包的点
      for (int j = 0; j < hullcount - 1; j++)
      {
        tempNode.x = hull[j].x;
        tempNode.y = hull[j].y;
        cvSeqPush(pointSeq, &tempNode);
      }
      // 将 CvSeq* 转换为 std::vector<cv::Point>
      std::vector<cv::Point> contour = convertCvSeqToVector(pointSeq);
      trackBox[i] = cv::boundingRect(contour);
    }

    if (isFirstDetectedG)
    {
      lastTrackBoxG = new Rect[contours.size()];
      for (int i = 0; i < contours.size(); i++)
        lastTrackBoxG[i] = trackBox[i];
      lastTrackNumG = contours.size();
      isFirstDetectedG = false;
    }
    else
    {
      for (int i = 0; i < contours.size(); i++)
      {
        for (int j = 0; j < lastTrackNumG; j++)
        {
          if (isIntersected(trackBox[i], lastTrackBoxG[j]))
          {
            result[resultNum] = trackBox[i];
            break;
          }
        }
        resultNum++;
      }
      delete[] lastTrackBoxG;
      lastTrackBoxG = new Rect[contours.size()];
      for (int i = 0; i < contours.size(); i++)
      {
        lastTrackBoxG[i] = trackBox[i];
      }
      lastTrackNumG = contours.size();
    }

    delete[] trackBox;
  }
  else
  {
    isFirstDetectedG = true;
    result = NULL;
  }
  cvReleaseMemStorage(&storage);

  if (result != NULL)
  {
    for (int i = 0; i < resultNum; i++)
    {
      area += result[i].area();
    }
  }
  delete[] result;

  return area;
}

// 确定两个矩形区域是否相交
bool VisualizeRects::isIntersected(Rect r1, Rect r2)
{
  int minX = max(r1.x, r2.x);
  int minY = max(r1.y, r2.y);
  int maxX = min(r1.x + r1.width, r2.x + r2.width);
  int maxY = min(r1.y + r1.height, r2.y + r2.height);

  if (minX < maxX && minY < maxY)
    return true;
  else
    return false;
}

// cv::Mat
// VisualizeRects::ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
// {
//   cv::Rect select_rect;
//   bool first_ = true;
//   const int center_threshold = 200;
//   cv::Mat final_image = in_image.clone();
//   cv::Mat imput_frame = in_image.clone();
//   for (auto const &object : in_objects->objects)
//   {
//     if (IsObjectValid(object))
//     {
//       cv::Rect rect;
//       rect.x = object.x;
//       rect.y = object.y;
//       rect.width = object.width;
//       rect.height = object.height;

//       if (rect.x + rect.width >= in_image.cols)
//         rect.width = in_image.cols - rect.x - 1; // 画线不超出图像

//       if (rect.y + rect.height >= in_image.rows)
//         rect.height = in_image.rows - rect.y - 1;

//       // 选择图像相对靠中的红绿灯
//       if (object.label == "traffic light")
//       {
//         if (first_)
//         {
//           first_ = false;
//           select_rect = rect;
//         }
//         else
//         {
//           if (pose_type_ == 0)
//           {
//             if (fabs(rect.x - in_image.cols / 2.0) < fabs(select_rect.x - in_image.cols / 2.0))
//             { // 取中间红绿灯
//               select_rect = rect;
//             }
//           }
//           else if (pose_type_ == 1)
//           {
//             if (fabs(rect.x - in_image.cols / 2.0) > fabs(select_rect.x - in_image.cols / 2.0))
//             { // 取2边
//               select_rect = rect;
//             }
//           }
//         }
//       }
//       // draw rectangle
//       cv::rectangle(final_image,
//                     rect,
//                     cv::Scalar(244, 134, 66),
//                     4,
//                     cv::LINE_AA);

//       // draw label
//       std::string label = "";
//       if (!object.label.empty() && object.label != "unknown")
//       {
//         label = object.label;
//       }
//       int font_face = cv::FONT_HERSHEY_DUPLEX;
//       double font_scale = 1.5;
//       int thickness = 1;

//       int baseline = 0;
//       cv::Size text_size = cv::getTextSize(label,
//                                            font_face,
//                                            font_scale,
//                                            thickness,
//                                            &baseline);
//       baseline += thickness;

//       cv::Point text_origin(object.x - text_size.height, object.y);

//       cv::rectangle(final_image,
//                     text_origin + cv::Point(0, baseline),
//                     text_origin + cv::Point(text_size.width, -text_size.height),
//                     cv::Scalar(0, 0, 0),
//                     cv::FILLED,
//                     cv::LINE_AA,
//                     0);

//       cv::putText(final_image,
//                   label,
//                   text_origin,
//                   font_face,
//                   font_scale,
//                   cv::Scalar::all(255),
//                   thickness,
//                   cv::LINE_AA,
//                   false);
//     }
//   }

//   autoware_msgs::TrafficLight state_msg;
//   state_msg.traffic_light = 2;
//   if (first_ == false)
//   {
//     int light_color = state_msg.traffic_light = brightnessDetect(imput_frame, select_rect);
//     state_msg.traffic_light = light_color;

//     // // 在图像中划出选中的红绿灯
//     // cv::Rect rect;
//     // rect = select_rect;

//     // 在图像中划出选中的红绿灯
//     // draw rectangle for selected traffic light
//     cv::rectangle(final_image,
//                   select_rect,
//                   cv::Scalar(0, 255, 0), // 使用绿色框选
//                   4,
//                   cv::LINE_AA);

//     // draw label for selected traffic light
//     std::string selected_label = "Selected Traffic Light";
//     int font_face = cv::FONT_HERSHEY_DUPLEX;
//     double font_scale = 1.5;
//     int thickness = 1;

//     int baseline = 0;
//     cv::Size text_size = cv::getTextSize(selected_label,
//                                          font_face,
//                                          font_scale,
//                                          thickness,
//                                          &baseline);
//     baseline += thickness;

//     cv::Point text_origin(select_rect.x, select_rect.y - 10); // 将文本显示在矩形的上方

//     cv::rectangle(final_image,
//                   text_origin + cv::Point(0, baseline),
//                   text_origin + cv::Point(text_size.width, -text_size.height),
//                   cv::Scalar(0, 0, 0),
//                   cv::FILLED,
//                   cv::LINE_AA,
//                   0);

//     cv::putText(final_image,
//                 selected_label,
//                 text_origin,
//                 font_face,
//                 font_scale,
//                 cv::Scalar::all(255),
//                 thickness,
//                 cv::LINE_AA,
//                 false);
//   }

//   signalState_pub.publish(state_msg);

//   return final_image;
// } // ObjectsToBoxes

cv::Mat VisualizeRects::ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
    cv::Rect select_rect; // 用于存储选择的红绿灯矩形
    bool first_ = true;  // 初始标记为第一个红绿灯
    const int center_threshold = 50; // 中心阈值
    cv::Mat final_image = in_image.clone(); // 克隆输入图像用于结果显示
    cv::Mat imput_frame = in_image.clone(); // 可能用于后续亮度检测

    // 遍历检测到的对象
    for (auto const &object : in_objects->objects)
    {
        if (IsObjectValid(object)) // 检查对象是否有效
        {
            cv::Rect rect; // 创建一个矩形来表示对象
            rect.x = object.x; // 对象的X坐标
            rect.y = object.y; // 对象的Y坐标
            rect.width = object.width; // 对象的宽度
            rect.height = object.height; // 对象的高度

            // 确保矩形不超出图像边界
            if (rect.x + rect.width >= in_image.cols)
                rect.width = in_image.cols - rect.x - 1; // 画线不超出图像

            if (rect.y + rect.height >= in_image.rows)
                rect.height = in_image.rows - rect.y - 1;

            // 处理红绿灯
            if (object.label == "traffic light") // 如果对象是红绿灯
            {
                // 计算红绿灯的中心点
                int object_center_x = rect.x + rect.width / 2;

                if (first_)
                {
                    // 第一次找到红绿灯，初始化选择的矩形
                    select_rect = rect; // 选择当前红绿灯
                    first_ = false; // 更新标志为不是第一个红绿灯
                }
                else
                {
                    // 根据 pose_type 选择最中间的红绿灯
                    if (pose_type_ == 0) // 选择中间的红绿灯
                    {
                        if (fabs(object_center_x - in_image.cols / 2.0) < fabs((select_rect.x + select_rect.width / 2.0) - in_image.cols / 2.0) )
                        {
                            // 如果当前红绿灯更靠近图像中心，更新选择的红绿灯
                            select_rect = rect;
                        }
                    }
                    else if (pose_type_ == 1) // 选择两边的红绿灯
                    {
                        if (fabs(object_center_x - in_image.cols / 2.0) > fabs((select_rect.x + select_rect.width / 2.0) - in_image.cols / 2.0))
                        {
                            // 如果当前红绿灯更远离图像中心，更新选择的红绿灯
                            select_rect = rect;
                        }
                    }
                }
            }

            // 绘制矩形框和标签
            cv::rectangle(final_image, rect, cv::Scalar(244, 134, 66), 4, cv::LINE_AA); 

            // 绘制标签
            std::string label = "";
            if (!object.label.empty() && object.label != "unknown")
            {
                label = object.label; // 如果标签有效，使用它
            }
            int font_face = cv::FONT_HERSHEY_DUPLEX; // 字体类型
            double font_scale = 1.0; // 字体缩放
            int thickness = 1; // 字体粗细

            int baseline = 0; // 基线
            // 获取标签文本的大小
            cv::Size text_size = cv::getTextSize(label, font_face, font_scale, thickness, &baseline);
            baseline += thickness; // 调整基线

            // 文本起始位置
            cv::Point text_origin(object.x - text_size.height, object.y);

            // 绘制文本背景
            cv::rectangle(final_image,
                          text_origin + cv::Point(0, baseline),
                          text_origin + cv::Point(text_size.width, -text_size.height),
                          cv::Scalar(0, 0, 0), // 黑色背景
                          cv::FILLED,
                          cv::LINE_AA,
                          0);

            // 在图像上绘制文本
            cv::putText(final_image, label, text_origin, font_face, font_scale, cv::Scalar::all(255), thickness, cv::LINE_AA, false);
        }
    }

    autoware_msgs::TrafficLight state_msg; // 创建交通灯状态消息
    state_msg.traffic_light = 2; // 默认状态

    // 如果没有找到红绿灯
    if (!first_) // 如果已经找到红绿灯
    {
        // 亮度检测
        int light_color = state_msg.traffic_light = brightnessDetect(imput_frame, select_rect);
        state_msg.traffic_light = light_color; // 更新交通灯状态

        // 在图像中划出选中的红绿灯
        cv::rectangle(final_image, select_rect, cv::Scalar(0, 255, 0), 4, cv::LINE_AA); // 使用绿色框选

        // 显示选中的红绿灯标签
        std::string selected_label = "Selected Traffic Light"; // 标签文本
        int font_face = cv::FONT_HERSHEY_DUPLEX; // 字体类型
        double font_scale = 1.5; // 字体缩放
        int thickness = 1; // 字体粗细

        int baseline = 0; // 基线
        // 获取标签文本的大小
        cv::Size text_size = cv::getTextSize(selected_label, font_face, font_scale, thickness, &baseline);
        baseline += thickness; // 调整基线

        // 文本起始位置
        cv::Point text_origin(select_rect.x, select_rect.y - 10); // 将文本显示在矩形的上方

        // 绘制文本背景
        cv::rectangle(final_image,
                      text_origin + cv::Point(0, baseline),
                      text_origin + cv::Point(text_size.width, -text_size.height),
                      cv::Scalar(0, 0, 0), // 黑色背景
                      cv::FILLED,
                      cv::LINE_AA,
                      0);

        // 在图像上绘制文本
        cv::putText(final_image,
                    selected_label,
                    text_origin,
                    font_face,
                    font_scale,
                    cv::Scalar::all(255),
                    thickness,
                    cv::LINE_AA,
                    false);
    }

    signalState_pub.publish(state_msg); // 发布交通灯状态消息

    return final_image; // 返回结果图像
} // ObjectsToRects
5

bool VisualizeRects::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      in_object.width < 0 ||
      in_object.height < 0 ||
      in_object.x < 0 ||
      in_object.y < 0)
  {
    return false;
  }
  return true;
} // end IsObjectValid
