#include "lidar_obstacle_detection/safe_range_core.h"

namespace SafeRangeNs
{

SafeRange::SafeRange()
{
    ros::NodeHandle nh, private_nh_("~");
    private_nh_.param<double>("update_rate", update_rate_, 30);

    private_nh_.param<double>("lidar_car_front_distance", m_dLidarCarFrontDistance, 1.0);
    private_nh_.param<double>("lidar_car_lear_distance", m_dLidarCarLearDistance, -1.0);
    private_nh_.param<double>("lidar_car_left_distance", m_dLidarCarLeftDistance, 0.5);
    private_nh_.param<double>("lidar_car_right_distance", m_dLidarCarRightDistance, -0.5);

    private_nh_.param<double>("front_safe_distance", m_dFrontSafeDistance, 0.5);
    private_nh_.param<double>("lear_safe_distance", m_dLearSafeDistance, 0.2);
    private_nh_.param<double>("left_safe_distance", m_dLeftSafeDistance, 0.2);
    private_nh_.param<double>("right_safe_distance", m_dRightSafeDistance, 0.2);

    private_nh_.param<double>("front_velocity_coefficient", m_dFrontVelocityCoefficient, 1.0);
    private_nh_.param<double>("lear_velocity_coefficient", m_dLearVelocityCoefficient, 0.2);
    private_nh_.param<double>("left_velocity_coefficient", m_dLeftVelocityCoefficient, 0.2);
    private_nh_.param<double>("right_velocity_coefficient", m_dRightVelocityCoefficient, 0.2);
    private_nh_.param<double>("m_dSafeMaxSpeedLimit", m_dSafeMaxSpeedLimit, 10.0);

    private_nh_.param<double>("front_slow_distance", m_dFrontSlowDistance, 2);
    private_nh_.param<double>("lear_slow_distance", m_dLearSlowDistance, 1);
    private_nh_.param<double>("left_slow_distance", m_dLeftSlowDistance, 0.5);
    private_nh_.param<double>("right_slow_distance", m_dRightSlowDistance, 0.5);

    private_nh_.param<std::string>("ultra_enable", str_UltraEnables, std::string("[true,true,true,true,true,true,true,true]"));

    sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &SafeRange::callbackGetPredictedObjects, this);
    sub_ultra_sound = nh.subscribe("/ultrasound_raw", 1, &SafeRange::callbackUltraSound, this);
    sub_current_pose = nh.subscribe("/current_pose", 10, &SafeRange::callbackGetCurrentPose, this);
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &SafeRange::callbackCurrentVelocity, this);
    sub_real_velocity = nh.subscribe("/real_velocity", 10, &SafeRange::callbackRealVelocity, this);

    pub_safe_ctrl = nh.advertise<autoware_msgs::Vehicle_Ctrl_Status>("safe_range_ctrl", 10, true);
    g_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("safe_range_marker", 10, true);
    slow_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("slow_range_marker", 10, true);
    ros::Publisher g_local_mark_pub;
    safe_detect = 1;
    safe_ultra = 1;

    YAML::Node ultras = YAML::Load(str_UltraEnables); // yaml文件格式解析数据
    size_t ultra_size = ultras.size();
    for (size_t i = 0; i < ultra_size; i++)
    {
        m_dUltraEnables.push_back(ultras[i].as<bool>()); // 把数据放在容器中
    }

    cur_angle = 0.0;
}

SafeRange::~SafeRange()
{
}


// geometry_msgs::Pose GetClosestPose(const autoware_msgs::DetectedObject &det_obj, geometry_msgs::Pose &cur_pos)
// {
//     geometry_msgs::Pose minPose;
//     for (unsigned int i = 0; i < det_obj.convex_hull.polygon.points.size(); i++)
//     {
//         geometry_msgs::Pose pose;
//         pose.position.x = det_obj.convex_hull.polygon.points.at(i).x;
//         pose.position.y = det_obj.convex_hull.polygon.points.at(i).y;
//         pose.position.z = det_obj.convex_hull.polygon.points.at(i).z;
//     }

//     return minPose;
// }
void SafeRange::createLocalsafeMarker(geometry_msgs::PoseArray &poses)
{

    visualization_msgs::Marker obj_marker;
    obj_marker.header.frame_id = "velodyne";
    obj_marker.header.stamp = ros::Time::now();
    obj_marker.ns = "safe_dis";
    obj_marker.id = 0;
    obj_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.scale.x = 0.2;
    obj_marker.scale.y = 0.2;
    obj_marker.scale.z = 0.2;
    obj_marker.color.r = 1.0;
    obj_marker.color.a = 1.0;
    obj_marker.frame_locked = true;

    g_waypoints_marker_array.markers.push_back(obj_marker);
    for (unsigned int i = 0; i < poses.poses.size(); i++)
    {
        geometry_msgs::Point point;
        point = poses.poses[i].position;
        obj_marker.points.push_back(point);
        g_waypoints_marker_array.markers.push_back(obj_marker);
    }

}

void SafeRange::createSafeRangeMarker(float_t insafe, double front, double lear, double left, double right)
{
    visualization_msgs::Marker waypoint_marker;
    waypoint_marker.header.frame_id = "velodyne";
    waypoint_marker.header.stamp = ros::Time();
    waypoint_marker.ns = "safe_range_box";
    waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.scale.x = 0.05;
    waypoint_marker.scale.y = 0.05;
    // waypoint_marker.scale.z = 0.1;
    waypoint_marker.frame_locked = false;
    if(insafe == 0){
        waypoint_marker.color.r = 1.0;
        waypoint_marker.color.g = 0.0;
        waypoint_marker.color.b = 0.0;
    }else{
        waypoint_marker.color.r = 1.0;
        waypoint_marker.color.g = 1.0;
        waypoint_marker.color.b = 0.0;
    }
    waypoint_marker.color.a = 0.6;

    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = front;
    p1.y = left;
    p1.z = -1.0;

    p2.x = front;
    p2.y = right;
    p2.z = -1.0;

    p3.x = lear;
    p3.y = right;
    p3.z = -1.0;

    p4.x = lear;
    p4.y = left;
    p4.z = -1.0;

    waypoint_marker.points.push_back(p1);
    waypoint_marker.points.push_back(p2);
    waypoint_marker.points.push_back(p3);
    waypoint_marker.points.push_back(p4);
    waypoint_marker.points.push_back(p1);

    g_waypoints_marker_array.markers.push_back(waypoint_marker);
}

void SafeRange::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
    static tf::TransformListener listener;

    while (1)
    {
        try
        {
            listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

void SafeRange::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg)
{
    float_t insafe = 1;
    float_t inslow = 1;
    float_t slow_proport[4] = {1,-1,1,1};
    geometry_msgs::PoseArray poses;
    m_bLSafeDir[0] = false;
    m_bLSafeDir[1] = false;

    m_dIncremental[0] = (cur_velocity * m_dFrontVelocityCoefficient) * (1 - sin(fabs(cur_angle))*1.5);
    m_dIncremental[1] = (-cur_velocity * m_dLearVelocityCoefficient) * (1 - sin(fabs(cur_angle))*1.5);
    m_dIncremental[2] = (fabs(cur_velocity) * m_dLeftVelocityCoefficient) * (sin(cur_angle)*1.5);
    m_dIncremental[3] = (fabs(cur_velocity) * m_dRightVelocityCoefficient) * (-sin(cur_angle)*1.5);

    for(int i=0; i < 4; i++){
        if(m_dIncremental[i] < 0){
            m_dIncremental[i] = 0;
        }
    }
    for (unsigned int i = 0; i < msg->objects.size(); i++)
    {
        if (1)//msg->objects.at(i).id > 0)
        {
            for (unsigned int j = 0; j < msg->objects[i].convex_hull.polygon.points.size(); j++)
            {
                geometry_msgs::Pose pose;
                geometry_msgs::Pose pose_maker;
                geometry_msgs::Point point;
                float x,y,z;
                float diff_x, diff_y,diff_z;
                float yaw;
                pose.position.x = msg->objects[i].convex_hull.polygon.points.at(j).x;
                pose.position.y = msg->objects[i].convex_hull.polygon.points.at(j).y;
                pose.position.z = msg->objects[i].convex_hull.polygon.points.at(j).z;

                tf::StampedTransform transform;
                GetTransformFromTF("map", "velodyne", transform);
                geometry_msgs::Pose m_OriginPos;
                m_OriginPos.position.x = transform.getOrigin().x();
                m_OriginPos.position.y = transform.getOrigin().y();
                m_OriginPos.position.z = transform.getOrigin().z();

#if 0
                // map转为base_link坐标
                diff_x = (pose.position.x - m_CurrentPos.position.x);
                diff_y = (pose.position.y - m_CurrentPos.position.y);
                diff_z = (pose.position.z - m_CurrentPos.position.z);
                // base_link转为velodyne坐标
                diff_x = diff_x  - 1.35;
                diff_y = diff_y  +  0;
                diff_z = diff_z  -  1.30;
                #else
                // map转为velodyne坐标
                diff_x = (pose.position.x - m_OriginPos.position.x);
                diff_y = (pose.position.y - m_OriginPos.position.y);
                diff_z = (pose.position.z - m_OriginPos.position.z);
                #endif

                yaw = tf::getYaw(m_CurrentPos.orientation);
                point.x = diff_x * cos(yaw) + diff_y * sin(yaw);
                point.y = diff_y * cos(yaw) - diff_x * sin(yaw);
                point.z = diff_z;

/***************增加减速功能****************/
                if (point.x < (m_dLidarCarFrontDistance + m_dFrontSlowDistance)
                                && point.x > (m_dLidarCarLearDistance -  m_dLearSlowDistance)
                                && point.y < (m_dLidarCarLeftDistance + m_dLeftSlowDistance)
                                && point.y > (m_dLidarCarRightDistance - m_dRightSlowDistance)
                                && point.z < 0.3)
                {
                    double slow_distance = sqrt(pow(point.x,2)+pow(point.y,2));
                    double slow_fron=1,slow_lear=1,slow_fronb=1,slow_learb=1;

                    if(point.x>(m_dLidarCarFrontDistance+m_dFrontSafeDistance)&&point.x>0)
                    {
                        slow_proport[0] = (slow_distance-(m_dLidarCarFrontDistance+m_dFrontSafeDistance))
                                            /((m_dLidarCarFrontDistance+m_dFrontSlowDistance)-(m_dLidarCarFrontDistance+m_dFrontSafeDistance));
                        if(slow_proport[0]>1)
                            slow_proport[0] = 1;
                        ROS_WARN("[激光雷达] 触发前面减速.    %f ",slow_proport[0]);
                    }
                    if(point.x<(m_dLidarCarLearDistance-m_dLearSafeDistance)&&point.x<0)
                    {
                        slow_proport[1] = (point.x-(m_dLidarCarLearDistance-m_dLearSafeDistance))
                                            /((m_dLidarCarLearDistance+m_dLearSlowDistance)-(m_dLidarCarLearDistance+m_dLearSafeDistance));
                        if(slow_proport[1]<-1)
                            slow_proport[1] = -1;
                        ROS_WARN("[激光雷达] 触发后面减速.    %f ",slow_proport[1]);
                        }
                    if(point.x<(m_dLidarCarFrontDistance+m_dFrontSafeDistance)&&point.x>0)
                    {
                        slow_proport[2] = (abs(point.y)-(m_dLidarCarLeftDistance+m_dLeftSafeDistance))
                                            /((m_dLidarCarLeftDistance+m_dLeftSlowDistance)-(m_dLidarCarLeftDistance+m_dLeftSafeDistance));
                        if(slow_proport[2]>1)
                            slow_proport[2] = 1;
                        ROS_WARN("[激光雷达] 触发前侧面减速.   %f ",slow_proport[2]);
                    }
                    if(point.x>(m_dLidarCarLearDistance-m_dLearSafeDistance)&&point.x<0)
                    {
                        slow_proport[3] = (abs(point.y)-(m_dLidarCarLeftDistance+m_dLeftSafeDistance))
                                            /((m_dLidarCarLeftDistance+m_dLeftSlowDistance)-(m_dLidarCarLeftDistance+m_dLeftSafeDistance));
                        if(slow_proport[3]>1)
                            slow_proport[3] = 1;
                        ROS_WARN("[激光雷达] 触发后侧面减速.   %f ",slow_proport[3]);
                    }
                    inslow = 0;
                    // slow_proport = std::min(std::min(slow_fron,slow_fronb),slow_learb);
                    // if(slow_proport>abs(slow_lear))
                    //     slow_proport = slow_lear;
                    // if(slow_proport>1)
                    //     slow_proport=1;
                }
/***************增加减速功能****************/


                m_dIncremental[0] = 0;
                m_dIncremental[1] = 0;
                m_dIncremental[2] = 0;
                m_dIncremental[3] = 0;
                // yaw = 3.14159 - yaw;
                if (point.x < (m_dLidarCarFrontDistance + (m_dIncremental[0] + m_dFrontSafeDistance)) 
                                && point.x > (m_dLidarCarLearDistance - (m_dIncremental[1] + m_dLearSafeDistance)) 
                                && point.y < (m_dLidarCarLeftDistance + (m_dIncremental[2] + m_dLeftSafeDistance)) 
                                && point.y > (m_dLidarCarRightDistance - (m_dIncremental[3] + m_dRightSafeDistance))
                                && point.z < 0.3)
                {
                    pose_maker.position = point;
                    poses.poses.push_back(pose_maker);
                    insafe = 0;
                    // ROS_ERROR("[激光雷达] 触发了紧急安全制动.");

                    if(point.x > ( m_dLidarCarLearDistance /2.0 ))
                    {
                        m_bLSafeDir[0] = true;
                        ROS_ERROR("[激光雷达] 触发了from紧急安全制动.");
                    }else
                    {
                        m_bLSafeDir[1] = true;
                        ROS_ERROR("[激光雷达] 触发了lear紧急安全制动.");
                    }
                }
            }
        }
    }
    g_waypoints_marker_array.markers.clear();
    createLocalsafeMarker(poses);
    createSafeRangeMarker(insafe, m_dLidarCarFrontDistance + (m_dIncremental[0] + m_dFrontSafeDistance),
                          m_dLidarCarLearDistance - (m_dIncremental[1] + m_dLearSafeDistance),
                          m_dLidarCarLeftDistance + (m_dIncremental[2] + m_dLeftSafeDistance),
                          m_dLidarCarRightDistance - (m_dIncremental[3] + m_dRightSafeDistance));
    g_mark_pub.publish(g_waypoints_marker_array);
    safe_detect = insafe;

    /***************增加显示减速范围****************/
    g_waypoints_marker_array.markers.clear();
    createLocalsafeMarker(poses);
    createSafeRangeMarker(inslow, m_dLidarCarFrontDistance +  m_dFrontSlowDistance,
                        m_dLidarCarLearDistance -  m_dLearSlowDistance,
                        m_dLidarCarLeftDistance +  m_dLeftSlowDistance,
                        m_dLidarCarRightDistance - m_dRightSlowDistance);
    slow_mark_pub.publish(g_waypoints_marker_array);
    // safe_slow = slow_proport;
    memcpy(safe_slow,slow_proport,sizeof(slow_proport));
    /***************增加显示减速范围****************/
}

// 激光雷达只保护最小安全距离，慢速(倒车等场景)
void SafeRange::callbackUltraSound(const autoware_msgs::UltraSoundArrayConstPtr &msg)
{
    float_t range=1;
    double _safe_distance;

    m_bUaSafeDir[0] = false;
    m_bUaSafeDir[1] = false;
    for (unsigned int i = 0; i < msg->ultras.size(); i++)
    {
        if (m_dUltraEnables[i] == 0){            // 过滤不使能的超声波
             continue;
        }
        
        uint8_t dir;
        uint8_t index = msg->ultras[i].direction;
        float_t distance = msg->ultras[i].distance / 1000.0;
        switch (index)
        {
        case autoware_msgs::UltraSound::FRONT_LEFT:
        case autoware_msgs::UltraSound::FRONT_RIGHT:
            // _safe_distance = cur_velocity * m_dFrontVelocityCoefficient + m_dFrontSafeDistance;
            _safe_distance = m_dFrontSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
                m_bUaSafeDir[0] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在前方,距离: %fm < %fm", distance, _safe_distance);
            }
            break;
        case autoware_msgs::UltraSound::LEFT_FRONT:
            // _safe_distance = cur_velocity * m_dLeftVelocityCoefficient + m_dLeftSafeDistance;
            _safe_distance = m_dLeftSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
               m_bUaSafeDir[0] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在左前方,距离:  %fm < %fm", distance, _safe_distance);
            }

            break;
        case autoware_msgs::UltraSound::LEFT_REAR:
            // _safe_distance = cur_velocity * m_dLeftVelocityCoefficient + m_dLeftSafeDistance;
            _safe_distance = m_dLeftSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
                m_bUaSafeDir[1] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在左后方,距离:  %fm < %fm", distance, _safe_distance);
            }

            break;
        case autoware_msgs::UltraSound::RIGHT_FRONT:
            // _safe_distance = cur_velocity * m_dRightVelocityCoefficient + m_dRightSafeDistance;
            _safe_distance = m_dRightSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
                m_bUaSafeDir[0] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在右前方,距离:  %fm < %fm", distance, _safe_distance);
            }

            break;
        case autoware_msgs::UltraSound::RIGHT_REAR:
            // _safe_distance = cur_velocity * m_dRightVelocityCoefficient + m_dRightSafeDistance;
            _safe_distance = m_dRightSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
                m_bUaSafeDir[1] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在右后方,距离:  %fm < %fm", distance, _safe_distance);
            }

            break;
        case autoware_msgs::UltraSound::REAR_LEFT:
        case autoware_msgs::UltraSound::REAR_RIGHT:
            // _safe_distance = cur_velocity * m_dLearVelocityCoefficient + m_dLearSafeDistance;
            _safe_distance = m_dLearSafeDistance;
            if (distance < _safe_distance)
            {
                range = 0;
                m_bUaSafeDir[1] = true;
                ROS_ERROR("[超声波] 触发了紧急安全制动.位置在后方,距离:  %fm < %fm", distance, _safe_distance);
            }

            break;
        default:
            break;
        }
    }
    safe_ultra = range;
}

void SafeRange::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    m_CurrentPos = msg->pose;
    bNewCurrentPos = true;
}

void SafeRange::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
//   cur_velocity = msg->twist.linear.x;
//   if(cur_velocity < 0)
//   {
//       cur_velocity =0;
//   }else if(cur_velocity > m_dSafeMaxSpeedLimit){      // 单速度大于5km/h时,安全距离达到最大
//       cur_velocity = m_dSafeMaxSpeedLimit;
//   }
}

void SafeRange::callbackRealVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  cur_velocity = msg->twist.linear.x;
  if(cur_velocity < -m_dSafeMaxSpeedLimit)
  {
      cur_velocity =-m_dSafeMaxSpeedLimit;
  }else if(cur_velocity > m_dSafeMaxSpeedLimit){      // 单速度大于5km/h时,安全距离达到最大
      cur_velocity = m_dSafeMaxSpeedLimit;
  }

  cur_angle = msg->twist.angular.z;
}

void SafeRange::MainLoop()
{
    ros::Rate loop_rate(update_rate_);

    std_msgs::Float64 safe_ratio;

    autoware_msgs::Vehicle_Ctrl_Status msg;

    while (ros::ok())
    {
        msg.safe_ctrl = std::min(safe_detect, safe_ultra);
        msg.safe_from = std::max(m_bLSafeDir[0], m_bUaSafeDir[0]);
        msg.safe_lear = std::max(m_bLSafeDir[1], m_bUaSafeDir[1]);
        msg.safe_slow[0] = safe_slow[0];
        msg.safe_slow[1] = safe_slow[1];
        msg.safe_slow[2] = safe_slow[2];
        msg.safe_slow[3] = safe_slow[3];
        // safe_ratio.data = std::min(safe_detect, safe_ultra);
        pub_safe_ctrl.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
}

}