/*
  订阅到数据处理后再发布
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

// ros 头文件
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/UltraSoundArray.h>
#include <autoware_msgs/UltraSound.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <autoware_msgs/Vehicle_Ctrl_Status.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

namespace SafeRangeNs
{

class SafeRange
{
public:
    SafeRange();
    ~SafeRange();
    void MainLoop();

private:
    // ros 相关变量
    ros::Subscriber sub_predicted_objects, sub_ultra_sound, sub_current_pose, sub_current_velocity, sub_real_velocity;
    ros::Publisher pub_safe_ctrl, g_mark_pub, slow_mark_pub;
    geometry_msgs::Pose m_CurrentPos;

    // params 获取参数服务器值
    double update_rate_; // replanning and publishing rate [Hz]
    // std::string vehicle_name_;
    double m_dLidarCarFrontDistance;
    double m_dLidarCarLearDistance;
    double m_dLidarCarLeftDistance;
    double m_dLidarCarRightDistance;

    double m_dFrontSafeDistance;
    double m_dLearSafeDistance;
    double m_dLeftSafeDistance;
    double m_dRightSafeDistance;

    double m_dFrontVelocityCoefficient;
    double m_dLearVelocityCoefficient;
    double m_dLeftVelocityCoefficient;
    double m_dRightVelocityCoefficient;

    double m_dFrontSlowDistance;
    double m_dLearSlowDistance;
    double m_dLeftSlowDistance;
    double m_dRightSlowDistance;

    std::string str_UltraEnables;
    std::vector<bool> m_dUltraEnables;
    double m_dSafeMaxSpeedLimit;
    // classes 组合
    // ros msg variables
    // geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;

    // mist variables
    float_t safe_value;
    float_t safe_detect;
    float_t safe_ultra;
    float_t safe_slow[4]={1,-1,1,1};
    bool bNewCurrentPos;
    double cur_velocity;
    double cur_angle;

    float m_dIncremental[4];

    bool m_bLSafeDir[2];
    bool m_bUaSafeDir[2];

    visualization_msgs::MarkerArray g_waypoints_marker_array;

        // callback
    void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg);
    void callbackUltraSound(const autoware_msgs::UltraSoundArrayConstPtr &msg);
    void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackRealVelocity(const geometry_msgs::TwistStampedConstPtr &msg);

    // fucntions
    // void publishCanInfo(autoware_can_msgs::CANPacket msg_);
    // geometry_msgs::Pose GetClosestPose(const autoware_msgs::DetectedObject &det_obj, geometry_msgs::Pose &cur_pos);
    void createLocalsafeMarker(geometry_msgs::PoseArray &poses);
    void createSafeRangeMarker(float_t insafe, double front, double lear, double left, double right);
    void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
};

}