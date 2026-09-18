#ifndef LOGINWIDGET_H
#define LOGINWIDGET_H

#include <QWidget>
#include <QDialog>
#include <QStandardItemModel>
#include <QVBoxLayout>

#include <QTableWidget>
#include <QTableWidgetItem>
#include <QComboBox>
#include <QSpinBox>
#include <QModelIndex>
#include <QFileDialog>
#include <QXmlStreamWriter>
#include <QCheckBox>

#include <QtConcurrent>//要记得添加该头文件
// #include <QValueAxis>
// #include <QLineSeries>
// #include <QGraphicsView>
#include <QSettings>
#include <QDir>
#include <QTabWidget>
#include <QToolButton>
#include <QAction>
#include <QMenu>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>

#include <QPixmap>

#include <QTimer>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/BatteryState.h"


#include "driverless_ui/HardwareInfo.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "yaml-cpp/yaml.h"


// #include "qrviz.hpp"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <QProcessEnvironment>

// /* 光添加头文件QChartView还不够，还需要引入QChart的命名空间 */
// using namespace QtCharts;


namespace Ui {
    class Driverless;
}


class Driverless : public QWidget
{
    Q_OBJECT

public:
    explicit Driverless(int argc, char **argv, QWidget *parent = 0);
    ~Driverless();

    // bool init(const std::string& master_url, const std::string& host_url);
    void ui_init();

public slots:
    void myThread();



private slots:
    void Choose_Vehicle();
    void Choose_Map();

    void sensorParam(void);
    void mapParam(void);//地图参数配置
    void mapLocalParam(void);
    void detectionParam(void);//感知参数配置
    void planningParam(void);//规划参数配置
    void controlParam(void);//控制参数配置
    void canClientParam(void);//底盘can参数配置
    void recordParam(void);//录制数据参数配置
    void mappingParam(void);//建图参数配置
    void makePathParam(void);//录制轨迹点参数配置
    void pathTrackParam(void);//路径追踪轨迹点参数配置
    void deleMapParam(void);//删除地图参数配置
    // void laneParam(void);//车道保持参数配置


    QString on_Btn_SetName_clicked();

private:

    Ui::Driverless *ui;

    QString strParam(QString param, QString data);
    QString toBool(QString data);
    bool boolTo(QString data);
    void delay_ms(int time);

    void connections();
    void module_connet(void);
    void function_connet(void);
    void rosTool_connet(void);

    void my_sensing(void);
    void my_localization(void);
    void my_detection(void);
    void my_mission(void);
    void my_motion(void);
    void my_auto(void);

    QStringList getFiles(const QString& bagName, const QString& dirName, bool path_en);
    bool copyDirectory(const QString& srcPath, const QString& dstPath, bool coverFileIfExist);
    void runLaunch(QString path, QString launch, bool en);

    QString writeYaml(QString module, QString func, QString param, QString data);
    QString readYaml(QString module, QString func, QString param);
    void readParam(void);

    void battery_callback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void hardware_info_callback(const driverless_ui::HardwareInfo::ConstPtr& msg);


private:
    QString env_map, env_vehicle, env_site;//系统变量
    bool use_developer;//开发者页面标志
    QDialog *dia_pathTrack;
    QCheckBox *check_pathTrack;
    QVBoxLayout *layout_pathTrack;


    struct Sensor
    { //传感器参数
        QString baud;
    }sensor;

    struct Map//地图参数
    {/* data */
        QString path_pcd, map_dir;
    }map;

    struct Local//定位参数
    {/* data */
        QString get_gnss_init_pose, init_pos_gnss, get_height, method_type, use_imu, use_gnss;
    }local;

    struct Detection//感知参数
    {/* data */
        QString lidar_front_car_range, lidar_back_car_range, lidar_left_car_range, lidar_right_car_range;
        QString front_safe_distance, lear_safe_distance, left_safe_distance, right_safe_distance;
        QString front_velocity_coefficient, lear_velocity_coefficient, left_velocity_coefficient, right_velocity_coefficient;
        QString use_gpu, cluster_size_min, cluster_size_max, lidar_back_range, lidar_total_range;
        QString clipping_height, min_point_distance;
        QString vehicle_width, vehicle_length, vector_map_filter_distance;
    }detect;
    struct Planning//感知参数
    {/* data */
        QString maxLocalPlanDistance, minFollowingDistance, minDistanceToAvoid, maxDistanceToAvoid, rollOutsNumber;
        QString horizontalSafetyDistance, verticalSafetyDistance, maxVelocity, minVelocity, maxAcceleration;
        QString maxDeceleration, width, length, wheelBaseLength, turningRadius, maxSteerAngle;
        QString enableLaneChange, enableReplan, enableSmoothing, enableRvizInput, onlyReplan, pathDensity;
    }plan;
    struct Control//控制参数
    {/* data */
        QString minimum_lookahead_distance, lookahead_ratio;
        QString wheel_base, lateral_accel_limit, lateral_jerk_limit, lowpass_gain_linear_x;
        QString lowpass_gain_angular_z, lowpass_gain_steering_angle;
    }control;

    struct CanClient//控制参数
    {/* data */
        QString speed_gain, stop_brake_increase, normal_brake_increase;
    }can;

    struct Record//录制数据参数
    {/* data */
        QString topic1, topic2, topic3, topic4, topic5, topic6, topic7, topic8;
    }record;

    struct Lane//车道保持参数
    {/* data */
        QString img_compress, h_size, h_off, h_top, v_size, v_off, speed, gray_threshold, hls_val;
    }lane;

    struct Mapping//建图参数
    {/* data */
        QString name, use_gps, use_bag, use_sim, filt, use_gpu, speed;
    }mapping;

protected:
    void closeEvent(QCloseEvent *event);

};


#endif // LOGINWIDGET_H
