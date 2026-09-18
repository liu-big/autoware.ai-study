#include <QCompleter>
#include <QDebug>
#include <QMessageBox>
#include <QMovie>
#include <QPainter>
#include <QPropertyAnimation>
#include <QStringListModel>
#include <QSettings>
#include <string>
#include <ros/network.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <qinputdialog.h>
#include "ros/ros.h"
#include "driverless.h"
#include "ui_driverless.h"
#include <iostream>
#include <ros/package.h>
#include <iostream>
#include <stdio.h>

using namespace std;

Driverless::Driverless(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Driverless)
{
    ui->setupUi(this);
    this->setWindowTitle("无人驾驶操作平台");
    ui_init();  //初始化
    connections();    //连接成功才能执行此函数
    QtConcurrent::run(this,&Driverless::myThread);//多线程执行循环启动，可以带参数
}

Driverless::~Driverless()
{

}

void Driverless::ui_init() {

    // YAML::Node config = YAML::LoadFile("install/driverless_ui/share/driverless_ui/config/local.yaml");
    // if(config["Local"]["setting"]["developer"]){
    //     QString readData = QString::fromStdString(config["Local"]["setting"]["developer"].as<string>());
    //     if(readData == "false")
    //         ui->tab_manager->removeTab(2);//删除该页，实际该页还在
    // }
    ui->check_sensor->setIcon(QIcon("://images/Sensor.png"));
    ui->check_map->setIcon(QIcon("://images/Map.png"));
    ui->check_locat->setIcon(QIcon("://images/Local.png"));
    ui->check_detect->setIcon(QIcon("://images/detect.png"));
    ui->check_plan->setIcon(QIcon("://images/Path.png"));
    ui->check_control->setIcon(QIcon("://images/control.png"));
    ui->check_auto->setIcon(QIcon("://images/can.png"));
    ui->check_rviz->setIcon(QIcon("://images/rviz.png"));

    ui->check_record->setIcon(QIcon("://images/record.png"));
    ui->check_pcd->setIcon(QIcon("://images/pcd.png"));
    ui->check_makePath->setIcon(QIcon("://images/makePath.png"));
    ui->check_pathTrack->setIcon(QIcon("://images/pathTrack.png"));
    ui->check_autoDrive->setIcon(QIcon("://images/autoDrive.png"));
    // ui->check_laneKeep->setIcon(QIcon("://images/lane.png"));  //车道保持
    ui->check_setGoal->setIcon(QIcon("://images/goal.png"));
    ui->check_deleMap->setIcon(QIcon("://images/delet.png"));

    QIcon icon1 = QIcon("://images/car.png");
    QPixmap m_pic1 = icon1.pixmap(icon1.actualSize(QSize(16, 16)));//size自行调整
    ui->label_car->setPixmap(m_pic1);
    QIcon icon2 = QIcon("://images/map.png");
    QPixmap m_pic2 = icon2.pixmap(icon2.actualSize(QSize(16, 16)));//size自行调整
    ui->label_map->setPixmap(m_pic2);
    QIcon icon3 = QIcon("://images/line.png");
    QPixmap m_pic3 = icon3.pixmap(icon3.actualSize(QSize(16, 16)));//size自行调整
    ui->label_line->setPixmap(m_pic3);
    QIcon icon4 = QIcon("://images/lidar.png");
    QPixmap m_pic4 = icon4.pixmap(icon4.actualSize(QSize(16, 16)));//size自行调整
    ui->label_lidar->setPixmap(m_pic4);
    QIcon icon5 = QIcon("://images/Camera.png");
    QPixmap m_pic5 = icon5.pixmap(icon5.actualSize(QSize(16, 16)));//size自行调整
    ui->label_camera->setPixmap(m_pic5);
    QIcon icon6 = QIcon("://images/imu.png");
    QPixmap m_pic6 = icon6.pixmap(icon6.actualSize(QSize(16, 16)));//size自行调整
    ui->label_imu->setPixmap(m_pic6);
    QIcon icon7 = QIcon("://images/mm.png");
    QPixmap m_pic7 = icon7.pixmap(icon7.actualSize(QSize(16, 16)));//size自行调整
    ui->label_mm->setPixmap(m_pic3);
    QIcon icon8 = QIcon("://images/um.png");
    QPixmap m_pic8 = icon8.pixmap(icon8.actualSize(QSize(16, 16)));//size自行调整
    ui->label_um->setPixmap(m_pic8);
    QIcon icon9 = QIcon("://images/power-v.png");
    QPixmap m_pic9 = icon9.pixmap(icon9.actualSize(QSize(16, 16)));//size自行调整
    ui->label_power->setPixmap(m_pic9);
    QIcon icon10 = QIcon("://images/canhe.png");
    QPixmap m_pic10 = icon10.pixmap(icon10.actualSize(QSize(16, 16)));//size自行调整
    ui->label_canhe->setPixmap(m_pic10);
    QIcon icon11 = QIcon("://images/vehicle.png");
    QPixmap m_pic11 = icon11.pixmap(icon11.actualSize(QSize(16, 16)));//size自行调整
    ui->label_vehicle_2->setPixmap(m_pic11);

    //获取系统变量
    env_vehicle = QProcessEnvironment::systemEnvironment().value("AUTOCAR_VEHICLE_TYPE");
    env_map = QProcessEnvironment::systemEnvironment().value("AUTOCAR_MAP_TYPE");
    env_site = QProcessEnvironment::systemEnvironment().value("AUTOCAR_MAP_SITE_TYPE");

    qDebug()<<"车辆:"<<env_vehicle<<"地图:"<<env_map<<"路线:"<<env_site;
    ui->comboBox_car->addItem(env_vehicle);
    ui->comboBox_map->addItem(env_map);
    ui->comboBox_line->addItem(env_site);

//底盘can窗口，定义全局否则重开窗口有问题
    dia_pathTrack = new QDialog(this);
    dia_pathTrack->setWindowTitle(QObject::tr("参数配置"));
    dia_pathTrack->resize(300,200);
    check_pathTrack = new QCheckBox("启动底盘CAN");
    layout_pathTrack = new QVBoxLayout;
    layout_pathTrack->addWidget(check_pathTrack);
    dia_pathTrack->setLayout(layout_pathTrack);
    connect(check_pathTrack, &QCheckBox::stateChanged, [=] (){
        if(check_pathTrack->isChecked())
            my_auto();
        else
            runLaunch("custom_launch","my_auto.launch",false);
    });
}

void Driverless::readParam(void)
{
    sensor.baud = readYaml("Sensor", "navigate", "baud");

    local.get_gnss_init_pose = readYaml("Local", "gnss", "get_gnss_init_pose");
    local.init_pos_gnss = readYaml("Local", "matching", "init_pos_gnss");
    local.get_height = readYaml("Local", "matching", "get_height");
    local.method_type = readYaml("Local", "matching", "method_type");
    local.use_imu = readYaml("Local", "matching", "use_imu");
    local.use_gnss = readYaml("Local", "matching", "use_gnss");

    detect.lidar_front_car_range = readYaml("Detection", "safeRange", "lidar_front_car_range");
    detect.lidar_back_car_range = readYaml("Detection", "safeRange", "lidar_back_car_range");
    detect.lidar_left_car_range = readYaml("Detection", "safeRange", "lidar_left_car_range");
    detect.lidar_right_car_range = readYaml("Detection", "safeRange", "lidar_right_car_range");
    detect.front_safe_distance = readYaml("Detection", "safeRange", "front_safe_distance");
    detect.lear_safe_distance = readYaml("Detection", "safeRange", "lear_safe_distance");
    detect.left_safe_distance = readYaml("Detection", "safeRange", "left_safe_distance");
    detect.right_safe_distance = readYaml("Detection", "safeRange", "right_safe_distance");
    detect.front_velocity_coefficient = readYaml("Detection", "safeRange", "front_velocity_coefficient");
    detect.lear_velocity_coefficient = readYaml("Detection", "safeRange", "lear_velocity_coefficient");
    detect.left_velocity_coefficient = readYaml("Detection", "safeRange", "left_velocity_coefficient");
    detect.right_velocity_coefficient = readYaml("Detection", "safeRange", "right_velocity_coefficient");
    detect.use_gpu = readYaml("Detection", "Cluster", "use_gpu");
    detect.cluster_size_min = readYaml("Detection", "Cluster", "cluster_size_min");
    detect.cluster_size_max = readYaml("Detection", "Cluster", "cluster_size_max");
    detect.lidar_back_range = readYaml("Detection", "Cluster", "lidar_back_range");
    detect.lidar_total_range = readYaml("Detection", "Cluster", "lidar_total_range");
    detect.clipping_height = readYaml("Detection", "Filter", "clipping_height");
    detect.min_point_distance = readYaml("Detection", "Filter", "min_point_distance");
    detect.vehicle_width = readYaml("Detection", "ContourTrack", "vehicle_width");
    detect.vehicle_length = readYaml("Detection", "ContourTrack", "vehicle_length");
    detect.vector_map_filter_distance = readYaml("Detection", "ContourTrack", "vector_map_filter_distance");

    plan.maxLocalPlanDistance = readYaml("Planning", "op_common", "maxLocalPlanDistance");
    plan.minFollowingDistance = readYaml("Planning", "op_common", "minFollowingDistance");
    plan.minDistanceToAvoid = readYaml("Planning", "op_common", "minDistanceToAvoid");
    plan.maxDistanceToAvoid = readYaml("Planning", "op_common", "maxDistanceToAvoid");
    plan.rollOutsNumber = readYaml("Planning", "op_common", "rollOutsNumber");
    plan.horizontalSafetyDistance = readYaml("Planning", "op_common", "horizontalSafetyDistance");
    plan.verticalSafetyDistance = readYaml("Planning", "op_common", "verticalSafetyDistance");
    plan.maxVelocity = readYaml("Planning", "op_common", "maxVelocity");
    plan.minVelocity = readYaml("Planning", "op_common", "minVelocity");
    plan.maxAcceleration = readYaml("Planning", "op_common", "maxAcceleration");
    plan.maxDeceleration = readYaml("Planning", "op_common", "maxDeceleration");
    plan.width = readYaml("Planning", "op_common", "width");
    plan.length = readYaml("Planning", "op_common", "length");
    plan.wheelBaseLength = readYaml("Planning", "op_common", "wheelBaseLength");
    plan.turningRadius = readYaml("Planning", "op_common", "turningRadius");
    plan.maxSteerAngle = readYaml("Planning", "op_common", "maxSteerAngle");

    plan.enableLaneChange = readYaml("Planning", "op_global", "enableLaneChange");
    plan.enableReplan = readYaml("Planning", "op_global", "enableReplan");
    plan.enableSmoothing = readYaml("Planning", "op_global", "enableSmoothing");
    plan.enableRvizInput = readYaml("Planning", "op_global", "enableRvizInput");
    plan.onlyReplan = readYaml("Planning", "op_global", "onlyReplan");
    plan.pathDensity = readYaml("Planning", "op_global", "pathDensity");

    control.minimum_lookahead_distance = readYaml("Control", "pursuit", "minimum_lookahead_distance");
    control.lookahead_ratio = readYaml("Control", "pursuit", "lookahead_ratio");
    control.wheel_base = readYaml("Control", "twist", "wheel_base");
    control.lateral_accel_limit = readYaml("Control", "twist", "lateral_accel_limit");
    control.lateral_jerk_limit = readYaml("Control", "twist", "lateral_jerk_limit");
    control.lowpass_gain_linear_x = readYaml("Control", "twist", "lowpass_gain_linear_x");
    control.lowpass_gain_angular_z = readYaml("Control", "twist", "lowpass_gain_angular_z");
    control.lowpass_gain_steering_angle = readYaml("Control", "twist", "lowpass_gain_steering_angle");

    lane.img_compress = readYaml("LaneFollow", "image", "img_compress");
    lane.h_size = readYaml("LaneFollow", "image", "h_size");
    lane.h_off = readYaml("LaneFollow", "image", "h_off");
    lane.h_top = readYaml("LaneFollow", "image", "h_top");
    lane.v_size = readYaml("LaneFollow", "image", "v_size");
    lane.v_off = readYaml("LaneFollow", "image", "v_off");
    lane.speed = readYaml("LaneFollow", "image", "speed");
    lane.gray_threshold = readYaml("LaneFollow", "image", "gray_threshold");
    lane.hls_val = readYaml("LaneFollow", "image", "hls_val");

    can.speed_gain = readYaml("CanClient", "drive", "speed_gain");
    can.stop_brake_increase = readYaml("CanClient", "drive", "stop_brake_increase");
    can.normal_brake_increase = readYaml("CanClient", "drive", "normal_brake_increase");

    record.topic1 = "/points_raw";
    record.topic2 = "/imu_raw";
    record.topic3 = "null";
    record.topic4 = "null";
    record.topic5 = "/fix";
    record.topic6 = "/image_raw";
    record.topic7 = "null";
    record.topic8 = "null";

    mapping.name = "Lio-sam";
    mapping.use_bag = "false";
    mapping.use_gps = "false";
    mapping.use_gpu = "0";
    mapping.filt = "0.2";
    mapping.speed = "2";

    map.path_pcd = "null";
    map.map_dir = "null";
}

void Driverless::connections() {

    //选择车辆槽函数
    connect(ui->comboBox_car,static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this,&Driverless::Choose_Vehicle);
    //选择地图槽函数
    connect(ui->comboBox_map,static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),this,&Driverless::Choose_Map);
    //模块
    module_connet();
    //功能
    function_connet();
    //ROS工具
    rosTool_connet();

}

QString Driverless::strParam(QString param, QString data)
{
    QString str = " "+param+":="+data+" ";
    return str;
}

QString Driverless::toBool(QString data)
{
    if(data == "0")
        return "false";
    else
        return "true";
}

bool Driverless::boolTo(QString data)
{
    if(data == "false")
        return 0;
    else
        return 1;
}
void Driverless::delay_ms(int time)
{
    QEventLoop loop;//定义一个新的事件循环
    QTimer::singleShot(time, &loop, SLOT(quit()));//创建定时器延时100毫秒，等待文件彻底删除
    loop.exec();
}

void Driverless::my_sensing(void)
{
    // QString str_cmd1;
    QString str_cmd1 = "my_sensing.launch"+strParam("baud", sensor.baud);
    runLaunch("custom_launch",str_cmd1,true);
}
void Driverless::my_localization(void)
{
    QString str_cmd = "my_localization.launch"+strParam("get_gnss_init_pose", local.get_gnss_init_pose);
    str_cmd.append(strParam("init_pos_gnss", local.init_pos_gnss));
    str_cmd.append(strParam("get_height", local.get_height));
    str_cmd.append(strParam("use_imu", local.use_imu));
    str_cmd.append(strParam("use_gnss", local.use_gnss));
    runLaunch("custom_launch",str_cmd,true);
}
void Driverless::my_detection(void)
{
    QString str_cmd = "my_detection.launch"+strParam("lidar_front_car_range", detect.lidar_front_car_range);
    str_cmd.append(strParam("lidar_back_car_range", detect.lidar_back_car_range));
    str_cmd.append(strParam("lidar_left_car_range", detect.lidar_left_car_range));
    str_cmd.append(strParam("lidar_right_car_range", detect.lidar_right_car_range));
    str_cmd.append(strParam("front_safe_distance", detect.front_safe_distance));
    str_cmd.append(strParam("lear_safe_distance", detect.lear_safe_distance));
    str_cmd.append(strParam("left_safe_distance", detect.left_safe_distance));
    str_cmd.append(strParam("right_safe_distance", detect.right_safe_distance));
    str_cmd.append(strParam("front_velocity_coefficient", detect.front_velocity_coefficient));
    str_cmd.append(strParam("lear_velocity_coefficient", detect.lear_velocity_coefficient));
    str_cmd.append(strParam("left_velocity_coefficient", detect.left_velocity_coefficient));
    str_cmd.append(strParam("right_velocity_coefficient", detect.right_velocity_coefficient));

    str_cmd.append(strParam("use_gpu", detect.use_gpu));
    str_cmd.append(strParam("cluster_size_min", detect.cluster_size_min));
    str_cmd.append(strParam("cluster_size_max", detect.cluster_size_max));
    str_cmd.append(strParam("lidar_back_range", detect.lidar_back_range));
    str_cmd.append(strParam("lidar_total_range", detect.lidar_total_range));

    str_cmd.append(strParam("clipping_height", detect.clipping_height));
    str_cmd.append(strParam("min_point_distance", detect.min_point_distance));

    str_cmd.append(strParam("vehicle_width", detect.vehicle_width));
    str_cmd.append(strParam("vehicle_length", detect.vehicle_length));
    str_cmd.append(strParam("vector_map_filter_distance", detect.vector_map_filter_distance));
    runLaunch("custom_launch",str_cmd,true);
}

void Driverless::my_mission(void)
{
    QString str_cmd = "my_mission_planning.launch"+strParam("maxLocalPlanDistance", plan.maxLocalPlanDistance);
    str_cmd.append(strParam("minFollowingDistance", plan.minFollowingDistance));
    str_cmd.append(strParam("minDistanceToAvoid", plan.minDistanceToAvoid));
    str_cmd.append(strParam("maxDistanceToAvoid", plan.maxDistanceToAvoid));
    str_cmd.append(strParam("rollOutsNumber", plan.rollOutsNumber));
    str_cmd.append(strParam("horizontalSafetyDistance", plan.horizontalSafetyDistance));
    str_cmd.append(strParam("verticalSafetyDistance", plan.verticalSafetyDistance));
    str_cmd.append(strParam("maxVelocity", plan.maxVelocity));
    str_cmd.append(strParam("minVelocity", plan.minVelocity));
    str_cmd.append(strParam("maxAcceleration", plan.maxAcceleration));
    str_cmd.append(strParam("maxDeceleration", plan.maxDeceleration));
    str_cmd.append(strParam("width", plan.width));
    str_cmd.append(strParam("length", plan.length));
    str_cmd.append(strParam("wheelBaseLength", plan.wheelBaseLength));
    str_cmd.append(strParam("turningRadius", plan.turningRadius));
    str_cmd.append(strParam("maxSteerAngle", plan.maxSteerAngle));

    str_cmd.append(strParam("enableLaneChange", plan.enableLaneChange));
    str_cmd.append(strParam("enableReplan", plan.enableReplan));
    str_cmd.append(strParam("enableSmoothing", plan.enableSmoothing));
    str_cmd.append(strParam("enableRvizInput", plan.enableRvizInput));
    str_cmd.append(strParam("onlyReplan", plan.onlyReplan));
    str_cmd.append(strParam("pathDensity", plan.pathDensity));
    runLaunch("custom_launch",str_cmd,true);
}
void Driverless::my_motion(void)
{
    QString str_cmd = "my_motion_planning.launch"+strParam("minimum_lookahead_distance", control.minimum_lookahead_distance);
    str_cmd.append(strParam("lookahead_ratio", control.lookahead_ratio));
    str_cmd.append(strParam("wheel_base", control.wheel_base));
    str_cmd.append(strParam("lateral_accel_limit", control.lateral_accel_limit));
    str_cmd.append(strParam("lateral_jerk_limit", control.lateral_jerk_limit));
    str_cmd.append(strParam("lowpass_gain_linear_x", control.lowpass_gain_linear_x));
    str_cmd.append(strParam("lowpass_gain_angular_z", control.lowpass_gain_angular_z));
    str_cmd.append(strParam("lowpass_gain_steering_angle", control.lowpass_gain_steering_angle));
    runLaunch("custom_launch",str_cmd,true);
}
void Driverless::my_auto(void)
{
    QString str_cmd = "my_auto.launch"+strParam("speed_gain", can.speed_gain);
    str_cmd.append(strParam("stop_brake_increase", can.stop_brake_increase));
    str_cmd.append(strParam("normal_brake_increase", can.normal_brake_increase));
    runLaunch("custom_launch",str_cmd,true);
}

void Driverless::module_connet(void)
{
    //传感器
    connect(ui->check_sensor, &QCheckBox::stateChanged, [=] (){
        if(ui->check_sensor->isChecked())
            my_sensing();
        else
            runLaunch("custom_launch","my_sensing.launch",false);
    });
    connect(ui->push_sensor, &QPushButton::clicked, this, &Driverless::sensorParam);


    //地图
    connect(ui->check_map, &QCheckBox::stateChanged, [=] (){
        if(ui->check_map->isChecked())
        {
            qDebug()<<"55555"<<map.map_dir;
            QString str_cmd = "my_map.launch";
            if(map.path_pcd != "null")
                str_cmd.append(strParam("path_pcd", map.path_pcd));
            if(map.map_dir != "null")
                str_cmd.append(strParam("map_dir", map.map_dir));

            qDebug()<<str_cmd;
            runLaunch("custom_launch",str_cmd,true);
        }
        else
            runLaunch("custom_launch","my_map.launch",false);
    });
    connect(ui->push_map, &QPushButton::clicked, this, &Driverless::mapParam);

    //定位
    connect(ui->check_locat, &QCheckBox::stateChanged, [=] (){
        if(ui->check_locat->isChecked())
            my_localization();
        else
            runLaunch("custom_launch","my_localization.launch",false);
    });
    connect(ui->push_locat, &QPushButton::clicked, this, &Driverless::mapLocalParam);

    //感知
    connect(ui->check_detect, &QCheckBox::stateChanged, [=] (){
        if(ui->check_detect->isChecked())
            my_detection();
        else
            runLaunch("custom_launch","my_detection.launch",false);
    });
    connect(ui->push_detect, &QPushButton::clicked, this, &Driverless::detectionParam);

    //规划
    connect(ui->check_plan, &QCheckBox::stateChanged, [=] (){
        if(ui->check_plan->isChecked())
            my_mission();
        else
            runLaunch("custom_launch","my_mission_planning.launch",false);
    });
    connect(ui->push_plan, &QPushButton::clicked, this, &Driverless::planningParam);

    //控制
    connect(ui->check_control, &QCheckBox::stateChanged, [=] (){
        if(ui->check_control->isChecked())
            my_motion();
        else
            runLaunch("custom_launch","my_motion_planning.launch",false);
    });
    connect(ui->push_control, &QPushButton::clicked, this, &Driverless::controlParam);

    //底盘can
    connect(ui->check_auto, &QCheckBox::stateChanged, [=] (){
        if(ui->check_auto->isChecked())
            my_auto();
        else
            runLaunch("custom_launch","my_auto.launch",false);
    });
    connect(ui->push_auto, &QPushButton::clicked, this, &Driverless::canClientParam);
}

void Driverless::function_connet(void)
{
    //录制数据
    connect(ui->check_record, &QCheckBox::stateChanged, [=] (){
        if(ui->check_record->isChecked())
        {
            QString str_cmd = "record_ros_bag.launch"+strParam("topic1", record.topic1);
            str_cmd.append(strParam("topic2", record.topic2));
            str_cmd.append(strParam("topic3", record.topic3));
            str_cmd.append(strParam("topic4", record.topic4));
            str_cmd.append(strParam("topic5", record.topic5));
            str_cmd.append(strParam("topic6", record.topic6));
            str_cmd.append(strParam("topic7", record.topic7));
            str_cmd.append(strParam("topic8", record.topic8));
            runLaunch("custom_launch/map",str_cmd,true);
        }
        else
            runLaunch("custom_launch/map","record_ros_bag.launch",false);
    });
    connect(ui->push_record, &QPushButton::clicked, this, &Driverless::recordParam);

    //制作PCD地图
    connect(ui->check_pcd, &QCheckBox::stateChanged, [=] (){
        static QString str_laun;
        if(ui->check_pcd->isChecked())
        {
            QString readFileMap = getFiles("startingup_ros", "data/Common/map", true)[0];

            std::string str_cmd1 = "rm -r " +readFileMap.toStdString();
            system(str_cmd1.c_str());//清除存在的地图

            std::string str_cmd2 = "mkdir -p " +readFileMap.toStdString() + "/pointcloud_map ";
            str_cmd2.append(readFileMap.toStdString() + "/site ");
            str_cmd2.append(readFileMap.toStdString() + "/vector_map ");
            str_cmd2.append(readFileMap.toStdString() + "/way_points");
            system(str_cmd2.c_str());//创建地图文件夹

            if(ui->comboBox_car->currentText()=="sim" | ui->comboBox_car->currentText()=="carla_sim")
                mapping.use_sim = "true";
            else
                mapping.use_sim = "false";
            if(mapping.name == "NDT")
                str_laun = "build_pcd_map.launch";
            else
                str_laun = "build_pcd_mapLio.launch";
            QString str_cmd = str_laun+strParam("use_bag", mapping.use_bag);
            str_cmd.append(strParam("use_gps", mapping.use_gps));
            str_cmd.append(strParam("use_gpu", mapping.use_gpu));
            str_cmd.append(strParam("use_sim", mapping.use_sim));
            str_cmd.append(strParam("filt", mapping.filt));
            str_cmd.append(strParam("speed", mapping.speed));
            runLaunch("custom_launch/map",str_cmd,true);
        }
        else
            runLaunch("custom_launch/map",str_laun,false);
    });
    connect(ui->push_pcd, &QPushButton::clicked, this, &Driverless::mappingParam);
    //制作轨迹点
    connect(ui->check_makePath, &QCheckBox::stateChanged, [=] (){
        if(ui->check_makePath->isChecked())
        {
            QString str_cmd = "my_save_points.launch"+strParam("init_pos_gnss", local.init_pos_gnss);
            str_cmd.append(strParam("use_gnss", local.use_gnss));
            runLaunch("custom_launch/points_nav",str_cmd,true);
        }
        else
            runLaunch("custom_launch/points_nav","my_save_points.launch",false);
    });
    connect(ui->push_makePath, &QPushButton::clicked, this, &Driverless::makePathParam);

     //路径跟踪
    connect(ui->check_pathTrack, &QCheckBox::stateChanged, [=] (){
        if(ui->check_pathTrack->isChecked())
        {
            QString str_cmd = "my_load_points.launch"+strParam("init_pos_gnss", local.init_pos_gnss);
            str_cmd.append(strParam("use_gnss", local.use_gnss));
            runLaunch("custom_launch/points_nav",str_cmd,true);
        }
        else
            runLaunch("custom_launch/points_nav","my_load_points.launch",false);
    });
    connect(ui->push_pathTrack, &QPushButton::clicked, this, &Driverless::pathTrackParam);

    //自动驾驶
    connect(ui->check_autoDrive, &QCheckBox::stateChanged, [=] (){
        QString rvizPath = getFiles("startingup_ros", "data/Common/rviz", true)[0];
        std::string str_cmd;
        if(ui->check_autoDrive->isChecked())
        {
            my_sensing();
            delay_ms(800);
            runLaunch("custom_launch","my_map.launch",true);
            delay_ms(800);
            my_localization();
            delay_ms(800);
            my_detection();
            my_mission();
            my_motion();
            // my_auto();

            str_cmd = "rviz -d " + rvizPath.toStdString() +"/default.rviz &";
            system(str_cmd.c_str());
        }
        else
        {
            system("pkill -f default.rviz &");
            runLaunch("custom_launch","my_sensing.launch",false);
            runLaunch("custom_launch","my_map.launch",false);
            runLaunch("custom_launch","my_localization.launch",false);
            runLaunch("custom_launch","my_detection.launch",false);
            runLaunch("custom_launch","my_mission_planning.launch",false);
            runLaunch("custom_launch","my_motion_planning.launch",false);
            runLaunch("custom_launch","my_auto.launch",false);
        }
    });
    connect(ui->push_autoDrive, &QPushButton::clicked, this, &Driverless::pathTrackParam);

    //车道保持  (2024.7.11)
    // connect(ui->check_laneKeep, &QCheckBox::stateChanged, [=] (){
    //     if(ui->check_laneKeep->isChecked())
    //     {
    //         QString str_cmd = "roslaunch lane_follow lane_follow_sim.launch "+strParam("img_compress", lane.img_compress);
    //         str_cmd.append(strParam("h_size", lane.h_size));
    //         str_cmd.append(strParam("h_off", lane.h_off));
    //         str_cmd.append(strParam("h_top", lane.h_top));
    //         str_cmd.append(strParam("v_size", lane.v_size));
    //         str_cmd.append(strParam("v_off", lane.v_off));
    //         str_cmd.append(strParam("speed", lane.speed));
    //         str_cmd.append(strParam("gray_threshold", lane.gray_threshold));
    //         str_cmd.append(" &");
    //         if(ui->comboBox_map->currentText()=="sim")
    //             system(str_cmd.toStdString().c_str());
    //         else
    //             system(str_cmd.toStdString().c_str());
    //     }
    //     else
    //     {
    //         if(ui->comboBox_map->currentText()=="sim")
    //             system("pkill -f lane_follow_sim.launch &");
    //         else
    //             system("pkill -f lane_follow.launch &");
    //     }
    // });
    // connect(ui->push_laneKeep, &QPushButton::clicked, this, &Driverless::laneParam);
    //设置目标点
    connect(ui->check_setGoal, &QCheckBox::stateChanged, [=] (){
        if(ui->check_setGoal->isChecked())
            system("roslaunch app app.launch &");
        else
            system("pkill -f app.launch &");
    });
    //删除地图
    connect(ui->check_deleMap, &QCheckBox::stateChanged, [=] (){
        if(ui->check_deleMap->isChecked())
            deleMapParam();
    });
}

void Driverless::rosTool_connet(void)
{
    //rviz工具
    connect(ui->check_rviz, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        QString filePath = getFiles("startingup_ros", "data/Common/rviz", true)[0];
        if(ui->check_rviz->isChecked())
            str_cmd = "rviz -d " + filePath.toStdString() +"/default.rviz &";
        else
            str_cmd = "pkill -f default.rviz &";

        system(str_cmd.c_str());
    });

    //rviz工具
    connect(ui->check_rviz_1, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        QString filePath = getFiles("startingup_ros", "data/Common/rviz", true)[0];
        if(ui->check_rviz_1->isChecked())
            str_cmd = "rviz -d " + filePath.toStdString() +"/default.rviz &";
        else
            str_cmd = "pkill -f default.rviz &";

        system(str_cmd.c_str());
    });
    //roscore
    connect(ui->check_roscore, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        if(ui->check_roscore->isChecked())
            str_cmd = "roscore &";
        else
            str_cmd = "pkill -f ros &";

        system(str_cmd.c_str());
    });
    //rqt_plot
    connect(ui->check_plot, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        if(ui->check_plot->isChecked())
            str_cmd = "rqt_plot &";
        else
            str_cmd = "pkill -f rqt_plot &";

        system(str_cmd.c_str());
    });
    //rqt
    connect(ui->check_rqt, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        if(ui->check_rqt->isChecked())
            str_cmd = "rqt &";
        else
            str_cmd = "pkill -f rqt &";

        system(str_cmd.c_str());
    });
    //kill ROS
    connect(ui->check_killros, &QCheckBox::stateChanged, [=] (){
        std::string str_cmd;
        if(ui->check_killros->isChecked())
            str_cmd = "pkill -f ros &";
        else
            str_cmd = "pkill -f ros &";

        system(str_cmd.c_str());
    });
    //命令执行
    connect(ui->pushButton_cmd, &QPushButton::clicked, [=] (){
        std::string str_cmd;
        str_cmd = ui->lineEdit->text().toStdString() + " &";
        system(str_cmd.c_str());
    });
}

void Driverless::Choose_Vehicle(){
    std::string str_cmd_1;
    qDebug()<<"选择车辆:"<< ui->comboBox_car->currentText();
    QString saveFileVeh = getFiles("startingup_ros", "data/Common", true)[0];
    QString readFileVeh = getFiles("startingup_ros", "data/Vehicle", true)[0] + "/" + ui->comboBox_car->currentText();
    //判断指定文件是否存在.也可以判断在指定路径下是否有文件和你将要保存的文件重名.
    if(QFile::exists(readFileVeh))
    {
        // qDebug()<<"readFileVeh   "<<readFileVeh;
        // qDebug()<<"saveFileVeh   "<<saveFileVeh;
        //文件拷贝
        if(copyDirectory(readFileVeh, saveFileVeh, true))
        {
            //自适应大小
            int fontsize = ui->comboBox_car->font().pointSize();//获取字体的磅值
            int wid = ui->comboBox_car->currentText().length();
            if(wid<6)
                wid = 6;
            ui->comboBox_car->setFixedWidth(fontsize*wid);
            // 删除 .bashrc 中的旧变量定义
            system("sed -i '/export AUTOCAR_VEHICLE_TYPE=/d' ~/.bashrc");
            QString str_cmd = "echo export AUTOCAR_VEHICLE_TYPE=" + ui->comboBox_car->currentText() + " >> ~/.bashrc";
            system(str_cmd.toStdString().c_str());
            // 在当前进程中设置新的环境变量
            qputenv("AUTOCAR_VEHICLE_TYPE", ui->comboBox_car->currentText().toLatin1());
            // 打印以确认环境变量已更新
            qDebug() << "AUTOCAR_VEHICLE_TYPE set to:" << qgetenv("AUTOCAR_VEHICLE_TYPE");
            // 重新启动应用程序
            QString program = QCoreApplication::applicationFilePath();
            QStringList arguments = QCoreApplication::arguments();
            QProcess::startDetached(program, arguments);
            QCoreApplication::exit(0); // 退出当前进程
            str_cmd_1 = "source ~/.bashrc &";
            system(str_cmd_1.c_str());
        }
        else
            QMessageBox::warning(NULL, "失败", "选择车辆失败！", QMessageBox::Yes , QMessageBox::Yes);
    }
    else
        QMessageBox::warning(NULL, "失败", "选择车辆不存在！", QMessageBox::Yes , QMessageBox::Yes);
    readParam();
}

void Driverless::Choose_Map(){
    std::string str_cmd_1;
    qDebug()<<"选择地图:"<< ui->comboBox_map->currentText();
    QString saveFileMap = getFiles("startingup_ros", "data/Common", true)[0];
    QString readFileMap = getFiles("startingup_ros", "data/Map", true)[0] + "/" + ui->comboBox_map->currentText();
    //判断指定文件是否存在.也可以判断在指定路径下是否有文件和你将要保存的文件重名.
    if(QFile::exists(readFileMap))
    {
        //自适应大小
        int fontsize = ui->comboBox_map->font().pointSize();//获取字体的磅值
        int wid = ui->comboBox_map->currentText().length();
        if(wid<6)
            wid = 6;
        ui->comboBox_map->setFixedWidth(fontsize*wid);

        std::string str_cmd = "rm -r "+saveFileMap.toStdString()+"/map";
        system(str_cmd.c_str());//删除地图再保存
        delay_ms(100);//创建定时器延时100毫秒，等待文件彻底删除
        //文件拷贝
        /*
        if(copyDirectory(readFileMap, saveFileMap, true))
        {
            system("sed -i '/export AUTOCAR_MAP_TYPE=/d' ~/.bashrc");
            QString str_cmd = "echo export AUTOCAR_MAP_TYPE=" + ui->comboBox_map->currentText() + " >> ~/.bashrc";
            system(str_cmd.toStdString().c_str());
            QString str_cmd2 = "export AUTOCAR_MAP_TYPE=" + ui->comboBox_map->currentText();
            system(str_cmd2.toStdString().c_str());
            system("source ~/.bashrc");
        }
        */
        if(copyDirectory(readFileMap, saveFileMap, true))
	    {
		// 删除 .bashrc 中的旧变量定义
            system("sed -i '/export AUTOCAR_MAP_TYPE=/d' ~/.bashrc");	
            // 将新的环境变量写入 .bashrc
            QString str_cmd = "echo export AUTOCAR_MAP_TYPE=" + ui->comboBox_map->currentText() + " >> ~/.bashrc";
            system(str_cmd.toStdString().c_str());
            // 在当前进程中设置新的环境变量
            qputenv("AUTOCAR_MAP_TYPE", ui->comboBox_map->currentText().toLatin1());
            // 打印以确认环境变量已更新
            qDebug() << "AUTOCAR_MAP_TYPE set to:" << qgetenv("AUTOCAR_MAP_TYPE");
            // 重新启动应用程序
            QString program = QCoreApplication::applicationFilePath();
            QStringList arguments = QCoreApplication::arguments();
            QProcess::startDetached(program, arguments);
            QCoreApplication::exit(0); // 退出当前进程
            str_cmd_1 = "source ~/.bashrc &";
            system(str_cmd_1.c_str());
        }
        else
            QMessageBox::warning(NULL, "失败", "选择地图失败！", QMessageBox::Yes , QMessageBox::Yes);
    }
    else
        QMessageBox::warning(NULL, "失败", "选择地图不存在！", QMessageBox::Yes , QMessageBox::Yes);

}

void Driverless::closeEvent(QCloseEvent *event)
{
    qDebug()<<"关闭所有ros程序........";

    system("pkill -f ros &");//kill broscore
    system("pkill -f driverless_ui &");//kill broscore
}




//获取ROS包路径子文件
QStringList Driverless::getFiles(const QString& bagName, const QString& dirName, bool path_en)
{
    char outBuffer[200] ;
    QString str = "realpath $(rospack find "+bagName + ")/"+dirName;
    FILE *FileName = popen(str.toStdString().c_str(), "r" );   //获取文件路径
    if(!FileName){  // 检测流管道
        perror("Fail to popen\n");
        // return 0;
    }
    while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
    }
    pclose(FileName);
    QString str_name  = QString::fromStdString(outBuffer);
    str_name  =   str_name.simplified();//去掉换行符号
    // qDebug()<<str_name;
    if(path_en)
    {
        QStringList path;
        path << str_name;
        return path;
    }

	// 获取所有文件夹名
    QString mFolderPath = str_name;
	QDir dir(mFolderPath);
    QStringList mImgNames;
	mFolderPath = dir.fromNativeSeparators(mFolderPath);//  "\\"转为"/"
	if (!dir.exists()) mImgNames = QStringList("");
	dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
	dir.setSorting(QDir::Name);
	mImgNames = dir.entryList();
    return mImgNames;
}

//文件复制函数
bool Driverless::copyDirectory(const QString& srcPath, const QString& dstPath, bool coverFileIfExist)
{
    QDir srcDir(srcPath);
    QDir dstDir(dstPath);
    if (!dstDir.exists()) { //目的文件目录不存在则创建文件目录
        if (!dstDir.mkdir(dstDir.absolutePath()))
            return false;
    }
    QFileInfoList fileInfoList = srcDir.entryInfoList();
    foreach(QFileInfo fileInfo, fileInfoList) {
        if (fileInfo.fileName() == "." || fileInfo.fileName() == "..")
            continue;
        if (fileInfo.isDir()) {    // 当为目录时，递归的进行copy
    //         // if (coverFileIfExist && dstDir.exists(dstDir.filePath(fileInfo.fileName()))) {
    //         //     QDir qDir(dstDir.filePath(fileInfo.fileName()));  //删除目录
	//         //     qDir.removeRecursively();
    //         // }
            if (!copyDirectory(fileInfo.filePath(),dstDir.filePath(fileInfo.fileName()),coverFileIfExist))
                return false;
        }
        else {            //当允许覆盖操作时，将旧文件进行删除操作 
            if (coverFileIfExist && dstDir.exists(fileInfo.fileName())) {
                dstDir.remove(fileInfo.fileName());
            }
            /// 进行文件copy
            if (!QFile::copy(fileInfo.filePath(), dstDir.filePath(fileInfo.fileName()))) {
                return false;
            }
        }
    }
    return true;
}

void Driverless::runLaunch(QString path, QString launch, bool en)
{
    std::string str_cmd;
    QString bagPath = getFiles("startingup_ros", "data/Common", true)[0]+"/"+path+"/"+launch;
    if(en)
        str_cmd = "roslaunch " +bagPath.toStdString() +" &";
    else
        str_cmd = "pkill -f " + launch.toStdString() +" &";

    system(str_cmd.c_str());
    qDebug()<<"bagPath:  "<<bagPath;
}

QString Driverless::writeYaml(QString module, QString func, QString param, QString data)
{
    QString filePath = getFiles("startingup_ros", "data/Common/config", true)[0]+"/config.yaml";
    YAML::Node config = YAML::LoadFile(filePath.toStdString());
    ofstream outF(filePath.toStdString()); //保存
    config[module.toStdString()][func.toStdString()][param.toStdString()] = data.toStdString();
    outF << config;
    outF.close();
    return data;
}

QString Driverless::readYaml(QString module, QString func, QString param)
{
    QString filePath = getFiles("startingup_ros", "data/Common/config", true)[0]+"/config.yaml";
    QFileInfo fileInfo(filePath);
    if(!fileInfo.isFile())
    {
        QMessageBox::information(NULL, "文件失败",
            "config配置文件不存在！",
            QMessageBox::Yes);
        YAML::Node config = YAML::LoadFile(filePath.toStdString());
        return "";
    }
    QString readData;
    YAML::Node config = YAML::LoadFile(filePath.toStdString());
    if(config[module.toStdString()][func.toStdString()][param.toStdString()]){
        readData = QString::fromStdString(config[module.toStdString()][func.toStdString()][param.toStdString()].as<string>());
        qDebug() << "readData: " << readData;
    }
    else
    {
        QMessageBox::information(NULL, "参数失败",
                        module+"-"+func+"-"+param+"参数错误！",
                        QMessageBox::Yes);
                        return "";
    }
    return readData;
}

void Driverless::sensorParam(void)
{
    QDialog *dia_sensor = new QDialog(this);
    dia_sensor->setWindowTitle(QObject::tr("传感器参数配置"));
    dia_sensor->resize(300,100);
    QLabel *label_1 = new QLabel("惯导波特率:");
    // QSpinBox *spinBox_1 = new QSpinBox();
    // spinBox_1->setMaximum(999999);
    QComboBox *comboBox_1 = new QComboBox();
    comboBox_1->addItem("230400");
    comboBox_1->addItem("460800");

    if(sensor.baud == "460800")
        comboBox_1->setCurrentIndex(1);
    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_1);
    layout_1->addWidget(comboBox_1);
    dia_sensor->setLayout(layout_1);

    connect(comboBox_1,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](){
        sensor.baud = comboBox_1->currentText();
        writeYaml("Sensor", "navigate", "baud", sensor.baud);
    });
    dia_sensor->show();
}

//地图参数配置
void Driverless::mapParam(void)
{
    QDialog *dia_map = new QDialog(this);
    dia_map->setWindowTitle(QObject::tr("地图参数配置"));
    dia_map->resize(420,200);
    QLabel *label_1 = new QLabel("点云地图:");
    static QLineEdit *lineEdit_1 = new QLineEdit();
    QToolButton *button_1 = new QToolButton();
    button_1->setText("...");
    static QCheckBox *check_1 = new QCheckBox();

    QLabel *label_2 = new QLabel("高精地图:");
    static QLineEdit *lineEdit_2 = new QLineEdit();
    QToolButton *button_2 = new QToolButton();
    button_2->setText("...");
    static QCheckBox *check_2 = new QCheckBox();

    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_1);
    layout_1->addWidget(lineEdit_1);
    layout_1->addWidget(button_1);
    layout_1->addWidget(check_1);

    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_2);
    layout_2->addWidget(lineEdit_2);
    layout_2->addWidget(button_2);
    layout_2->addWidget(check_2);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(layout_1);
    layout->addLayout(layout_2);
    dia_map->setLayout(layout);

    QString readFileMap = getFiles("startingup_ros", "data/Map", true)[0];
    connect(button_1, &QToolButton::clicked, [=] (){
        QString path_pcd = QFileDialog::getExistingDirectory(this, "请选择文件夹路径...", readFileMap);
        if(path_pcd!="null")
            lineEdit_1->setText(path_pcd);
    });
    connect(button_2, &QToolButton::clicked, [=] (){
        QString map_dir = QFileDialog::getExistingDirectory(this, "请选择文件夹路径...", readFileMap);
        if(map_dir!="null")
            lineEdit_2->setText(map_dir);

    });
    connect(check_1, &QCheckBox::stateChanged, [=] (){
        if(check_1->isChecked())
            map.path_pcd = lineEdit_1->text();
        else
            map.path_pcd = "null";


    });
    connect(check_2, &QCheckBox::stateChanged, [=] (){
        if(check_2->isChecked())
            map.map_dir = lineEdit_2->text();
        else
            map.map_dir = "null";
    });
    dia_map->show();
}


//定位参数配置
void Driverless::mapLocalParam(void)
{
    QDialog *dia_mapLocal = new QDialog(this);
    dia_mapLocal->setWindowTitle(QObject::tr("定位参数配置"));
    dia_mapLocal->resize(300,200);

    QCheckBox *check_localInit = new QCheckBox("保存GNSS初始位置");

    QCheckBox *check_height = new QCheckBox("获取高度");
    QCheckBox *check_imu = new QCheckBox("使用IMU定位");
    QCheckBox *check_gnss = new QCheckBox("使用GNSS定位");
    QCheckBox *check_init_pos = new QCheckBox("初始化GNSS位置");
    QVBoxLayout *layout_1 = new QVBoxLayout;
    layout_1->addWidget(check_localInit);
    layout_1->addWidget(check_init_pos);
    layout_1->addWidget(check_height);
    layout_1->addWidget(check_imu);
    layout_1->addWidget(check_gnss);
    dia_mapLocal->setLayout(layout_1);

    check_localInit->setChecked(local.get_gnss_init_pose.toInt());
    connect(check_localInit, &QCheckBox::stateChanged, [=] (){
        local.get_gnss_init_pose = QString::number(check_localInit->isChecked());
        writeYaml("Local", "gnss", "get_gnss_init_pose", local.get_gnss_init_pose);
    });
    check_height->setChecked(local.get_height.toInt());
    connect(check_height, &QCheckBox::stateChanged, [=] (){
        local.get_height = QString::number(check_height->isChecked());
        writeYaml("Local", "matching", "get_height", local.get_height);
    });
    check_imu->setChecked(local.use_imu.toInt());
    connect(check_imu, &QCheckBox::stateChanged, [=] (){
        local.use_imu = QString::number(check_imu->isChecked());
        writeYaml("Local", "matching", "use_imu", local.use_imu);
    });
    check_gnss->setChecked(local.use_gnss.toInt());
    connect(check_gnss, &QCheckBox::stateChanged, [=] (){
        local.use_gnss = QString::number(check_gnss->isChecked());
        writeYaml("Local", "matching", "use_gnss", local.use_gnss);
    });
    check_init_pos->setChecked(local.init_pos_gnss.toInt());
    connect(check_init_pos, &QCheckBox::stateChanged, [=] (){
        local.init_pos_gnss = QString::number(check_init_pos->isChecked());
        writeYaml("Local", "matching", "init_pos_gnss", local.init_pos_gnss);
    });


    dia_mapLocal->show();
}

//感知参数配置
void Driverless::detectionParam(void)
{
    QDialog *dia_detection = new QDialog(this);
    dia_detection->setWindowTitle(QObject::tr("感知参数配置"));
    dia_detection->resize(600,200);

    QTabWidget *m_tabWidget = new QTabWidget(dia_detection);
    QWidget* widget1 = new QWidget(m_tabWidget);
    QWidget* widget2 = new QWidget(m_tabWidget);
    QWidget* widget3 = new QWidget(m_tabWidget);
    QWidget* widget4 = new QWidget(m_tabWidget);
    m_tabWidget->addTab(widget1, "安全");
    m_tabWidget->addTab(widget2, "聚类");
    m_tabWidget->addTab(widget3, "过滤");
    m_tabWidget->addTab(widget4, "KF跟踪");

    QLabel *label_2 = new QLabel("雷达到车头的距离:");
    QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
    QLabel *label_3 = new QLabel("雷达到车尾的距离:");
    QDoubleSpinBox *spinBox_3 = new QDoubleSpinBox();
    spinBox_3->setMinimum(-99);
    QLabel *label_4 = new QLabel("雷达到车左侧的距离:");
    QDoubleSpinBox *spinBox_4 = new QDoubleSpinBox();
    QLabel *label_5 = new QLabel("雷达到车右侧的距离:");
    QDoubleSpinBox *spinBox_5 = new QDoubleSpinBox();
    spinBox_5->setMinimum(-99);
    QLabel *label_6 = new QLabel("前方障碍物最小安全距离:");
    QDoubleSpinBox *spinBox_6 = new QDoubleSpinBox();
    QLabel *label_7 = new QLabel("后方障碍物最小安全距离:");
    QDoubleSpinBox *spinBox_7 = new QDoubleSpinBox();
    QLabel *label_8 = new QLabel("左方障碍物最小安全距离:");
    QDoubleSpinBox *spinBox_8 = new QDoubleSpinBox();
    QLabel *label_9 = new QLabel("右方障碍物最小安全距离:");
    QDoubleSpinBox *spinBox_9 = new QDoubleSpinBox();
    QLabel *label_10 = new QLabel("前方障碍物安全速度系数:");
    QDoubleSpinBox *spinBox_10 = new QDoubleSpinBox();
    QLabel *label_11 = new QLabel("后方障碍物安全速度系数:");
    QDoubleSpinBox *spinBox_11 = new QDoubleSpinBox();
    QLabel *label_12 = new QLabel("左方障碍物安全速度系数:");
    QDoubleSpinBox *spinBox_12 = new QDoubleSpinBox();
    QLabel *label_13 = new QLabel("右方障碍物安全速度系数:");
    QDoubleSpinBox *spinBox_13 = new QDoubleSpinBox();

    QPushButton *button_1 = new QPushButton("确认");

    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_2);
    layout_1->addWidget(spinBox_2);
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_3);
    layout_2->addWidget(spinBox_3);
    QHBoxLayout *layout_3 = new QHBoxLayout;
    layout_3->addWidget(label_4);
    layout_3->addWidget(spinBox_4);
    QHBoxLayout *layout_4 = new QHBoxLayout;
    layout_4->addWidget(label_5);
    layout_4->addWidget(spinBox_5);
    QHBoxLayout *layout_5 = new QHBoxLayout;
    layout_5->addWidget(label_6);
    layout_5->addWidget(spinBox_6);
    QHBoxLayout *layout_6 = new QHBoxLayout;
    layout_6->addWidget(label_7);
    layout_6->addWidget(spinBox_7);
    QHBoxLayout *layout_7 = new QHBoxLayout;
    layout_7->addWidget(label_8);
    layout_7->addWidget(spinBox_8);
    QHBoxLayout *layout_8 = new QHBoxLayout;
    layout_8->addWidget(label_9);
    layout_8->addWidget(spinBox_9);
    QHBoxLayout *layout_9 = new QHBoxLayout;
    layout_9->addWidget(label_10);
    layout_9->addWidget(spinBox_10);
    QHBoxLayout *layout_10 = new QHBoxLayout;
    layout_10->addWidget(label_11);
    layout_10->addWidget(spinBox_11);
    QHBoxLayout *layout_11 = new QHBoxLayout;
    layout_11->addWidget(label_12);
    layout_11->addWidget(spinBox_12);
    QHBoxLayout *layout_12 = new QHBoxLayout;
    layout_12->addWidget(label_13);
    layout_12->addWidget(spinBox_13);

    QVBoxLayout *layout1 = new QVBoxLayout;
    layout1->addLayout(layout_1);
    layout1->addLayout(layout_2);
    layout1->addLayout(layout_3);
    layout1->addLayout(layout_4);
    layout1->addLayout(layout_5);
    layout1->addLayout(layout_6);
    layout1->addLayout(layout_7);
    layout1->addLayout(layout_8);
    layout1->addLayout(layout_9);
    layout1->addLayout(layout_10);
    layout1->addLayout(layout_11);
    layout1->addLayout(layout_12);
    layout1->addWidget(button_1);

    QLabel *label_14 = new QLabel("集群有效的最小点数:");
    QSpinBox *spinBox_14 = new QSpinBox();
    QLabel *label_15 = new QLabel("集群有效的最大点数:");
    QSpinBox *spinBox_15 = new QSpinBox();
    spinBox_15->setMaximum(999999);
    QLabel *label_16 = new QLabel("雷达到车后方的有效距离:");
    QDoubleSpinBox *spinBox_16 = new QDoubleSpinBox();
    spinBox_16->setMinimum(-99);
    QLabel *label_17 = new QLabel("雷达有效探测范围:");
    QDoubleSpinBox *spinBox_17 = new QDoubleSpinBox();
    spinBox_17->setMaximum(9999);
    QCheckBox *check_gpu = new QCheckBox("使用GPU");
    QPushButton *button_2 = new QPushButton("确认");

    QHBoxLayout *layout_13 = new QHBoxLayout;
    layout_13->addWidget(label_14);
    layout_13->addWidget(spinBox_14);
    QHBoxLayout *layout_14 = new QHBoxLayout;
    layout_14->addWidget(label_15);
    layout_14->addWidget(spinBox_15);
    QHBoxLayout *layout_15 = new QHBoxLayout;
    layout_15->addWidget(label_16);
    layout_15->addWidget(spinBox_16);
    QHBoxLayout *layout_16 = new QHBoxLayout;
    layout_16->addWidget(label_17);
    layout_16->addWidget(spinBox_17);

    QVBoxLayout *layout2 = new QVBoxLayout;
    layout2->addWidget(check_gpu);
    layout2->addLayout(layout_13);
    layout2->addLayout(layout_14);
    layout2->addLayout(layout_15);
    layout2->addLayout(layout_16);
    layout2->addWidget(button_2);

    QLabel *label_18 = new QLabel("删除高于此高度值的点:");
    QDoubleSpinBox *spinBox_18 = new QDoubleSpinBox();
    QLabel *label_19 = new QLabel("移除距离传感器原点比这个距离更近的点:");
    QDoubleSpinBox *spinBox_19 = new QDoubleSpinBox();
    QPushButton *button_3 = new QPushButton("确认");

    QHBoxLayout *layout_17 = new QHBoxLayout;
    layout_17->addWidget(label_18);
    layout_17->addWidget(spinBox_18);
    QHBoxLayout *layout_18 = new QHBoxLayout;
    layout_18->addWidget(label_19);
    layout_18->addWidget(spinBox_19);

    QVBoxLayout *layout3 = new QVBoxLayout;
    layout3->addLayout(layout_17);
    layout3->addLayout(layout_18);
    layout3->addWidget(button_3);

    QLabel *label_20 = new QLabel("车辆宽度:");
    QDoubleSpinBox *spinBox_20 = new QDoubleSpinBox();
    QLabel *label_21 = new QLabel("车辆长度:");
    QDoubleSpinBox *spinBox_21 = new QDoubleSpinBox();
    QCheckBox *check_vector_filter = new QCheckBox("使用高精地图");
    QPushButton *button_4 = new QPushButton("确认");

    QHBoxLayout *layout_19 = new QHBoxLayout;
    layout_19->addWidget(label_20);
    layout_19->addWidget(spinBox_20);
    QHBoxLayout *layout_20 = new QHBoxLayout;
    layout_20->addWidget(label_21);
    layout_20->addWidget(spinBox_21);

    QVBoxLayout *layout4 = new QVBoxLayout;
    layout4->addLayout(layout_19);
    layout4->addLayout(layout_20);
    layout4->addWidget(check_vector_filter);
    layout4->addWidget(button_4);


    widget1->setLayout(layout1);
    widget2->setLayout(layout2);
    widget3->setLayout(layout3);
    widget4->setLayout(layout4);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(m_tabWidget);
    dia_detection->setLayout(mainLayout);


    spinBox_2->setValue(detect.lidar_front_car_range.toDouble());
    spinBox_3->setValue(detect.lidar_back_car_range.toDouble());
    spinBox_4->setValue(detect.lidar_left_car_range.toDouble());
    spinBox_5->setValue(detect.lidar_right_car_range.toDouble());
    spinBox_6->setValue(detect.front_safe_distance.toDouble());
    spinBox_7->setValue(detect.lear_safe_distance.toDouble());
    spinBox_8->setValue(detect.left_safe_distance.toDouble());
    spinBox_9->setValue(detect.right_safe_distance.toDouble());
    spinBox_10->setValue(detect.front_velocity_coefficient.toDouble());
    spinBox_11->setValue(detect.lear_velocity_coefficient.toDouble());
    spinBox_12->setValue(detect.left_velocity_coefficient.toDouble());
    spinBox_13->setValue(detect.right_velocity_coefficient.toDouble());

    check_gpu->setChecked(boolTo(detect.use_gpu));
    spinBox_14->setValue(detect.cluster_size_min.toInt());
    spinBox_15->setValue(detect.cluster_size_max.toInt());
    spinBox_16->setValue(detect.lidar_back_range.toDouble());
    spinBox_17->setValue(detect.lidar_total_range.toDouble());

    spinBox_18->setValue(detect.clipping_height.toDouble());
    spinBox_19->setValue(detect.min_point_distance.toDouble());

    spinBox_20->setValue(detect.vehicle_width.toDouble());
    spinBox_21->setValue(detect.vehicle_length.toDouble());
    check_vector_filter->setChecked(detect.vector_map_filter_distance.toInt());
    connect(button_1, &QPushButton::clicked, [=] (){
        detect.lidar_front_car_range = QString::number(spinBox_2->value());
        detect.lidar_back_car_range = QString::number(spinBox_3->value());
        detect.lidar_left_car_range = QString::number(spinBox_4->value());
        detect.lidar_right_car_range = QString::number(spinBox_5->value());
        detect.front_safe_distance = QString::number(spinBox_6->value());
        detect.lear_safe_distance = QString::number(spinBox_7->value());
        detect.left_safe_distance = QString::number(spinBox_8->value());
        detect.right_safe_distance = QString::number(spinBox_9->value());
        detect.front_velocity_coefficient = QString::number(spinBox_10->value());
        detect.lear_velocity_coefficient = QString::number(spinBox_11->value());
        detect.left_velocity_coefficient = QString::number(spinBox_12->value());
        detect.right_velocity_coefficient = QString::number(spinBox_13->value());

        writeYaml("Detection", "safeRange", "lidar_front_car_range", detect.lidar_front_car_range);
        writeYaml("Detection", "safeRange", "lidar_back_car_range", detect.lidar_back_car_range);
        writeYaml("Detection", "safeRange", "lidar_left_car_range", detect.lidar_left_car_range);
        writeYaml("Detection", "safeRange", "lidar_right_car_range", detect.lidar_right_car_range);
        writeYaml("Detection", "safeRange", "front_safe_distance", detect.front_safe_distance);
        writeYaml("Detection", "safeRange", "lear_safe_distance", detect.lear_safe_distance);
        writeYaml("Detection", "safeRange", "left_safe_distance", detect.left_safe_distance);
        writeYaml("Detection", "safeRange", "right_safe_distance", detect.right_safe_distance);
        writeYaml("Detection", "safeRange", "front_velocity_coefficient", detect.front_velocity_coefficient);
        writeYaml("Detection", "safeRange", "lear_velocity_coefficient", detect.lear_velocity_coefficient);
        writeYaml("Detection", "safeRange", "left_velocity_coefficient", detect.left_velocity_coefficient);
        writeYaml("Detection", "safeRange", "right_velocity_coefficient", detect.right_velocity_coefficient);

    });
    connect(button_2, &QPushButton::clicked, [=] (){
        detect.use_gpu = toBool(QString::number(check_gpu->isChecked()));
        detect.cluster_size_min = QString::number(spinBox_14->value());
        detect.cluster_size_max = QString::number(spinBox_15->value());
        detect.lidar_back_range = QString::number(spinBox_16->value());
        detect.lidar_total_range = QString::number(spinBox_17->value());
        writeYaml("Detection", "Cluster", "use_gpu", detect.use_gpu);
        writeYaml("Detection", "Cluster", "cluster_size_min", detect.cluster_size_min);
        writeYaml("Detection", "Cluster", "cluster_size_max", detect.cluster_size_max);
        writeYaml("Detection", "Cluster", "lidar_back_range", detect.lidar_back_range);
        writeYaml("Detection", "Cluster", "lidar_total_range", detect.lidar_total_range);
    });
    connect(button_3, &QPushButton::clicked, [=] (){
        detect.clipping_height = QString::number(spinBox_18->value());
        detect.min_point_distance = QString::number(spinBox_19->value());
        writeYaml("Detection", "Filter", "clipping_height", detect.clipping_height);
        writeYaml("Detection", "Filter", "min_point_distance", detect.min_point_distance);
    });
    connect(button_4, &QPushButton::clicked, [=] (){
        detect.vehicle_width = QString::number(spinBox_20->value());
        detect.vehicle_length = QString::number(spinBox_21->value());
        detect.vector_map_filter_distance = QString::number(check_vector_filter->isChecked());
        writeYaml("Detection", "ContourTrack", "vehicle_width", detect.vehicle_width);
        writeYaml("Detection", "ContourTrack", "vehicle_length", detect.vehicle_length);
        writeYaml("Detection", "ContourTrack", "vector_map_filter_distance", detect.vector_map_filter_distance);
    });
    dia_detection->show();

}

//规划参数配置
void Driverless::planningParam(void)
{
    QDialog *dia_detection = new QDialog(this);
    dia_detection->setWindowTitle(QObject::tr("规划参数配置"));
    dia_detection->resize(600,200);

    QTabWidget *m_tabWidget = new QTabWidget(dia_detection);
    QWidget* widget1 = new QWidget(m_tabWidget);
    QWidget* widget2 = new QWidget(m_tabWidget);

    m_tabWidget->addTab(widget1, "局部规划");
    m_tabWidget->addTab(widget2, "全局规划");

    QLabel *label_2 = new QLabel("局部规划最大距离:");
    QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
    spinBox_2->setMaximum(99999);
    QLabel *label_3 = new QLabel("多远检测障碍物:");
    QDoubleSpinBox *spinBox_3 = new QDoubleSpinBox();
    spinBox_3->setMaximum(99999);
    QLabel *label_4 = new QLabel("检测到障碍物多远开始绕行:");
    QDoubleSpinBox *spinBox_4 = new QDoubleSpinBox();
    spinBox_4->setMaximum(99999);
    QLabel *label_5 = new QLabel("车子开始启动的最小距离:");
    QDoubleSpinBox *spinBox_5 = new QDoubleSpinBox();
    QLabel *label_6 = new QLabel("局部规划线数量");
    QSpinBox *spinBox_6 = new QSpinBox();
    QLabel *label_7 = new QLabel("左右安全距离:");
    QDoubleSpinBox *spinBox_7 = new QDoubleSpinBox();
    QLabel *label_8 = new QLabel("前后安全距离:");
    QDoubleSpinBox *spinBox_8 = new QDoubleSpinBox();
    QLabel *label_9 = new QLabel("最大速度:");
    QDoubleSpinBox *spinBox_9 = new QDoubleSpinBox();
    QLabel *label_10 = new QLabel("最小速度:");
    QDoubleSpinBox *spinBox_10 = new QDoubleSpinBox();
    QLabel *label_11 = new QLabel("最大加速度:");
    QDoubleSpinBox *spinBox_11 = new QDoubleSpinBox();
    QLabel *label_12 = new QLabel("最大减速度:");
    QDoubleSpinBox *spinBox_12 = new QDoubleSpinBox();
    spinBox_12->setMinimum(-99);
    QLabel *label_13 = new QLabel("车辆宽度:");
    QDoubleSpinBox *spinBox_13 = new QDoubleSpinBox();
    QLabel *label_14 = new QLabel("车辆长度:");
    QDoubleSpinBox *spinBox_14 = new QDoubleSpinBox();
    QLabel *label_15 = new QLabel("轴距:");
    QDoubleSpinBox *spinBox_15 = new QDoubleSpinBox();
    QLabel *label_16 = new QLabel("最小转为半径:");
    QDoubleSpinBox *spinBox_16 = new QDoubleSpinBox();
    QLabel *label_17 = new QLabel("最大转角:");
    QDoubleSpinBox *spinBox_17 = new QDoubleSpinBox();

    QPushButton *button_1 = new QPushButton("确认");

    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_2);
    layout_1->addWidget(spinBox_2);
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_3);
    layout_2->addWidget(spinBox_3);
    QHBoxLayout *layout_3 = new QHBoxLayout;
    layout_3->addWidget(label_4);
    layout_3->addWidget(spinBox_4);
    QHBoxLayout *layout_4 = new QHBoxLayout;
    layout_4->addWidget(label_5);
    layout_4->addWidget(spinBox_5);
    QHBoxLayout *layout_5 = new QHBoxLayout;
    layout_5->addWidget(label_6);
    layout_5->addWidget(spinBox_6);
    QHBoxLayout *layout_6 = new QHBoxLayout;
    layout_6->addWidget(label_7);
    layout_6->addWidget(spinBox_7);
    QHBoxLayout *layout_7 = new QHBoxLayout;
    layout_7->addWidget(label_8);
    layout_7->addWidget(spinBox_8);
    QHBoxLayout *layout_8 = new QHBoxLayout;
    layout_8->addWidget(label_9);
    layout_8->addWidget(spinBox_9);
    QHBoxLayout *layout_9 = new QHBoxLayout;
    layout_9->addWidget(label_10);
    layout_9->addWidget(spinBox_10);
    QHBoxLayout *layout_10 = new QHBoxLayout;
    layout_10->addWidget(label_11);
    layout_10->addWidget(spinBox_11);
    QHBoxLayout *layout_11 = new QHBoxLayout;
    layout_11->addWidget(label_12);
    layout_11->addWidget(spinBox_12);
    QHBoxLayout *layout_12 = new QHBoxLayout;
    layout_12->addWidget(label_13);
    layout_12->addWidget(spinBox_13);
    QHBoxLayout *layout_13 = new QHBoxLayout;
    layout_13->addWidget(label_14);
    layout_13->addWidget(spinBox_14);
    QHBoxLayout *layout_14 = new QHBoxLayout;
    layout_14->addWidget(label_15);
    layout_14->addWidget(spinBox_15);
    QHBoxLayout *layout_15 = new QHBoxLayout;
    layout_15->addWidget(label_16);
    layout_15->addWidget(spinBox_16);
    QHBoxLayout *layout_16 = new QHBoxLayout;
    layout_16->addWidget(label_17);
    layout_16->addWidget(spinBox_17);


    QVBoxLayout *layout1 = new QVBoxLayout;
    layout1->addLayout(layout_1);
    layout1->addLayout(layout_2);
    layout1->addLayout(layout_3);
    layout1->addLayout(layout_4);
    layout1->addLayout(layout_5);
    layout1->addLayout(layout_6);
    layout1->addLayout(layout_7);
    layout1->addLayout(layout_8);
    layout1->addLayout(layout_9);
    layout1->addLayout(layout_10);
    layout1->addLayout(layout_11);
    layout1->addLayout(layout_12);
    layout1->addLayout(layout_13);
    layout1->addLayout(layout_14);
    layout1->addLayout(layout_15);
    layout1->addLayout(layout_16);
    layout1->addWidget(button_1);

    QCheckBox *check_laneChange = new QCheckBox("变道");
    QCheckBox *check_replan = new QCheckBox("多点规划");
    QCheckBox *check_smoothing = new QCheckBox("规划平滑");
    QCheckBox *check_rvizInput = new QCheckBox("RvizInput");
    QCheckBox *check_onlyReplan = new QCheckBox("循环规划");
    QLabel *label_18 = new QLabel("航点相隔距离:");
    QSpinBox *spinBox_18 = new QSpinBox();


    QPushButton *button_2 = new QPushButton("确认");

    QHBoxLayout *layout_19 = new QHBoxLayout;
    layout_19->addWidget(label_18);
    layout_19->addWidget(spinBox_18);

    QVBoxLayout *layout2 = new QVBoxLayout;
    layout2->addWidget(check_laneChange);
    layout2->addWidget(check_replan);
    layout2->addWidget(check_smoothing);
    layout2->addWidget(check_rvizInput);
    layout2->addWidget(check_onlyReplan);
    layout2->addLayout(layout_19);
    layout2->addWidget(button_2);

    widget1->setLayout(layout1);
    widget2->setLayout(layout2);


    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(m_tabWidget);
    dia_detection->setLayout(mainLayout);

    spinBox_2->setValue(plan.maxLocalPlanDistance.toDouble());
    spinBox_3->setValue(plan.minFollowingDistance.toDouble());
    spinBox_4->setValue(plan.minDistanceToAvoid.toDouble());
    spinBox_5->setValue(plan.maxDistanceToAvoid.toDouble());
    spinBox_6->setValue(plan.rollOutsNumber.toInt());
    spinBox_7->setValue(plan.horizontalSafetyDistance.toDouble());
    spinBox_8->setValue(plan.verticalSafetyDistance.toDouble());
    spinBox_9->setValue(plan.maxVelocity.toDouble());
    spinBox_10->setValue(plan.minVelocity.toDouble());
    spinBox_11->setValue(plan.maxAcceleration.toDouble());
    spinBox_12->setValue(plan.maxDeceleration.toDouble());
    spinBox_13->setValue(plan.width.toDouble());
    spinBox_14->setValue(plan.length.toDouble());
    spinBox_15->setValue(plan.wheelBaseLength.toDouble());
    spinBox_16->setValue(plan.turningRadius.toDouble());
    spinBox_17->setValue(plan.maxSteerAngle.toDouble());

    check_laneChange->setChecked(boolTo(plan.enableLaneChange));
    check_replan->setChecked(boolTo(plan.enableReplan));
    check_smoothing->setChecked(boolTo(plan.enableSmoothing));
    check_rvizInput->setChecked(boolTo(plan.enableRvizInput));
    check_onlyReplan->setChecked(boolTo(plan.onlyReplan));
    spinBox_18->setValue(plan.pathDensity.toDouble());


    connect(button_1, &QPushButton::clicked, [=] (){
        plan.maxLocalPlanDistance = QString::number(spinBox_2->value());
        plan.minFollowingDistance = QString::number(spinBox_3->value());
        plan.minDistanceToAvoid = QString::number(spinBox_4->value());
        plan.maxDistanceToAvoid = QString::number(spinBox_5->value());
        plan.rollOutsNumber = QString::number(spinBox_6->value());
        plan.horizontalSafetyDistance = QString::number(spinBox_7->value());
        plan.verticalSafetyDistance = QString::number(spinBox_8->value());
        plan.maxVelocity = QString::number(spinBox_9->value());
        plan.minVelocity = QString::number(spinBox_10->value());
        plan.maxAcceleration = QString::number(spinBox_11->value());
        plan.maxDeceleration = QString::number(spinBox_12->value());
        plan.width = QString::number(spinBox_13->value());
        plan.length = QString::number(spinBox_14->value());
        plan.wheelBaseLength = QString::number(spinBox_15->value());
        plan.turningRadius = QString::number(spinBox_16->value());
        plan.maxSteerAngle = QString::number(spinBox_17->value());

        writeYaml("Planning", "op_common", "maxLocalPlanDistance", plan.maxLocalPlanDistance);
        writeYaml("Planning", "op_common", "minFollowingDistance", plan.minFollowingDistance);
        writeYaml("Planning", "op_common", "minDistanceToAvoid", plan.minDistanceToAvoid);
        writeYaml("Planning", "op_common", "maxDistanceToAvoid", plan.maxDistanceToAvoid);
        writeYaml("Planning", "op_common", "rollOutsNumber", plan.rollOutsNumber);
        writeYaml("Planning", "op_common", "horizontalSafetyDistance", plan.horizontalSafetyDistance);
        writeYaml("Planning", "op_common", "verticalSafetyDistance", plan.verticalSafetyDistance);
        writeYaml("Planning", "op_common", "maxVelocity", plan.maxVelocity);
        writeYaml("Planning", "op_common", "minVelocity", plan.minVelocity);
        writeYaml("Planning", "op_common", "maxAcceleration", plan.maxAcceleration);
        writeYaml("Planning", "op_common", "maxDeceleration", plan.maxDeceleration);
        writeYaml("Planning", "op_common", "width", plan.width);
        writeYaml("Planning", "op_common", "length", plan.length);
        writeYaml("Planning", "op_common", "wheelBaseLength", plan.wheelBaseLength);
        writeYaml("Planning", "op_common", "turningRadius", plan.turningRadius);
        writeYaml("Planning", "op_common", "maxSteerAngle", plan.maxSteerAngle);
    });
    connect(button_2, &QPushButton::clicked, [=] (){
        plan.enableLaneChange = toBool(QString::number(check_laneChange->isChecked()));
        plan.enableReplan = toBool(QString::number(check_replan->isChecked()));
        plan.enableSmoothing = toBool(QString::number(check_smoothing->isChecked()));
        plan.enableRvizInput = toBool(QString::number(check_rvizInput->isChecked()));
        plan.onlyReplan = toBool(QString::number(check_onlyReplan->isChecked()));
        plan.pathDensity = QString::number(spinBox_18->value());

        writeYaml("Planning", "op_global", "enableLaneChange", plan.enableLaneChange);
        writeYaml("Planning", "op_global", "enableReplan", plan.enableReplan);
        writeYaml("Planning", "op_global", "enableSmoothing", plan.enableSmoothing);
        writeYaml("Planning", "op_global", "enableRvizInput", plan.enableRvizInput);
        writeYaml("Planning", "op_global", "onlyReplan", plan.onlyReplan);
        writeYaml("Planning", "op_global", "pathDensity", plan.pathDensity);
    });

    dia_detection->show();

}

//控制参数配置
void Driverless::controlParam(void)
{
    QDialog *dia_control = new QDialog(this);
    dia_control->setWindowTitle(QObject::tr("控制参数配置"));
    dia_control->resize(300,200);

    QLabel *label_1 = new QLabel("前视距离:");
    QDoubleSpinBox *spinBox_1 = new QDoubleSpinBox();
    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_1);
    layout_1->addWidget(spinBox_1);

    QLabel *label_2 = new QLabel("转弯半径:");
    QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_2);
    layout_2->addWidget(spinBox_2);

    QLabel *label_3 = new QLabel("轮距:");
    QDoubleSpinBox *spinBox_3 = new QDoubleSpinBox();
    QHBoxLayout *layout_3 = new QHBoxLayout;
    layout_3->addWidget(label_3);
    layout_3->addWidget(spinBox_3);

    QLabel *label_4 = new QLabel("横向加速度限制:");
    QDoubleSpinBox *spinBox_4 = new QDoubleSpinBox();
    QHBoxLayout *layout_4 = new QHBoxLayout;
    layout_4->addWidget(label_4);
    layout_4->addWidget(spinBox_4);

    QLabel *label_5 = new QLabel("横向冲击极限:");
    QDoubleSpinBox *spinBox_5 = new QDoubleSpinBox();
    QHBoxLayout *layout_5 = new QHBoxLayout;
    layout_5->addWidget(label_5);
    layout_5->addWidget(spinBox_5);

    QLabel *label_6 = new QLabel("低筒增益线性x:");
    QDoubleSpinBox *spinBox_6 = new QDoubleSpinBox();
    QHBoxLayout *layout_6 = new QHBoxLayout;
    layout_6->addWidget(label_6);
    layout_6->addWidget(spinBox_6);

    QLabel *label_7 = new QLabel("低通增益角z:");
    QDoubleSpinBox *spinBox_7 = new QDoubleSpinBox();
    QHBoxLayout *layout_7 = new QHBoxLayout;
    layout_7->addWidget(label_7);
    layout_7->addWidget(spinBox_7);

    QLabel *label_8 = new QLabel("低通转向角:");
    QDoubleSpinBox *spinBox_8 = new QDoubleSpinBox();
    QHBoxLayout *layout_8 = new QHBoxLayout;
    layout_8->addWidget(label_8);
    layout_8->addWidget(spinBox_8);

    QPushButton *button_1 = new QPushButton("确认");

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(layout_1);
    mainLayout->addLayout(layout_2);
    mainLayout->addLayout(layout_3);
    mainLayout->addLayout(layout_4);
    mainLayout->addLayout(layout_5);
    mainLayout->addLayout(layout_6);
    mainLayout->addLayout(layout_7);
    mainLayout->addLayout(layout_8);
    mainLayout->addWidget(button_1);
    dia_control->setLayout(mainLayout);

    spinBox_1->setValue(control.minimum_lookahead_distance.toDouble());
    spinBox_2->setValue(control.lookahead_ratio.toDouble());
    spinBox_3->setValue(control.wheel_base.toDouble());
    spinBox_4->setValue(control.lateral_accel_limit.toDouble());
    spinBox_5->setValue(control.lateral_jerk_limit.toDouble());
    spinBox_6->setValue(control.lowpass_gain_linear_x.toDouble());
    spinBox_7->setValue(control.lowpass_gain_angular_z.toDouble());
    spinBox_8->setValue(control.lowpass_gain_steering_angle.toDouble());
    connect(button_1, &QPushButton::clicked, [=] (){
        control.minimum_lookahead_distance = QString::number(spinBox_1->value());
        control.lookahead_ratio = QString::number(spinBox_2->value());
        control.wheel_base = QString::number(spinBox_3->value());
        control.lateral_accel_limit = QString::number(spinBox_4->value());
        control.lateral_jerk_limit = QString::number(spinBox_5->value());
        control.lowpass_gain_linear_x = QString::number(spinBox_6->value());
        control.lowpass_gain_angular_z = QString::number(spinBox_7->value());
        control.lowpass_gain_steering_angle = QString::number(spinBox_8->value());

        writeYaml("Control", "pursuit", "minimum_lookahead_distance", control.minimum_lookahead_distance);
        writeYaml("Control", "pursuit", "lookahead_ratio", control.lookahead_ratio);
        writeYaml("Control", "twist", "wheel_base", control.wheel_base);
        writeYaml("Control", "twist", "lateral_accel_limit", control.lateral_accel_limit);
        writeYaml("Control", "twist", "lateral_jerk_limit", control.lateral_jerk_limit);
        writeYaml("Control", "twist", "lowpass_gain_linear_x", control.lowpass_gain_linear_x);
        writeYaml("Control", "twist", "lowpass_gain_angular_z", control.lowpass_gain_angular_z);
        writeYaml("Control", "twist", "lowpass_gain_steering_angle", control.lowpass_gain_steering_angle);
    });

    dia_control->show();
}

//底盘can参数配置
void Driverless::canClientParam(void)
{
    QDialog *dia_can = new QDialog(this);
    dia_can->setWindowTitle(QObject::tr("底盘CAN参数配置"));
    dia_can->resize(300,200);

    QLabel *label_1 = new QLabel("速度倍率:");
    QDoubleSpinBox *spinBox_1 = new QDoubleSpinBox();
    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_1);
    layout_1->addWidget(spinBox_1);

    QLabel *label_2 = new QLabel("急刹值:");
    QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_2);
    layout_2->addWidget(spinBox_2);

    QLabel *label_3 = new QLabel("刹车值:");
    QDoubleSpinBox *spinBox_3 = new QDoubleSpinBox();
    QHBoxLayout *layout_3 = new QHBoxLayout;
    layout_3->addWidget(label_3);
    layout_3->addWidget(spinBox_3);

    QPushButton *button_1 = new QPushButton("确认");

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(layout_1);
    mainLayout->addLayout(layout_2);
    mainLayout->addLayout(layout_3);
    mainLayout->addWidget(button_1);
    dia_can->setLayout(mainLayout);

    spinBox_1->setValue(can.speed_gain.toDouble());
    spinBox_2->setValue(can.stop_brake_increase.toDouble());
    spinBox_3->setValue(can.normal_brake_increase.toDouble());
    connect(button_1, &QPushButton::clicked, [=] (){
        can.speed_gain = QString::number(spinBox_1->value());
        can.stop_brake_increase = QString::number(spinBox_2->value());
        can.normal_brake_increase = QString::number(spinBox_3->value());

        writeYaml("CanClient", "drive", "speed_gain", can.speed_gain);
        writeYaml("CanClient", "drive", "stop_brake_increase", can.stop_brake_increase);
        writeYaml("CanClient", "drive", "normal_brake_increase", can.normal_brake_increase);
    });

    dia_can->show();
}

//录制数据参数配置
void Driverless::recordParam(void)
{
    QDialog *dia_record = new QDialog(this);
    dia_record->setWindowTitle(QObject::tr("录制数据参数配置"));
    dia_record->resize(300,200);

    QCheckBox *check_1 = new QCheckBox();
    QCheckBox *check_2 = new QCheckBox();
    QCheckBox *check_3 = new QCheckBox();
    QCheckBox *check_4 = new QCheckBox();
    QCheckBox *check_5 = new QCheckBox();
    QCheckBox *check_6 = new QCheckBox();
    QCheckBox *check_7 = new QCheckBox();
    QCheckBox *check_8 = new QCheckBox();
    QLineEdit *lineEdit1= new QLineEdit("/points_raw");
    QLineEdit *lineEdit2= new QLineEdit("/imu_raw");
    QLineEdit *lineEdit3= new QLineEdit("/nmea_sentence");
    QLineEdit *lineEdit4= new QLineEdit("/can_box_raw");
    QLineEdit *lineEdit5= new QLineEdit("/fix");
    QLineEdit *lineEdit6= new QLineEdit("/image_raw");
    QLineEdit *lineEdit7= new QLineEdit("/can_info");
    QLineEdit *lineEdit8= new QLineEdit("/image_raw/compressed");

    QHBoxLayout *layout1 = new QHBoxLayout;
    layout1->addWidget(lineEdit1);
    layout1->addWidget(check_1);
    QHBoxLayout *layout2 = new QHBoxLayout;
    layout2->addWidget(lineEdit2);
    layout2->addWidget(check_2);
    QHBoxLayout *layout3 = new QHBoxLayout;
    layout3->addWidget(lineEdit3);
    layout3->addWidget(check_3);
    QHBoxLayout *layout4 = new QHBoxLayout;
    layout4->addWidget(lineEdit4);
    layout4->addWidget(check_4);
    QHBoxLayout *layout5 = new QHBoxLayout;
    layout5->addWidget(lineEdit5);
    layout5->addWidget(check_5);
    QHBoxLayout *layout6 = new QHBoxLayout;
    layout6->addWidget(lineEdit6);
    layout6->addWidget(check_6);
    QHBoxLayout *layout7 = new QHBoxLayout;
    layout7->addWidget(lineEdit7);
    layout7->addWidget(check_7);
    QHBoxLayout *layout8 = new QHBoxLayout;
    layout8->addWidget(lineEdit8);
    layout8->addWidget(check_8);

    QPushButton *button_1 = new QPushButton("确认");

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(layout1);
    layout->addLayout(layout2);
    layout->addLayout(layout3);
    layout->addLayout(layout4);
    layout->addLayout(layout5);
    layout->addLayout(layout6);
    layout->addLayout(layout7);
    layout->addLayout(layout8);
    layout->addWidget(button_1);

    dia_record->setLayout(layout);

    if(record.topic1!="null")
        check_1->setChecked(true);
    if(record.topic2!="null")
        check_2->setChecked(true);
    if(record.topic3!="null")
        check_3->setChecked(true);
    if(record.topic4!="null")
        check_4->setChecked(true);
    if(record.topic5!="null")
        check_5->setChecked(true);
    if(record.topic6!="null")
        check_6->setChecked(true);
    if(record.topic7!="null")
        check_7->setChecked(true);
    if(record.topic8!="null")
        check_8->setChecked(true);

    connect(button_1, &QPushButton::clicked, [=] (){
        if(check_1->isChecked())
            record.topic1 = lineEdit1->text();
        else
            record.topic1 = "null";
        if(check_2->isChecked())
            record.topic2 = lineEdit2->text();
        else
            record.topic2 = "null";
        if(check_3->isChecked())
            record.topic3 = lineEdit3->text();
        else
            record.topic3 = "null";
        if(check_4->isChecked())
            record.topic4 = lineEdit4->text();
        else
            record.topic4 = "null";
        if(check_5->isChecked())
            record.topic5 = lineEdit5->text();
        else
            record.topic5 = "null";
        if(check_6->isChecked())
            record.topic6 = lineEdit6->text();
        else
            record.topic6 = "null";
        if(check_7->isChecked())
            record.topic7 = lineEdit7->text();
        else
            record.topic7 = "null";
        if(check_8->isChecked())
            record.topic8 = lineEdit8->text();
        else
            record.topic8  = "null";
    });
    dia_record->show();
}

//建图参数配置
void Driverless::mappingParam(void)
{
    QDialog *dia_mapping = new QDialog(this);
    dia_mapping->setWindowTitle(QObject::tr("建图参数配置"));
    dia_mapping->resize(300,200);

    QComboBox *comboBox_1 = new QComboBox();
    comboBox_1->addItem("Lio-sam");
    comboBox_1->addItem("NDT");

    QCheckBox *check_1 = new QCheckBox("是否使用数据包");
    QLabel *label_2 = new QLabel("速率:");
    QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
    spinBox_2->setMaximum(8.0);
    spinBox_2->setSingleStep(0.1);
    spinBox_2->setValue(2);
    spinBox_2->setEnabled(false);
    QHBoxLayout *layout_3 = new QHBoxLayout;
    layout_3->addWidget(check_1,2);
    layout_3->addWidget(label_2,1);
    layout_3->addWidget(spinBox_2,1);

    QCheckBox *check_2 = new QCheckBox("使用GPS");
    QCheckBox *check_3 = new QCheckBox("使用GPU");
    QHBoxLayout *layout_4 = new QHBoxLayout;
    layout_4->addWidget(check_2,1);
    layout_4->addWidget(check_3,1);

    QLabel *label_1 = new QLabel("降采样:");
    QDoubleSpinBox *spinBox_1 = new QDoubleSpinBox();
    spinBox_1->setMaximum(1.0);
    spinBox_1->setSingleStep(0.1);
    spinBox_1->setValue(0.2);
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_1,1);
    layout_2->addWidget(spinBox_1,2);

    QPushButton *button_1 = new QPushButton("保存地图");

    QVBoxLayout *layout_1 = new QVBoxLayout;
    layout_1->addWidget(comboBox_1);
    layout_1->addLayout(layout_3);
    layout_1->addLayout(layout_4);
    layout_1->addLayout(layout_2);
    layout_1->addWidget(button_1);
    dia_mapping->setLayout(layout_1);

    if(mapping.name == "NDT")
        comboBox_1->setCurrentIndex(1);
    check_1->setChecked(boolTo(mapping.use_bag));
    if(check_1->isChecked())
        spinBox_2->setEnabled(true);
    check_2->setChecked(boolTo(mapping.use_gps));
    if(mapping.use_gpu=="2")
        check_3->setChecked(true);
    else
        check_3->setChecked(false);
    spinBox_1->setValue(mapping.filt.toDouble());
    spinBox_2->setValue(mapping.speed.toDouble());

    connect(comboBox_1,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](){
        mapping.name = comboBox_1->currentText();
    });
    connect(check_1, &QCheckBox::stateChanged, [=] (){
        mapping.use_bag = toBool(QString::number(check_1->isChecked()));
        if(check_1->isChecked())
            spinBox_2->setEnabled(true);
        else
            spinBox_2->setEnabled(false);
    });
    connect(check_2, &QCheckBox::stateChanged, [=] (){
        mapping.use_gps = toBool(QString::number(check_2->isChecked()));
    });
    connect(check_3, &QCheckBox::stateChanged, [=] (){
        if(check_3->isChecked())
            mapping.use_gpu = "2";
        else
            mapping.use_gpu = "0";
    });
    connect(spinBox_2, QOverload<double>::of(&QDoubleSpinBox::valueChanged),[=](double d){
        mapping.speed = QString::number(spinBox_2->value());
    });
    connect(spinBox_1, QOverload<double>::of(&QDoubleSpinBox::valueChanged),[=](double d){
        mapping.filt = QString::number(spinBox_1->value());
    });

    connect(button_1, &QPushButton::clicked, [=] (){
        QString mapName = on_Btn_SetName_clicked();
        QStringList saveMaps = getFiles("startingup_ros", "data/Map", false);
        for (int i = 0; i < saveMaps.size(); ++i)
        {
            if(saveMaps[i] == mapName)
            {
                QMessageBox::information(NULL, "保存失败",
                        "已存在相同地图！",
                        QMessageBox::Yes);
                        return ;
            }
        }
        if(!mapName.isEmpty())
        {
            std::string str_cmd;
            if(mapping.name == "NDT")
                runLaunch("custom_launch/map","build_pcd_map.launch",false);
            else
                runLaunch("custom_launch/map","build_pcd_mapLio.launch",false);

            QString saveFileMap = getFiles("startingup_ros", "data/Map", true)[0] + "/" + mapName;
            QString readFileMap = getFiles("startingup_ros", "data/Common", true)[0] + "/";

            str_cmd = "mkdir " +saveFileMap.toStdString() +" &";
            system(str_cmd.c_str());

            std::string str_cmd1 = "cp -r " + readFileMap.toStdString()+"map " + readFileMap.toStdString()+"param " + readFileMap.toStdString()+"tf " + saveFileMap.toStdString() +" &";
            system(str_cmd1.c_str());
            ui->comboBox_map->addItem(mapName);
            dia_mapping->close();
        }
    });

    dia_mapping->show();
}

QString Driverless::on_Btn_SetName_clicked()
{
    bool bOk = false;
    QString sName = QInputDialog::getText(this,
                                        "温馨提示!",
                                        "请输入地图名",
                                        QLineEdit::Normal,
                                        "map01",
                                        &bOk
                                        );

    if (bOk && !sName.isEmpty()) {
        return sName;
    }
    return "";
}

//录制轨迹点参数配置
void Driverless::makePathParam(void)
{
    QDialog *dia_makePath = new QDialog(this);
    dia_makePath->setWindowTitle(QObject::tr("录制轨迹点参数配置"));
    dia_makePath->resize(300,200);

    static bool flag = false;
    QCheckBox *check_1 = new QCheckBox("开始记录轨迹点");
    QPushButton *button_1 = new QPushButton("保存轨迹点");

    QVBoxLayout *layout_1 = new QVBoxLayout;

    layout_1->addWidget(check_1);

    layout_1->addWidget(button_1);
    dia_makePath->setLayout(layout_1);
    check_1->setChecked(flag);

    if(!ui->check_makePath->isChecked())
    {
        check_1->setChecked(false);
    }

    connect(check_1, &QCheckBox::stateChanged, [=] (){
        if(check_1->isChecked())
        {
            flag = true;
            QString wayPath1 = getFiles("startingup_ros", "data/Common/map/way_points", true)[0]+"/driving_lane.csv";
            QString wayPath2 = getFiles("startingup_ros", "data/Map", true)[0]+"/"+ui->comboBox_map->currentText()+"/map/way_points/driving_lane.csv";
            qDebug()<<wayPath1;
            qDebug()<<wayPath2;
            std::string str_cmd1 = "rm " +wayPath1.toStdString();
            std::string str_cmd2 = "true > " +wayPath2.toStdString();
            std::string str_cmd3 = "ln " +wayPath2.toStdString() + " " + wayPath1.toStdString();
            system(str_cmd1.c_str());
            system(str_cmd2.c_str());
            system(str_cmd3.c_str());

            QString str_cmd = "my_save_points2.launch"+strParam("map_type", ui->comboBox_map->currentText());
            runLaunch("custom_launch/points_nav",str_cmd,true);
          
        }
        else
            runLaunch("custom_launch/points_nav","my_save_points2.launch",false);
    });

    connect(button_1, &QPushButton::clicked, [=] (){
        runLaunch("custom_launch/points_nav","my_save_points.launch",false);
        runLaunch("custom_launch/points_nav","my_save_points2.launch",false);
        dia_makePath->close();
    });

    dia_makePath->show();
}

//路径追踪参数配置
void Driverless::pathTrackParam(void)
{
    // QDialog *dia_pathTrack = new QDialog(this);
    // dia_pathTrack->setWindowTitle(QObject::tr("参数配置"));
    // dia_pathTrack->resize(300,200);

    // static QCheckBox *check_1 = new QCheckBox("启动底盘CAN");

    // QVBoxLayout *layout_1 = new QVBoxLayout;

    // layout_1->addWidget(check_1);

    // dia_pathTrack->setLayout(layout_1);
    // connect(check_1, &QCheckBox::stateChanged, [=] (){
    //     if(check_1->isChecked())
    //         my_auto();
    //     else
    //         runLaunch("custom_launch","my_auto.launch",false);
    // });

    dia_pathTrack->show();
}

//车道保持参数配置
// void Driverless::laneParam(void)
// {
//     QDialog *dia_control = new QDialog(this);
//     dia_control->setWindowTitle(QObject::tr("车道保持参数配置"));
//     dia_control->resize(300,200);

//     QLabel *label_1 = new QLabel("原图像缩放倍数:");
//     QDoubleSpinBox *spinBox_1 = new QDoubleSpinBox();
//     QHBoxLayout *layout_1 = new QHBoxLayout;
//     layout_1->addWidget(label_1);
//     layout_1->addWidget(spinBox_1);

//     QLabel *label_2 = new QLabel("截取宽度(%):");
//     QDoubleSpinBox *spinBox_2 = new QDoubleSpinBox();
//     spinBox_2->setMaximum(100);
//     QHBoxLayout *layout_2 = new QHBoxLayout;
//     layout_2->addWidget(label_2);
//     layout_2->addWidget(spinBox_2);

//     QLabel *label_3 = new QLabel("摄像头偏向调整(%):");
//     QDoubleSpinBox *spinBox_3 = new QDoubleSpinBox();
//     spinBox_3->setMinimum(-100);
//     spinBox_3->setMaximum(100);
//     QHBoxLayout *layout_3 = new QHBoxLayout;
//     layout_3->addWidget(label_3);
//     layout_3->addWidget(spinBox_3);

//     QLabel *label_4 = new QLabel("顶端宽度(%):");
//     QDoubleSpinBox *spinBox_4 = new QDoubleSpinBox();
//     spinBox_4->setMaximum(100);
//     QHBoxLayout *layout_4 = new QHBoxLayout;
//     layout_4->addWidget(label_4);
//     layout_4->addWidget(spinBox_4);

//     QLabel *label_5 = new QLabel("截取高度(%):");
//     QDoubleSpinBox *spinBox_5 = new QDoubleSpinBox();
//     spinBox_5->setMaximum(100);
//     QHBoxLayout *layout_5 = new QHBoxLayout;
//     layout_5->addWidget(label_5);
//     layout_5->addWidget(spinBox_5);

//     QLabel *label_6 = new QLabel("速度值(%):");
//     QDoubleSpinBox *spinBox_6 = new QDoubleSpinBox();
//     spinBox_6->setMaximum(5);
//     QHBoxLayout *layout_6 = new QHBoxLayout;
//     layout_6->addWidget(label_6);
//     layout_6->addWidget(spinBox_6);

//     // QLabel *label_9 = new QLabel("速度值:");
//     // QDoubleSpinBox *spinBox_9 = new QDoubleSpinBox();
//     // spinBox_9->setMaximum(5);
//     // QHBoxLayout *layout_9 = new QHBoxLayout;
//     // layout_9->addWidget(label_9);
//     // layout_9->addWidget(spinBox_9);

//     QLabel *label_7 = new QLabel("灰度阀值:");
//     QDoubleSpinBox *spinBox_7 = new QDoubleSpinBox();
//     spinBox_7->setMaximum(255);
//     QHBoxLayout *layout_7 = new QHBoxLayout;
//     layout_7->addWidget(label_7);
//     layout_7->addWidget(spinBox_7);

//     QLabel *label_8 = new QLabel("去除车头图像(%):");
//     QDoubleSpinBox *spinBox_8 = new QDoubleSpinBox();
//     spinBox_8->setMaximum(100);
//     QHBoxLayout *layout_8 = new QHBoxLayout;
//     layout_8->addWidget(label_8);
//     layout_8->addWidget(spinBox_8);

//     QPushButton *button_1 = new QPushButton("确认");

//     QVBoxLayout *mainLayout = new QVBoxLayout;
//     mainLayout->addLayout(layout_1);
//     mainLayout->addLayout(layout_2);
//     mainLayout->addLayout(layout_3);
//     mainLayout->addLayout(layout_4);
//     mainLayout->addLayout(layout_5);
//     mainLayout->addLayout(layout_6);
//     mainLayout->addLayout(layout_7);
//     mainLayout->addLayout(layout_8);
//     mainLayout->addWidget(button_1);
//     dia_control->setLayout(mainLayout);

//     spinBox_1->setValue(lane.img_compress.toDouble());
//     spinBox_2->setValue(lane.h_size.toDouble());
//     spinBox_3->setValue(lane.h_off.toDouble());
//     spinBox_4->setValue(lane.h_top.toDouble());
//     spinBox_5->setValue(lane.v_size.toDouble());
//     spinBox_6->setValue(lane.speed.toDouble());
//     spinBox_7->setValue(lane.gray_threshold.toDouble());
//     spinBox_8->setValue(lane.v_off.toDouble());
//     // spinBox_9->setValue(control.lowpass_gain_steering_angle.toDouble());
//     connect(button_1, &QPushButton::clicked, [=] (){
//         lane.img_compress = QString::number(spinBox_1->value());
//         lane.h_size = QString::number(spinBox_2->value());
//         lane.h_off = QString::number(spinBox_3->value());
//         lane.h_top = QString::number(spinBox_4->value());
//         lane.v_size = QString::number(spinBox_5->value());
//         lane.speed = QString::number(spinBox_6->value());
//         lane.gray_threshold = QString::number(spinBox_7->value());
//         lane.v_off = QString::number(spinBox_8->value());

//         writeYaml("LaneFollow", "image", "img_compress", lane.img_compress);
//         writeYaml("LaneFollow", "image", "h_size", lane.h_size);
//         writeYaml("LaneFollow", "image", "h_off", lane.h_off);
//         writeYaml("LaneFollow", "image", "h_top", lane.h_top);
//         writeYaml("LaneFollow", "image", "v_size", lane.v_size);
//         writeYaml("LaneFollow", "image", "v_off", lane.v_off);
//         writeYaml("LaneFollow", "image", "speed", lane.speed);
//         writeYaml("LaneFollow", "image", "gray_threshold", lane.gray_threshold);
//     });

//     dia_control->show();
// }

//删除地图参数配置
void Driverless::deleMapParam(void)
{
    QDialog *dia_deleMap = new QDialog(this);
    dia_deleMap->setWindowTitle(QObject::tr("删除地图"));
    dia_deleMap->resize(300,200);

    QComboBox *comboBox_1 = new QComboBox();
    QStringList mapNames = getFiles("startingup_ros", "data/Map", false);

    for (int i = 0; i < mapNames.size(); ++i)
    {
        comboBox_1->addItem(mapNames[i]);
    }

    QLabel *label_1 = new QLabel("选择地图:");
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_1,1);
    layout_2->addWidget(comboBox_1,2);

    QPushButton *button_1 = new QPushButton("确认删除");

    QVBoxLayout *layout_1 = new QVBoxLayout;
    layout_1->addLayout(layout_2);
    layout_1->addWidget(button_1);
    dia_deleMap->setLayout(layout_1);

    connect(button_1, &QPushButton::clicked, [=] (){
        QString readFileMap = getFiles("startingup_ros", "data/Map", true)[0] + "/" + comboBox_1->currentText();

        std::string str_cmd = "rm -r " + readFileMap.toStdString() + " &";
        system(str_cmd.c_str());
        delay_ms(100);

        for (int i = 0; i < mapNames.size(); ++i)
        {
            // qDebug()<<ui->comboBox_map->itemText(i);
            if(ui->comboBox_map->itemText(i) == comboBox_1->currentText())
            {
                ui->comboBox_map->removeItem(i);
            }
        }
        ui->check_deleMap->setChecked(false);
        dia_deleMap->close();
    });

    dia_deleMap->show();
}

void Driverless::myThread()
{
    QStringList vehicleNames = getFiles("startingup_ros", "data/Vehicle", false);
    for (int i = 0; i < vehicleNames.size(); ++i)
    {
        if(vehicleNames[i]!=env_vehicle)
        {
            ui->comboBox_car->addItem(vehicleNames[i]);
            qDebug() << "vehicleNames: " << vehicleNames[i];
        }
    }
    QStringList mapNames = getFiles("startingup_ros", "data/Map", false);
    for (int i = 0; i < mapNames.size(); ++i)
    {
        if(mapNames[i]!=env_map)
        {
            ui->comboBox_map->addItem(mapNames[i]);
            qDebug() << "mapNames: " << mapNames[i];
        }
    }

    // system("roscore &");
    std::map<std::string, std::string> remappings;
    remappings["__master"] = "http://127.0.0.1:11311/";
    remappings["__hostname"] = "127.0.0.1";
    ros::init(remappings, "driverless_ui");
    if (!ros::master::check()) {
        // return false;
    }
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // 仿真台架接收不到下列话题
    ros::Subscriber battery_sub = nh.subscribe("battery_state", 10, &Driverless::battery_callback, this);
    ros::Subscriber hardware_sub = nh.subscribe("hardware_info", 10, &Driverless::hardware_info_callback, this);

    private_nh.getParam("use_developer", use_developer);//选择是否使用开发工具
    // if(!use_developer)
        ui->tab_manager->removeTab(2);//删除该页，实际该页还在
        
    readParam();
    ros::spin();
}

void Driverless::battery_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    // qDebug()<<msg->percentage;
    ui->label_battery_state->setText(QString::number(msg->percentage)+"%");
}

void Driverless::hardware_info_callback(const driverless_ui::HardwareInfo::ConstPtr& msg)
{
    if(msg->lidar == 0)
    {
        ui->label_lidar_2->setText("Ok");
        ui->label_lidar_2->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_lidar_2->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->lidar == 1)
    {
        ui->label_lidar_2->setText("Warn");
        // ui->label_lidar_2->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->lidar == 2)
    {
        ui->label_lidar_2->setText("Ok");
        // ui->label_lidar_2->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->camera == 0)
    {
        ui->label_camera_2->setText("Ok");
        ui->label_camera_2->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_camera_2->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->camera == 1)
    {
        ui->label_camera_2->setText("Warn");
        // ui->label_camera_2->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->camera == 2)
    {
        ui->label_camera_2->setText("Ok");
        // ui->label_camera_2->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->gnss == 0)
    {
        ui->label_gnss->setText("Ok");
        ui->label_gnss->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_gnss->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->gnss == 1)
    {
        ui->label_gnss->setText("Warn");
        // ui->label_gnss->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->gnss == 2)
    {
        ui->label_gnss->setText("Ok");
        // ui->label_gnss->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->milli == 0)
    {
        ui->label_milli->setText("Ok");
        ui->label_milli->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_milli->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->milli == 1)
    {
        ui->label_milli->setText("Warn");
        // ui->label_milli->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->milli == 2)
    {
        ui->label_milli->setText("Ok");
        // ui->label_milli->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->ultra == 0)
    {
        ui->label_ultra->setText("Ok");
        ui->label_ultra->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_ultra->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->ultra == 1)
    {
        ui->label_ultra->setText("Warn");
        // ui->label_ultra->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->ultra == 2)
    {
        ui->label_ultra->setText("Ok");
        // ui->label_ultra->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->can == 0)
    {
        ui->label_can->setText("Ok");
        ui->label_can->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_can->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->can == 1)
    {
        ui->label_can->setText("Warn");
        // ui->label_can->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->can == 2)
    {
        ui->label_can->setText("Ok");
        // ui->label_can->setStyleSheet("color: rgb(138, 226, 52);");
    }

    if(msg->vehicle == 0)
    {
        ui->label_vehicle->setText("Ok");
        ui->label_vehicle->setStyleSheet("color: rgb(138, 226, 52);");
        // ui->label_vehicle->setStyleSheet("color: rgb(239, 41, 41);");
    }else if(msg->vehicle == 1)
    {
        ui->label_vehicle->setText("Warn");
        // ui->label_vehicle->setStyleSheet("color: rgb(237, 212, 0);");
    }else if(msg->vehicle == 2)
    {
        ui->label_vehicle->setText("Ok");
        // ui->label_vehicle->setStyleSheet("color: rgb(138, 226, 52);");
    }


}