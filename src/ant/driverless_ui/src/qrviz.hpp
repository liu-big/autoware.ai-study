#ifndef QRVIZ_H
#define QRVIZ_H

#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>

#include <QDebug>
#include <QException>
#include <QThread>
#include <QVBoxLayout>

#include "rviz/image/ros_image_texture.h"
class QRviz : public QThread {
  Q_OBJECT
 public:
  QRviz(QVBoxLayout* layout, QString node_name);
  void run();
  void createDisplay(QString display_name, QString topic_name);
  //显示Grid
  void Display_Grid(bool enable, QString Reference_frame, int Plan_Cell_count,
                    QColor color = QColor(125, 125, 125));
  //显示map
  void Display_Map(bool enable, QString topic, double Alpha,
                   QString Color_Scheme);
  //设置全局显示属性
  void SetGlobalOptions(QString frame_name, QColor backColor, int frame_rate);
  //显示激光雷达点云
  void Display_LaserScan(bool enable, QString topic);

  void Display_PointCloud2(bool enable, QString topic);
  void Display_PcdMap(bool enable, QString topic);
  void Display_VectorMap(bool enable, QString topic);
  void Display_center_lines(bool enable, QString topic);
  void setViews(QString frame_name,QString view);//设置图层

  //显示导航相关控件
  void Display_Navigate(bool enable, QString Global_topic,
                        QString Global_planner, QString Local_topic,
                        QString Local_planner);
  void Display_Polygon(bool enable, QString topic);
  //显示tf坐标变换
  void Display_TF(bool enable);
  void Set_Pos();
  void Set_Goal();
  void Set_MoveCamera();
  void Set_Select();
  void Set_mutil_Goal();
  //发布goal话题的坐标
  void Send_Goal_topic();
  void show();
  void hide();
  void update();
  //显示robotmodel
  void Display_RobotModel(bool enable);
  //显示导航点
  void Display_MarkerArray(bool enable, QString topic);
//显示局部地图
  void Local_Costmap(bool enable);
  void Display_Imu(bool enable, QString topic);
  void Display_ImuAcc(bool enable, QString topic);
  void Display_Stracc(bool enable, QString topic);
  void Display_Strangul(bool enable, QString topic);
  void Display_Strangle(bool enable, QString topic);

 private:
  // rviz显示容器
  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* manager_;
  rviz::ViewManager* viewManager;
  rviz::Display* grid_ = NULL;

  //显示tf坐标变换
  rviz::Display* TF_ = NULL;
  rviz::Display* map_ = NULL;
  rviz::Display* laser_ = NULL;
  rviz::Display* polygon_ = NULL;
  rviz::Display* Navigate_localmap = NULL;
  rviz::Display* Navigate_localplanner = NULL;
  rviz::Display* Navigate_globalmap = NULL;
  rviz::Display* Navigate_globalplanner = NULL;
  rviz::Display* Navigate_amcl = NULL;
  rviz::Display* markerArray_ = NULL;
  rviz::Display* markerArray2_ = NULL;
  rviz::Display* markerArray3_ = NULL;
  rviz::Display *costmap_ = NULL;
  rviz::Display* RobotModel_ = NULL;

  rviz::Display* pointcloud2 = NULL;
  rviz::Display* pcdmap = NULL;
  rviz::Display* imu_ = NULL;
  rviz::Display* imu2_ = NULL;
  rviz::Display* stracc_ = NULL;
  rviz::Display* strangul = NULL;
  rviz::Display* strangle = NULL;

 

  // rviz工具
  rviz::Tool* current_tool;
  // rviz工具控制器
  rviz::ToolManager* tool_manager_;
  QVBoxLayout* layout;
  QString nodename;
 private slots:
  void addTool(rviz::Tool*);

  //   rviz::VisualizationManager *manager_=NULL;
  //    rviz::RenderPanel *render_panel_;
};

#endif  // QRVIZ_H
