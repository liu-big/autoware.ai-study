#include "qrviz.hpp"
#include <QTimer>
QRviz::QRviz(QVBoxLayout* layout, QString node_name) {
  this->layout = layout;
  this->nodename = node_name;

  //创建rviz容器
  render_panel_ = new rviz::RenderPanel;
  render_panel_->resize(900, 900);
  //向layout添加widget
  layout->addWidget(render_panel_);
  //初始化rviz控制对象
  manager_ = new rviz::VisualizationManager(render_panel_);
  viewManager = manager_->getViewManager();//设置图层
  ROS_ASSERT(manager_ != NULL);
  //获取当前rviz控制对象的 tool控制对象
  tool_manager_ = manager_->getToolManager();
  ROS_ASSERT(tool_manager_ != NULL);
  //初始化camera 这行代码实现放大 缩小 平移等操作
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  tool_manager_->initialize();
  manager_->removeAllDisplays();
  manager_->startUpdate();
    ROS_INFO("------------------------创建rviz容器");
}
void QRviz::show() { render_panel_->show(); }
void QRviz::hide() { render_panel_->hide(); }
void QRviz::update() {   manager_->startUpdate(); }

//显示robotModel
void QRviz::Display_RobotModel(bool enable) {
  if (RobotModel_ == NULL) {
    RobotModel_ =
        manager_->createDisplay("rviz/RobotModel", "Qrviz RobotModel", enable);
  } else {
    delete RobotModel_;
    RobotModel_ =
        manager_->createDisplay("rviz/RobotModel", "Qrviz RobotModel", enable);
  }
}
//显示grid
void QRviz::Display_Grid(bool enable, QString Reference_frame,
                        int Plan_Cell_count, QColor color) {
  if (grid_ == NULL) {
    grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    // Configure the GridDisplay the way we like it.
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Color")->setValue(color);
    grid_->subProp("Reference Frame")->setValue(Reference_frame);
    grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);

  } else {
    delete grid_;
    grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    // Configure the GridDisplay the way we like it.
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Color")->setValue(color);
    grid_->subProp("Reference Frame")->setValue(Reference_frame);
    grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);
  }
  grid_->setEnabled(enable);
  manager_->startUpdate();
}
//显示map
void QRviz::Display_Map(bool enable, QString topic, double Alpha,
                        QString Color_Scheme) {
  if (!enable && map_) {
    map_->setEnabled(false);
    return;
  }
  if (map_ == NULL) {
    qDebug()<<"map is ok";
    map_ = manager_->createDisplay("rviz/Map", "QMap", true);
    ROS_ASSERT(map_);
    map_->subProp("Topic")->setValue(topic);
    map_->subProp("Alpha")->setValue(Alpha);
    map_->subProp("Color Scheme")->setValue(Color_Scheme);

  } 
  else {
    ROS_ASSERT(map_);
    qDebug() << "asdasdasd:" << topic << Alpha;

    delete map_;
    map_ = manager_->createDisplay("rviz/Map", "adjustable map", true);
    ROS_ASSERT(map_);
    map_->subProp("Topic")->setValue(topic);
    map_->subProp("Alpha")->setValue(Alpha);
    map_->subProp("Color Scheme")->setValue(Color_Scheme);
  }

  map_->setEnabled(enable);
  manager_->startUpdate();
}


//显示激光雷达三维点云
void QRviz::Display_PointCloud2(bool enable, QString topic) {
  if (pointcloud2 == NULL) {
    pointcloud2 = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", enable);
    ROS_ASSERT(pointcloud2);
    pointcloud2->subProp("Topic")->setValue(topic);
    pointcloud2->subProp("Size (m)")->setValue("0.05");
  } else {
    delete pointcloud2;
    pointcloud2 = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", enable);
    ROS_ASSERT(pointcloud2);
    pointcloud2->subProp("Topic")->setValue(topic);
    pointcloud2->subProp("Style")->setValue("Squares");
    pointcloud2->subProp("Size (m)")->setValue("0.05");
  }
  pointcloud2->setEnabled(enable);
  manager_->startUpdate();
}

//显示三维点云地图
void QRviz::Display_PcdMap(bool enable, QString topic) {
  if (pcdmap == NULL) {
    pcdmap = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", enable);
    ROS_ASSERT(pcdmap);
    pcdmap->subProp("Topic")->setValue(topic);
    pcdmap->subProp("Style")->setValue("Points");
    pcdmap->subProp("Size (Pixels)")->setValue("2");
    pcdmap->subProp("Alpha")->setValue("0.05");
    // pcdmap->subProp("Color Transformer")->setValue("FlatColor");
    // pcdmap->subProp("Color")->setValue(QColor(255, 255, 255));  //设置为白色
    pcdmap->subProp("Color Transformer")->setValue("AxisColor");
  } else {
    delete pcdmap;
    pcdmap = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", enable);
    ROS_ASSERT(pcdmap);
    pcdmap->subProp("Topic")->setValue(topic);
    pcdmap->subProp("Style")->setValue("Points");
    pcdmap->subProp("Size (Pixels)")->setValue("2");
    pcdmap->subProp("Alpha")->setValue("0.05");
    // pcdmap->subProp("Color Transformer")->setValue("FlatColor");
    // pcdmap->subProp("Color")->setValue(QColor(255, 255, 255));  //设置为白色
    pcdmap->subProp("Color Transformer")->setValue("AxisColor");
  }
  pcdmap->setEnabled(enable);
  manager_->startUpdate();
}

//显示高精地图
void QRviz::Display_VectorMap(bool enable, QString topic) {
  if (markerArray2_ == NULL) {
    markerArray2_ = manager_->createDisplay("rviz/MarkerArray", "adjustable markerArray2", enable);
    ROS_ASSERT(markerArray2_);
    markerArray2_->subProp("Marker Topic")->setValue(topic);
  } else {
    delete markerArray2_;
    markerArray2_ = manager_->createDisplay("rviz/MarkerArray", "adjustable markerArray2", enable);
    ROS_ASSERT(markerArray2_);
    markerArray2_->subProp("Marker Topic")->setValue(topic);
  }
  markerArray2_->setEnabled(enable);
  manager_->startUpdate();
}

//车道线
void QRviz::Display_center_lines(bool enable, QString topic) {
  if (markerArray3_ == NULL) {
    markerArray3_ = manager_->createDisplay("rviz/MarkerArray", "adjustable markerArray2", enable);
    ROS_ASSERT(markerArray3_);
    markerArray3_->subProp("Marker Topic")->setValue(topic);
  } else {
    delete markerArray3_;
    markerArray3_ = manager_->createDisplay("rviz/MarkerArray", "adjustable markerArray2", enable);
    ROS_ASSERT(markerArray3_);
    markerArray3_->subProp("Marker Topic")->setValue(topic);
  }
  markerArray3_->setEnabled(enable);
  manager_->startUpdate();
}

void QRviz::Display_Polygon(bool enable, QString topic) {
  if (polygon_ == NULL) {
    polygon_ = manager_->createDisplay("rviz/Polygon", "QPolygon", enable);
    ROS_ASSERT(polygon_);
    polygon_->subProp("Topic")->setValue(topic);
  } else {
    delete polygon_;
    polygon_ = manager_->createDisplay("rviz/Polygon", "QPolygon", enable);
    ROS_ASSERT(polygon_);
    polygon_->subProp("Topic")->setValue(topic);
  }
  polygon_->setEnabled(enable);
  manager_->startUpdate();
}
//显示激光雷达
void QRviz::Display_LaserScan(bool enable, QString topic) {
  if (laser_ == NULL) {
    laser_ = manager_->createDisplay("rviz/LaserScan", "QLaser", enable);
    ROS_ASSERT(laser_);
    laser_->subProp("Topic")->setValue(topic);
    laser_->subProp("Size (m)")->setValue("0.05");
  } else {
    delete laser_;
    laser_ = manager_->createDisplay("rviz/LaserScan", "QLaser", enable);
    ROS_ASSERT(laser_);
    laser_->subProp("Topic")->setValue(topic);
    laser_->subProp("Size (m)")->setValue("0.05");
  }
  laser_->setEnabled(enable);
  manager_->startUpdate();
}

//显示多个导航的坐标点
void QRviz::Display_MarkerArray(bool enable, QString topic) {
  if (markerArray_ == NULL) {
    markerArray_ = manager_->createDisplay("rviz/MarkerArray", "adjustable MarkerArray", enable);
    ROS_ASSERT(markerArray_);
    markerArray_->subProp("Marker Topic")->setValue(topic);
  } else {
    delete markerArray_;
    markerArray_ = manager_->createDisplay("rviz/MarkerArray", "adjustable MarkerArray", enable);
    ROS_ASSERT(markerArray_);
    markerArray_->subProp("Marker Topic")->setValue(topic);
  }
  markerArray_->setEnabled(enable);
  manager_->startUpdate();
}

void QRviz::Local_Costmap(bool enable){
    costmap_ = manager_->createDisplay( "rviz/Map", "adjustable MarkerArray", enable );
    costmap_->subProp( "Topic" )->setValue( "/move_base/local_costmap/costmap" );
    costmap_->subProp( "Alpha" )->setValue( "0.7" );
    costmap_->subProp("Color Scheme")->setValue("map");
    costmap_->subProp("Width")->setValue("80");
    costmap_->subProp("Height")->setValue("80");
}

//设置全局显示
void QRviz::SetGlobalOptions(QString frame_name, QColor backColor,
                             int frame_rate) {
  manager_->setFixedFrame(frame_name);
  manager_->setProperty("Background Color", backColor);
  manager_->setProperty("Frame Rate", frame_rate);
  manager_->startUpdate();


}

void QRviz::setViews(QString frame_name,QString view)
{
  viewManager->setRenderPanel(render_panel_);
  viewManager->setCurrentViewControllerType(view);
  viewManager->getCurrent()->subProp("Target Frame")->setValue(frame_name);
}

// "rviz/MoveCamera";
// "rviz/Interact";
// "rviz/Select";
// "rviz/SetInitialPose";
// "rviz/SetGoal";
//设置机器人导航初始位置
void QRviz::Set_Pos() {  
//  QTimer::singleShot(2500,[=](){
//      map_rviz->Display_Navigate(Navigation_enable,GlobalMap_topic,GlobalMap_paln,LocalMap_topic,LocalMap_plan);
//  });
  //获取设置Pos的工具
  //添加工具

  current_tool = tool_manager_->addTool("rviz/SetInitialPose");
  //设置当前使用的工具为SetInitialPose（实现在地图上标点）
  tool_manager_->setCurrentTool(current_tool);
  manager_->startUpdate();

  //     tool_manager_->setCurrentTool()
}
//设置机器人导航目标点
void QRviz::Set_Goal() {
  //添加工具
  current_tool = tool_manager_->addTool("rviz/SetGoal");
  //设置goal的话题
  rviz::Property* pro = current_tool->getPropertyContainer();
  pro->subProp("Topic")->setValue("/move_base_simple/goal");
  //设置当前frame
  manager_->setFixedFrame("map");
  //设置当前使用的工具为SetGoal（实现在地图上标点）
  tool_manager_->setCurrentTool(current_tool);

  manager_->startUpdate();
}


//设置机器人多个导航目标点
void QRviz::Set_mutil_Goal() {
  //添加工具
  current_tool = tool_manager_->addTool("rviz/PublishPoint");
  //设置goal的话题
  rviz::Property* pro = current_tool->getPropertyContainer();
  pro->subProp("Topic")->setValue("/clicked_point");
  //设置当前frame
  manager_->setFixedFrame("map");
  //设置当前使用的工具为SetGoal（实现在地图上标点）
  tool_manager_->setCurrentTool(current_tool);

  manager_->startUpdate();
}

void QRviz::Set_MoveCamera() {
  //获取设置Pos的工具
  //添加工具

  current_tool = tool_manager_->addTool("rviz/MoveCamera");
  //设置当前使用的工具为SetInitialPose（实现在地图上标点）
  tool_manager_->setCurrentTool(current_tool);
  manager_->startUpdate();
}
void QRviz::Set_Select() {
  //获取设置Pos的工具
  //添加工具

  current_tool = tool_manager_->addTool("rviz/Select");
  //设置当前使用的工具为SetInitialPose（实现在地图上标点）
  tool_manager_->setCurrentTool(current_tool);
  manager_->startUpdate();
}
//显示tf坐标变换
void QRviz::Display_TF(bool enable) {
  if (TF_) {
    delete TF_;
    TF_ = NULL;
  }
  TF_ = manager_->createDisplay("rviz/TF", "QTF", enable);
}
//显示导航相关
void QRviz::Display_Navigate(bool enable, QString Global_topic,
                             QString Global_planner, QString Local_topic,
                             QString Local_planner) {
  if (Navigate_localmap) {
    delete Navigate_localmap;
    Navigate_localmap = NULL;
  }
  if (Navigate_localplanner) {
    delete Navigate_localplanner;
    Navigate_localplanner = NULL;
  }
  if (Navigate_globalmap) {
    delete Navigate_globalmap;
    Navigate_globalmap = NULL;
  }
  if (Navigate_globalplanner) {
    delete Navigate_globalplanner;
    Navigate_globalplanner = NULL;
  }
    QTimer::singleShot(1000,[=](){
        // local map
        Navigate_localmap = manager_->createDisplay("rviz/Map", "Qlocalmap", enable);
        Navigate_localmap->subProp("Topic")->setValue(Local_topic);
        Navigate_localmap->subProp("Color Scheme")->setValue("costmap");
    });
    QTimer::singleShot(2000,[=](){
        Navigate_localplanner =
            manager_->createDisplay("rviz/Path", "QlocalPath", enable);
        Navigate_localplanner->subProp("Topic")->setValue(Local_planner);
        Navigate_localplanner->subProp("Color")->setValue(QColor(0, 12, 255));
    });
    QTimer::singleShot(3000,[=](){
        // global map
        // Navigate_globalmap =
        //     manager_->createDisplay("rviz/Map", "QGlobalmap", enable);
        // Navigate_globalmap->subProp("Topic")->setValue(Global_topic);
        // Navigate_globalmap->subProp("Color Scheme")->setValue("costmap");
    });
    QTimer::singleShot(4000,[=](){
        Navigate_globalplanner =
            manager_->createDisplay("rviz/Path", "QGlobalpath", enable);
        Navigate_globalplanner->subProp("Topic")->setValue(Global_planner);
        Navigate_globalplanner->subProp("Color")->setValue(QColor(255, 0, 0));
    });
  //更新画面显示
  manager_->startUpdate();
}
void QRviz::addTool(rviz::Tool*) {}
void QRviz::createDisplay(QString display_name, QString topic_name) {}
void QRviz::run() {}
