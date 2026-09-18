#include <QCompleter>
#include <QDebug>
//#include <QHostAddress>
#include <QMessageBox>
#include <QMovie>
//#include <QNetworkInterface>
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

#include "app.h"
#include "ui_app.h"
#include <iostream>
#include <ros/package.h>
#include <iostream>
#include <stdio.h>
#include "autoware_msgs/SitePoseArray.h" 


VizlibTest::VizlibTest(int argc, char **argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VizlibTest)
{
    ui->setupUi(this);
    //窗体标题
    this->setWindowTitle("车辆部署软件");

    ui_init();   
    //读取配置
    connections();    //连接成功才能执行此函数

}

VizlibTest::~VizlibTest()
{
    delete ui;
}


//UI界面初始化
void VizlibTest::ui_init() {
  ROS_INFO("UI界面初始化");

  ui->tableWidget->setRowCount(0); //设置表格⾏数
  ui->tableWidget->setColumnCount(5);// 设置表格列数
  QStringList headerLabels = QStringList()<<QObject::tr("坐标名")
                                              <<QObject::tr("X")<<QObject::tr("Y")<<QObject::tr("Z")<<QObject::tr(" ");
  ui->tableWidget->setHorizontalHeaderLabels(headerLabels);  //设置表头
  ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);//设置表格不可被编辑

  ui->tableWidget_2->setRowCount(0); //设置表格⾏数
  ui->tableWidget_2->setColumnCount(5);// 设置表格列数
  QStringList headerLabels_2 = QStringList()<<QObject::tr("站点名")
                                              <<QObject::tr("属性")<<QObject::tr("触发1")<<QObject::tr("触发2")<<QObject::tr(" ");

  ui->tableWidget_2->setHorizontalHeaderLabels(headerLabels_2);  //设置表头
  ui->tableWidget_2->setEditTriggers(QAbstractItemView::NoEditTriggers);//设置表格不可被编辑


  // ui->push_read_list->setStyleSheet("color: rgb(0, 0, 0); background-color: rgb(255, 0, 0);");

}


void VizlibTest::connections() {
    
    connect(ui->tableWidget, SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(onItemClicked(QTableWidgetItem*)));//表格1连接信号
    connect(ui->tableWidget_2, SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(onItemClicked_2(QTableWidgetItem*)));//表格2连接信号
    //保存表格1为xml
  connect(ui->push_save_list, SIGNAL(clicked()), this, SLOT(on_btn_saveas_xml_clicked()));
  //读取表格1为xml
  connect(ui->push_read_list, SIGNAL(clicked()), this, SLOT(on_btn_readfromxml_clicked()));

    //保存表格2为xml
  connect(ui->push_save_list_2, SIGNAL(clicked()), this, SLOT(on_btn_saveas_site_clicked()));
  //读取表格2为xml
  connect(ui->push_read_list_2, SIGNAL(clicked()), this, SLOT(on_btn_readfromsite_clicked()));

////设置站点
  connect(ui->pushAdd_site, SIGNAL(clicked()), this, SLOT(Add_Site()));
//设置坐标点
  connect(ui->set_goal_btn_2, SIGNAL(clicked()), this, SLOT(coordinate_set())); 

   //加载点云地图连接
  connect(ui->set_points_map, SIGNAL(clicked()), this, SLOT(Load_Points_Map()));
//加载高精地图连接
  connect(ui->set_vector_map, SIGNAL(clicked()), this, SLOT(Load_Vector_Map()));

   //发布站点
  connect(ui->pushButton_site, &QPushButton::clicked,[=](){ 
    pub_data();
  });

  connect(ui->set_views, &QCheckBox::stateChanged, [=] (){

    qDebug()<<"状态改变信号";
    if(ui->set_views->isChecked())
      map_rviz->setViews("map","rviz/TopDownOrtho");
    else
      map_rviz->setViews("map","rviz/Orbit");
  });

    // system("roscore &");//打开roscore
    // QTimer::singleShot(2000,[=](){  //计时进入，等待roscore打开
        slot_btn_connect();
  // });
    
}

//ROS节点初始化的函数*********************************
bool VizlibTest::init(const std::string& master_url, const std::string& host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
        ROS_INFO("---------------%s",remappings["__master"].c_str());
        ROS_INFO("---------------%s",remappings["__hostname"].c_str());
  ros::init(remappings, "app_monitor", ros::init_options::AnonymousName);
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
  SubAndPubTopic();    //创建订阅者与发布者
  return true;
}

//创建订阅者与发布者
void VizlibTest::SubAndPubTopic() 
{
    ros::NodeHandle nh;

  // 创建一个Publisher，发布名为launch_pub的topic，消息类型为std_msgs::Strin
  pub_SitePose  = nh.advertise<autoware_msgs::SitePoseArray>("/multi_goal", 1);
  markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);
  // markerPub = nh.advertise<app::SiteArray>("TEXT_VIEW_FACING", 1);
  //  app_pub =   h.advertise<std_msgs::String>("app_pub", 1000);
  sub_SitePose    = nh.subscribe("move_base_simple/goal", 1000,
                            &VizlibTest::chatterCallBack, this);

}

void VizlibTest::pub_marker(void) {
  map_rviz->Display_MarkerArray(true,"/MarkerArray");
    visualization_msgs::MarkerArray markerArray;
    int count = ui->tableWidget->rowCount();
    for (size_t i = 0; i < count; i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id="map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =i;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;//文字的大小
        marker.color.b = 25;
        marker.color.g = 0;
        marker.color.r = 25;//文字的颜色
        marker.color.a = 1;//必写，否则rviz无法显示;


        geometry_msgs::Pose pose;
        pose.position.x =  QString(ui->tableWidget->item(i,1)->text()).toFloat();
        pose.position.y =  QString(ui->tableWidget->item(i,2)->text()).toFloat();
        pose.position.z =  QString(ui->tableWidget->item(i,3)->text()).toFloat();
        std::ostringstream str;
        // str<<"哈TT";
        // marker.text= str.str();
        marker.text= ui->tableWidget->item(i,0)->text().toStdString();   
        // ROS_INFO("哈TTr");
        marker.pose=pose;
        markerArray.markers.push_back(marker);
        markerArrayPub.publish(markerArray);
}
    ros::spinOnce();
}


//发布launch命令信息
void VizlibTest::pub_data(void) {
  autoware_msgs::SitePoseArray site_data;
  autoware_msgs::SitePose  siteposes;

  if(ROS_LINK){

    site_data.header.frame_id = "map";
    site_data.header.stamp = ros::Time();

    for(int i =0;i<ui->tableWidget_2->rowCount();i++)
    {
        siteposes.pose.position.x = Site_X_List.at(i);
        siteposes.pose.position.y = Site_Y_List.at(i);
        siteposes.pose.position.z = Site_Z_List.at(i);

        siteposes.pose.orientation.x = Site_orieX_List.at(i);
        siteposes.pose.orientation.y = Site_orieY_List.at(i);
        siteposes.pose.orientation.z = Site_orieZ_List.at(i);
        siteposes.pose.orientation.w = Site_orieW_List.at(i);

        siteposes.site_name = ui->tableWidget_2->item(i,0)->text().toStdString();

        if(ui->tableWidget_2->item(i,1)->text() == "终点")
            siteposes.Attributes = siteposes.START_SITE;
        else if(ui->tableWidget_2->item(i,1)->text() == "停靠点")
            siteposes.Attributes = siteposes.DOCK_SITE;
        else if(ui->tableWidget_2->item(i,1)->text() == "车库")
            siteposes.Attributes = siteposes.GARAGE__SITE;

        if(ui->tableWidget_2->item(i,2)->text().contains("时间",Qt::CaseSensitive) == true)
        {
            QString tmp,str;
            str = ui->tableWidget_2->item(i,2)->text();
            siteposes.trigger_1 = siteposes.TIME_TRI;
            for(int j = 0;j<str.length();j++)
            {
              if(str[j]>'0'&&str[j]<'9')
              tmp.append(str[j]);
            }
            siteposes.time =tmp.toInt();
        }
        else if(ui->tableWidget_2->item(i,2)->text() == "按键")
            siteposes.trigger_1 = siteposes.BUTTON;
        else if(ui->tableWidget_2->item(i,2)->text() == "指令")
            siteposes.trigger_1 = siteposes.INSTRUCTION;

        if(ui->tableWidget_2->item(i,3)->text().contains("时间",Qt::CaseSensitive) == true)
        {
            QString tmp2,str;
            str = ui->tableWidget_2->item(i,3)->text();
            siteposes.trigger_2 = siteposes.TIME_TRI;
            for(int j = 0;j<str.length();j++)
            {
              if(str[j]>='0'&&str[j]<='9')
              tmp2.append(str[j]);
            }
            siteposes.time =tmp2.toInt();
        }
        else if(ui->tableWidget_2->item(i,3)->text() == "按键")
            siteposes.trigger_2 = siteposes.BUTTON;
        else if(ui->tableWidget_2->item(i,3)->text() == "指令")
            siteposes.trigger_2 = siteposes.INSTRUCTION;

        if(ui->tableWidget_2->item(i,2)->text().contains("时间",Qt::CaseSensitive) == false && ui->tableWidget_2->item(i,3)->text().contains("时间",Qt::CaseSensitive) == false)
            siteposes.time = 0;
        site_data.sites.push_back(siteposes);
    }

    // pose.orientation.x = 123;

      pub_SitePose.publish(site_data);

    // 发布消息
    // ROS_INFO("发布消息---%s", msg.data.c_str());
    // 循环等待回调函数
    ros::spinOnce();
  }
}

//APP 监听回调函数
void VizlibTest::chatterCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("frame_id :  %s",msg->header.frame_id.c_str());
    ROS_INFO("Position(x,y,z)   %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Orientation(x,y,z,w)   %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    ROS_INFO("Time  %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

// QTableWidgetItem* item2 = new QTableWidgetItem(QIcon(QObject::tr(":/icons/dial.png")), QObject::tr("Dial"));

    // ui->tableView->setItem(coord_num,0,item2);
    // model->setItem(coord_num,1,new QStandardItem(QString::number(msg->pose.position.x)));
    // model->setItem(coord_num,2,new QStandardItem(QString::number(msg->pose.position.y)));
    // model->setItem(coord_num,3,new QStandardItem(QString::number(msg->pose.position.z)));
    if(SET_FLAG){
        int row_count = ui->tableWidget->rowCount(); //获取表单行数
        ui->tableWidget->insertRow(row_count); //插入新行
        ui->tableWidget->setItem(coord_num,0,new QTableWidgetItem(coordiname));
        ui->tableWidget->setItem(coord_num,1,new QTableWidgetItem(QString::number(msg->pose.position.x)));
        ui->tableWidget->setItem(coord_num,2,new QTableWidgetItem(QString::number(msg->pose.position.y)));
        ui->tableWidget->setItem(coord_num,3,new QTableWidgetItem(QString::number(msg->pose.position.z)));

        Coord_orieX_List.append(msg->pose.orientation.x);
        Coord_orieY_List.append(msg->pose.orientation.y);
        Coord_orieZ_List.append(msg->pose.orientation.z);
        Coord_orieW_List.append(msg->pose.orientation.w);

        ui->tableWidget->setItem(coord_num,4,new QTableWidgetItem("删除"));
        
        // QString str = ui->tableWidget->item(coord_num,0)->text();//取出字符串
        // //qDebug()<<"cur value"<<model->record(index.row()).value("packages").toString();
        //qDebug()<<ui->tableWidget->rowCount()<< str;
        pub_marker();//将坐标点发布显示出来
        coord_num++;
        SET_FLAG = false;//设置坐标点标志位
     }
}


    void VizlibTest::Load_Points_Map(void)
    {
        qDebug()<<"加载点云地图";

      char outBuffer[200] ;
          FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/pointcloud_map", "r" );   //获取文件路径
          if(!FileName){  // 检测流管道
              perror("Fail to popen\n");
              return ;
          }
          while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
            // printf("输出: %s",outBuffer); // 打印输出
          }
          pclose(FileName); 

          //  std::cout << outBuffer << std::endl;
          QString str_name  = QString::fromStdString(outBuffer);
          str_name  =   str_name.simplified();//去掉换行符号
          qDebug()<<str_name;

        std::string path;
        QStringList filename = QFileDialog::getOpenFileNames(this,"选择地图文件",QString(str_name), tr("PCD(*.pcd);;"));
        qDebug()<<filename;
        QString str;
        for(int i=0;i<filename.count();i++)
        {
          str = str + filename.at(i) + ",";
          //  qDebug()<<str;
        }
        str = "roslaunch map_file  points_map_loader.launch path_pcd:="+str+" &";
        // qDebug()<<str;
        path = str.toStdString();

        system("pkill -f map_file &");//先kill掉全局规划节点
        map_rviz->Display_PcdMap(true, "/points_map");
        system(path.c_str());//执行命令
    }
    void VizlibTest::Load_Vector_Map(void)
    {
      qDebug()<<"加载高精地图";

      char outBuffer[200] ;
          FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/vector_map", "r" );   //获取文件路径
          if(!FileName){  // 检测流管道
              perror("Fail to popen\n");
              return ;
          }
          while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
            // printf("输出: %s",outBuffer); // 打印输出
          }
          pclose(FileName); 

          //  std::cout << outBuffer << std::endl;
          QString str_name  = QString::fromStdString(outBuffer);
          str_name  =   str_name.simplified();//去掉换行符号
          qDebug()<<str_name;

        std::string path;
        QStringList filename = QFileDialog::getOpenFileNames(this,"选择地图文件",QString(str_name), tr("CSV(*.csv);;"));
        qDebug()<<filename;
        QString str;
        for(int i=0;i<filename.count();i++)
        {
          str = str + filename.at(i) + " ";
          //  qDebug()<<str;
        }
        str = "rosrun map_file vector_map_loader "+str+" &";
        // qDebug()<<str;
        path = str.toStdString();

        system("pkill -f vector_map_loader &");//先kill掉高精地图节点
        system("pkill -f op_global_planner &");//先kill掉全局规划节点
        map_rviz->Display_VectorMap(true,"/vector_map");
        map_rviz->Display_center_lines(true,"/vector_map_center_lines_rviz");

        system(path.c_str());//打开执行高精地图命令
        system("roslaunch op_global_planner op_global_planner.launch &");//打开执行全局规划命令
    }


void VizlibTest::Add_Site(void)
{
    // qDebug()<<"表格信号";
    QDialog *window = new QDialog(this);
    window->setWindowTitle(QObject::tr("增加站点"));
    window->resize(300,200);

    QLabel *label_1 = new QLabel("站点名:");
    QComboBox *comboBox_1 = new QComboBox();
    for(int i=0;i<ui->tableWidget->rowCount();i++)//显示出所有添加的坐标
    {
      QString str = ui->tableWidget->item(i,0)->text();//取出字符串
      comboBox_1->addItem(str);
      // qDebug()<<"读取站点信息"<<str;
    }
      
    QHBoxLayout *layout_1 = new QHBoxLayout;
    layout_1->addWidget(label_1);
    layout_1->addWidget(comboBox_1);

    QLabel *label_2 = new QLabel("属性:");
    QComboBox *comboBox_2 = new QComboBox();
    comboBox_2->addItem("终点");
    comboBox_2->addItem("停靠点");
    comboBox_2->addItem("充电站");
    comboBox_2->setCurrentIndex(1);
    QHBoxLayout *layout_2 = new QHBoxLayout;
    layout_2->addWidget(label_2);
    layout_2->addWidget(comboBox_2);


    QLabel *label_3 = new QLabel("触发1");
    QComboBox *comboBox_3 = new QComboBox();
    comboBox_3->addItem("时间");
    comboBox_3->addItem("按键");
    comboBox_3->addItem("指令");
    QSpinBox *spinbox= new QSpinBox;
    spinbox->setMaximum(99999);
    QLabel *label_5 = new QLabel("秒");
    QHBoxLayout *layout_3 = new QHBoxLayout;
    QHBoxLayout *layout_3_1 = new QHBoxLayout;
    QHBoxLayout *layout_3_2 = new QHBoxLayout;
    layout_3_1->addWidget(label_3);
    layout_3_1->addWidget(spinbox);
    layout_3_1->addWidget(label_5);
    layout_3_2->addWidget(comboBox_3); 
    

    layout_3->addLayout(layout_3_1,1);
    layout_3->addLayout(layout_3_2,1);
    // layout_3->setStretch(0,1,2,3,4); 
    // layout_3->setStretch(4);

    QLabel *label_4 = new QLabel("触发2");
    QComboBox *comboBox_4 = new QComboBox();
    comboBox_4->addItem("时间");
    comboBox_4->addItem("按键");
    comboBox_4->addItem("指令");
    QHBoxLayout *layout_4 = new QHBoxLayout;
    layout_4->addWidget(label_4);
    layout_4->addWidget(comboBox_4);

    
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton *button = new QPushButton("确定");
    layout->addLayout(layout_1,1);
    layout->addLayout(layout_2,1);
    layout->addLayout(layout_3,1);
    layout->addLayout(layout_4,1);


    layout->addWidget(button,1);


    window->setLayout(layout);


    connect(comboBox_3,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](){
      if(comboBox_3->currentIndex() !=0 && comboBox_4->currentIndex() !=0){
          spinbox->setEnabled(false);
      }
      else
          spinbox->setEnabled(true);
    });
    connect(comboBox_4,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](){
      if(comboBox_3->currentIndex() !=0 && comboBox_4->currentIndex() !=0)
          spinbox->setEnabled(false);
      else
          spinbox->setEnabled(true);
    });

    connect(button, &QPushButton::clicked,[=](){
      // qDebug()<<"选中行"<<comboBox_1->currentIndex();
      if(comboBox_1->currentIndex()>-1)
      {
            int row_count = ui->tableWidget_2->rowCount(); //获取表单行数
            // qDebug()<<"表格2行数"<<row_count;
            ui->tableWidget_2->insertRow(row_count); //插入新行
            ui->tableWidget_2->setItem(site_num,0,new QTableWidgetItem(ui->tableWidget->item(comboBox_1->currentIndex(),0)->text()));

            Site_time_List.append(spinbox->value());
            Site_X_List.append(QString(ui->tableWidget->item(comboBox_1->currentIndex(),1)->text()).toFloat());//将X轴数据保存到列表
            Site_Y_List.append(QString(ui->tableWidget->item(comboBox_1->currentIndex(),2)->text()).toFloat());//将Y轴数据保存到列表
            Site_Z_List.append(QString(ui->tableWidget->item(comboBox_1->currentIndex(),3)->text()).toFloat());//将Z轴数据保存到列表

            Site_orieX_List.append(Coord_orieX_List.at(comboBox_1->currentIndex()));
            Site_orieY_List.append(Coord_orieY_List.at(comboBox_1->currentIndex()));
            Site_orieZ_List.append(Coord_orieZ_List.at(comboBox_1->currentIndex()));
            Site_orieW_List.append(Coord_orieW_List.at(comboBox_1->currentIndex()));

          qDebug()<<"X轴"<<Site_X_List<<Site_X_List.count();
          qDebug()<<"Y轴"<<Site_Y_List<<Site_Y_List.count();
          qDebug()<<"Z轴"<<Site_Z_List<<Site_Z_List.count();
          qDebug()<<"orieX"<<Site_X_List<<Site_orieX_List.count();
          qDebug()<<"orieY"<<Site_Y_List<<Site_orieY_List.count();
          qDebug()<<"orieZ"<<Site_Z_List<<Site_orieZ_List.count();
          qDebug()<<"orieW"<<Site_Z_List<<Site_orieW_List.count();

            switch (comboBox_2->currentIndex())
            {
            case 0:
              ui->tableWidget_2->setItem(site_num,1,new QTableWidgetItem("终点"));
              break;
            case 1:
              ui->tableWidget_2->setItem(site_num,1,new QTableWidgetItem("停靠点"));
              break;
            case 2:
              ui->tableWidget_2->setItem(site_num,1,new QTableWidgetItem("充电站"));
              break;

            default:
              break;
            }
            QString time_str = "时间  "+QString::number(spinbox->value())+"S";
            switch (comboBox_3->currentIndex())
            {        
            case 0:          
              ui->tableWidget_2->setItem(site_num,2,new QTableWidgetItem(time_str));
              break;
            case 1:
              ui->tableWidget_2->setItem(site_num,2,new QTableWidgetItem("按键"));
              break;
            case 2:
              ui->tableWidget_2->setItem(site_num,2,new QTableWidgetItem("指令"));
              break;
            
            default:
              break;
            }
            switch (comboBox_4->currentIndex())
            {
            case 0:
              ui->tableWidget_2->setItem(site_num,3,new QTableWidgetItem(time_str));
              break;
            case 1:
              ui->tableWidget_2->setItem(site_num,3,new QTableWidgetItem("按键"));
              break;
            case 2:
              ui->tableWidget_2->setItem(site_num,3,new QTableWidgetItem("指令"));
              break;
            
            default:
              break;
            }

            ui->tableWidget_2->setItem(site_num,4,new QTableWidgetItem("删除"));
            site_num++;
            // pub_data();
            window->close(); 
      }
    });

    window->show();
}

void VizlibTest::onItemClicked(QTableWidgetItem* item)

{
    // qDebug()<<"表格1信号";
    int row = item->row();

    int colum = item->column();

    // qDebug()<<tr("row: %1, colum: %2").arg(row).arg(colum);
    if(colum == 4){//判断是否按下删除格
        ui->tableWidget->removeRow(row);
        Coord_orieX_List.removeAt(row);
        Coord_orieY_List.removeAt(row);
        Coord_orieZ_List.removeAt(row);
        Coord_orieW_List.removeAt(row);
        pub_marker();//将坐标点发布显示出来
        coord_num--;
    }

}

void VizlibTest::onItemClicked_2(QTableWidgetItem* item)

{
    // qDebug()<<"表格2信号";
    int row = item->row();

    int colum = item->column();

    // qDebug()<<tr("row: %1, colum: %2").arg(row).arg(colum);
    if(colum == 4){
        ui->tableWidget_2->removeRow(row);
        Site_X_List.removeAt(row);
        Site_Y_List.removeAt(row);
        Site_Z_List.removeAt(row);

        Site_time_List.removeAt(row);
        Site_orieX_List.removeAt(row);
        Site_orieY_List.removeAt(row);
        Site_orieZ_List.removeAt(row);
        Site_orieW_List.removeAt(row);
        // qDebug()<<"X轴"<<Site_X_List<<Site_X_List.count();
        // qDebug()<<"Y轴"<<Site_Y_List<<Site_Y_List.count();
        // qDebug()<<"Z轴"<<Site_Z_List<<Site_Z_List.count();
        // pub_data();//将站点发布显示出来
        site_num--;
    }

}


void VizlibTest::on_btn_saveas_xml_clicked(void)
{
    char outBuffer[200] ;
    FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/site", "r" );   //获取文件路径
    if(!FileName){  // 检测流管道
        perror("Fail to popen\n");
        return ;
    }
    while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
      // printf("输出: %s",outBuffer); // 打印输出
    }
    pclose(FileName); 

    //  std::cout << outBuffer << std::endl;
     QString str  = QString::fromStdString(outBuffer);
     str  =   str.simplified();//去掉换行符号
    qDebug()<<str;
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save as xml"),QString(str), "XML files (*.xml);;");


    if (filepath == "") return;
    QFile file(filepath);
    if (!file.open(QFile::WriteOnly | QIODevice::Text)) return;

    QXmlStreamWriter writer(&file);
    writer.setAutoFormatting(true);
    writer.writeStartDocument();//版本号，默认1.0
    writer.writeStartElement("testtable");
    int row = ui->tableWidget->rowCount();
    int col = ui->tableWidget->columnCount();
    // qDebug()<<"row"<<row<<"col"<<col;
    for(int i = 0; i < row; i++)
    {
        writer.writeStartElement("row");
        for(int j = 0; j < col; j++)
        {
            writer.writeTextElement("col",ui->tableWidget->item(i,j)->text());
        }
        // qDebug()<<"fff"<<QString::number(Coord_orieX_List.at(i))<<row;
        writer.writeTextElement("orie_X",QString::number(Coord_orieX_List.at(i)));
        writer.writeTextElement("orie_Y",QString::number(Coord_orieY_List.at(i)));
        writer.writeTextElement("orie_Z",QString::number(Coord_orieZ_List.at(i)));
        writer.writeTextElement("orie_W",QString::number(Coord_orieW_List.at(i)));
        writer.writeEndElement();
    }
    // qDebug()<<"Coord_orieX_List"<<Coord_orieX_List<<Coord_orieX_List.count();
    writer.writeEndElement();
    writer.writeEndDocument();
    file.close();
}

void VizlibTest::on_btn_readfromxml_clicked(void)
{    
  char outBuffer[200] ;
    FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/site/", "r" );   //获取文件路径
    if(!FileName){  // 检测流管道
        perror("Fail to popen\n");
        return ;
    }
    while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
      // printf("输出: %s",outBuffer); // 打印输出
    }
    pclose(FileName); 

    //  std::cout << outBuffer << std::endl;
     QString str  = QString::fromStdString(outBuffer);
     str  =   str.simplified();//去掉换行符号
    qDebug()<<str;

    QString filename_xml = QFileDialog::getOpenFileName(this,"选择线路文件",QString(str), tr("XML(*.xml);;"));
    QFile file(filename_xml);
    if (!file.open(QFile::ReadOnly)) return;

     for (size_t i = 0; i < ui->tableWidget->rowCount(); i++)  //清除所有
    {
      ui->tableWidget->removeRow( i);
        Coord_orieX_List.removeAt( i);
        Coord_orieY_List.removeAt( i);
        Coord_orieZ_List.removeAt( i);
        Coord_orieW_List.removeAt( i);
    }
    coord_num = 0;

    ui->tableWidget->setRowCount(0);
    ui->tableWidget->setColumnCount(5);
    QXmlStreamReader xmlreader(&file);
    int i = -1;//行号
    int j = -1;//列号
    xmlreader.readNext();

    while(!xmlreader.atEnd())
    {
        xmlreader.readNext();
        if(xmlreader.isStartElement() && xmlreader.name() == "row")
        {
            i++;
            j = -1;//初始化列号
            int row_count = ui->tableWidget->rowCount(); //获取表单行数
            ui->tableWidget->insertRow(row_count); //插入新行
        }
        else if(xmlreader.name() == "col")
        {
            j++;
            // QLineEdit *qle = new QLineEdit;
            // qle->setText(xmlreader.readElementText());
            ui->tableWidget->setItem(i,j,new QTableWidgetItem(xmlreader.readElementText()));
        }
        else if(xmlreader.name() == "orie_X")
        {
            j++;
            Coord_orieX_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_Y")
        {
            j++;
            Coord_orieY_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_Z")
        {
            j++;
            Coord_orieZ_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_W")
        {
            j++;
            Coord_orieW_List.append(QString(xmlreader.readElementText()).toFloat());
        }
    }
    // qDebug()<<"Coord_orieX_List"<<Coord_orieX_List<<Coord_orieX_List.count();
    // qDebug()<<"Coord_orieY_List"<<Coord_orieY_List<<Coord_orieY_List.count();
    // qDebug()<<"Coord_orieZ_List"<<Coord_orieZ_List<<Coord_orieZ_List.count();
    // qDebug()<<"Coord_orieW_List"<<Coord_orieW_List<<Coord_orieW_List.count();
    coord_num = ui->tableWidget->rowCount(); //获取表单当前行数
    pub_marker();//将坐标点发布显示出来
    file.close();
}

void VizlibTest::on_btn_saveas_site_clicked(void)//保存站点列表
{
    char outBuffer[200] ;
    FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/site/", "r" );   //获取文件路径
    if(!FileName){  // 检测流管道
        perror("Fail to popen\n");
        return ;
    }
    while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
      // printf("输出: %s",outBuffer); // 打印输出
    }
    pclose(FileName); 

    //  std::cout << outBuffer << std::endl;
    QString str  = QString::fromStdString(outBuffer);
    str  =   str.simplified();//去掉换行符号
    qDebug()<<str;
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save as xml"),QString(str), "XML files (*.xml);;");

    if (filepath == "") return;
    QFile file(filepath);
    if (!file.open(QFile::WriteOnly | QIODevice::Text)) return;

    QXmlStreamWriter writer(&file);
    writer.setAutoFormatting(true);
    writer.writeStartDocument();//版本号，默认1.0
    writer.writeStartElement("testtable");
    int row = ui->tableWidget_2->rowCount();
    int col = ui->tableWidget_2->columnCount();
    // qDebug()<<"row"<<row<<"col"<<col;
    for(int i = 0; i < row; i++)
    {
        writer.writeStartElement("row");
        for(int j = 0; j < col; j++)
        {
            writer.writeTextElement("col",ui->tableWidget_2->item(i,j)->text());
        }
        // qDebug()<<"fff"<<QString::number(Coord_orieX_List.at(i))<<row;
        
        writer.writeTextElement("Time",QString::number(Site_time_List.at(i)));
        writer.writeTextElement("X",QString::number(Site_X_List.at(i)));
        writer.writeTextElement("Y",QString::number(Site_Y_List.at(i)));
        writer.writeTextElement("Z",QString::number(Site_Z_List.at(i)));

        writer.writeTextElement("orie_X",QString::number(Site_orieX_List.at(i)));
        writer.writeTextElement("orie_Y",QString::number(Site_orieY_List.at(i)));
        writer.writeTextElement("orie_Z",QString::number(Site_orieZ_List.at(i)));
        writer.writeTextElement("orie_W",QString::number(Site_orieW_List.at(i)));
        writer.writeEndElement();
    }
    // qDebug()<<"Coord_orieX_List"<<Coord_orieX_List<<Coord_orieX_List.count();
    writer.writeEndElement();
    writer.writeEndDocument();
    file.close();
}


void VizlibTest::on_btn_readfromsite_clicked(void)
{
    char outBuffer[200] ;
    FILE *FileName = popen("realpath $(rospack find startingup_ros)/data/Map/${AUTOCAR_MAP_TYPE}/map/site/", "r" );   //获取文件路径${AUTOCAR_MAP_TYPE}
    if(!FileName){  // 检测流管道
        perror("Fail to popen\n");
        return ;
    }
    while(fgets(outBuffer, 1024, FileName) != NULL){ // 获取输出
      // printf("输出: %s",outBuffer); // 打印输出
    }
    pclose(FileName); 

    //  std::cout << outBuffer << std::endl;
     QString str  = QString::fromStdString(outBuffer);
     str  =   str.simplified();//去掉换行符号
    

    QString filename_xml = QFileDialog::getOpenFileName(this,"选择线路文件",QString(str), tr("XML(*.xml);;"));
    QFile file(filename_xml);
    if (!file.open(QFile::ReadOnly)) return;

     for (size_t i = 0; i <= ui->tableWidget_2->rowCount(); i++)
    {
        ui->tableWidget_2->removeRow(i);
        // Site_X_List.removeAt(i);
        // Site_Y_List.removeAt(i);
        // Site_Z_List.removeAt(i);

        // Site_time_List.removeAt(i);
        // Site_orieX_List.removeAt(i);
        // Site_orieY_List.removeAt(i);
        // Site_orieZ_List.removeAt(i);
        // Site_orieW_List.removeAt(i);
    }
        Site_X_List.clear();
        Site_Y_List.clear();
        Site_Z_List.clear();

        Site_time_List.clear();
        Site_orieX_List.clear();
        Site_orieY_List.clear();
        Site_orieZ_List.clear();
        Site_orieW_List.clear();
    qDebug()<<"str"<<ui->tableWidget_2->rowCount();
    site_num = 0;//站点数

    ui->tableWidget_2->setRowCount(0);
    ui->tableWidget_2->setColumnCount(5);
    QXmlStreamReader xmlreader(&file);
    int i = -1;//行号
    int j = -1;//列号
    xmlreader.readNext();

    while(!xmlreader.atEnd())
    {
        xmlreader.readNext();
        if(xmlreader.isStartElement() && xmlreader.name() == "row")
        {
            i++;
            j = -1;//初始化列号
            int row_count = ui->tableWidget_2->rowCount(); //获取表单行数
            ui->tableWidget_2->insertRow(row_count); //插入新行
        }
        else if(xmlreader.name() == "col")
        {
            j++;
            // QLineEdit *qle = new QLineEdit;
            // qle->setText(xmlreader.readElementText());
            ui->tableWidget_2->setItem(i,j,new QTableWidgetItem(xmlreader.readElementText()));
        }
        else if(xmlreader.name() == "Time")
        {
            j++;
            Site_time_List.append(QString(xmlreader.readElementText()).toInt());
        }
        else if(xmlreader.name() == "X")
        {
            j++;
            Site_X_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "Y")
        {
            j++;
            Site_Y_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "Z")
        {
            j++;
            Site_Z_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_X")
        {
            j++;
            Site_orieX_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_Y")
        {
            j++;
            Site_orieY_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_Z")
        {
            j++;
            Site_orieZ_List.append(QString(xmlreader.readElementText()).toFloat());
        }
        else if(xmlreader.name() == "orie_W")
        {
            j++;
            Site_orieW_List.append(QString(xmlreader.readElementText()).toFloat());
        }
    }
    qDebug()<<"Site_X_List"<<Site_X_List<<Site_X_List.count();
    qDebug()<<"Site_Y_List"<<Site_Y_List<<Site_Y_List.count();
    qDebug()<<"Site_Z_List"<<Site_Z_List<<Site_Z_List.count();

    qDebug()<<"Site_orieX_List"<<Site_orieX_List<<Site_orieX_List.count();
    qDebug()<<"Site_orieY_List"<<Site_orieY_List<<Site_orieY_List.count();
    qDebug()<<"Site_orieZ_List"<<Site_orieZ_List<<Site_orieZ_List.count();
    qDebug()<<"Site_orieW_List"<<Site_orieW_List<<Site_orieW_List.count();
    site_num = ui->tableWidget_2->rowCount(); //获取表单当前行数
    // pub_data();//将站点发布出来
    file.close();
}

//坐标轴设置
void VizlibTest::coordinate_set()
{
    // qDebug()<<"坐标设置";

    QDialog *window = new QDialog(this);
    window->setWindowTitle(QObject::tr("增加坐标点"));
    window->resize(300,50);

    QLabel *label = new QLabel("坐标名:");
    QPushButton *button = new QPushButton("确定");
    QLineEdit *linetdit = new QLineEdit;
    // linetdit->setReadOnly(true);  //设置只读
    // linetdit->setText("--------------");

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(label);
    layout->addWidget(linetdit);
    layout->addWidget(button);

    window->setLayout(layout);
        

    connect(button, &QPushButton::clicked,[=](){
        // qDebug()<<"坐标名"<<linetdit->text();
        if(linetdit->text() != NULL){
            SET_FLAG = true;//设置坐标点标志位
            coordiname = linetdit->text();
            if(ROS_LINK)
                map_rviz->Set_Goal();
                window->close();
        }

    });
    window->show();
}

void VizlibTest::display_rviz() {

  QTimer::singleShot(500,[=](){
      map_rviz->Display_Grid(true, "QGrid", 20,
                            QColor(160, 160, 160));
  });

      QTimer::singleShot(500,[=](){
      ROS_INFO("模型显示");
      map_rviz->Display_RobotModel(true);
    });

    map_rviz->setViews("map","rviz/Orbit");
    map_rviz->Display_MarkerArray(true,"/MarkerArray");
    map_rviz->Display_PcdMap(true, "/points_map");
    map_rviz->Display_VectorMap(true,"/vector_map");
    map_rviz->Display_center_lines(true,"/vector_map_center_lines_rviz");

}

void VizlibTest::slot_btn_connect()
{
         ROS_WARN("连接");
        bool isConnect = init("http://127.0.0.1:11311/", "127.0.0.1");
        // qDebug()<<"isConnect"<<isConnect;
        if(isConnect){
              
                ui->mapViz_2->hide();
                // ui->widget_rviz_2->show();
                if (map_rviz == NULL) {
                    map_rviz = new QRviz(ui->verticalLayout_build_map_2, "qrviz");
                    map_rviz->SetGlobalOptions("map",  QColor(48, 48, 48), 30);//建立全局坐标系否则图片显示卡顿
                }
                display_rviz();   //显示RVIZ

                ROS_LINK = true;    //ROS连接成功标志
          }
        else{
                QMessageBox::information(NULL, "连接失败",
                                      "连接失败！请检查你的连接配置或重启重试！",
                                      QMessageBox::Yes);
                  ROS_LINK = false;
          }
}

void VizlibTest::closeEvent(QCloseEvent *event)
{
  if(ROS_LINK)
      ros::shutdown();
  // this->close();
  // system("pkill -f ros &");//先kill掉全局规划节点
}
