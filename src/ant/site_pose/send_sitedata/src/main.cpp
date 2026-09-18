
/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/ /*****************************************************************************
** Includes
*****************************************************************************/
#include <QWidget>
#include <QDialog>

#include <QDebug>
#include <QFileDialog>
#include <QXmlStreamWriter>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"  
#include "geometry_msgs/PoseArray.h"  
#include "visualization_msgs/Marker.h" 
#include "visualization_msgs/MarkerArray.h" 
#include "autoware_msgs/SitePoseArray.h" 
#include<iostream>
#include <sstream>

#include <locale.h>

std::string  _file_path = "";

// #include "loginwidget.h"
QList<float>Site_X_List;   //  构建一个QString类型列表类
QList<float>Site_Y_List;   //  构建一个QString类型列表类
QList<float>Site_Z_List;   //  构建一个QString类型列表类
QList<float>Site_orieX_List;   //  构建一个QString类型列表类
QList<float>Site_orieY_List;   //  构建一个QString类型列表类
QList<float>Site_orieZ_List;   //  构建一个QString类型列表类
QList<float>Site_orieW_List;   //  构建一个QString类型列表类

QList<float>Coord_orieX_List;   //  构建一个QString类型列表类
QList<float>Coord_orieY_List;   //  构建一个QString类型列表类
QList<float>Coord_orieZ_List;   //  构建一个QString类型列表类
QList<float>Coord_orieW_List;   //  构建一个QString类型列表类

QList<QString>Site_name_List;   //  构建一个QString类型列表类
QList<QString>Attributes;   //  构建一个QString类型列表类
QList<QString>trigger_1;   //  构建一个QString类型列表类
QList<QString>trigger_2;   //  构建一个QString类型列表类
QList<int>time_site;   //  构建一个QString类型列表类

ros::Publisher pub_SitePose;
/*****************************************************************************
** Main
*****************************************************************************/
void Readfrom_Site(void)
{
    // QString filename_xml = "/home/lao/sor_ws/test_2.xml";
    QFile file(QString::fromStdString(_file_path));
    if (!file.open(QFile::ReadOnly)) return;

//     ui->tableWidget_2->setRowCount(0);
//     ui->tableWidget_2->setColumnCount(5);
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
            // int row_count = ui->tableWidget_2->rowCount(); //获取表单行数
            // ui->tableWidget_2->insertRow(row_count); //插入新行
        }
        else if(xmlreader.name() == "col")
        {
            j++;
            switch (j)
            {
            case 0:
                Site_name_List.append(xmlreader.readElementText());
                break;
            case 1:
                Attributes.append(xmlreader.readElementText());
                break;
            case 2:
                trigger_1.append(xmlreader.readElementText());
                break;
            case 3:
                trigger_2.append(xmlreader.readElementText());
                break;
            // case 4:
            //     time_site.append(QString(xmlreader.readElementText()).toInt());
            //     break;
            
            default:
                break;
            }
           
        }
        else if(xmlreader.name() == "Time")
        {
            j++;
            time_site.append(QString(xmlreader.readElementText()).toInt());
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

    qDebug()<<"Site_name_List"<<Site_name_List<<Site_name_List.count();
    qDebug()<<"Attributes"<<Attributes<<Attributes.count();
    qDebug()<<"trigger_1"<<trigger_1<<trigger_1.count();
    qDebug()<<"trigger_2"<<trigger_2<<trigger_2.count();
    qDebug()<<"time_site"<<time_site<<time_site.count();
//     coord_num = ui->tableWidget->rowCount(); //获取表单当前行数
//     // pub_marker();//将坐标点发布显示出来
    file.close();
}

//发布launch命令信息
void pub_data(void) {
  autoware_msgs::SitePoseArray site_data;
  autoware_msgs::SitePose  siteposes;
//   send_sitedata::Mode  mode;


    site_data.header.frame_id = "map";
    site_data.header.stamp = ros::Time();

    // qDebug()<<"44444"<<ui->tableWidget_2->rowCount()<<Site_X_List.count();
    for(int i =0;i<Site_X_List.count();i++)
    {
        siteposes.pose.position.x = Site_X_List.at(i);
        siteposes.pose.position.y = Site_Y_List.at(i);
        siteposes.pose.position.z = Site_Z_List.at(i);

        siteposes.pose.orientation.x = Site_orieX_List.at(i);
        siteposes.pose.orientation.y = Site_orieY_List.at(i);
        siteposes.pose.orientation.z = Site_orieZ_List.at(i);
        siteposes.pose.orientation.w = Site_orieW_List.at(i);

        
        siteposes.site_name= Site_name_List.at(i).toStdString();   
        if(Attributes.at(i) == "终点")
            siteposes.Attributes = siteposes.START_SITE;
        else if(Attributes.at(i) == "停靠点")
            siteposes.Attributes = siteposes.DOCK_SITE;
        else if(Attributes.at(i) == "充电站")
            siteposes.Attributes = siteposes.GARAGE__SITE;

        if(trigger_1.at(i).contains("时间",Qt::CaseSensitive) == true)
            siteposes.trigger_1 = siteposes.TIME_TRI;
        else if(trigger_1.at(i) == "按键")
            siteposes.trigger_1 = siteposes.BUTTON;
        else if(trigger_1.at(i) == "指令")
            siteposes.trigger_1 = siteposes.INSTRUCTION;

        if(trigger_2.at(i).contains("时间",Qt::CaseSensitive) == true)
            siteposes.trigger_2 = siteposes.TIME_TRI;
        else if(trigger_2.at(i) == "按键")
            siteposes.trigger_2 = siteposes.BUTTON;
        else if(trigger_2.at(i) == "指令")
            siteposes.trigger_2 = siteposes.INSTRUCTION;

        if(trigger_1.at(i).contains("时间",Qt::CaseSensitive) == true || trigger_2.at(i).contains("时间",Qt::CaseSensitive) == true)
            siteposes.time = time_site.at(i);
        else 
             siteposes.time = 0;
        // ROS_INFO("test");
        // ROS_INFO("进入循环");
        site_data.sites.push_back(siteposes);
    }
    pub_SitePose.publish(site_data);
}

int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE,"zh_CN.utf8");
    // ROS节点初始化
    ros::init(argc, argv, "Site_rArrayPub");     
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    pub_SitePose = nh.advertise<autoware_msgs::SitePoseArray>("/multi_goal", 1,true);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    private_nh.getParam("file_path", _file_path);

    std::cout << "file_path is " << _file_path<<std::endl;

    Readfrom_Site();

    // while (ros::ok())
    // {
    //     pub_data();

    //     // 按照循环频率延时
    //     loop_rate.sleep();
    // }

  while (0 == pub_SitePose.getNumSubscribers()) {
    ROS_INFO_ONCE("Waiting for subscribers to connect");
    ros::Duration(0.1).sleep();
  }
    
  pub_data();
  sleep(3);
  // ros::spin();
}
