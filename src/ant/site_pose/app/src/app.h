#ifndef LOGINWIDGET_H
#define LOGINWIDGET_H

#include <QWidget>
#include <QDialog>
#include <QStandardItemModel>

#include <QTableWidget>
#include <QTableWidgetItem>
#include <QComboBox>
#include <QSpinBox>
#include <QModelIndex>
#include <QFileDialog>
#include <QXmlStreamWriter>


#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>

#include <QTimer>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"  
#include "geometry_msgs/PoseArray.h"  
#include "visualization_msgs/Marker.h" 
#include "visualization_msgs/MarkerArray.h" 
// #include "app/SiteArray.h" 
#include<iostream>
#include <sstream>

#include "qrviz.hpp"
namespace Ui {
class VizlibTest;
}


class VizlibTest : public QWidget
{
    Q_OBJECT

public:
    explicit VizlibTest(int argc, char **argv, QWidget *parent = 0);
    ~VizlibTest();

    bool init(const std::string& master_url, const std::string& host_url);
    void ui_init();

 public slots:

    void on_btn_saveas_xml_clicked(void);
    void on_btn_readfromxml_clicked(void);
    void on_btn_saveas_site_clicked(void);//保存站点列表
    void on_btn_readfromsite_clicked(void);
    void coordinate_set();

    void Load_Points_Map(void);
    void Load_Vector_Map(void);

private slots:
    void slot_btn_connect();
    // void slot_btn_quit();
    void onItemClicked(QTableWidgetItem* item);
    void onItemClicked_2(QTableWidgetItem* item);
    void Add_Site(void);

private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel * render_panel_;
    Ui::VizlibTest *ui;

    QRviz *map_rviz = NULL;
    void display_rviz();
    void connections();
    void SubAndPubTopic();
    void pub_data(void);
    void pub_marker(void);
    void chatterCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);


private:

        bool ROS_LINK = false;
        bool SET_FLAG = false;//设置坐标点标志位

        ros::Publisher pub_SitePose;
        ros::Publisher markerArrayPub;
        ros::Subscriber sub_SitePose;
        

        QStandardItemModel  *model; //创建一个标准的条目模型

        QString coordiname;//坐标名称
        int coord_num = 0;//坐标数
       int site_num = 0;//站点数
        QList<float>Site_X_List;   //  构建一个QString类型列表类
        QList<float>Site_Y_List;   //  构建一个QString类型列表类
        QList<float>Site_Z_List;   //  构建一个QString类型列表类
        QList<float>Site_orieX_List;   //  构建一个QString类型列表类
        QList<float>Site_orieY_List;   //  构建一个QString类型列表类
        QList<float>Site_orieZ_List;   //  构建一个QString类型列表类
        QList<float>Site_orieW_List;   //  构建一个QString类型列表类
        QList<int>Site_time_List;   //  构建一个QString类型列表类

        QList<float>Coord_orieX_List;   //  构建一个QString类型列表类
        QList<float>Coord_orieY_List;   //  构建一个QString类型列表类
        QList<float>Coord_orieZ_List;   //  构建一个QString类型列表类
        QList<float>Coord_orieW_List;   //  构建一个QString类型列表类

protected:
    void closeEvent(QCloseEvent *event);
    
};


#endif // LOGINWIDGET_H
