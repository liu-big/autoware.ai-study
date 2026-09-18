/*
  订阅到数据处理后再发布
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// ros 头文件
#include <geometry_msgs/PoseStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_can_msgs/CANPacket.h>

#include <autoware_msgs/UltraSoundArray.h>
#include <autoware_msgs/UltraSound.h>

// class DirParams
// {
// public:
//     int front_left;
//     int front_right;
//     int left_front;
//     int left_rear;
//     int right_front;
//     int right_rear;
//     int rear_left;
//     int rear_right;

//     DirParams()
//     {
//         front_left=0;
//         front_right=1;
//         left_front=2;
//         left_rear = 3;
//         right_front=4;
//         right_rear=5;
//         rear_left = 6;
//         rear_right=7;
//     }
// };

class UltraBf19
{
public:
    UltraBf19() : nh_(), private_nh_("~")
    {
        private_nh_.param<int>("index0", dirparams_[0], 0);
        private_nh_.param<int>("index1", dirparams_[1], 0);
        private_nh_.param<int>("index2", dirparams_[2], 0);
        private_nh_.param<int>("index3", dirparams_[3], 0);
        private_nh_.param<int>("index4", dirparams_[4], 0);
        private_nh_.param<int>("index5", dirparams_[5], 0);
        private_nh_.param<int>("index6", dirparams_[6], 0);
        private_nh_.param<int>("index7", dirparams_[7], 0);
        private_nh_.param<int>("index8", dirparams_[8], 0);
        private_nh_.param<int>("index9", dirparams_[9], 0);
        private_nh_.param<int>("index10", dirparams_[10], 0);
        private_nh_.param<int>("index11", dirparams_[11], 0);

        private_nh_.param<int>("ultra_type", m_iUltraType, 0xb1);

        box_sub_ = nh_.subscribe("can_box_raw", 100, &UltraBf19::UltraBoxCallback, this);
        box_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_cmd", 10);

        ultra_pub_ = nh_.advertise<autoware_msgs::UltraSoundArray>("ultrasound_raw", 10, true);
        data_ok = false;

        // while(data_ok == false){
        //     publishCanBoxCmd();
        //     sleep(0.1);
        // }


      pthread_t th;
      if (pthread_create(&th, nullptr, &UltraBf19::publishthread, this) != 0)
      {
        std::perror("pthread_create");
        std::exit(1);
      }
    };
    ~UltraBf19(){};

private:
    // ros 相关变量
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber box_sub_;
    ros::Publisher box_pub_, ultra_pub_;
    bool data_ok;

    // params 获取参数服务器值
    double update_rate_; // replanning and publishing rate [Hz]
    std::string vehicle_name_;

    // classes 组合
    // VehicleInferface *ivehicle; // 必须为指针
    // DirParams dirparams_;
    int dirparams_[12];

    // ros msg variables
    // geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
    autoware_msgs::UltraSoundArray ultras_;

    // mist variables
    bool current_pose_initialized_;
    int m_iUltraType;
#if 0
    uint8_t transter_dir(uint8_t index)
    {
        uint8_t dir;
        if (index == dirparams_.front_left)
        {
            dir = autoware_msgs::UltraSound::FRONT_LEFT;
        }
        else if (index == dirparams_.front_right)
        {
            dir = autoware_msgs::UltraSound::FRONT_RIGHT;
        }
        else if (index == dirparams_.left_front)
        {
            dir = autoware_msgs::UltraSound::LEFT_FRONT;
        }
        else if (index == dirparams_.left_rear)
        {
            dir = autoware_msgs::UltraSound::LEFT_REAR;
        }
        else if (index == dirparams_.right_front)
        {
            dir = autoware_msgs::UltraSound::RIGHT_FRONT;
        }
        else if (index == dirparams_.right_rear)
        {
            dir = autoware_msgs::UltraSound::RIGHT_REAR;
        }
        else if (index == dirparams_.rear_left)
        {
            dir = autoware_msgs::UltraSound::REAR_LEFT;
        }
        else if (dirparams_.rear_right)
        {
            dir = autoware_msgs::UltraSound::REAR_RIGHT;
        }
        else
        {
            dir = autoware_msgs::UltraSound::FRONT_LEFT;
        }

        return dir;
    }
#else
    uint8_t transter_dir(uint8_t index)
    {
        uint8_t dir;
        if (index == 0)
        {
            dir = autoware_msgs::UltraSound::FRONT_LEFT;
        }
        else if (index == 1)
        {
            dir = autoware_msgs::UltraSound::FRONT_RIGHT;
        }
        else if (index == 2)
        {
            dir = autoware_msgs::UltraSound::LEFT_FRONT;
        }
        else if (index == 3)
        {
            dir = autoware_msgs::UltraSound::LEFT_REAR;
        }
        else if (index == 4)
        {
            dir = autoware_msgs::UltraSound::RIGHT_FRONT;
        }
        else if (index == 5)
        {
            dir = autoware_msgs::UltraSound::RIGHT_REAR;
        }
        else if (index == 6)
        {
            dir = autoware_msgs::UltraSound::REAR_LEFT;
        }
        else if (7)
        {
            dir = autoware_msgs::UltraSound::REAR_RIGHT;
        }
        else
        {
            dir = autoware_msgs::UltraSound::FRONT_LEFT;
        }

        return dir;
    }
#endif
    // callback
    void UltraBoxCallback(const autoware_can_msgs::CANPacket::ConstPtr &msg)
    {
        uint16_t can_id = msg->id;
        uint8_t data[12]={0};
        memcpy(&data, &msg->dat, sizeof(msg->dat));

        static int count;
        ultras_.header.stamp = ros::Time::now();
        ultras_.header.seq = count++;
        static bool low_flag = false;
        static bool hight_flag = false;
        static bool sub_hight_flag = false;
        int offset = 0;
        if (can_id == 0x611)
        {       
            if(low_flag == true)
            {
                ultras_.ultras.clear();
            }
            low_flag = true;
            offset = 0;
        }else if(can_id==0x612 ){       
            if(hight_flag == true)
            {
                ultras_.ultras.clear();
            }
            hight_flag = true;
            offset = 4;
        }else if(can_id==0x613 ){       
            if(sub_hight_flag == true)
            {
                ultras_.ultras.clear();
            }
            sub_hight_flag = true;
            offset = 8;
        }else{
            return;
        }

        for(int i=0;i < 4;i++)
        {
            autoware_msgs::UltraSound ultra;
            ultra.header.stamp = ros::Time::now();
            ultra.index = i + offset;

            if(dirparams_[ultra.index] == -1){                  // 不正常,过滤。id都发布
                continue;
            }
            ultra.direction = transter_dir(dirparams_[ultra.index]);
            ultra.distance = ((data[i*2] << 8) | data[i*2 + 1]) & 0xffffff;
            char str[20];
            sprintf(str,"%x",ultra.distance);
            ultra.distance = atol(str);
            ultras_.ultras.push_back(ultra);
        }

        if (low_flag == true && hight_flag == true && sub_hight_flag == true)
        {
            if(data[0] > 0xf0 || data[3] > 0xf0){
                return;
            }
            ultra_pub_.publish(ultras_);
            low_flag = false;
            hight_flag = false;
            sub_hight_flag = false;
            ultras_.ultras.clear();
            data_ok = true;
        }
    }

    // fucntions
    void publishCanBoxCmd()
    {
        static int count=0;
        autoware_can_msgs::CANPacket msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "Ultra_cmd";
        msg.header.seq = count;
        msg.id = 0x601;
        msg.count = count++;
        msg.dat[0] = m_iUltraType;
        msg.dat[1] = 0x1f;
        msg.dat[2] = 0xff;
        msg.len = 0x03;
        box_pub_.publish(msg);
    }

    static void *publishthread(void *arg)
    {
        UltraBf19 *_this = (UltraBf19 *)arg;
        //   ros::Rate rate(_this->update_rate_);
        while(_this->data_ok == false){
            _this->publishCanBoxCmd();
            sleep(1);
        }

    //   uint8_t recvdata[100];
    //   while (ros::ok())
    //   {
    //     _this->ivehicle->run();
    //     ros::spinOnce();
    //     rate.sleep();
    //   }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultra_bf19");
    UltraBf19 obj;

    ros::spin();
    return 0;
};
