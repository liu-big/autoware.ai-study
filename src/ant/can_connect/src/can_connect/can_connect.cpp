/* 和底层打交道,需要循环去读取数据,所有需要while */
#include "can_connect/can_connect.h"

//1 构造函数
CanConnect::CanConnect() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("update_rate", update_rate_, 30);
  private_nh_.param<double>("can_port", can_port_, 8002);

  can_pub_ = nh_.advertise<autoware_can_msgs::CANPacket>("can_box_raw", 100, true);
  can_sub_ = nh_.subscribe("can_box_cmd", 100, &CanConnect::vehicleCanWriteCallback, this);

  // 选择不同 can设备进行通信.可选择socket或物理can
  ican_ctrl = new CanSocket(can_port_);
  if(!ican_ctrl->init())
  {
    exit(0);
  }
}

CanConnect::~CanConnect()
{
}

void CanConnect::vehicleCanWriteCallback(const autoware_can_msgs::CANPacket &msg)
{
  uint32_t id_ = msg.id;
  uint32_t send_len_;
  uint8_t data_[100];
  uint8_t data_t[8];

  memcpy(data_t, &msg.dat, sizeof(data_t));

  ican_ctrl->encapValue(data_, send_len_, id_, data_t,msg.len);

  if (ican_ctrl->can_is_ok() == true){
      ican_ctrl->send(data_, send_len_);
  }
}

void CanConnect::publishCanData(uint8_t *can_data)
{
  // SensorType type;
  uint32_t id_;
  uint8_t data_[8];
  if (ican_ctrl->parseCanValue(can_data, id_, data_)) // 得到某类型设备数据进行数据发布
  {
    autoware_can_msgs::CANPacket msg;
    msg.header.frame_id = "can_raw";
    msg.header.stamp = ros::Time::now();

    msg.id = id_;
    msg.len = sizeof(data_);
    for (int i = 0; i < sizeof(data_); i++){
      msg.dat[i] = data_[i];
    }

    can_pub_.publish(msg); // 自定义数据类型为 id+data
  }
}

void *CanConnect::recvCandata(void *arg)
{
  CanConnect *_this = (CanConnect *)arg;
  uint8_t recvdata[100];

  while (ros::ok())
  {
    if (_this->ican_ctrl->can_is_ok() == true)
    {
      if(_this->ican_ctrl->read(recvdata))
      {
        _this->publishCanData(recvdata);
      }
    }
  }
}

// 循环读取can数据
void CanConnect::run()
{
  // ros::Rate rate(update_rate_);

  // uint8_t recvdata[100];
  // while (ros::ok())
  // {
  //   ros::spinOnce();  
  //   if (ican_ctrl->can_is_ok() == true)
  //   {
  //     if(ican_ctrl->read(recvdata))
  //     {
  //       publishCanData(recvdata);
  //     }
  //   }

  //   rate.sleep();
  // }

  pthread_t th;
  if (pthread_create(&th, nullptr, CanConnect::recvCandata, this))
  {
    std::perror("pthread_create can_connect");
    return;
  }
  
  int ret = pthread_detach(th);
  if (ret != 0)
  {
    std::perror("pthread_detach can_connect");
    return;
  }

}

