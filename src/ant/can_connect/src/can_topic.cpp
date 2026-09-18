bool parseCanValue(uint8_t *data)
{
    bool ret = false;
    uint8_t can_data[13];
    for (int i = 0; i < 13; i++)
    {
        can_data[i] = _can_data[i];
    }

    if (can_data[0] == 0x08)
    {
        // 校验数据的有效性,数据累计值
        // if(!ctrl->check_ok(&can_data[5])){
        //   return 0;
        // }
        uint16_t id_ = (can_data[3] << 8) | can_data[4];

        switch (id_)
        {
        case 0x200:
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
            type = SensorType::Vehicle;
            break;
        case 0x401:
        case 0x402:
        case 0x403:
        case 0x404:
            type = SensorType::Ultrasound;
            break;
        case 0xa01:
            type = SensorType::MillimeterWave;
            break;
        default:
            break;
        }
        ret = true;
    }

    return ret;
}

void CanConnect::publishCanData(const uint8_t *can_data)
{
    // SensorType type;
    uint16_t id_;
    uint8_t data[8];
    if (ictrl->parseCanValue(can_data, &id_, &data)) // 得到某类型设备数据进行数据发布
    {
        switch (type_)
        {
        case SensorType::Vehicle: // 车辆底盘
        {
            std_msgs::UInt8MultiArray msg;
            // msg.data = &data;
            vehicle_can_pub_.publish(msg); // 自定义数据类型为 id+data
        }
        break;
        case SensorType::Ultrasound: // 超声波
            break;
        case SensorType::MillimeterWave: // 毫米波
            break;
        default:
            break;
        }
    }
}

vehicle_can_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/vehicle_can_read", 100, true);
vehicle_can_sub_ = nh_.subscribe("/vehicle_can_write", 100, &CanConnect::vehicleCanReadCallback, this);

enum class SensorType
{
    Vehicle,
    Ultrasound,
    MillimeterWave,
};

