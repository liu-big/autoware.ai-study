
#include "can_box/can_socket.hpp"

CanSocket::CanSocket(int port) : _client_sock(-1)
{
    _listen_port = port;
}

CanSocket::~CanSocket()
{
}

bool CanSocket::init()
{
    _sock = socket(AF_INET, SOCK_STREAM, 0);
    if (_sock == -1)
    {
        std::perror("socket");
        return false;
    }

    sockaddr_in addr;

    std::memset(&addr, 0, sizeof(sockaddr_in));
    addr.sin_family = PF_INET;
    addr.sin_port = htons(_listen_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    int yes = 1;
    setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(int));
    int ret = bind(_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret == -1)
    {
        std::perror("bind");
        goto error;
    }

    ret = listen(_sock, 20);
    if (ret == -1)
    {
        std::perror("listen");
        goto error;
    }

    // 启动线程,监听socket连接
    pthread_t th_connet;
    if (pthread_create(&th_connet, nullptr, run, (void *)this))
    {
        std::perror("socket connet pthread_create");
        goto error;
    }

    ret = pthread_detach(th_connet);
    if (ret != 0)
    {
        std::perror("socket connet pthread_detach");
        goto error;
    }

    return true;
error:
    close(_sock);
    return false;
}

bool CanSocket::send(uint8_t *data, uint32_t len)
{
    int ret = true;

    ssize_t n = write(_client_sock, data, len);
    if (n != len)
        ret = false;

    return ret;
}

bool CanSocket::read(uint8_t *data)
{
    int ret = true;

    uint8_t recvdata[13];
    ssize_t n = recv(_client_sock, recvdata, sizeof(recvdata), 0);
    if (n <= 0)
    {
        std::perror("recv");
        _client_sock = -1;
        ret = false;
    }else{
        memcpy(data, recvdata, sizeof(recvdata));
    }

    return ret;
}

bool CanSocket::can_is_ok()
{
    if (_client_sock != -1){
        return true;
    }else{
        return false;
    }
}

void CanSocket::encapValue(uint8_t *data, uint32_t &send_len, uint32_t &id, uint8_t *value,uint8_t data_len)
{
    uint8_t can_data[13];
    can_data[0] = data_len;
    can_data[1] = id >> 24 & 0xff;
    can_data[2] = id >> 16 & 0xff;
    can_data[3] = (id >> 8) & 0xff;
    can_data[4] = id & 0xff;

    for (int i = 0; i < 8; i++)
    {
        can_data[5+i] = value[i];
    }

    send_len = sizeof(can_data);
    memcpy(data, can_data, send_len);
}

// 解析can数据的设备类型
bool CanSocket::parseCanValue(uint8_t *data, uint32_t &id, uint8_t *value)
{
    bool ret = true;
    uint8_t can_data[13];
    for (int i = 0; i < 13; i++)
    {
        can_data[i] = data[i];
        // printf("%0x ",can_data[i]);
    }
    // printf("\n");
    if (can_data[0] == 0x08)
    {
        // 校验数据的有效性,数据累计值
        // if(!ctrl->check_ok(&can_data[5])){
        //   return 0;
        // }
        uint16_t tmp = (can_data[3] << 8) | can_data[4];
        id = tmp;

        for (int i = 0; i < 8; i++){
            value[i] = can_data[5 + i];
        }
    }
    else
    {
        ret = false;
    }

    return ret;
}

void *CanSocket::run(void *arg)
{
    CanSocket *_this = (CanSocket *)arg;

    sockaddr_in client;
    socklen_t len = sizeof(client);
    while (true)
    {
        // get connect to android
        std::cout << "Waiting access..." << std::endl;

        // 得到最新连接的句柄,覆盖原来老的句柄
        _this->_client_sock = accept(_this->_sock, reinterpret_cast<sockaddr *>(&client), &len);
        if (_this->_client_sock == -1)
        {
            std::perror("accept");
            break;
        }
        printf("client IP：[%s] \n", inet_ntoa(client.sin_addr));
        printf("client PORT：[%d] \n", ntohs(client.sin_port));
    }
}