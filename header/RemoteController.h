#pragma once

#ifndef RM_REMOTECONTROLLER_H
#define RM_REMOTECONTROLLER_H

#include "General.h"
#include "serialport.h"

using namespace cv;
using namespace ml;

class RemoteController
{
private:
    double BeginTime;
public:
    // 无参数构造函数
    RemoteController();
    // 从下位机接收数据
    void paraGetCar();
    // 上位机接收数据发送给下位机
    void paraReceiver();
    // 获取时间
    double getClock();
    // 汇总（接收）角度解算的数据，为下一步发送数据到下位机做准备
    void ArmorToData(pattern Car_model, double pitch, double yaw, int light_len);
};

// 全局收发数变量
extern RemoteController Send;
extern RemoteController Recive;

#endif //RM_REMOTECONTROLLER_H
