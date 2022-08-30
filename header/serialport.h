#pragma once

#ifndef RM_SERIALPORT_H
#define RM_SERIALPORT_H

// #define WINDOWS      // Windows模式下
#define LINUX           // Linux模式下

#include <iostream>
#include "General.h"
#include<cstdio>		/* 标准输入输出定义 */
#include<cstdlib>		/* 标准函数库定义 */
#include<unistd.h>		/* Unix 标准函数定义 */		// 在linux上自带的，VS中没有 需要在VS自带头文件目录中手动添加这个头文件
#include<sys/types.h>	/* 数据类型，比如一些XXX_t的那种 */
#include<sys/stat.h>	/* 定义了一些返回值的结构，没看明白 */
#include<fcntl.h>		/* 文件控制定义 */
#include<cerrno>		/* 错误号定义 */
#include<errno.h>		/* 通过错误码来回报错误资讯的宏 */

#ifdef LINUX
#include<termios.h>		/* PPSIX 终端控制定义 用于控制非同步通信端口 */		// linux自带 在windows中无法使用
#endif // LINUX
#ifdef WINDOWS
#include<Windows.h>
#endif // WINDOWS

#define serialport_xy

using namespace std;

struct serial_transmit_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0x80;		// 头帧 后续跟嵌入式沟通决定
    unsigned char end = 0x7f;		// 尾帧 后续跟嵌入式沟通决定
    void get_xy_data(int16_t x, int16_t y, int light_len);		// 对需要传输的数据进行处理（高八位低八位处理）
};
struct serial_receive_data
{
    unsigned char raw_data[20];
    int size;
    unsigned char head = 0x80;		// 头帧 后续跟嵌入式沟通决定
    unsigned char end = 0x7f;		// 尾帧 后续跟嵌入式沟通决定
};

class SerialPort {
public:
    SerialPort();
    // 串口初始化
    SerialPort(const char* filename, int buadrate, int& fd, int& last_fd);		// 参数：串口名称   波特率
    // 重启串口
    void restart_serial(int& fd, int& last_fd);
    // 发送数据
    void send_data(const struct serial_transmit_data& data, int& fd, int& last_fd);
    // 接收数据
    bool read_data(const struct serial_receive_data* data, int& mode, float& bulletspeed, float& buff_offset_x, float& buff_offset_y, int& col, int& fd, int& last_fd);
    // 关闭串口
    void close_port(int& fd);

#ifdef LINUX
    
#endif // LINUX
    bool success_;

#ifdef WINDOWS
    HANDLE hcom;
#endif

private:
    const char* file_name_;
    int buadrate_;
    float last_bullet_speed;
};

#endif //RM_SERIALPORT_H
