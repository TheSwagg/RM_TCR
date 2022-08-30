#pragma once

#ifndef RM_GENERAL_H
#define RM_GENERAL_H

#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
// 互斥量类是一个同步原语，可以用来保护共享数据不被多个线程同时访问
#include<mutex>

#define DEBUG_MODE
#define RELEASE_MODE
#define PI 3.14159
#define IMSHOW
// #define Serial_Path "/dev/485_USB"      // 串口路径
#define Serial_Path "/dev/ttyUSB0"	   // linux默认串口路径
#define Serial_Baud 0                   // 串口波特率(0表示115200    1表示921600)

using namespace std;
using namespace cv;
// 外部变量
// extern pthread_mutex_t Globalmutex; // 由于图像更新导致的线程冲突
// extern pthread_cond_t GlobalCondCV; // 由于图像更新导致的线程冲突
// extern bool imageReadable;          // 由于图像更新导致的线程冲突
extern cv::Mat src;                    // 定义源图像src
extern string Armor_videopath;         // 装甲板测试视频路径
extern string Buff_videopath;          // 能量机关测试视频路径
extern bool bool_Run;		// 程序是否运行
extern mutex reciveRes;     //线程读取资源锁
extern string paramFileName;    // 参数读取路径
extern string paramCameraName;	// 相机内参外参读取路径
extern bool read_succeed;	// 是否读取成功

// 图像更新线程
// void* imageUpdatingThread(void* PARAM);

// 装甲板识别线程
// void* armorDetectingThread(void* PARAM);

// 战车击打模式
enum HitType
{
    SightType = 0,      // 自瞄模式
    EnergyType = 1      // 能量机关模式
};

// 是否发射子弹
enum BoolHit
{
    Hit_yes = 0,        // 发射子弹
    Hit_no = 1          // 不发射子弹
};

// 装甲板类型
enum ArmorType
{
    SMALL_ARMOR = 0,    // 小装甲板 步兵，英雄
    BIG_ARMOR = 1       // 大装甲板 哨兵
};

// 敌方战车颜色（灯条颜色） B蓝 G绿 R红
enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};
extern Color ENEMYCOLOR;        // 所需击打战车的灯条颜色
extern Color last_color;	//读取上一次的颜色

// 射击模式
enum pattern {
    FirstFind,       // 首次识别
    Shoot,           // 连续射击
    Stop,            // 非连续
    buffering        // 缓冲
};

// 读取图像模式
typedef enum {
    VIDEO=0,              // 视频读取
    DAHENG=1,             // 大恒相机
    BUFF=2,                // 大符
} ImageGetMode;

// 上位机接收当前的yaw和pitch值
struct CarData {
    double pitch = 0;       // 获取战车当前的pitch值
    double yaw = 0;         // 获取战车当前的yaw值
    float ShootSpeed = 16;  // 子弹射速
    bool IsBuffMode = false;
    double BeginToNowTime = 0;      // 获取时间
    Color c = GREEN;
    ImageGetMode model_ = BUFF;
};

// 击打缓冲计算返回
typedef struct {
    float pitch;
    float yaw;
    float t;     // 击打弹道时间
    float angle;
} Angle_t;

// 采入图像
struct ImageDate {
    cv::Mat SrcImage;
    ImageGetMode mode;      // 读取图像模式
    CarData ReciveStm32;    // 下位机通过串口传输而来
};
//待击打大符信息
struct RM_BuffData{
    float tx = 0;
    float ty = 0;
    float tz = 0;
    float pitch = 0;
    float yaw = 0;
    RotatedRect box;
    Point2f point[4];
};
typedef enum{
    BUFF_FIRST_SHOOT,
    BUFF_CONTINUE_SHOOT,
    BUFF_BUFFER_SHOOT
} BuffStatus  ;

extern CarData getStm32;    // 读取下位机发送的数据

// 获取两个点的距离
float getPointsDistance(const cv::Point2f& a, const cv::Point2f& b);

// 根据打击优先级增加装甲板的打击度
void setNumScore(const int& armorNum, const int& targetNum, float& armorScore);

//
void ArmorToData(pattern Car_model, double pitch, double yaw, int light_len);

/* -------------------相机参数--------------------- */
// 相机参数读取目录
extern string CameraFileName;

//阈值
extern int GrayValue;
extern int RGrayWeightValue;
extern int BGrayWeightValue;

//hsv
extern int RLowH ;
extern int RHighH;
extern int RLowS ;
extern int RHighS ;
extern int RLowV ;
extern int RHighV ;

extern int BLowH ;
extern int BHighH;
extern int BLowS;
extern int BHighS;
extern int BLowV ;
extern int BHighV;

extern int V_ts;

//相机曝光
extern int GXExpTime ;             //大恒相机曝光值
extern int GXExpTimeValue;         //控制大恒相机曝光值动态变化
extern int GXGain ;                //大恒相机增益
extern int GXGainValue;            //控制大恒相机曝光增益值动态变化

//装甲符号
extern int ShootArmorNumber;

double GetDistance(Point2f a,Point2f b);
double GetDistance(Point3f a,Point3f b);                        //空间两点距离
Point2f GetCircle(Point2f point_one, Point2f point_two,Point2f point_three);
Point3f solveCenterPointOfCircle(vector<Point3f> pt);           //空间3点得圆心
Point2f getArmorCenter(Point2f p[4]);               //得到装甲中心
float getArmorAngle(Point2f p[4]);
double GetAngle(Point2f a,Point2f b,Point2f c);         //平面三点得a的夹角
double GetAngle(Point3f a,Point3f b,Point3f c);         //空间三点得a的夹角

//得到两点距离
inline double GetDistance(Point2f a,Point2f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

//得到空间两点距离
inline double GetDistance(Point3f a,Point3f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2)+pow(a.z - b.z,2));
}

//得到装甲中心
inline Point2f getArmorCenter(Point2f p[4]){
    return Point2f((p[0].x+p[1].x+p[2].x+p[3].x)/4,(p[0].y+p[1].y+p[2].y+p[3].y)/4);
}

//得到装甲倾斜角度
inline float getArmorAngle(Point2f p[4]){
    return((atan2(p[0].y - p[1].y,p[1].x - p[0].x)+atan2(p[3].y - p[2].y,p[2].x - p[3].x))/2);
}
//平面三点拟合得到圆心
inline Point2f GetCircle(Point2f point_one, Point2f point_two,Point2f point_three){
    //x1，y1为box_buff中心点，x2，y2为last_box_buff中心点
    float x1 = point_one.x;
    float x2 = point_two.x;
    float x3 = point_three.x;
    float y1 = point_one.y;
    float y2 = point_two.y;
    float y3 = point_three.y;

    //******************利用三点求圆心************
    float A1 = 2*(x2-x1);
    float B1 = 2*(y2-y1);
    float C1 = x2*x2+y2*y2-x1*x1-y1*y1;
    float A2 = 2*(x3-x2);
    float B2 = 2*(y3-y2);
    float C2 = x3*x3+y3*y3-x2*x2-y2*y2;
    float X = ((C1*B2)-(C2*B1))/((A1*B2)-(A2*B1));
    float Y = ((A1*C2)-(A2*C1))/((A1*B2)-(A2*B1));
    return Point2f(X,Y);
    //**********************************************
}

//空间3点得圆心
inline Point3f solveCenterPointOfCircle(vector<Point3f> pt)
{
    double a1, b1, c1, d1;
    double a2, b2, c2, d2;
    double a3, b3, c3, d3;

    double x1 = pt[0].x, y1 = pt[0].y, z1 = pt[0].z;
    double x2 = pt[1].x, y2 = pt[1].y, z2 = pt[1].z;
    double x3 = pt[2].x, y3 = pt[2].y, z3 = pt[2].z;

    a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
    b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
    c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
    d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    a2 = 2 * (x2 - x1);
    b2 = 2 * (y2 - y1);
    c2 = 2 * (z2 - z1);
    d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

    a3 = 2 * (x3 - x1);
    b3 = 2 * (y3 - y1);
    c3 = 2 * (z3 - z1);
    d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

    double x = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
               /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    double y =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
                /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    double z = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
               /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

    cout<<"计算所得圆心:x:"<<x<<" y:"<<y<<"   z:"<<z<<endl;
    return Point3f(x,y,z);
}


//根据余弦定理，得到夹角,a为顶角
/**
 * @brief GetAngle
 * @param a
 * @param b
 * @param c
 * @param src
 * @return
 * @remark 2维空间,a为顶角
 */
inline double GetAngle(Point2f a,Point2f b,Point2f c){
    double DistanceA = GetDistance(b,c);
    double DistanceB = GetDistance(a,c);
    double DistanceC = GetDistance(b,a);

    double angle_cos = (DistanceB*DistanceB+DistanceC*DistanceC-DistanceA*DistanceA)/(2*DistanceB*DistanceC);
    cout<<angle_cos<<endl;
    double angle = acos(angle_cos);
    return 180*angle/PI;
}
//三维空间,a为顶角
inline double GetAngle(Point3f a,Point3f b,Point3f c){
    double DistanceA = GetDistance(b,c);
    double DistanceB = GetDistance(a,c);
    double DistanceC = GetDistance(b,a);

    double angle_cos = (DistanceB*DistanceB+DistanceC*DistanceC-DistanceA*DistanceA)/(2*DistanceB*DistanceC);
    cout<<angle_cos<<endl;
    double angle = acos(angle_cos);
    return 180*angle/PI;
}
#endif //RM_GENERAL_H
