/*
	* Date: 2022.3.26
	* Details: 主要相关联全局变量以及函数定义
*/
#include "../header/General.h"
#include "../header/RemoteController.h"

cv::Mat src = cv::Mat::zeros(600, 800, CV_8UC3);   // Transfering buffer

string Armor_videopath = "../Video/Armor1.mp4";         // 装甲板测试视频路径
string Buff_videopath = "";          // 能量机关测试视频路径

bool bool_Run = true;		// 程序是否运行

mutex reciveRes;		//线程读取资源锁

CarData getStm32;		//读取数据

string paramFileName;    // 参数读取路径

Color ENEMYCOLOR = GREEN;     // 所需击打的装甲板颜色
bool read_succeed = false;	// 是否读取成功
Color last_color = GREEN;	//读取上一次的颜色

/* -------------------相机参数--------------------- */
// 相机参数读取目录
string CameraFileName = "../DebugParam.xml";
// 相机内参外参读取路径
string paramCameraName = "../cameraParamsGalaxy6mm1.xml";

//阈值
int GrayValue;
int RGrayWeightValue;
int BGrayWeightValue;

//hsv
int RLowH ;
int RHighH;
int RLowS ;
int RHighS ;
int RLowV ;
int RHighV ;

int BLowH ;
int BHighH;
int BLowS;
int BHighS;
int BLowV ;
int BHighV;

int V_ts;

//相机曝光
int GXExpTime ;             //大恒相机曝光值
int GXExpTimeValue;         //控制大恒相机曝光值动态变化
int GXGain ;                //大恒相机增益
int GXGainValue;            //控制大恒相机曝光增益值动态变化

void ArmorToData(pattern Car_model, double pitch, double yaw, int light_len) {
    Send.ArmorToData(Car_model, pitch, yaw, light_len);
}
