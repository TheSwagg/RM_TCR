/*
	* Date: 2022.4.4
	* Details: 相机图像读取，图像处理，收发数线程初始化
	* Remarks: 利用多线程异步处理图像的获取与处理步骤，提高cpu的利用率，提高程序运行速度，图像的获取与处理控制采用生产者和消费者的方法
*/

#include <iostream>
#include <thread>		// 线程
#include "header/General.h"
#include "header/ImageProcess.h"
#include "header/RemoteController.h"

/**
 * When I wrote this, only God and I understood what I was doing
 * Now, God only knows
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *.............................................
 *          佛祖保佑             永无BUG
 *
 **/

// 获取相机参数
void getData();

int main()
{
    // 获取外部xml文件中的数据
    getData();

#ifdef DEBUG_MODE
    // 创建窗口
    // namedWindow("曝光");
    // namedWindow("BGR");
    // namedWindow("R-HSV");
    // namedWindow("B-HSV");

    // 曝光
    // createTrackbar("大恒相机曝光值", "曝光", &GXExpTime, 200);
    // createTrackbar("大恒相机曝光增益值", "曝光", &GXGain, 200);
    // rgb
    // createTrackbar("灰度二值化阈值", "BGR", &GrayValue, 255);
    // createTrackbar("R权重", "BGR", &RGrayWeightValue, 100);
    // createTrackbar("B权重", "BGR", &BGrayWeightValue, 100);

    // R-hsv
    // cvCreateTrackbar("RLowH", "R-HSV", &RLowH, 360); //Hue (0 - 179)
    // cvCreateTrackbar("RHighH", "R-HSV", &RHighH, 360);

    // cvCreateTrackbar("RLowS", "R-HSV", &RLowS, 255); //Saturation (0 - 255)
    // cvCreateTrackbar("RHighS", "R-HSV", &RHighS, 255);

    // cvCreateTrackbar("RLowV", "R-HSV", &RLowV, 255); //Value (0 - 255)
    // cvCreateTrackbar("RHighV", "R-HSV", &RHighV, 255);
    // cvCreateTrackbar("V", "R-HSV", &V_ts, 255);

    // B-hsv
    // cvCreateTrackbar("BLowH", "B-HSV", &RLowH, 360); //Hue (0 - 179)
    // cvCreateTrackbar("BHighH", "B-HSV", &RHighH, 360);

    // cvCreateTrackbar("BLowS", "B-HSV", &RLowS, 255); //Saturation (0 - 255)
    // cvCreateTrackbar("BHighS", "B-HSV", &RHighS, 255);

    // cvCreateTrackbar("BLowV", "B-HSV", &RLowV, 255); //Value (0 - 255)
    // cvCreateTrackbar("BHighV", "B-HSV", &RHighV, 255);
#endif

    // 线程创建
    ImageProcess process;
    std::thread t1(&ImageProcess::ImageProducter, process);		// 创建图像生产线程
    std::thread t2(&ImageProcess::ImageConsumer, process);		// 创建图像消费线程
    std::thread t4(&RemoteController::paraGetCar, Recive);		// 创建收数线程
    std::thread t3(&RemoteController::paraReceiver, Send);		// 创建发数线程
    t4.join();
    t3.join();
    t2.join();
    t1.join();

    return 0;
}

void getData() {
    // 参数读入
    FileStorage fs_param(CameraFileName,FileStorage::READ);
    if(!fs_param.isOpened()){
        cout<<"参数文件打开失败!"<<endl;
        exit(1);
    }
    // 参数传入
    // 传入rgb参数
    fs_param["RGB_BinaryValue"]["GrayValue"]>>GrayValue;
    fs_param["RGB_BinaryValue"]["RGrayValue"]>>RGrayWeightValue;
    fs_param["RGB_BinaryValue"]["BGrayValue"]>>BGrayWeightValue;
    // 传入hsv参数
    fs_param["HSV_BinaryValue"]["RLowH"]>>RLowH;
    fs_param["HSV_BinaryValue"]["RLowS"]>>RLowS;
    fs_param["HSV_BinaryValue"]["RLowV"]>>RLowV;
    fs_param["HSV_BinaryValue"]["RHighH"]>>RHighH;
    fs_param["HSV_BinaryValue"]["RHighS"]>>RHighS;
    fs_param["HSV_BinaryValue"]["RHighV"]>>RHighV;

    fs_param["HSV_BinaryValue"]["BLowH"]>>BLowH;
    fs_param["HSV_BinaryValue"]["BLowS"]>>BLowS;
    fs_param["HSV_BinaryValue"]["BLowV"]>>BLowV;
    fs_param["HSV_BinaryValue"]["BHighH"]>>BHighH;
    fs_param["HSV_BinaryValue"]["BHighS"]>>BHighS;
    fs_param["HSV_BinaryValue"]["BHighV"]>>BHighV;
    fs_param["HSV_BinaryValue"]["V_ts"]>>V_ts;

    // 传入曝光参数
    // fs_param["ExpTime"]["UsbExpTime"]>>UsbExpTime;
    fs_param["ExpTime"]["DahengExpTime"]>>GXExpTime;
    fs_param["ExpTime"]["DahengGain"]>>GXGain;
    // fs_param["ExpTime"]["UsbAspTime"]>>UsbAspTime;

    // UsbExpTimeValue = UsbExpTime;
    GXExpTimeValue = GXExpTime;
    GXGainValue = GXGain;
    // UsbAspTimeValue = UsbAspTime;

    fs_param.release();
}
