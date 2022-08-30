#include "../header/Armor.h"

/*
	* Date: 2022.2.27
	* Details: 灯条识别类源文件
*/

// 灯条无参数初始化构造函数
LightBar::LightBar() {
    lightRect = RotatedRect();	// 创建旋转矩形
    length = 0;		// 灯条长度（即为旋转矩形的高度）
    center = Point2f();		// 灯条中心（即为旋转矩形的质心）
    angle = 0;		// 灯条与竖直方向的夹角（即为旋转矩形的旋转角度）
}

// 灯条有参数构造函数
LightBar::LightBar(const RotatedRect& lightRect) {
    this->lightRect = lightRect;
    length = MAX(lightRect.size.height, lightRect.size.width);
    center = lightRect.center;
    if (lightRect.angle > 90)	// 保证角度在0 - 90度之间
        angle = lightRect.angle - 180;
    else
        angle = lightRect.angle;
}

// 析构函数
LightBar::~LightBar() {}