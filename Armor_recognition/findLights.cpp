#include "../header/Armor.h"

/*
	* Date: 2022.2.27
	* Details: 灯条检测相关函数源文件
*/

// 检测出所有可能的装甲板
void ArmorDetector::findLights() {
    vector<vector<Point>> lightContours;  //定义候选灯光轮廓
    Mat contourImg;
    srcImg_binary.copyTo(contourImg); //将图像复制到contourImg，防止findContours改变roiImg
    findContours(contourImg, lightContours, 0, 2); //CV_RETR_EXTERNAL = 0, CV_CHAIN_APPROX_SIMPLE = 2       最耗时的操作，需要进行优化
    RotatedRect lightRect;  // 拟合椭圆来的灯条旋转矩形
    LightBar light;  // 定义临时灯条

    for (const auto& lightContour : lightContours) {	// 迭代lightContours
        if (lightContour.size() < 6)	// 如果轮廓点数小于6，不可拟合椭圆
            continue;
        if (contourArea(lightContour) < armorParam.min_area)	// 面积筛选，滤去小的发光点
            continue;

        lightRect = fitEllipse(lightContour);	// 拟合椭圆
        light = LightBar(lightRect);			// 构造为灯条

        if (abs(light.angle) > armorParam.max_angle)	// 角度筛选，滤去一些竖直偏角偏大的发光点
            continue;

        lights.emplace_back(light);		// 在vector容器的尾部添加一个元素
    }
    // 灯条少于两条则设置状态为没找到灯条（要有超过两条的灯条才可以构造成一个装甲板）
    if (lights.size() < 2) {
        state = LIGHTS_NOT_FOUND;
        return;
    }

    // 依据灯条中心从左往右排序
    sort(lights.begin(), lights.end(), [](LightBar& a1, LightBar& a2) { return a1.center.x < a2.center.x; });
    state = LIGHTS_FOUND;
    return;
}