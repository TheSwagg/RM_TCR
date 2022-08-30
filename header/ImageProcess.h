#pragma once

/*
	* Date: 2022.3.24
	* Details: 图像获取和处理控制
*/

#ifndef RM_IMAGEPROCESS_H
#define RM_IMAGEPROCESS_H

#include "Armor.h"
#include "AngleSolver_2.h"

class ImageProcess {
public:
    // 无参数构造函数
    ImageProcess();

    // 获取图像函数
    void ImageProducter();

    // 图像处理函数
    void ImageConsumer();

    // 保存参数函数
    void SaveData();
};

#endif //RM_IMAGEPROCESS_H
