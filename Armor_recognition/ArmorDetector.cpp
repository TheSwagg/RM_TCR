#include "../header/Armor.h"

/*
	* Date: 2022.2.27
	* Details: 装甲板识别子类源文件
*/

ArmorParam armorParam = ArmorParam();

// 初始化无参数装甲板识别类
ArmorDetector::ArmorDetector() {
    state = LIGHTS_NOT_FOUND;	// 初始化识别类型为LIGHTS_NOT_FOUND（未找到灯条）
}

// 析构函数
ArmorDetector::~ArmorDetector() {}

// 设置敌方战车颜色 B篮=0 G绿-1 R红=2 （通过下位机串口传输而来）
void ArmorDetector::setEnemyColor(Color enemyColor) {
    this->enemyColor = enemyColor;
}

// 操作手设定的所需要击打装甲板的数字（通过下位机串口传输而来）
void ArmorDetector::setTargetNum(const int& targetNum) {
    this->targetNum = targetNum;
}

// 重设检测器（删除原有的灯条和装甲板状态），以便进行下一帧的检测
void ArmorDetector::resetDetector() {
    state = LIGHTS_NOT_FOUND;
    lights.clear();
    armors.clear();
}

// 载入源图像并设置ROI区域（当ROI模式开启，且上一帧找到目标装甲板时）
void ArmorDetector::setImg(Mat& src) {
    src.copyTo(srcImg);  // 深（值）拷贝给srcImg
    classifier.loadImg(srcImg); // 载入srcImg，用于透射变换剪切出装甲板图（对透视变换的图片做预处理）
    srcImg_binary = Mat::zeros(srcImg.size(), CV_8UC1);

    // 通道相减法的自定义形式，利用指针访问(uchar*)，免去了split、substract和thresh操作，加速了1.7倍
    // .data:  uchar类型的指针，指向Mat数据矩阵的首地址
    uchar* pdata = (uchar*)srcImg.data;
    uchar* qdata = (uchar*)srcImg_binary.data;
    int srcData = srcImg.rows * srcImg.cols;

    /*
        * Method: 通道相减法
        * Theory: 如果需要识别红色灯条，那么红灯区域的R通道值远大于B通道值，那么使用R通道值减去B通道值再进行二值化处理，就可以提取红灯区域
        *		  如果需要识别蓝色灯条，更上面方法相反
        * Remark: 由于图像处理后图像的所有通道数都在同一行上，因此 *(pdata + 2) 表示R通道值，*pdata 表示B通道时
    */
    // 提取红色灯条
    if (enemyColor == RED)
    {
        for (int i = 0; i < srcData; i++)
        {
            if (*(pdata + 2) - *pdata > armorParam.color_threshold)
                *qdata = 255;
            pdata += 3;
            qdata++;
        }
    }
        // 提取蓝色灯条
    else if (enemyColor == BLUE)
    {
        for (int i = 0; i < srcData; i++)
        {
            if (*pdata - *(pdata + 2) > armorParam.color_threshold)
                *qdata = 255;
            pdata += 3;
            qdata++;
        }
    }

    /*
        * getStructuringElement()函数返回指定形状和尺寸的结构元素
        * Param: shape 表示内核的形状  esize 指定的内核尺寸
    */
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3)); // 矩形内核形状，(3, 3)内核尺寸
    dilate(srcImg_binary, srcImg_binary, kernel); // 对srcImg_binary进行膨胀操作，使灯条区域更加平滑
}

// 载入SVM模型
void ArmorDetector::loadSVM(const char* model_path, Size armorImgSize) {
    classifier.loadSvmModel(model_path, armorImgSize);
}

// 装甲板识别集成函数
ArmorDate ArmorDetector::run(Mat& src) {
    ArmorDate Arm;
    //载入并预处理图像
    this->setImg(src);

    // 重设detector的内容，清空在上一帧中找到的灯条和装甲板，同时检测器状态重置为LIGHTS_NOT_FOUND（最低状态）
    resetDetector();

    // 检测视野中所有可能的灯条
    findLights();

    /*
        * 当检测到的灯条数量少于2条的时候（灯条数过少，无法构成装甲板）
        * 接下来进行判断当前未识别到目标次数是否大于设定的缓冲次数
    */
    if (state == LIGHTS_NOT_FOUND) {
        Outof_buffer();
        lastArmor = targetArmor;
    }

        // 当找到超过两个灯条的时候（只有检测到两个灯条时才可以构成一个装甲板）
    else if (state == LIGHTS_FOUND)
    {
        // 将每两个灯条匹配为一个装甲板，如果匹配出来的装甲板是合适的，则压入armors中
        matchArmors();

        // 如果找到了灯条，则设置好目标装甲板和上一个装甲板
        // 根据打击得分获取最佳打击装甲板
        setTargetArmor();
    }

    Arm.BestArmor = targetArmor;
    Arm.Car_model = model;
    Arm.centerPoint = targetArmor.center;
    Arm.contourPoints = targetArmor.armorVertices;
    Arm.type = targetArmor.type;
    return Arm;
}

/*
	* Goal: 获取detector的结果
	* Output information: 装甲板四角顶点，装甲板中心点，装甲板类型[大型装甲板 or 小型装甲板]
*/
void ArmorDetector::getTargetInfo(ArmorBox& BestArmor, vector<Point2f>& armorVertices, Point2f& centerPoint, ArmorType& type, pattern& Car_model) {
    armorVertices.clear();
    BestArmor = targetArmor;
    armorVertices = targetArmor.armorVertices;
    centerPoint = targetArmor.center;
    type = targetArmor.type;
    Car_model = model;
}

/*
	* 识别程序是否识别到装甲板
	* return: NOT_FOUND(return 0) FOUND(return 1)
*/
bool ArmorDetector::isFoundArmor() {
    if (state == ARMOR_FOUND)
        return 1;
    else
        return 0;
}


/*
	******************************************************* Debug Function  ************************************************************
*/

// 在图像中显示所有找到的灯条（LightBar）
void showLights(Mat& image, const vector<LightBar>& lights) {
    Mat lightDisplay;	// 用于显示灯条的图像
    image.copyTo(lightDisplay);		// 获取源图像的拷贝，以防对原图像进行修改

    // 如果找到了灯条
    if (!lights.empty())
    {
        putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); // 显示大标题 “LIGHTS FOUND!”
        // 画出所有灯条轮廓以及灯条中心和中心位置坐标
        // auto相当于iterator（迭代）
        for (auto light : lights)
        {
            Point2f lightVertices[4];
            light.lightRect.points(lightVertices);	// 灯条旋转矩形的四个顶点
            // 画出所有灯条的轮廓
            for (size_t i = 0; i < 4; i++)
            {
                line(lightDisplay, lightVertices[i], lightVertices[(i + 1) % 4], Scalar(255, 0, 255), 1, 8, 0);
            }

            // 画出灯条中心
            circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

            // 显示灯条的中心坐标点
            putText(lightDisplay, to_string(int(light.center.x)), light.center, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, to_string(int(light.center.y)), light.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
        }
    }
        // 如果没找到灯条
    else
    {
        putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);// 显示大标题 “LIGHTS NOT FOUND!”
    }

    // 显示结果图
    imshow("Lights Monitor", lightDisplay);
}

// 在图像中显示所有找到的装甲板（Armors）
void showArmors(Mat& image, const vector<ArmorBox>& armors, const ArmorBox& targetArmor, double light_len) {
    Mat armorDisplay; // 展示装甲板的图像
    image.copyTo(armorDisplay); // 获取源图像的拷贝，以防对原图像进行修改

    // 如果找到了装甲板
    if (!armors.empty())
    {
        putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); // 显示大标题 “ARMOR FOUND!”
	string name = "ARMOR FOUND!";
	name += to_string(light_len);
	putText(armorDisplay, name, Point(100, 500), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false);// 显示灯条长度
        // 画出所有装甲板的顶点边以及中心和中心位置坐标
        for (auto armor : armors)
        {
            // 画中心
            circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);
            // 画出所有装甲板边框
            for (size_t i = 0; i < 4; i++)
            {
                line(armorDisplay, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4], Scalar(255, 255, 255), 2, 8, 0);
            }
            // 显示中点坐标以及SVM识别出的装甲板上的数字
            putText(armorDisplay, to_string(int(armor.center.x)), armor.center, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, to_string(int(armor.center.y)), armor.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, to_string(int(armor.type)), armor.center + Point2f(15, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
        }
        // 画出最佳击打装甲板轮廓
        for (size_t i = 0; i < 4; i++)
        {
            line(armorDisplay, targetArmor.armorVertices[i], targetArmor.armorVertices[(i + 1) % 4], Scalar(255, 255, 0), 2, 8, 0);
        }
    }
        // 如果没找到装甲板
    else
    {
        putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);// 显示大标题 “ARMOR NOT FOUND!”
    }

    // 显示结果图
    imshow("Armor Monitor", armorDisplay);
}

// 在控制台输出找到的灯条的中心和角度
void textLights(vector<LightBar>& lights) {
    cout << "\n*********************** L I G H T S ***********************" << endl;
    if (lights.empty()) {
        cout << "LIGHTS NOT FOUND!" << endl;
    }
    else
    {
        cout << "LIGHTS FOUND!" << endl;
        for (size_t i = 0; i < lights.size(); i++)
        {
            cout << "------------------------------------------------" << endl;
            cout << "Light Center:" << lights[i].center << endl;
            cout << "Light Angle:" << lights[i].angle << endl;
        }
        cout << "------------------------------------------------" << endl;
    }
}

// 在控制台输出找到装甲板的中心，数字，匹配信息
void textArmors(vector<ArmorBox>& armors)
{
    cout << "\n*********************** A R M O R S ***********************" << endl;
    if (armors.empty()) {
        cout << "ARMORS NOT FOUND!" << endl;
    }
    else
    {
        cout << "ARMOR FOUND!" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            cout << "------------------------------------------------" << endl;
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Armor Number: " << armors[i].armorNum << endl;

            if (armors[i].type == SMALL_ARMOR)
                cout << "Armor Type: SMALL ARMOR" << endl;
            else if (armors[i].type == BIG_ARMOR)
                cout << "Armor Type: BIG ARMOR" << endl;

            cout << "\n------------- matching information -------------" << endl;
            cout << "Angle difference: " << armors[i].getAngleDiff() << endl;
            cout << "Deviation Angle: " << armors[i].getDeviationAngle() << endl;
            cout << "X Dislocation Ration: " << armors[i].getDislocationX() << endl;
            cout << "Y Dislocation Ration: " << armors[i].getDislocationY() << endl;
            cout << "Length Ration: " << armors[i].getLengthRation() << endl;
        }
        cout << "------------------------------------------------" << endl;
    }
}

void textScores(vector<ArmorBox>& armors, ArmorBox& lastArmor)
{
    if (!armors.empty())
    {
        cout << "\n*********************** S C O R E S ***********************" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            float score = 0;  // 记录armor的打击度
            cout << "------------------------------------------------" << endl;
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Area: " << armors[i].armorRect.area() << endl;
            score += armors[i].armorRect.area(); // 面积得分

            // 获取a、b装甲板综合分数
            // setNumScore(armors[i].armorNum, armors[i].armorNum, score);

            if (lastArmor.armorNum != 0) {  // 上一帧图像中存在目标装甲板
                float a_distance = getPointsDistance(armors[i].center, lastArmor.center);
                cout << "Distance: " << a_distance << endl;
                score -= a_distance * 2;	// 装甲板距离得分，算负分
            }
        }
        cout << "------------------------------------------------" << endl;
    }
}

// 所有调试数据输出
void ArmorDetector::showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON, bool textLights_ON, bool textArmors_ON, bool textScores_ON, double light_len)
{
    if (showSrcImg_ON)
        imshow("srcImg", srcImg);
    if (showSrcBinary_ON)
        imshow("srcImg_Binary", srcImg_binary);
    if (showLights_ON)
        showLights(srcImg, lights);
    if (showArmors_ON)
        showArmors(srcImg, armors, targetArmor, light_len);
    if (textLights_ON)
        textLights(lights);
    if (textArmors_ON)
        textArmors(armors);
    if (textScores_ON)
        textScores(armors, lastArmor);
}
