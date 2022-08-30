#pragma once

/*
	* Date: 2022.2.27
	* Details: 装甲板识别模块头文件
*/

#ifndef RM_ARMOR_H
#define RM_ARMOR_H

#include "General.h"

using namespace cv;
using namespace ml;
using namespace std;

// 自瞄模式
enum DetectorState
{
    LIGHTS_NOT_FOUND = 0,
    LIGHTS_FOUND = 1,
    ARMOR_NOT_FOUND = 2,
    ARMOR_FOUND = 3
};

// 装甲板识别中所需用到的各种参数
struct ArmorParam
{

    // 图像预处理
    int color_threshold;   // 通道相减的colorImg使用的二值化阈值
    int bright_threshold;  // 亮度图二值化阈值

    // 检测灯条
    float min_area;		// 灯条允许的最小面积
    float max_angle;	// 灯条允许的最大偏角

    // 匹配装甲板
    float max_angle_diff; // 左右两个灯条之间允许的最大角度差
    float max_lengthDiff_ratio; // 左右两个灯条之间允许的最大长度差比值
    float max_deviation_angle; // 左右两灯条最大错位角(两灯条中心连线与水平线夹角)

    float max_x_diff_ratio;  // 灯条x方向最大位置差距
    float max_y_diff_ratio;  // 灯条y方向最大位置差距

    // 定义缓冲次数
    int Buffer_Num;					  // 定义buffer缓冲次数
    const int Max_Buffer_Num = 10;    // 定义max的buffer缓冲次数

    // 给各参数设定默认值(预设值)
    ArmorParam() {
        // 不同的相机曝光需要设置不同的值（在不同曝光以及不同远近的情况下相机所拍摄到的红色阈值不同）
        color_threshold = 50;
        bright_threshold = 90;

        min_area = 30;
        max_angle = 20;

        max_angle_diff = 8;
        max_lengthDiff_ratio = 0.5;
        max_deviation_angle = 10;

        max_y_diff_ratio = 0.5;
        max_x_diff_ratio = 4.2;

        Buffer_Num = 0;
    }
};
extern ArmorParam armorParam;

// 装甲板两侧灯条的相关信息
class LightBar
{
public:

    // 灯条无参数构造函数
    LightBar();

    // 灯条有参构造函数
    // 拟合椭圆获得的旋转矩形来构造灯条
    // 参数：灯条的旋转矩形
    LightBar(const RotatedRect& light);

    ~LightBar();

public:
    RotatedRect lightRect; // 灯条的旋转矩形（椭圆拟合获得）
    float length;	// 灯条长度
    Point2f center; // 灯条中心
    float angle;	// 灯条长度方向与竖直方向的夹角，左偏为0~90,右偏为0~-90 (不论左偏还是右偏，角度都限制在0-90之间)
};

// 装甲板相关的数据信息
class ArmorBox
{
public:

    // 装甲板无参构造函数
    ArmorBox();

    // 装甲板有参构造函数
    // 参数：左右两个灯条
    ArmorBox(const LightBar& l_light, const LightBar& r_light);
    ~ArmorBox();

    // 获取装甲板左右灯条角度差
    float getAngleDiff() const;

    // 获取左右灯条长度差比值
    float getLengthRation() const;

    // 获取灯条错位度角(两灯条中心连线与水平线夹角)
    float getDeviationAngle() const;

    // 灯条位置差距 获取两灯条中心X方向差距比值
    float getDislocationX() const;

    // 灯条位置差距 获取两灯条中心Y方向差距比值
    float getDislocationY() const;

    // 判断某个装甲板是否合适(是否能够构成一个装甲板)
    bool isSuitableArmor() const;

public:
    LightBar l_light, r_light; // 定义装甲板的左右灯条
    int l_index, r_index; // 左右灯条的下标(默认为-1，仅作为ArmorDetector类成员时生效)
    int armorNum;  // 存储SVM识别后装甲板上的数字（用SVM识别得到）
    vector<Point2f> armorVertices;  // 存储装甲板的四个顶点 左下 左上 右上 右下
    ArmorType type; // 装甲板类型（大型装甲板 小型装甲板）
    Point2f center;	// 存储装甲板的中心
    Rect armorRect;  // 存储装甲板的矩形所获取的roi区域
    float armorAngle;// 存储装甲板角度(灯条角度的平均值)
    Mat armorImg;	// 装甲板的图片（透射变换后的装甲板图片）
    double tx;
    double ty;
    double tz;
};


// 利用透射变换截取装甲板图片（SVM模型大小）,并利用SVM来识别装甲板数字
class ArmorNumClassifier
{
public:
    ArmorNumClassifier();
    ~ArmorNumClassifier();

    // 载入SVM模型（用于识别装甲板数字）
    // 参数：待载入SVM模型的路径 模型的图片尺寸
    void loadSvmModel(const char* model_path, Size armorImgSize = Size(40, 40));

    // 载入roiImage（并且对图像进行预处理）
    void loadImg(Mat& srcImg);

    // 利用透视变换获得装甲板图片
    void getArmorImg(ArmorBox& armor);

    // 利用SVM实现装甲板数字识别
    void setArmorNum(ArmorBox& armor);

private:
    Ptr<SVM>svm;  // SVM模型
    Mat p;		// 载入到SVM中识别的矩阵
    Size armorImgSize; // SVM模型识别图片大小 Size类型（=训练集的图片大小）

    Mat warpPerspective_src; // 透射变换的原图
    Mat warpPerspective_dst; // 透射变换后生成的图像
    Mat warpPerspective_mat; // 透射变换的变换矩阵
    Point2f srcPoints[4];   // 透射变换的原图上的目标点 左上 右上 右下 左下
    Point2f dstPoints[4];	// 透射变换的目标图中的点 左上 右上 右下 左下
};


// 返回最佳装甲板数据
struct ArmorDate {
    vector<Point2f> contourPoints;
    ArmorType type;
    Point2f centerPoint;
    ArmorBox BestArmor;
    pattern Car_model;
};

// 装甲板识别类，实现装甲板两侧灯条的检测，装甲板的灯条匹配，装甲板的筛选，装甲板数字识别，选择目标等功能
class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();

    // 载入SVM模型（用于识别装甲板数字）
    // 参数：待载入SVM模型的路径 模型的图片尺寸
    void loadSVM(const char* model_path, Size armorImgSize = Size(40, 40));

    // 设置敌方颜色 B蓝=0 G绿=1 R红=2
    void setEnemyColor(Color enemyColor);

    // 操作手用，设置目标装甲板数字
    // 通过串口接受的信息
    void setTargetNum(const int& targetNum);

    // 重设检测器（删除原有的灯条和装甲板状态），以便进行下一帧的检测
    void resetDetector();

    // 载入源图像并进行图像预处理
    void setImg(Mat& src);

    // 检测所有可能的灯条
    void findLights();

    /*
        * 判断当前未识别到目标次数是否大于设定的缓冲次数
        * 如果大于设定的缓冲次数，设定为未识别到
        * 如果小于或者等于设定的缓冲次数，赋值和上一次识别结果相同
    */
    void Outof_buffer();

    // 将识别到的灯条拟合为装甲板
    void matchArmors();

    // 将上一帧的目标装甲板（targearmor）作为lastArmor，然和选择本帧图像中所有装甲板里面价值最大的装甲板作为目标装甲板
    void setTargetArmor();

    // 集成的装甲板检测识别函数
    ArmorDate run(Mat& src);

    /*
        * 识别程序是否识别到装甲板
        * return: NOT_FOUND(0) FOUND(1)
    */
    bool isFoundArmor();

    // 在窗体上显示所有信息，用于debug时使用
    void showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON, bool textLights_ON, bool textArmors_ON, bool textScores_ON, double light_len);

    // 获取detector的结果 （-->装甲板四角顶点，装甲板中心点，装甲板类型[大型装甲板 or 小型装甲板]<--）
    void getTargetInfo(ArmorBox& BestArmor, vector<Point2f>& armorVertices, Point2f& centerPoint, ArmorType& type, pattern& model);

private:
    Mat srcImg;  // 从相机采集的当前的图像帧
    Mat srcImg_binary; // 源图像的二值图
    Color enemyColor;  // 敌方颜色（下位机通过串口发送的数据）
    int targetNum; // 操作手设定的目标装甲板数字（下位机通过串口发送的数据）
    vector<LightBar> lights; // 找到的所有可能的灯条
    vector<ArmorBox> armors; // 识别到的所有装甲板
    ArmorBox targetArmor; // 当前图像帧对应的目标装甲板（targearmor），即为最佳装甲板
    ArmorBox lastArmor;  // 上一帧图像的目标装甲板（lastArmor）
    ArmorNumClassifier classifier; // 获取装甲板图像及识别装甲板数字的类（SVM）
    DetectorState state; // 装甲板检测器的状态，随着装甲板进程的执行而不断更新
    pattern model;		// 战车射击状态（初始化为stop）
};

#endif //RM_ARMOR_H
