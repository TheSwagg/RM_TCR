#pragma once
/*
	* Date: 2022.2.28
	* Details: 角度结算相关函数头文件
*/

#ifndef RM_ANGLESOLVER_2_H
#define RM_ANGLESOLVER_2_H

#include "General.h"
#include "Filter.h"
#include "Armor.h"

#define G 9.8

// 角度解算类
class AngleSolver
{
public:
    // 无参数构造函数
    AngleSolver();
    // 析构函数
    ~AngleSolver();

    /*
        * Goal: 设置相机参数
        * Param: camera_matrix: 相机内在矩阵
        * @param distortion_coeff: 相机畸变系数
    */
    void setCameraParam(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeff);
    // 重载函数，filePath为xml类型文件
    int setCameraParam(const char* filePath, int camId);

    /*
        * Goal: 设置装甲板的尺寸
        * Param: type 装甲板类型[SMALL_ARMOR or BIG_ARMOR]  width 装甲板的宽度(mm)  height 装甲板的高度(mm)
    */
    void setArmorSize(ArmorType type, double width, double height);

    /*
        * Goal: 设置子弹打击速度
        * Param: 子弹的打击速度(m/s)
    */
    void setBulletSpeed(int bulletSpeed);

    /*
        * Goal: 设置装甲板的四角顶点和中心点
        * Param: Armor 最佳击打装甲板
        * Param: contoursPoints 装甲板四角顶点的数组
        * Param: centerPoint 装甲板中心点坐标
        * Param: type 装甲板类型[SMALL_ARMOR or BIG_ARMOR]
    */
    void setTarget(pattern model, ArmorBox Armor, vector<Point2f> contoursPoints, Point2f centerPoint, ArmorType type);

    // 获取装甲板2D坐标
    void GetPoint2D(ArmorBox& BestArmor, vector<cv::Point2f>& point2D);

    // 获取装甲板3D坐标
    void GetPoint3D(ArmorBox& BestArmor, vector<cv::Point3f>& point3D);

    // pnp转换,得到目标坐标
    void CountAngleXY(const vector<Point2f>& point2D, const vector<Point3f>& point3D);

    // 相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
    void ShootAdjust(double& tx, double& ty, double& tz, double Carpitch, double Caryaw);

    // 计算击打时间及仰角
    Angle_t ComputeShootTime(float tx, float ty, float distance, CarData CarDatas);

    // 移动预测
    void ComputeRecoup(ArmorBox& BestArmor, CarData CarDatas, Point3f RelativePoisition);

    // 首次识别到目标，保留值和卡尔曼滤波对象初始化
    void First_Find(ArmorBox BestArmor, CarData carDatas);

    // 首次进入连续滤波，对卡尔曼滤波进行第一次赋初值
    void FirstSetFilter(ArmorBox& BestArmor, CarData carDatas);

    // 进行连续滤波
    void ContinueSetFilter(ArmorBox& BestArmor, CarData carDatas);

    // 获取卡尔曼滤波处理后的position
    Point3f SetKF(ArmorBox& BestArmor, CarData CarDatas, double t);

    // 将绝对坐标转化为相对坐标
    Point3f GetAbsToRelative(Point3f p, float Carpitch, float Caryaw);

    /*
        * Step1: 使用solvePnP函数计算出坐标点从世界坐标系旋转到相机坐标系的旋转和平移向量
        * Step2: 通过计算出的平移向量
    */
    void solveAngles(CarData CarDatas);

    // P4P method（使用相机成像原理）
    void P4P_solver();

    // PinHole method（使用小孔成像原理）
    void PinHole_solver();

    // 角度补偿综合函数
    void compensateAngle();

    // 对机筒与相机之间在y方向上的距离进行俯仰补偿（相机装在机筒的正上方 所以只需要对y方向上的差距进行补偿就可以）
    void compensateOffset();

    // 重力的俯仰补偿
    void compensateGravity();

    /*
        * Goal: 使用solvePnP根据二维坐标点来获取yaw方向和pitch方向的角度以及distance距离
        * Param: Armor 最佳击打装甲板
        * Param: contourPoints 装甲板的四角顶点
        * Param: centerPoint 装甲板的中心点
        * Param: type 装甲板的类型[SMALL_ARMOR or BIG_ARMOR]
        * Param: yaw 机筒所需转动的yaw值（负值表示需要向左，正值表示需要向右）
        * Param: pitch 机筒所需转动的pitch值（负值表示需要向下，正值表示需要向上）
        * Param: evaluateDistance 所计算出的距离（单位为 mm）
    */
    void getAngle(pattern model, struct ArmorBox& Armor, CarData CarDatas, vector<Point2f>& contourPoints, Point2f centerPoint, ArmorType type, double& yaw, double& pitch, double& evaluateDistance, double& light_len);

    // 缓冲状态击打
    void BufferSetFilter(struct ArmorBox& BestArmor, CarData CarDatas, double& pitch, double& yaw);

    // 显示Debug的数据
    void showDebugInfo(CarData CarDatas, bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams);

private:

    // 相机参数(相机标定出来的参数)
    Mat CAMERA_MATRIX;    //IntrinsicMatrix(相机内参矩阵)	 fx,fy,cx,cy
    Mat DISTORTION_COEFF; //DistortionCoefficients(相机畸变系数)  k1,k2,p1,p2

    // 装甲板的世界坐标
    vector<Point3d> SMALL_ARMOR_POINTS_3D;
    vector<Point3d> BIG_ARMOR_POINTS_3D;
    // Mat SMALL_ARMOR_POINTS;
    // Mat BIG_ARMOR_POINTS;

    // 子弹速度(用于计算重力和气压补偿)
    double BULLET_SPEED;

    /*
        * 相机与机筒之间的y轴距离 barrel_y = camera_y + barrel_camera_y
        * Remark: 如果摄像头在机筒的上方，那么GUN_CAM_DISTANCE_Y的值为正，反之为负
    */
    double GUN_CAM_DISTANCE_Y;

    // 目标装甲板的基本参数
    ArmorBox BestArmor;
    vector<Point2f> targetContour;
    Point2f targetCenter;
    ArmorType targetType;
    pattern Car_model;

    // solvePnP计算得出的结果
    // s[R|t]=s'  s->世界坐标  s`->相机坐标
    Mat rVec;    //摄像机与目标中心的rot转动(记录旋转向量)
    Mat tVec;  // 摄像机与目标中心之间的平移(记录平移向量)

    // 计算输出参数
    float y_yaw;		// yaw值
    float x_pitch;		// pitch值
    double distance;		// 距离 (mm)
    float test_y_yaw;		// yaw值
    float test_x_pitch;		// pitch值

    // 装甲板大小（尺寸）
    float fBigArmorWidth;
    float fBigArmorHeight;
    float fSmallArmorWidth;
    float fSmallArmorHeight;

    float ptz_camera_pitch;            // 对应绕x旋转角度，弧度，角度乘0.017453
    float ptz_camera_yaw;              // 对应绕y旋转角度，弧度
    float ptz_camera_roll;             // 对应绕z旋转角度，弧度

    // 坐标系转换
    float ptz_camera_y;                // 相机与云台垂直方向距离差
    float ptz_camera_z;                // 相机与云台轴向方向距离差
    float ptz_camera_x;                // 相机与云台水平方向距离差

    //相机内参
    Mat GuangcaremaMatrix;
    Mat GuangdistCoeffs;
    // Mat GuangcaremaMatrix = Mat(Size(3, 3), CV_64FC1);	// 大恒相机内参
    // Mat GuangdistCoeffs = Mat(Size(1, 5), CV_64FC1);		// 大恒相机外参
};

#endif //RM_ANGLESOLVER_H
