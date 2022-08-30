#ifndef BUFFANGLESOLVER_H
#define BUFFANGLESOLVER_H
#include"../header/General.h"
#include"../header/Filter.h"
#include "../buff_include/FindBuff.h"
class BuffAngleSolver{
private:
    float BuffWidth;
    float BuffHeight;
    float ChassisToPtz_x;
    float ChassisToPtz_y;
    float ChassisToPtz_z;
    float ChassisToPtz_Pitch;
    float ChassisToPtz_Yaw;
    float ChassisToPtz_Roll;

    void GetPoint2D( RM_BuffData & BestArmor,std::vector<cv::Point2f>&point2D);
    void GetPoint3D( RM_BuffData & BestArmor,std::vector<cv::Point3f>&point3D);
    void CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, RM_BuffData & BestArmor);
    void GetBuffrAngle(RM_BuffData & BestArmor);
    float GetBuffCentralAngle(Point3f ObjectPoistion,Point3f CircleCenter,Point3f LowerBoundaryPoint,Point3f LefterBoundaryPoint);
    Point3f GetBuffCenter(RM_BuffData * BestArmor);
    Point3f GetAveBuffCenter();
    Point3f GetNormalVector(Point3f new_vector,Point3f old_vector);
    float ShootAdjust(float & x,float & y,float & z,float pitch,float yaw);
    void BuffAngleSpeedFilter(float & AngleSpeed,KF_two Filter,CarData carDatas);
    Point3f GetShootPoistion(float angle,Point3f center,float r);
    void ChassisToPtz(RM_BuffData & BestArmor);
    Angle_t ComputeBuffShootTime(float tx, float ty, float distance,struct CarData CarDatas);

    //底盘usb
    cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
            1281.2415311337063031,-0.3089685211182275,654.0926382032376978,
    0,1283.0023551375293209,510.1706263660812510,
    0,0,1);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.0728917229903051, -0.1316730024507187, -0.0024528112888193, -0.0006391130756001 ,0.0);


public:
    BuffAngleSolver();                  //构造函数

    void GetBuffShootAngle(RM_BuffData * BestArmor,BuffStatus BuffShootStatus,CarData carDatas);

};

#endif // BUFFANGLESOLVER_H
