/*
	* Date: 2022.3.1
	* Details: 角度结算相关函数源文件
	* functions:
	*	AngleSolver()		无参数构造函数
	*	~AngleSolver()		析构函数
	*	setCameraParam()	设置相机参数
	*	setCameraParam()	重载函数，读取xml文件
	*	setArmorSize()		设置装甲板尺寸
	*	setBulletSpeed()	设置子弹打击速度
	*	setTarget()			设置装甲板的四角顶点和中心点位置坐标
	*	GetPoint2D()		获取装甲板2D坐标
	*	GetPoint3D()		获取装甲板3D坐标
	*	CountAngleXY()		pnp转换，初步得到目标坐标（获取初步的tx, ty, tz, x_pitch, y_yaw）
	*	ShootAdjust()		相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
	*	ComputeShootTime()	计算击打时间以及机筒仰角
	*	First_Find()		首次识别到目标，保留值和卡尔曼滤波对象的初始化
	*	FirstSetFilter()	首次进入连续滤波，卡尔曼滤波状态向量初始化
	*	ContinueSetFilter()	进入连续滤波，持续进行卡尔曼滤波处理目标坐标
	*	SetKF()				获取卡尔曼滤波处理后的position结果
	*	GetAbsToRelative()	将绝对坐标转换为相对坐标
	*	ComputeRecoup()		移动预测综合函数
	*	solveAngles()		角度解算综合函数
	*	P4P_solver()		P4P method（使用相机成像原理），计算初步的x_pitch, y_yaw
	*	PinHole_solver()	PinHole method（使用小孔成像原理），计算初步的x_pitch, y_yaw
	*	compensateAngle()	角度补偿综合函数（弃用）
	*	compensateOffset()	机筒和相机之间仰角补偿（弃用）
	*	compensateGravity()	重力仰角补偿（弃用）
	*	getAngle()			综合函数，起到最终角度解算后的结果输出的作用
	*	BufferSetFilter()	缓冲状态击打函数
	*	showDebugInfo()		Debug数据显示函数
*/

#include "../header/AngleSolver_2.h"
#include "../header/RemoteController.h"

#define SHOOT_DELAY_TIME 120                // 子弹发弹延迟
#define USING_POISITION_FILTER		// 使用位置滤波
#define Mobile_prediction			// 进行移动预测

CarData old_carDatas;		// 保留上一次接收到的CarData

// 定义卡尔曼类型
KF_two KF_tz;
KF_two KF_forecast;		// 预测一阶滤波

// 设置tz卡尔曼滤波
bool isSetKF_tz = false;		// 判断是否进行卡尔曼滤波的初始化
bool isSetAngleKF = false;          	//是否设置过角度滤波
float tz_old;

// 线速度保留量
float v_tx_old;
float v_ty_old;
float v_tz_old;
// 位置保留量
float p_tx_old;
float p_ty_old;
float p_tz_old;
Point3f old_objectP;

// 用于缓冲计算
float SendPitch = 0;              // 记录上一帧pitch发送角度量
float SendYaw = 0;                // 记录上一帧yaw发送角度量

// 记录前后时间
double  old_CarTime = 0;

// 无参数构造函数
AngleSolver::AngleSolver() {
    // 装甲板尺寸初始化
    fBigArmorWidth = 23.0;
    fBigArmorHeight = 8.85;	// 12.7
    fSmallArmorWidth = 13.5;
    fSmallArmorHeight = 8.85;	// 12.5

    ptz_camera_pitch = -9.3 * PI / 180;            // 对应绕x旋转角度,弧度，角度乘0.017453
    ptz_camera_yaw = -18 * PI / 180;              // 对应绕y旋转角度，弧度
    ptz_camera_roll = 0 * PI / 180;           // 对应绕z旋转角度，弧度

    // 坐标系转换(暂定)
    ptz_camera_y = 0;                  // 相机与云台垂直方向距离差(后期需要进行修改)
    ptz_camera_z = 0;                  // 相机与云台轴向方向距离差
    ptz_camera_x = 0;                  // 相机与云台水平方向距离差

    // 读取相机内参外参
    FileStorage fs(paramCameraName, FileStorage::READ);
    if(!fs.isOpened()){
        cout<<"相机内参外参参数文件打开失败!"<<endl;
        exit(1);
    }
    else {
	cout<<"打开成功！" <<endl;
    }
    fs["IntrinsicCam"] >> GuangcaremaMatrix;
    fs["DistortionCam"] >> GuangdistCoeffs;

    fs.release();

    //测距滤波初始化,仅需执行一次
    if (!isSetKF_tz) {
        isSetKF_tz = true;

        // 状态协方差矩阵附初值
        Eigen::MatrixXd P_in = Eigen::MatrixXd(2, 2);
        P_in << 1.0, 0.0,
                0.0, 1.0;
        KF_tz.P = P_in;

        // 过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(2, 2);
        Q_in << 1.0, 0.0,
                0.0, 1e-1;
        KF_tz.Q = Q_in;

        // 测量矩阵附初值
        Eigen::MatrixXd H_in(1, 2);
        H_in << 1.0, 0.0;
        KF_tz.H = H_in;

        // 测量噪声矩阵附初值
        Eigen::MatrixXd R_in(1, 1);
        R_in << 10;
        KF_tz.R = R_in;

        Eigen::MatrixXd F_in(2, 2);
        F_in << 1.0, 1.0,
                0.0, 1.0;
        KF_tz.F = F_in;
    }
}

// 析构函数
AngleSolver::~AngleSolver() {
}

/*
		* Goal: 设置相机参数
		* Param: camera_matrix: 相机内在矩阵
		* @param distortion_coeff: 相机畸变系数
	*/
void AngleSolver::setCameraParam(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeff) {
    camera_matrix.copyTo(CAMERA_MATRIX);
    distortion_coeff.copyTo(DISTORTION_COEFF);
}

// 重载函数，filePath为xml类型文件
int AngleSolver::setCameraParam(const char* filePath, int camId) {
    FileStorage fsRead;
    fsRead.open(filePath, FileStorage::READ);	// 打开并读取xml文件
    if (!fsRead.isOpened())
    {
        cout << "Failed to open xml" << endl;
        return -1;
    }

    // 获取枪口中心点和相机中心点在y轴上的差距
    fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

    Mat camera_matrix;
    Mat distortion_coeff;
    switch (camId)
    {
        case 1:
            fsRead["CAMERA_MATRIX_1"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_1"] >> distortion_coeff;
            break;
        case 2:
            fsRead["CAMERA_MATRIX_2"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_2"] >> distortion_coeff;
            break;
        case 3:
            fsRead["CAMERA_MATRIX_3"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_3"] >> distortion_coeff;
            break;
        default:
            cout << "WRONG CAMID GIVEN!" << endl;
            break;
    }
    setCameraParam(camera_matrix, distortion_coeff);
    fsRead.release();
    return 0;
}

/*
	* Goal: 设置装甲板的尺寸
	* Param: type 装甲板类型[SMALL_ARMOR or BIG_ARMOR]  width 装甲板的宽度(mm)  height 装甲板的高度(mm)
*/
void AngleSolver::setArmorSize(ArmorType type, double width, double height) {
    double half_x = width / 2.0;
    double half_y = height / 2.0;
    switch (type)
    {
        case SMALL_ARMOR:
	    SMALL_ARMOR_POINTS_3D.clear();
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x, half_y, 0.0));   // 左上角
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(half_x, half_y, 0.0));    // 右上角
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(half_x, -half_y, 0.0));   // 右下角
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0.0));  // 左下角
	    // Mat(SMALL_ARMOR_POINTS_3D).convertTo(SMALL_ARMOR_POINTS, CV_32F);
            break;

        case BIG_ARMOR:
	    BIG_ARMOR_POINTS_3D.clear();
            BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x, half_y, 0.0));   // 右下角
            BIG_ARMOR_POINTS_3D.push_back(Point3f(half_x, half_y, 0.0));    // 右上角
            BIG_ARMOR_POINTS_3D.push_back(Point3f(half_x, -half_y, 0.0));   // 左上角
            BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0.0));  // 左下角
	    // Mat(BIG_ARMOR_POINTS_3D).convertTo(BIG_ARMOR_POINTS, CV_32F);
            break;
        default: break;
    }
}

/*
	* Goal: 设置子弹打击速度
	* Param: 子弹的打击速度(mm/s)
*/
void AngleSolver::setBulletSpeed(int bulletSpeed) {
    BULLET_SPEED = bulletSpeed;
}

/*
	* Goal: 设置装甲板的四角顶点和中心点
	* Param: contoursPoints 装甲板四角顶点的数组  centerPoint 装甲板中心点坐标  type 装甲板类型[SMALL_ARMOR or BIG_ARMOR]
*/
void AngleSolver::setTarget(pattern model, ArmorBox Armor, vector<Point2f> contourPoints, Point2f centerPoint, ArmorType type) {
    Car_model = model;
    BestArmor = Armor;
    targetContour = contourPoints;
    targetCenter = centerPoint;
    targetType = type;
}

// 获取装甲板2D坐标
void AngleSolver::GetPoint2D(ArmorBox& BestArmor, vector<cv::Point2f>& point2D) {
    cv::Point2f lu, ld, ru, rd;        //right_down right_up left_up left_down

    lu = BestArmor.armorVertices[0];
    ld = BestArmor.armorVertices[3];
    ru = BestArmor.armorVertices[1];
    rd = BestArmor.armorVertices[2];

    //     cout<<"lu:"<<lu<<endl;
    point2D.clear();		// 先清空再存入
    point2D.push_back(lu);
    point2D.push_back(ru);
    point2D.push_back(rd);
    point2D.push_back(ld);
}

// 获取装甲板3D坐标
void AngleSolver::GetPoint3D(ArmorBox& BestArmor, vector<cv::Point3f>& point3D) {
    float fHalfX = 0;
    float fHalfY = 0;
    if (BestArmor.type == SMALL_ARMOR)
    {
        // cout << "小" << endl;
        fHalfX = fSmallArmorWidth / 2.0;
        fHalfY = fSmallArmorHeight / 2.0;
    }
    else {
        // cout<<"大"<<endl;
        fHalfX = fBigArmorWidth / 2.0;
        fHalfY = fBigArmorHeight / 2.0;
    }
    point3D.push_back(Point3f(-fHalfX, -fHalfY, 0.0));
    point3D.push_back(Point3f(fHalfX, -fHalfY, 0.0));
    point3D.push_back(Point3f(fHalfX, fHalfY, 0.0));
    point3D.push_back(Point3f(-fHalfX, fHalfY, 0.0));
}

// pnp转换,得到目标坐标
void AngleSolver::CountAngleXY(const vector<Point2f>& point2D, const vector<Point3f>& point3D) {
    // 初始化旋转向量和移动向量
    rVec = cv::Mat::zeros(3, 1, CV_64FC1);
    tVec = cv::Mat::zeros(3, 1, CV_64FC1);

    switch (targetType)
    {
        case SMALL_ARMOR:
            /*
                * Theory: 利用多个控制点在三维场景中的坐标及其在图像中的透视投影坐标
                          即可求解出摄像机坐标系与表示三维场景结构的世界坐标系之间的绝对位姿关系
                          包括绝对平移向量t以及旋转矩阵R
                * Remark: 这里的控制点是指准确知道三维空间坐标位置，同时也知道对应图像平面坐标的点
                * Param: objectPoints 世界坐标系下的控制点的坐标 (InputArray的数据类型)
                * Param: imagePoints 在图像坐标系下对应的控制点的坐标 (InputArray的数据类型)
                * Param: GuangcaremaMatrix 相机的内参矩阵 (通过相机标定得到)
                * Param: GuangdistCoeffs 相机的畸变系数 (通过相机标定得到)
                * Param: rvec 输出的旋转向量 (使坐标点从世界坐标系旋转到相机坐标系)
                * Param: tvec 输出的平移向量 (使坐标点从世界坐标系平移到相机坐标系)
                * Param: useExtrinsicGuess 默认为false
                * Param: flags 默认使用CV_ITERATIV迭代法
            */
            solvePnP(SMALL_ARMOR_POINTS_3D, targetContour, GuangcaremaMatrix, GuangdistCoeffs, rVec, tVec, false, SOLVEPNP_ITERATIVE);
            break;
        case BIG_ARMOR:
            solvePnP(BIG_ARMOR_POINTS_3D, targetContour, GuangcaremaMatrix, GuangdistCoeffs, rVec, tVec, false, SOLVEPNP_ITERATIVE);
            break;
        default:
            break;
    }

    /*
        * 相机与机筒之间的y轴方向的距离
        * Remark: 假定摄像头安装在机筒正上方，所以GUN_CAM_DISTANCE_Y的值为正值
    */
    // GUN_CAM_DISTANCE_Y = 0;		// 需要根据情况修改
    // tVec.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;

    // 计算距离机筒和装甲板之间的距离
    double tx = tVec.at<double>(0, 0);
    double ty = -tVec.at<double>(1, 0);
    double tz = tVec.at<double>(2, 0);

    // 计算距离
    distance = sqrt(tx * tx + ty * ty + tz * tz);
    cout << "distance=============================" << distance / 1.2 << endl;

    // if (Car_model == FirstFind) {
	// tz_old = tz;
	// KF_tz.is_set_x = false;
    // }
    // else {
	// if (!KF_tz.is_set_x) {
	    // Eigen::VectorXd _x(2, 1);
	    // _x << tz, tz- tz_old;
	    // KF_tz.set_x(_x);
	// }
	// else {
	    // KF_tz.Prediction(KF_tz.F);
	    // Eigen::VectorXd _z(1, 1);
	    // _z << tz;
	    // KF_tz.update(_z, KF_tz.F);
	// }
	// tz = KF_tz.get_x()(0);
    // }

    BestArmor.tx = tx;
    BestArmor.ty = ty;
    BestArmor.tz = tz;

    // x_pitch = atan2(BestArmor.ty, BestArmor.tz) * 180 / CV_PI;
    // y_yaw = atan2(BestArmor.tx, BestArmor.tz) * 180 / CV_PI;
    // test_x_pitch = atan2(BestArmor.ty, BestArmor.tz) * 180 / CV_PI;
    // test_y_yaw = atan2(BestArmor.tx, BestArmor.tz) * 180 / CV_PI;
    // x_pitch = atan2(BestArmor.ty, BestArmor.tz) * 180 / CV_PI;
    // y_yaw = atan2(BestArmor.tx, BestArmor.tz) * 180 / CV_PI;

    // 目标距离太远，使用小孔成像原理（PinHole）来计算pitch值和yaw值
    if (distance > 5000)	// 目标距离大于5米
    {
        PinHole_solver();
    }
    // 目标距离适中，使用相机成像原理（PNP）来计算pitch值和yaw值
    else	//目标距离小于5米
    {
        P4P_solver();
    }
}

// 相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
void AngleSolver::ShootAdjust(double& tx, double& ty, double& tz, double Carpitch, double Caryaw) {
    //角度转弧度
    Carpitch *= CV_PI / 180;
    Caryaw *= CV_PI / 180;

    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3, 3);
    r_Roll << 1,					0,					   0,
            0,					cos(ptz_camera_roll),  sin(ptz_camera_roll),
            0,					-sin(ptz_camera_roll), cos(ptz_camera_roll);
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3, 3);
    r_Pitch << cos(ptz_camera_pitch),		0,			-sin(ptz_camera_pitch),
            0,							1,			0,
            sin(ptz_camera_pitch),		0,			cos(ptz_camera_pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3, 3);
    r_Yaw << cos(ptz_camera_yaw),		sin(ptz_camera_yaw),		0,
            -sin(ptz_camera_yaw),		cos(ptz_camera_yaw),		0,
            0,							0,							1;

    // 定义初始向量
    Eigen::VectorXd original(3, 1);          //按z，x，y传入，即变化对应到左手坐标系
    original << tz, tx, ty;

    // 平移变换,先平移再旋转
    Eigen::VectorXd translation(3, 1);
    translation << ptz_camera_z, ptz_camera_x, ptz_camera_y;
    original = original + translation;

    // 旋转变换
    Eigen::VectorXd change(3, 1);
    change = r_Roll * original;
    change = r_Pitch * change;
    change = r_Yaw * change;

    // 去掉车云台旋转相对影响,坐标系转换到相对初始位的绝对坐标
    // pitch转换
    Eigen::MatrixXd r_pitch_car(3, 3);
    r_pitch_car << cos(Carpitch),			0,			-sin(Carpitch),
            0,						1,			0,
            sin(Carpitch),			0,			cos(Carpitch);
    // yaw转换
    Eigen::MatrixXd r_yaw_car(3, 3);
    r_yaw_car << cos(Caryaw),		sin(Caryaw),		0,
            -sin(Caryaw),		cos(Caryaw),		0,
            0,					0,					1;

    change = r_pitch_car * change;
#ifdef USING_POISITION_FILTER
    // 使用位置滤波
    // 如果角度滤波无需将ty和tz转换为绝对角度
    change = r_yaw_car * change;
#endif

    tx = change(1);
    ty = change(2);
    tz = change(0);
}

// 计算击打时间及仰角
// Angle_t{ pitch; yaw; t; angle; }
Angle_t AngleSolver::ComputeShootTime(float tx, float ty, float distance, CarData CarDatas) {
    /*
        * g表示重力加速度
        * LevelDistance表示水平距离,公式(sqrt(pow(tz,2)+pow(tx,2))
        * HeightDistance表示垂直高度ty
        * v表示射速
        * tan_angle表示所要求俯仰角的正切值
        * 计算公式: -0.5g*pow(LevelDistance,2)/pow(V,2)*pow(tan_angle,2) + tan_angle*LevelDistance - 0.5g*pow(LevelDistance,2)/pow(V,2) - HeightDistance
        * 计算时间使用t = LevelDistance/(v*cos_angle)
    */

    // 单位转换
    tx /= 100.0;
    ty /= 100.0;
    float tz = distance / 100.0;

    // 子弹发射速度
    // float speed = CarDatas.ShootSpeed;

    // a b c可以看作三角形的三条边（这个三角形为机筒到装甲板所形成的三角形）
    double a = -0.5 * G * (pow(tz, 2) + pow(tx, 2));
    double b = sqrt((pow(tz, 2) + pow(tx, 2))) * pow(BULLET_SPEED, 2);
    double c = -0.5 * G * (pow(tz, 2) + pow(tx, 2)) - pow(BULLET_SPEED, 2) * ty;
    double Discriminant = pow(a, 2) + pow(b, 2) - 4 * a * c;		// 三角形判别式：a^2 + b^2 - 4*a*c

    // 初始化Angle_t
    Angle_t ShootBuff = { 0, 0, 0, 0 };
    if (Discriminant < 0)
        return ShootBuff;

    double angle_tan_1 = atan((-b + sqrt(Discriminant)) / (2 * a)) * 180 / CV_PI;
    double angle_tan_2 = atan((-b - sqrt(Discriminant)) / (2 * a)) * 180 / CV_PI;
    // 角度取舍,并转换为相对角度
    cout << "计算所得打击角度1:" << angle_tan_1 << "  计算所得打击角度2:" << angle_tan_2 << endl;
    if (fabs(angle_tan_1) <= fabs(angle_tan_2) && fabs(angle_tan_1) < 45) {
        ShootBuff.pitch = angle_tan_1 - CarDatas.pitch;
    }
    else if (fabs(angle_tan_2) < 45) {
        ShootBuff.pitch = angle_tan_2 - CarDatas.pitch;
    }
    else {      // 都不符合要求
        cout << "计算解不符合实际" << endl;
        return ShootBuff;
    }

    // Remark: atan(a / b) 和 atan2(a, b) 的表示方法不同，但是最后求出的结果都是一样的
    ShootBuff.yaw = atan2(tx, tz) * 180 / CV_PI;
    cout << "缓冲计算tx:" << tx << "  ty:" << ty << " tz:" << tz << "yaw" << ShootBuff.yaw << "最小角度" << atan2(ty, tz) * 180 / CV_PI << endl;

    // 计算子弹击打时间
    ShootBuff.t = tz / (BULLET_SPEED * cos(ShootBuff.pitch * CV_PI / 180)) * 1000;
    cout << "击打时间:" << ShootBuff.t << endl;
    return ShootBuff;
}

// 首次识别到目标
void AngleSolver::First_Find(ArmorBox BestArmor, CarData carDatas) {
    // 进行状态值的保留
    cout << "第一次发现目标" << endl;
    // 保留位置变量
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;
}

// 首次进入连续滤波，对卡尔曼滤波进行第一次赋初值
void AngleSolver::FirstSetFilter(ArmorBox& BestArmor, CarData carDatas) {
    cout << "第一次滤波" << endl;
    double t = carDatas.BeginToNowTime - old_carDatas.BeginToNowTime;        // 计算两帧经过的时间
    if (t == 0) {
        t = 15;         // 未收到数，自定义赋值，单位ms
    }
    // 目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old) / t;
    float v_ty_now = (BestArmor.ty - p_ty_old) / t;
    float v_tz_now = (BestArmor.tz - p_tz_old) / t;

    Eigen::VectorXd x(6, 1);
    x << BestArmor.tx, BestArmor.ty, BestArmor.tz, v_tx_now * 1000, v_ty_now * 1000, v_tz_now * 1000;
    KF_forecast.set_x(x);

    // 进行变量的保存 用于进行接下来的卡尔曼滤波处理操作
    // 位置保留变量
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

    old_objectP.x = p_tx_old;
    old_objectP.y = p_ty_old;
    old_objectP.z = p_tz_old;

    // 速度保留变量
    v_tx_old = v_tx_now;
    v_ty_old = v_ty_now;
    v_tz_old = v_tz_now;

    // 保留这一帧的carDatas 用于下一步的计算
    old_carDatas = carDatas;
}

// 进行连续滤波
void AngleSolver::ContinueSetFilter(ArmorBox& BestArmor, CarData carDatas) {
    cout << "连续滤波" << endl;
    double t = carDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    //不发数的时候t=0
    if (t == 0) {
        t = 15;         //单位ms
    }
    //得到真实测量值
    //目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old) / t;
    float v_ty_now = (BestArmor.ty - p_ty_old) / t;
    float v_tz_now = (BestArmor.tz - p_tz_old) / t;

    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

    Eigen::VectorXd z(6, 1);
    z << BestArmor.tx, BestArmor.ty, BestArmor.tz, v_tx_now * 1000, v_ty_now * 1000, v_tz_now * 1000;

    //得到状态转移矩阵
    Eigen::MatrixXd F_in(6, 6);
    F_in << 1.0, 0.0, 0.0, t / 1000, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, t / 1000, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, t / 1000,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    //预测上一最佳状态值
    KF_forecast.Prediction(F_in);

    //更新状态量
    KF_forecast.update(z, F_in);

    // 传参
    p_tx_old = KF_forecast.x_(0);
    p_ty_old = KF_forecast.x_(1);
    p_tz_old = KF_forecast.x_(2);

    // 保留这一帧的carDatas 用于下一步的计算
    old_carDatas = carDatas;
}

// 获取卡尔曼滤波处理后的position
Point3f AngleSolver::SetKF(ArmorBox& BestArmor, CarData CarDatas, double t) {
    if (Car_model == FirstFind) {
        // 首次发现目标 保留值和卡尔曼滤波对象初始化
        First_Find(BestArmor, CarDatas);
        // 判断卡尔曼是否赋初值，设置为false
        KF_forecast.is_set_x = false;
        // 状态协方差矩阵重新复位
        Eigen::MatrixXd P_in = Eigen::MatrixXd(6, 6);
        P_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        KF_forecast.P = P_in;
        return Point3f(BestArmor.tx, BestArmor.ty, BestArmor.tz);
    }
    else if (Car_model == Shoot) {
        // 连续射击
        if (!KF_forecast.is_set_x) {
            // 第一次连续射击(连续滤波)，进行卡尔曼滤波状态向量初始化
            cout << "位置首次连续滤波" << endl;
            FirstSetFilter(BestArmor, CarDatas);
            // 判断卡尔曼是否赋初值，设置为true
            KF_forecast.is_set_x = true;
        }
        else {
            // 进行连续滤波
            cout << "位置连续滤波" << endl;
            ContinueSetFilter(BestArmor, CarDatas);
        }
    }
    else {
        cout << "卡尔曼滤波状态传入出错" << endl;
    }

    // 时间为击打时间与弹道延迟之和
    double ShootTime = t + SHOOT_DELAY_TIME + Recive.getClock() - CarDatas.BeginToNowTime;
    cout << "x速度：" << KF_forecast.x_(3) << " y速度:" << KF_forecast.x_(4) << " z速度:" << KF_forecast.x_(5) << endl;
    cout << "时间：" << ShootTime << endl;

    Point3f position;
    position.x = KF_forecast.x_(0) + KF_forecast.x_(3) / 1000 * ShootTime;
    position.y = KF_forecast.x_(1) + KF_forecast.x_(4) / 1000 * ShootTime;
    position.z = KF_forecast.x_(2) + KF_forecast.x_(5) / 1000 * ShootTime;

    v_tx_old = KF_forecast.x_(3) / 1000;
    v_ty_old = KF_forecast.x_(4) / 1000;
    v_tz_old = KF_forecast.x_(5) / 1000;

    cout << "1阶滤波位置:" << position.x << "  " << position.y << "  " << position.z << endl;
    return position;
}

// 将绝对坐标转化为相对坐标
Point3f AngleSolver::GetAbsToRelative(Point3f p, float Carpitch, float Caryaw) {
    Carpitch *= CV_PI / 180;
    Caryaw *= CV_PI / 180;
    cout << "转换前:x:" << p.x << "  y:" << p.y << " z:" << p.z << " Carpitch:" << Carpitch << "  Caryaw:" << Caryaw << endl;
    // 绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3, 3);
    r_Pitch << cos(Carpitch),		0,		-sin(Carpitch),
            0,					1,		0,
            sin(Carpitch),		0,		cos(Carpitch);
    // 绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3, 3);
    r_Yaw << cos(Caryaw),		sin(Caryaw),		0,
            -sin(Caryaw),		cos(Caryaw),		0,
            0,					0,					1;

    Eigen::VectorXd original(3, 1);          //按z，x，y传入，即变化对应到左手坐标系
    original << p.z, p.x, p.y;

    Eigen::VectorXd change(3, 1);
    //旋转变换
    change = r_Pitch * original;
    change = r_Yaw * change;

    p.x = change(1);
    p.y = change(2);
    p.z = change(0);
    cout << "转换后:x:" << p.x << "  y:" << p.y << " z:" << p.z << endl;
    return p;
}

// 移动预测综合函数
void AngleSolver::ComputeRecoup(ArmorBox& BestArmor, CarData CarDatas, Point3f RelativePoisition) {
    // 抬升角度计算，得到绝对角度
    Angle_t ShootBuff = ComputeShootTime(RelativePoisition.x, RelativePoisition.y, RelativePoisition.z, CarDatas);

    if (ShootBuff.t == 0) {		// 计算出的击打时间为0
        // 出现可能性极小的无解情况,出现时往往是算法错误
        cout << "角度计算无解" << endl;
        ShootBuff.t = 15;
    }
    else {
        // 传参
        x_pitch = ShootBuff.pitch;
        y_yaw = ShootBuff.yaw;
    }

#ifdef USING_POISITION_FILTER		// 使用位置滤波适合大恒相机
    // 卡尔曼滤波
    // position存储卡尔曼滤波处理后的tx, ty, tz
    Point3f position = SetKF(BestArmor, CarDatas, ShootBuff.t);
    float y = position.y;					// 保留y的绝对坐标
    position = GetAbsToRelative(position, -CarDatas.pitch, CarDatas.yaw);              // 将绝对坐标x, y, z转化为相对坐标
    position.y = y;                           // y继续使用绝对坐标

    Angle_t AT = ComputeShootTime(position.x, position.y, position.z, CarDatas);  // 利用卡尔曼滤波处理后的位置坐标计算打击时间和偏向角
    // 更新打击偏向角
    x_pitch = AT.pitch;
    y_yaw = AT.yaw;

    // 数据的保留 可能在后续的缓冲计算时需要用到
    SendPitch = AT.pitch;
    SendYaw = AT.yaw;
    BestArmor.tx = position.x;
    BestArmor.ty = position.y;
    BestArmor.tz = position.z;
    old_objectP.y = y;                                                 // 保留位置坐标,为计算缓冲做准备
    old_objectP.x = BestArmor.tx;
    old_objectP.z = BestArmor.tz;

    return;
#endif
}

// 根据距离使用P4P或PinHole来求解角度
void AngleSolver::solveAngles(CarData CarDatas) {
    vector<Point2f> point2D;
    vector<Point3f> point3D;

    GetPoint2D(BestArmor, point2D);		// 矩阵转换为2d坐标
    GetPoint3D(BestArmor, point3D);		// 矩阵转换为3d坐标

    CountAngleXY(point2D, point3D);		// pnp转换,得到目标坐标

    //相机坐标与云台初始枪口坐标转换,坐标系做云台当前角度反向旋转得到绝对坐标
    Point3f RelativePoisition = Point3f(BestArmor.tx, BestArmor.ty, BestArmor.tz);
    // 相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
    ShootAdjust(BestArmor.tx, BestArmor.ty, BestArmor.tz, CarDatas.pitch, -CarDatas.yaw);
    // ty坐标使用绝对坐标
    // 其中tx和tz为相对坐标，ty为绝对坐标
    RelativePoisition.y = BestArmor.ty;

    // 接下来进行移动预测
#ifdef Mobile_prediction
    ComputeRecoup(BestArmor, CarDatas, RelativePoisition);
    if (Car_model != buffering) {
        // 当前不处于缓冲状态就更新记录时间
        // 缓冲状态应不参与滤波计算,根据最后的滤波所得速度计算缓冲时间内绝对坐标,由绝对坐标计算相对角度,会产生丢失目标僵直状态
        old_CarTime = CarDatas.BeginToNowTime;
    }
#endif
}

// P4P method（使用相机成像原理）
void AngleSolver::P4P_solver() {
    double x_pos = BestArmor.tx;
    double y_pos = BestArmor.ty;
    double z_pos = BestArmor.tz;

    /*
        * Formula: tan<pitch> = ty / sqrt(ty^2 + tz^2)
        *		   tan<yaw> = tx / tz
    */
    double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;
    test_x_pitch = -atan(tan_pitch) / CV_PI * 180;
    test_y_yaw = atan(tan_yaw) / CV_PI * 180;
    // x_pitch = -atan(tan_pitch) / CV_PI * 180;
    // y_yaw = atan(tan_yaw) / CV_PI * 180;
}

// PinHole method（使用小孔成像原理）
void AngleSolver::PinHole_solver() {
    // CAMERA_MATRIX 相机内参矩阵
    double fx = CAMERA_MATRIX.at<double>(0, 0);
    double fy = CAMERA_MATRIX.at<double>(1, 1);
    double cx = CAMERA_MATRIX.at<double>(0, 2);
    double cy = CAMERA_MATRIX.at<double>(1, 2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(targetCenter);

    /*
        * Goal: 将拍摄的图像矫正为正常的视角，便于检测
        * undistortPoints() 根据相机参数和观测到点坐标位置计算实际坐标位置
    */
    undistortPoints(in, out, CAMERA_MATRIX, DISTORTION_COEFF, noArray(), CAMERA_MATRIX);
    // .front() 用于获取 vector<cv::Point2f> 数组中的第一个
    pnt = out.front();

    /*
        * Goal: 去畸变后的比值
        * Formula: tan<pitch> = X / Z = (x<screen> - cx) / fx
        *		   tan<yaw> = Y / Z = (y<screen> - cy) / fy
    */
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;

    test_x_pitch = -atan(ryNew) / CV_PI * 180;
    test_y_yaw = atan(rxNew) / CV_PI * 180;
    // x_pitch = -atan(ryNew) / CV_PI * 180;
    // y_yaw = atan(rxNew) / CV_PI * 180;
}

// 角度补偿综合函数
void AngleSolver::compensateAngle() {
    compensateOffset();		// 机筒相机仰角补偿
    compensateGravity();	// 子弹重力补偿
}

/*
	* 对机筒与相机之间在y方向上的距离进行俯仰补偿（相机装在机筒的正上方 所以只需要对y方向上的差距进行补偿就可以）
	* Remark: 假定摄像头安装在机筒正上方，所以GUN_CAM_DISTANCE_Y的值为正值
*/
void AngleSolver::compensateOffset() {
    float camera_target_height = distance * sin(x_pitch / 180 * CV_PI);
    float gun_target_height = camera_target_height + GUN_CAM_DISTANCE_Y;
    float gun_pitch_tan = gun_target_height / (distance * cos(x_pitch / 180 * CV_PI));
    x_pitch = atan(gun_pitch_tan) / CV_PI * 180;
}

/*
	* 重力的俯仰补偿
	* Remark: 公式暂定，后期根据战车情况进行进一步优化修改
*/
void AngleSolver::compensateGravity() {
    float compensateGravity_pitch_tan = tan(x_pitch / 180 * CV_PI) + (0.5 * 9.8 * (distance / BULLET_SPEED) * (distance / BULLET_SPEED)) / cos(x_pitch / 180 * CV_PI);
    x_pitch = atan(compensateGravity_pitch_tan) / CV_PI * 180;
}

/*
	* Goal: 使用solvePnP根据二维坐标点来获取yaw方向和pitch方向的角度以及distance距离
	* Param: contourPoints 装甲板的四角顶点
	* Param: centerPoint 装甲板的中心点
	* Param: type 装甲板的类型[SMALL_ARMOR or BIG_ARMOR]
	* Param: yaw 机筒所需转动的yaw值（负值表示需要向左，正值表示需要向右）
	* Param: pitch 机筒所需转动的pitch值（负值表示需要向下，正值表示需要向上）
	* Param: evaluateDistance 所计算出的距离（单位为 mm）
*/
void AngleSolver::getAngle(pattern model, struct ArmorBox& Armor, CarData CarDatas, vector<Point2f>& contourPoints, Point2f centerPoint, ArmorType type, double& yaw, double& pitch, double& evaluateDistance, double& light_len)
{
    setTarget(model, Armor, contourPoints, centerPoint, type);
    setBulletSpeed(CarDatas.ShootSpeed);
    solveAngles(CarDatas);
    // compensateAngle();
    // cout << "y_yaw======" << test_y_yaw << endl;
    // cout << "x_pitch======" << test_x_pitch << endl;
    // distance = (int)(sqrt(pow(BestArmor.tz, 2) + pow(BestArmor.ty, 2) + pow(BestArmor.tx, 2)) / 100) + 1;
    // cout << "distance======" << distance << endl;

    double left_x = contourPoints[3].x;
    double right_x = contourPoints[2].x;
    std::cout << "-------1-------" << contourPoints << endl;
    std::cout << "-------lower_l.x-------" << left_x << endl;
    std::cout << "-------lower_r.x-------" << right_x << endl;
    std::cout << "-------light_length-------   " << abs(left_x - right_x) << endl;
    light_len = abs(left_x - right_x);

    // 要往右，数值加，往左，数值减
    yaw = test_y_yaw + 14.5;
    // if (yaw < 0)
    // {
	// yaw = -(abs(yaw) + 1);
    // }
    // else if (yaw > 0)
    // {
	// yaw = yaw + 1;
    // }
    // else
    // {
	// yaw = yaw;
    // }
    // pitch = test_x_pitch + 9.3;
    double plus = 0;
    if (type == 1) {
    	if (light_len >= 63)
	    plus = 8;
        else if (light_len < 63 && light_len >= 53)
	    plus = light_len/100 + 737/100;
        else if (light_len < 53 && light_len >= 47)
	    plus = light_len/30 + 92/15;
        else if (light_len < 47 && light_len >= 42)
	    plus = 3*light_len/100 + 629/100;
        else if (light_len < 42)
	    plus = light_len/20 + 109/20;
    }
    else if (type == 0){
	if (light_len >= 62)
	    plus = 8.85;
        else if (light_len < 62 && light_len >= 52)
	    plus = 0.005 * light_len + 8.54;
        else if (light_len < 52 && light_len >= 46)
	    plus = 0.0083 * light_len + 8.3667;
        else if (light_len < 46 && light_len >= 41)
	    plus = 0.02 * light_len + 7.83;
        else if (light_len < 41 && light_len >= 35)
	    plus = 0.0167 * light_len + 7.9667;
	else if (light_len < 35 && light_len >= 32)
	    plus = 0.0333 * light_len + 7.3833;
	else if (light_len < 32 && light_len >= 28)
	    plus = 0.025 * light_len + 7.65;
	else if (light_len < 28 && light_len >= 25)
	    plus = 0.0833 * light_len + 6.0167;
	else if (light_len < 25 && light_len >= 23)
	    plus = 0.1 * light_len + 5.6;
	else if (light_len < 23 && light_len >= 21)
	    plus = 0.075 * light_len + 6.175;
	else if (light_len < 21 && light_len >= 20)
	    plus = 0.1 * light_len + 5.65;
	else if (light_len < 20 && light_len >= 17)
	    plus = 0.05 * light_len + 6.65;
	else if (light_len < 17 && light_len >= 16)
	    plus = 0.02 * light_len + 7.16;
	else if (light_len < 16 && light_len >= 14)
	    plus = 0.03 * light_len + 7;
	else if (light_len < 14 && light_len >= 13)
	    plus = 7.42;
	else if (light_len < 13 && light_len >= 12)
	    plus = -0.03 * light_len + 7.81;
	else if (light_len < 12)
	    plus = 7.41;
	// plus = 8.85;
    }
    cout << "plus=================" << plus << endl;
    // 要往上，数值减，往下，数值加
    pitch = test_x_pitch + plus + 1;
    // pitch = test_x_pitch + 7.35;
    // yaw = y_yaw;
    // pitch = x_pitch;
    evaluateDistance = distance;
}

// 缓冲状态击打
void AngleSolver::BufferSetFilter(struct ArmorBox& BestArmor, CarData CarDatas, double& pitch, double& yaw) {
    // Send为发送角度，run为一定时间t内云台转动角度量
    // add为t内目标估计运动量，add = v*t
    // 新的发送量Send_now = Send + add - run
    if (!KF_forecast.is_set_x) {
        x_pitch = SendPitch - (CarDatas.pitch - old_carDatas.pitch);
        y_yaw = SendYaw - (CarDatas.yaw - old_carDatas.yaw);

        return;
    }
    double t = CarDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    BestArmor.tx = old_objectP.x + KF_forecast.x_(3) / 1000 * t;
    BestArmor.ty = old_objectP.y + KF_forecast.x_(4) / 1000 * t;
    BestArmor.tz = old_objectP.z + KF_forecast.x_(5) / 1000 * t;
    Angle_t at = ComputeShootTime(BestArmor.tx, BestArmor.ty, BestArmor.tz, CarDatas);
    x_pitch = at.pitch - (CarDatas.pitch - old_carDatas.pitch);
    y_yaw = at.yaw - (CarDatas.yaw - old_carDatas.yaw);
    BestArmor.tx = old_objectP.x;
    BestArmor.ty = old_objectP.y;
    BestArmor.tz = old_objectP.z;

    pitch = x_pitch;
    yaw = y_yaw;
}

// 显示Debug的数据
void AngleSolver::showDebugInfo(CarData CarDatas, bool showCurrentResult, bool showTVec, bool showP4P, bool showPinHole, bool showCompensation, bool showCameraParams)
{
    if (showCurrentResult)
    {
        Mat angleImage = Mat::zeros(250, 600, CV_8UC3);
        putText(angleImage, "Yaw: " + to_string(y_yaw), Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Pitch: " + to_string(x_pitch), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Distance: " + to_string(distance), Point(100, 150), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "X:" + to_string((int)(BestArmor.tx)), Point(50, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Y:" + to_string((int)(BestArmor.ty)), Point(250, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        putText(angleImage, "Z:" + to_string((int)(BestArmor.tz)), Point(400, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
        imshow("AngleSolver", angleImage);
    }
    if (showTVec)
    {
        cout << "tVec:" << endl;
        cout << " X:" << BestArmor.tx;
        cout << " Y:" << BestArmor.ty;
        cout << " Z:" << BestArmor.tz;
        cout << endl;
        cout << "-----------------------------------------------" << endl;
    }
    float yaw_temp = y_yaw, pitch_temp = x_pitch;
    if (showP4P)
    {
        P4P_solver();
        cout << "P4P Solver:" << endl;
        cout << "Yaw: " << y_yaw << "Pitch: " << x_pitch << endl;
        cout << "-----------------------------------------------" << endl;
        y_yaw = yaw_temp; x_pitch = pitch_temp;
    }
    if (showPinHole)
    {
        PinHole_solver();
        cout << "PinHole Solver:" << endl;
        cout << "Yaw: " << y_yaw << "Pitch: " << x_pitch << endl;
        cout << "-----------------------------------------------" << endl;
        y_yaw = yaw_temp; x_pitch = pitch_temp;
    }
    if (showCompensation)
    {
        solveAngles(CarDatas);
        float raw_pitch;
        raw_pitch = x_pitch;
        compensateOffset();
        cout << "Gun-Camera Offset:" << x_pitch - raw_pitch << endl;
        cout << "Yaw: " << y_yaw << "Pitch: " << x_pitch << endl;
        cout << "-----------------------------------------------" << endl;

        solveAngles(CarDatas);
        compensateGravity();
        cout << "Gravity:" << x_pitch - raw_pitch << endl;
        cout << "Yaw: " << y_yaw << "Pitch: " << x_pitch << endl;
        cout << "-----------------------------------------------" << endl;

        solveAngles(CarDatas);
        compensateAngle();
        cout << "Compensation:" << endl;
        cout << "Pitch Compensation:" << x_pitch - raw_pitch << endl;
        cout << "Yaw: " << y_yaw << "Pitch: " << x_pitch << endl;
        cout << "-----------------------------------------------" << endl;
        y_yaw = yaw_temp; x_pitch = pitch_temp;
    }
    if (showCameraParams)
    {
        cout << "CANERA_MATRIX:" << endl;
        cout << CAMERA_MATRIX << endl;
        cout << "-----------------------------------------------" << endl;
        cout << "DISTORTION_COEFF" << endl;
        cout << DISTORTION_COEFF << endl;
        cout << "-----------------------------------------------" << endl;
    }
}

/*
	* Remark: 如果矩阵类型为 CV_8U,则使用 Mat.at<uchar>(i,j)
			  如果矩阵类型为 CV_8S,则使用 Mat.at<schar>(i,j)
			  如果矩阵类型为 CV_16U,则使用 Mat.at<ushort>(i,j)
			  如果矩阵类型为 CV_16S,则使用 Mat.at<short>(i,j)
			  如果矩阵类型为 CV_32s,则使用 Mat.at<int>(i,j)
			  如果矩阵类型为 CV_32F,则使用 Mat.at<float>(i,j)
			  如果矩阵类型为 CV_64F,则使用 Mat.at<double>(i,j)
	* Remark: 单通道图像： Mat.at<存储类型名称>(行，列)
			  多通道图像： Mat.at<存储类型名称>(行，列)[通道]
*/
