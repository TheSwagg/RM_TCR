/*
    * Date: 2022.3.9
    * Details: 卡尔曼滤波工具类
*/

#include "../header/Filter.h"

//一阶卡尔曼预测

// 无参数构造函数
KF_two::KF_two() {
    //状态协方差矩阵附初值,搭配绝对位置的移动预测
    Eigen::MatrixXd P_in = Eigen::MatrixXd(6, 6);
    P_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    P = P_in;

    //过程噪声矩阵附初值
    Eigen::MatrixXd Q_in(6, 6);
    Q_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Q = Q_in;

    //测量矩阵附初值
    Eigen::MatrixXd H_in(6, 6);
    H_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.00,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    H = H_in;

    //测量噪声矩阵附初值
    Eigen::MatrixXd R_in(6, 6);
    R_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    R = R_in;

}

/*
    * 有参数构造函数
    * KF_two类初始化重载
    * P_in状态协方差矩阵    Q_in过程噪声矩阵    H_in测量矩阵    R_in测量噪声矩阵
*/
KF_two::KF_two(Eigen::MatrixXd P_in, Eigen::MatrixXd Q_in, Eigen::MatrixXd H_in, Eigen::MatrixXd R_in) {
    P = P_in;
    Q = Q_in;
    H = H_in;
    R = R_in;
}


/*
    * 给状态向量附初值
    * x 状态向量初值      _F状态转移矩阵
*/
void KF_two::set_x(Eigen::VectorXd x, Eigen::MatrixXd _F) {
    F = _F;
    x_ = x;
    is_set_x = true;
}

// 给状态向量附初值
void KF_two::set_x(Eigen::VectorXd x) {
    x_ = x;
    is_set_x = true;
}

/*
    * 传入状态矩阵,进行预测部分计算
    * Remark: _F对应当前状态的状态转移矩阵
*/
void KF_two::Prediction(Eigen::MatrixXd _F) {
    F = _F;
    //得到预测值
    x_ = F * x_;
    P = F * P * F.transpose() + Q;
}

/*
    * 有参数，代表与传入状态矩阵相乘
    * Remark: _F对应当前状态的状态转移矩阵
    * Return:  状态向量
*/
Eigen::VectorXd KF_two::GetPrediction(Eigen::MatrixXd _F) {
    return _F * x_;
}

/*
    * 无参数,代表直接使用类内状态向量和状态矩阵相乘,返回相乘向量
    * Return:  状态向量
*/
Eigen::VectorXd KF_two::GetPrediction() {
    return F * x_;
}

/*
    * 返回状态向量，获得预测值
    * Return 状态向量
*/
Eigen::VectorXd KF_two::get_x() {
    return x_;
}

// 状态更新
void KF_two::update(Eigen::VectorXd z, Eigen::MatrixXd _F) {
    F = _F;
    Eigen::MatrixXd y = z - H * x_;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x_ = x_ + (K * y);
    int size = x_.size();

    // Eigen::MatrixXd I = Eigen::MatrixXd(size,size);
    if (size == 4) {
        Eigen::MatrixXd I(4, 4);
        I << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        P = (I - K * H) * P;
    }

    if (size == 2) {
        Eigen::MatrixXd I(2, 2);
        I << 1, 0,
                0, 1;
        P = (I - K * H) * P;
    }

    if (size == 6) {
        Eigen::MatrixXd I(6, 6);
        I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        P = (I - K * H) * P;
    }

    if (size == 4) {
        Eigen::MatrixXd I(4, 4);
        I << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        P = (I - K * H) * P;
    }
}