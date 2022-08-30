#include "../header/Armor.h"

/*
	* Date: 2022.2.27
	* Details: 装甲板类源文件
*/

// 按照bl（左下）, tl（左上）, tr（右上）, br（右下）的顺序计算这四个点的交点 （用于计算装甲板的中心点）
const Point2f crossPointof(const Point2f& bl, const Point2f& tl, const Point2f& tr, const Point2f& br) {
    float a1 = tr.y - bl.y;		// y方向的差距
    float b1 = tr.x - bl.x;		// x方向的差距
    float c1 = bl.x * tr.y - tr.x * bl.y;

    float a2 = br.y - tl.y;
    float b2 = br.x - tl.x;
    float c2 = tl.x * br.y - br.x * tl.y;

    float d = a1 * b2 - a2 * b1;

    if (d == 0.0) {
        return Point2f(FLT_MAX, FLT_MAX);
    }
    else {
        return cv::Point2f((b2 * c1 - b1 * c2) / d, (c1 * a2 - c2 * a1) / d);
    }
}

// 利用两个LightBar的旋转矩形lightRect来构造装甲板的四个顶点（ArmorVertices）
void setArmorVertices(const LightBar& l_light, const LightBar& r_light, ArmorBox& armor) {
    //处理两个灯条
    cv::Size exLSize(int(l_light.lightRect.size.width), int(l_light.lightRect.size.height * 2));
    cv::Size exRSize(int(r_light.lightRect.size.width), int(r_light.lightRect.size.height * 2));
    cv::RotatedRect exLLight(l_light.center, exLSize, armor.armorAngle);
    cv::RotatedRect exRLight(r_light.center, exRSize, armor.armorAngle);

    // 选装矩阵的.points[4]中为顺时针的旋转矩阵四个顶点，[0]和[3]之间的距离为旋转矩阵的宽
    cv::Point2f pts_l[4];
    exLLight.points(pts_l);
    cv::Point2f upper_l = pts_l[2];
    cv::Point2f lower_l = pts_l[3];

    cv::Point2f pts_r[4];
    exRLight.points(pts_r);
    cv::Point2f upper_r = pts_r[1];
    cv::Point2f lower_r = pts_r[0];

    armor.armorVertices.clear();
    // armor.armorVertices[0] = upper_l;
    // armor.armorVertices[1] = upper_r;
    // armor.armorVertices[2] = lower_r;
    // armor.armorVertices[3] = lower_l;
    armor.armorVertices.push_back(upper_l);
    armor.armorVertices.push_back(upper_r);
    armor.armorVertices.push_back(lower_r);
    armor.armorVertices.push_back(lower_l);
}

// 装甲板相关数据信息无参数初始化
ArmorBox::ArmorBox() {
    l_index = -1;
    r_index = -1;
    l_light = LightBar();
    r_light = LightBar();
    armorNum = 0;
    // armorVertices.resize(4);
    type = SMALL_ARMOR;
    center = Point2f();
    armorRect = Rect();
    armorImg = Mat();
}

// 装甲板相关数据信息有参数构造函数
ArmorBox::ArmorBox(const LightBar& l_light, const LightBar& r_light) {
    // 定义左右灯条
    this->l_light = l_light;
    this->r_light = r_light;

    armorNum = 0;
    // 装甲板角度即为两灯条的角度平均值
    armorAngle = (l_light.angle + r_light.angle) / 2;

    //获取装甲板的四个顶点 bl->tl->tr->br 左下 左上 右上 右下
    armorVertices.resize(4);
    setArmorVertices(l_light, r_light, *this);    // '*this' 就表示为当前装甲板

    // 利用装甲板的四个顶点获取装甲板的中心点（vector<Point2f>）
    center = crossPointof(armorVertices[0], armorVertices[1], armorVertices[2], armorVertices[3]);

    // 得到对象轮廓后，用boundingRect()函数得到包覆此轮廓的最小正矩形（装甲板的roi区域）
    armorRect = boundingRect(armorVertices);

    // 判断装甲板的类型
    // 根据两灯条中心X方向差距的比值判断（大于一定数值时判定为大装甲板，即为哨兵装甲板）
    if (this->getDislocationX() > 2.7)
        type = BIG_ARMOR;	// 1
    else
	type = SMALL_ARMOR;	// 0
}

// 析构函数
ArmorBox::~ArmorBox() {}

// 获取装甲板左右灯条角度差
float ArmorBox::getAngleDiff() const {
    float angle_diff = abs(l_light.angle - r_light.angle); // 灯条的角度差（abs()函数取绝对值）
    return angle_diff;
}

// 左右灯条长度差比值
float ArmorBox::getLengthRation() const {
    float length_diff = abs(l_light.length - r_light.length);
    float lengthDiffRation = length_diff / MAX(l_light.length, r_light.length);
    return lengthDiffRation;
}

// 获取灯条错位度角(两灯条中心连线与水平线夹角) 
float ArmorBox::getDeviationAngle() const {
    float delta_x = r_light.center.x - l_light.center.x; // Δx
    float delta_y = r_light.center.y - l_light.center.y; // Δy
    float deviationAngle = abs(atan(delta_y / delta_x)) * 180 / CV_PI; // tanθ=Δy/Δx （atan()函数用于求解tan的角度）
    return deviationAngle;
}

// 灯条位置差距 两灯条中心X方向差距比值
float ArmorBox::getDislocationX() const {
    float meanLen = (l_light.length + r_light.length) / 2;
    float xDiff = abs(l_light.center.x - r_light.center.x); //x distance ration x轴方向上的距离比值（x轴距离与灯条平均值的比）
    float xDiff_ratio = xDiff / meanLen;
    return xDiff_ratio;
}

// 灯条位置差距 两灯条中心Y方向差距比值
float ArmorBox::getDislocationY() const {
    float meanLen = (l_light.length + r_light.length) / 2;
    float yDiff = abs(l_light.center.y - r_light.center.y);  //y轴方向上的距离比值（y轴距离与灯条平均值的比）getDislocationX
    float yDiff_ratio = yDiff / meanLen;
    return yDiff_ratio;
}

// 判断本装甲板是否是合适的装甲板
bool ArmorBox::isSuitableArmor() const
{
    return
            this->getAngleDiff() < armorParam.max_angle_diff &&				// 灯条角度差判断，需小于允许的最大角差
            this->getDeviationAngle() < armorParam.max_deviation_angle &&	// 灯条错位度角(两灯条中心连线与水平线夹角)判断
            this->getDislocationX() < armorParam.max_x_diff_ratio &&		// 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
            this->getDislocationY() < armorParam.max_y_diff_ratio &&		// 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
            this->getLengthRation() < armorParam.max_lengthDiff_ratio;		// 左右两个灯条之间允许的最大长度差比值不可偏大（用比值作为衡量依据）
}
