#include "../header/Armor.h"

/*
	* Date: 2022.2.27
	* Details: 装甲板匹配相关函数源文件
*/

// 删除游离灯条导致的错误装甲板
void eraseErrorRepeatArmor(vector<ArmorBox>& armors);
bool armorCompare(const ArmorBox& a_armor, const ArmorBox& b_armor, const ArmorBox& lastArmor, const int& targetNum);

/*
	* 判断当前未识别到目标次数是否大于设定的缓冲次数
	* 如果大于设定的缓冲次数，设定为未识别到
	* 如果小于或者等于设定的缓冲次数，赋值和上一次识别结果相同
*/
void ArmorDetector::Outof_buffer() {
    if (armorParam.Buffer_Num >= armorParam.Max_Buffer_Num) {
        targetArmor = ArmorBox();
        armorParam.Buffer_Num++;
        model = Stop;
    }
    else if (armorParam.Buffer_Num < armorParam.Max_Buffer_Num) {
        targetArmor = lastArmor;
        armorParam.Buffer_Num++;
        model = buffering;
    }
}

// 将识别到的灯条拟合成装甲板
void ArmorDetector::matchArmors() {
    for (int i = 0; i < lights.size() - 1; i++)		// 迭代识别到的灯条
    {
        for (int j = i + 1; j < lights.size(); j++) // 从左至右，每个灯条与其他灯条进行一次匹配判断
        {
            ArmorBox armor = ArmorBox(lights[i], lights[j]); // 利用左右灯条构建装甲板
            // 利用ArmorBox类中的子函数判断这个装甲板是否是一个合适的装甲板
            if (armor.isSuitableArmor()) // 如果是合适的装甲板，则设置其他装甲板信息
            {
                armor.l_index = i; // 左灯条在lights数组中的下标
                armor.r_index = j; // 右灯条在lights数组中的下标
                classifier.getArmorImg(armor);// 透视变换装甲板图像
                classifier.setArmorNum(armor);// 利用SVM实现装甲板数字识别
                armors.emplace_back(armor); // 将识别到的装甲板存入armors数组中
            }
        }

        eraseErrorRepeatArmor(armors);// 删除游离灯条导致的错误装甲板
    }
    if (armors.empty()) {
        state = ARMOR_NOT_FOUND; // 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
        return;
    }
    else {
        state = ARMOR_FOUND; //如果非空（有装甲板），则设置状态ARMOR_FOUND
        return;
    }
}

// 将上一帧的目标装甲板作为lastArmor选择本帧图像中所有装甲板里面价值最大的装甲板作为目标装甲板
void ArmorDetector::setTargetArmor() {
    if (state == ARMOR_NOT_FOUND)		// 如果状态为没有找到装甲板，则将lastArmor设置为默认的ArmorBox
        Outof_buffer();
    else if (state == ARMOR_FOUND) {
        armorParam.Buffer_Num = 0;		// 重置缓冲次数
        ArmorBox mva = armors[0]; // 初始最适合打击的装甲板为识别到的第一个装甲板
        for (int i = 1; i < armors.size(); i++) // 通过遍历装甲板获取最佳打击装甲板
        {
            if (armorCompare(armors[i], mva, lastArmor, targetNum))
                mva = armors[i];
        }
        targetArmor = mva;	 // 设置找到的最佳打击装甲板为这一帧的最佳打击装甲板（targetArmor）
        if (model == buffering || model == Stop)
            model = FirstFind;
        else
            model = Shoot;
    }
    lastArmor = targetArmor;	// 将这一帧的targetArmor设置为下一帧的lastArmor
}

// 针对游离灯条导致的错误装甲板进行检测和删除
void eraseErrorRepeatArmor(vector<ArmorBox>& armors) {
    int length = armors.size();
    vector<ArmorBox>::iterator it = armors.begin();		// 迭代检测到的装甲板
    // 根据同一位置上的灯条由于游离灯条可能构造出其他错误的装甲板，因此根据灯条错位度角删除错位角大的装甲板
    for (size_t i = 0; i < length; i++) {
        for (size_t j = i + 1; j < length; j++)
        {
            if (armors[i].l_index == armors[j].l_index ||
                armors[i].l_index == armors[j].r_index ||
                armors[i].r_index == armors[j].l_index ||
                armors[i].r_index == armors[j].r_index)
            {
                // .erase() 删除所迭代到的错误装甲板
                armors[i].getDeviationAngle() > armors[j].getDeviationAngle() ? armors.erase(it + i) : armors.erase(it + j);
            }
        }
    }
}

// 获取两点之间的距离
float getPointsDistance(const Point2f& a, const Point2f& b) {
    float delta_x = a.x - b.x;
    float delta_y = a.y - b.y;
    // 两点之间距离公式
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

// 根据打击优先级增加装甲板的打击度
void setNumScore(const int& armorNum, const int& targetNum, float& armorScore) {
    if (targetNum == 0)
    {
        if (armorNum == 1) armorScore += 1000;
        else if (armorNum == 2) armorScore += 2000;
        else if (armorNum == 3) armorScore += 3000;
        else if (armorNum == 4) armorScore += 4000;
        else if (armorNum == 5) armorScore += 5000;
        else if (armorNum == 6) armorScore += 6000;
    }
    if (armorNum == targetNum) armorScore += 100000;
}

/*
	* Goal: 比较a_armor装甲板与b_armor装甲板的打击度，判断a_armor是否比b_armor更适合打击
	* Method: 通过装甲板数字是否与目标装甲板数字匹配，装甲板与lastArmor的距离以及装甲板的面积大小判断
*/
bool armorCompare(const ArmorBox& a_armor, const ArmorBox& b_armor, const ArmorBox& lastArmor, const int& targetNum)
{
    float a_score = 0;  // a_armor的打击度
    float b_score = 0;  // b_armor的打击度
    // 根据面积获取得分，距离机筒越远的装甲板面积越小，得分越低
    a_score += a_armor.armorRect.area(); // a_armor面积得分（面积越大得分越高）
    b_score += b_armor.armorRect.area(); // b_armor面积得分

    // 获取a、b装甲板综合分数
    // setNumScore(a_armor.armorNum, targetNum, a_score);
    // setNumScore(b_armor.armorNum, targetNum, b_score);
    cout << "lastArmor.center==============" << lastArmor.center << endl;
    float a_distance = getPointsDistance(a_armor.center, lastArmor.center);
    float b_distance = getPointsDistance(b_armor.center, lastArmor.center);
    a_score -= a_distance * 5;
    b_score -= b_distance * 5;

    /*
        * Goal: 判断上一帧中是否存在目标装甲板，如果存在目标装甲板，那么根据现在的目标装甲板位置和上一帧装甲板距离远近进行算分
        * Remark: 装甲板距离得分，算负分，距离上一帧装甲板距离越近扣分越少
    */
    // if (lastArmor.armorNum != 0) {  // 上一帧图像中存在目标装甲板
        // float a_distance = getPointsDistance(a_armor.center, lastArmor.center);
        // float b_distance = getPointsDistance(b_armor.center, lastArmor.center);
        // a_score -= a_distance * 2;
        // b_score -= b_distance * 2;
    // }
    return a_score > b_score; // 根据打击度判断a是否比b更适合打击，得分越高越合适
}
