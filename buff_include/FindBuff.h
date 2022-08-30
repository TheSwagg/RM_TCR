#ifndef FINDBUFF_H
#define FINDBUFF_H
#include"../header/General.h"

class FindBuff{
private:
    void PreDelBuff(Mat Src, Mat &dst);
    vector<RotatedRect> FindBestBuff(Mat Src,Mat & dst);
    RotatedRect GetShootBuff(vector<RotatedRect> box_buffs,Mat Src);

public:
    RM_BuffData* BuffModeSwitch(Mat Src);
};

#endif // FINDBUFF_H
