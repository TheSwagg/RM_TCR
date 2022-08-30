#include "DaHengCamera.h"

// 构造函数,函数初始化
DaHengCamera::DaHengCamera() {
    // 初始化库
    // 该函数相当于开辟一个空间，申请资源
    status = GXInitLib();
    // 检测初始化是否成功
    if(status != GX_STATUS_SUCCESS){
        cout<<"相机库初始化失败!!!"<<endl;
    }
}

/*
 * Param serial_number为要打开设备的序列号
 * Return 返回检测到的连接相机个数
*/
int DaHengCamera::StartDevice(int serial_number) {
    uint32_t nDeviceNum = 0;
    // 枚举设备列表,超时时间受用户使用环境限制（1000表示限制时间在1000ms）
    // 返回的nDeviceNum就是当前网络上存在的相机数量
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if(serial_number > nDeviceNum){
        cout<<"设备号错误，超过所枚举数量"<<endl;
        return -1;
    }
    // 打开设备
    status = GXOpenDeviceByIndex(serial_number, &hDevice);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"设备打开成功!"<<endl;
        return nDeviceNum;
    } else {
        cout<<"设备打开失败!"<<endl;
        return -1;
    }
}


/*
 * 设置设备开始采集，设置分辨率应在采集图像之前
 * Reture 返回是否设置成功
*/
bool DaHengCamera::SetStreamOn() {
    // 设置buffer数量
    // 设置采集buffer个数
    status = GXSetAcqusitionBufferNumber(hDevice,2);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"buffer设置成功!"<<endl;
    } else {
        cout<<"buffer设置失败!"<<endl;
    }
    // 开采
    status = GXStreamOn(hDevice);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"开始采集图像!"<<endl;
        return true;
    } else {
        cout<<"采集失败!"<<endl;
        return false;
    }
}

/*
 * 设置分辨率
 * Param width_scale   宽比例
 *       height_scale  高比例
 * Return bool 返回是否成功
*/
bool DaHengCamera::SetResolution(int width_scale, int height_scale) {
    // 配置一个 2x2 的 Binning 和 2x2 的 Decimation
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t nBinningH = width_scale;
    int64_t nBinningV = height_scale;
    int64_t nDecimationH= width_scale;
    int64_t nDecimationV= height_scale;

    // 设置水平和垂直 Binning 模式为 Sum 模式
    // GXSetEnum()函数用于设置相机的图像采集模式
    status = GXSetEnum(hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,
                       GX_BINNING_HORIZONTAL_MODE_AVERAGE);
    status = GXSetEnum(hDevice,GX_ENUM_BINNING_VERTICAL_MODE,
                       GX_BINNING_VERTICAL_MODE_AVERAGE);
    status = GXSetInt(hDevice, GX_INT_BINNING_HORIZONTAL, nBinningH);
    status = GXSetInt(hDevice, GX_INT_BINNING_VERTICAL, nBinningV);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
    status = GXSetInt(hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"分辨率设置成功"<<endl;
        return true;
    } else {
        cout<<"分辨率设置失败"<<endl;
        return false;
    }
}

/*
 * 设置曝光值
 * Param ExposureTime  具体曝光值
 * Return bool 返回是否设置成功
*/
bool DaHengCamera::SetExposureTime(int ExposureTime) {
    // 设置曝光值
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, ExposureTime);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"曝光值设置成功"<<endl;
        return true;
    } else {
        cout<<"曝光值设置失败"<<endl;
        return false;
    }
}


/*
 * 手动设置曝光增益
 * Param value 选择曝光增益通道 0-B,1-G,2-R,3-All
 *       ExpGain 具体增益值 范围0-16
 * Return
*/
bool DaHengCamera::SetGAIN(int value,int ExpGain) {
    if(value == 0) {
        // 选择增益通道类型
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
    } else if(value == 1) {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
    } else if(value == 2) {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
    } else {
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    }
    // 设置曝光值
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, ExpGain);
    if(status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}

/*
 * 枚举变量为0是表示关闭，1为开启
 * Return bool 返回是否设置成功
*/
bool DaHengCamera::Set_BALANCE_AUTO(int value) {
    // 设置连续自动白平衡
    status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,value);
    if(status == GX_STATUS_SUCCESS) {
        cout<<"自动白平衡设置成功"<<endl;
        return true;
    } else {
        cout<<"自动白平衡设置失败"<<endl;
        return false;
    }
}

/*
 * 手动白平衡,设置之前必须先关闭自动白平衡,因为其具有记忆功能
 * Param value 选择平衡通道 0-B,1-G,2-R
 *       value_number 平衡系数
 * Return 返回是否设置成功
*/
bool DaHengCamera::Set_BALANCE(int value, int value_number) {
    if(value == 0){
        // 选择白平衡通道
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE);
    }else if(value == 1){
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN);
    }else{
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED);
    }
    status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, (float)value_number/10.0);
    if(status == GX_STATUS_SUCCESS)
        return true;
    else
        return false;
}

/*
 * 读取图像
 * Param Src 引入方式传递
 * Return bool 返回读取图像是否成功
*/
bool DaHengCamera::GetMat(Mat &Src) {

    // 清空缓冲队列
    // GXFlushQueue(hDevice);

    // 开始计时
    double time0 = static_cast<double>(getTickCount());

    int64_t nPayLoadSize = 0;   // 储存图像buffer大小
    // 获取图像 buffer 大小,下面动态申请内存
    status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
    if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0)
    {
        // 定义 GXGetImage 的传入参数
        GX_FRAME_DATA stFrameData;
        // 根据获取的图像 buffer 大小 nPayLoadSize 申请 buffer
        stFrameData.pImgBuf= malloc((size_t)nPayLoadSize);

        // 发送开始采集命令
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        if (status == GX_STATUS_SUCCESS)
        {
            // 调用 GXGetImage 读取一帧图像
            status = GXGetImage(hDevice, &stFrameData, 100);
            // time0=((double)getTickCount()-time0)/getTickFrequency();
            // printf("采集图像 %gms\n",time0*1000);

            if(stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
                char *pRGB24Buf = new char[stFrameData.nWidth * stFrameData.nHeight * 3]; // 输出图像RGB数据
                if (pRGB24Buf == NULL) {
                    return false;
                } else {
                    // 缓冲区初始化
                    memset(pRGB24Buf, 0, stFrameData.nWidth * stFrameData.nHeight * 3 * sizeof( char));
                }
                DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR3;  // 选择插值算法
                DX_PIXEL_COLOR_FILTER nBayerType = DX_PIXEL_COLOR_FILTER(BAYERBG);
                // 选择图像 Bayer 格式
                bool bFlip = false;
                VxInt32 DxStatus = DxRaw8toRGB24(stFrameData.pImgBuf, pRGB24Buf, stFrameData.nWidth, stFrameData.nHeight, cvtype, nBayerType, bFlip);
                if (DxStatus != DX_OK)
                {
                    if (pRGB24Buf != NULL) {
                        delete []pRGB24Buf;
                        pRGB24Buf = NULL;
                    }
                    return false;
                }

                Mat src = Mat(stFrameData.nHeight, stFrameData.nWidth, CV_8UC3);
                memcpy(src.data, pRGB24Buf, stFrameData.nWidth * stFrameData.nHeight * 3);
                src.copyTo(Src);
                time0=((double)getTickCount()-time0) / getTickFrequency();
                printf("采集图像 %gms\n", time0*1000);
                delete []pRGB24Buf;
                pRGB24Buf = NULL;
                free(stFrameData.pImgBuf);
                return true;
            }
            free(stFrameData.pImgBuf);
            return false;
        }
    }
    return false;
}

/*
 * 得到时间戳锁存值（还有问题没解决，可能是不支持此功能）
 * Return _time 单位ms
*/
double DaHengCamera::Get_TIMESTAMP() {
    int64_t time = 0;
    status = GXGetInt(hDevice, GX_INT_TIMESTAMP_LATCH_VALUE, &time);
    // 设置更新频率为125000000Hz
    double _time = (double)time/125000.0;
    return _time;
}

/*
 * 析构函数用于关闭设备
*/
DaHengCamera::~DaHengCamera() {
    // 停止图像的采集
    status = GXStreamOff(hDevice);
    // 关闭设备链接
    status = GXCloseDevice(hDevice);
    // 释放库
    status = GXCloseLib();
    cout<<"析构，相机设备已关闭"<<endl;
}
