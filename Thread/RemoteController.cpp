#include "../header/RemoteController.h"

// 全局收发数变量
RemoteController Send;
RemoteController Recive;

#define BUFFER_SIZE_ 1
SerialPort serial;
serial_receive_data rx_data;
serial_transmit_data tx_data;
volatile unsigned int SerialBuffer;
int fd;
int last_fd;

RemoteController::RemoteController() {
    BeginTime = (double)getTickCount();
}

// 从下位机接收数据
void RemoteController::paraGetCar() {
    while (1) {
        int mode = 4;
	float pitch = 0;
	float yaw = 0;
	float ShootSpeed = 0;
	int col = 3;
	if (fd != 0)
	{
	    cout << "串口开始读取数据" << endl;
            serial.read_data(&rx_data, mode, ShootSpeed, pitch, yaw, col, fd, last_fd);
            CarData GetCar;
            GetCar.pitch = pitch;
            GetCar.yaw = yaw;
            GetCar.ShootSpeed = ShootSpeed;
	    cout << "-------------GetCar.ShootSpeed=" << ShootSpeed << endl;
            if (mode == 2)
                GetCar.model_ = BUFF;
            else if (mode == 1)
                GetCar.model_ = DAHENG;
	    else
		GetCar.model_ = VIDEO;
            cout << "-------------GetCar.model_=" << mode << endl;
            double timeing = ((double)getTickCount() - BeginTime) / getTickFrequency();
            cout<<timeing;
            GetCar.BeginToNowTime = timeing * 1000.0;
            if (col == 0)
	    {
		GetCar.c = BLUE;
		last_color = GetCar.c;
	    }
            else if (col == 1)
	    {
                GetCar.c = RED;
		last_color = GetCar.c;
	    }
	    else
		GetCar.c = last_color;

            // 挂起资源锁
            reciveRes.lock();
            getStm32.pitch = GetCar.pitch;
            getStm32.yaw = GetCar.yaw;
            getStm32.ShootSpeed = GetCar.ShootSpeed;
            getStm32.BeginToNowTime = GetCar.BeginToNowTime;
            getStm32.c = GetCar.c;
            getStm32.model_ = GetCar.model_;
            // 解锁
            reciveRes.unlock();
	}
    }
}

// 上位机接收数据发送给下位机
void RemoteController::paraReceiver() {
    serial = SerialPort(Serial_Path, Serial_Baud, fd, last_fd);		// 打开并初始化串口
    while (1)
    {
        while (SerialBuffer >= BUFFER_SIZE_);
        SerialBuffer = BUFFER_SIZE_;
        serial.send_data(tx_data, fd, last_fd);
    }
    serial.close_port(fd);
}

// 汇总（接收）角度解算的数据，为下一步发送数据到下位机做准备
void RemoteController::ArmorToData(pattern Car_model, double pitch, double yaw, int light_len) {
    cout << "pitch==================" << pitch << endl;
    cout << "yaw====================" << yaw << endl;

//    if (Car_model == Stop)
//        tx_data.get_xy_data(0, 0);
//    else
	// if (pitch < 1 && pitch > -1)
	    // pitch = 0;
	// if (yaw < 1 && yaw > -1)
	    // yaw = 0;
	// if (pitch < -5 || pitch > 5)
	    // pitch = 0;
	// if (yaw < -5 || yaw > 5)
	    // yaw = 0;
        tx_data.get_xy_data(int16_t(pitch * 32767 / 90), int16_t(yaw * 32767 / 90), light_len);
	// tx_data.get_xy_data(int16_t(-20.0), int16_t(-30.0));
    SerialBuffer--;
}

// 获取时间
double RemoteController::getClock() {
    double timeing = ((double)getTickCount() - BeginTime) / getTickFrequency();
    return timeing * 1000.0;
}
