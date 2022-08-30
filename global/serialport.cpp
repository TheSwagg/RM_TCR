#include "../header/serialport.h"
#include <fcntl.h>		/*文件控制定义*/

/*
    * 接受下位机数据

        | 头帧 | 数据1 | 数据2 | 数据3 | 数据4 | 数据5 | 尾帧 |
        | 0xaa |  模式 | pitch |  yaw  |  弹速 | 敌方颜色 | 0xbb |

        ``` 
        数据1 模式 0为自瞄模式 1为能量机关模式
        数据2 当前pitch值
        数据3 当前yaw值
        数据4 当前弹速
        数据5 敌方颜色（0为蓝色 1为红色）
        ```

    * 发送至下位机数据

        | 头帧 | 数据1| 数据2 | 数据3| 数据4| 尾帧 |
        | 0xaa | yaw角度低八位 | yaw角度高八位 | pitch角度低八位 | pitch角度高八位 |0xbb |

        ``` 
        数据1|2 yaw角度
        数据3|4 pitch角度
        ```
*/

typedef union {
    float f;
    unsigned char c[4];
}float2uchar;

// 无参数构造函数
SerialPort::SerialPort() {}

// 串口初始化
SerialPort::SerialPort(const char* filename, int buadrate, int& fd, int& last_fd) {
    file_name_ = filename;
    buadrate_ = buadrate;
    success_ = false;

#ifdef LINUX    // linux情况下
    /*
        * 参数：file_name_为串口名称   模式选择
        * O_RDWR: 可读可写
        * O_NOCTTY: 该参数不会使打开的文件成为该进程的控制终端。如果没有指定这个标志，那么任何一个 输入都将会影响用户的进程。
        * O_SYNC: 等待物理 I/O 结束后再 write，包括更新文件属性的 I/O
    */
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);     // 打开串口
    last_fd = fd;
    if (fd == -1)
    {
        printf("open_port wait to open %s... \n",  file_name_);
        return;
    }
    else if (fd != -1)
    {
        // fcntl函数是linux下的一个文件锁函数，用以加密文件，给文件上锁，防止文件同时被多个进程操作。
        fcntl(fd, F_SETFL, 0);
        printf("port is open %s... \n", file_name_);
    }

    // 提供了一个常规的终端接口，用于控制非同步通信端口
    struct termios port_settings;
    if (buadrate_ == 0)
    {
        // B115200 115200波特
        cfsetispeed(&port_settings, B115200);   // 设置输入波特率
        cfsetospeed(&port_settings, B115200);   // 设置输出波特率
    }
    else if (buadrate_ == 1) 
    {
        // B921600 921600波特
        cfsetispeed(&port_settings, B921600);   // 设置输入波特率
        cfsetospeed(&port_settings, B921600);   // 设置输出波特率
    }

    /*
        * c_iflag：输入模式标志，控制终端输入方式
        * c_oflag：输出模式标志，控制终端输出方式
        * c_cflag：控制模式标志，指定终端硬件控制信息
        * c_lflag：本地模式标志，控制终端编辑功能
    */

    /*
        * 设置数据位
        * & ~CSIZE     把数据位清零
        * | CS8        把数据位设置为8位
    */
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;
    // ~IGNBRK       禁用忽略BREAK键输入（对不匹配的速度测试禁用IGNBRK，否则收到破坏）
    port_settings.c_iflag &= ~IGNBRK;         // 禁止中断处理
    port_settings.c_lflag = 0;                // 设置为零，表示没有信号字符，没有回音
    // 无规范处理
    port_settings.c_oflag = 0;                // 设置为零，表示没有重新映射，没有延迟
    port_settings.c_cc[VMIN] = 0;             // 最小字符，设置为零，表示读取不会被阻止
    port_settings.c_cc[VTIME] = 5;            // 等待时间，设置为五，表示0.5秒读取超时
    /*
        * ~IXON     禁止允许输入时对XON流进行控制
        * ~XOFF     禁止允许输入时对XOFF流进行控制
        * ~IXANY    禁止输入任何字符将重启停止的输出
    */
    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    /*
        * 通过位掩码的方式激活本地连接和接受使能选项
        * CLOCAL    略调制解调器线路状态（打开本地连接模式）
        * CREAD     使用接收器（开启串行数据接收）
    */
    port_settings.c_cflag |= (CLOCAL | CREAD);
    /*
        * 设置校验位
        * ~PARENB   禁止使用奇偶校验
        * ~PARODD   禁止对输入使用奇偶校验，禁止对输出使用偶校验
    */
    port_settings.c_cflag &= ~(PARENB | PARODD);        // 关闭奇偶校验
    port_settings.c_cflag |= 0;
    // 设置一位停止位
    // 一位停止位 &= ~CSTOPB            两位停止位 |= CSTOPB;
    port_settings.c_cflag &= ~CSTOPB;
    // 禁止使用RTS/CTS流控制
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // 最小字符，设置为10，表示读取被阻止10个字节
    port_settings.c_cc[VTIME] = 5;           // 等待时间，设置为五，表示0.5秒读取超时

    /*
        * 参数1: fd为打开的终端文件描述符
        * 参数2: optional_actions用于控制修改起作用的时间
        *        TCSANOW：不等数据传输完毕就立即改变属性
        *        TCSADRAIN：等待所有数据传输结束才改变属性
        *        TCSAFLUSH：等待所有数据传输结束,清空输入输出缓冲区才改变属性
        * 参数3: 结构体termios_p中保存了要修改的参数
    */
    tcsetattr(fd, TCSANOW, &port_settings);             // 将设置应用到串口
#endif  // LINUX

#ifdef WINDOWS
    /*
        * 参数1: 串口名
        * 参数2: 支持读写（GENERIC_READ | GENERIC_WRITE）
        * 参数3: 独占方式，串口不支持共享（0）
        * 参数4: 安全属性指针（默认为NULL）
        * 参数5: 打开现有的串口文件（OPEN_EXISTING）
        * 参数6: 同步方式（0）（异步方式为FILE_FLAG_OVERLAPPED）
        * 参数7: 用于复制文件句柄，默认值为NULL，对串口而言该参数必须设置为NULL
    */
    hcom = CreateFileA(file_name_, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hcom == (HANDLE)-1)
    {
        cout << "serialport open failed..." << endl;
        return;
    }
    else
        cout << "serialport open successful..." << endl;

    // 配置缓冲区大小
    if (!SetupComm(hcom, 1024, 1024))
    {
        cout << "set the buffer failed..." << endl;
        return;
    }

    // 配置参数
    DCB dcb;
    // 获取当前串口配置参数
    GetCommState(hcom, &dcb);
    if (buadrate_ == 0)
        dcb.BaudRate = 115200;     // 波特率
    else if (buadrate_ == 1)
        dcb.BaudRate = 921600;
    dcb.ByteSize = 10;             // 数据位
    dcb.Parity = NOPARITY;         // 无校验
    dcb.StopBits = ONESTOPBIT;     // 1位停止位
    SetCommState(hcom, &dcb);      // 设置配置参数到串口
     
    // 超时处理，单位：毫秒
    // 总超市 = 时间系数*读或写的字符数 + 时间常量
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = 1000;        // 读间隔超时
    timeouts.ReadTotalTimeoutConstant = 5000;   // 读时间常量
    timeouts.ReadTotalTimeoutMultiplier = 500;  // 读时间系数
    timeouts.WriteTotalTimeoutConstant = 2000;  // 写时间常量
    timeouts.WriteTotalTimeoutMultiplier = 500; // 写时间系数
    SetCommTimeouts(hcom, &timeouts);       // 设置超时处理参数到串口

    PurgeComm(hcom, PURGE_TXCLEAR | PURGE_RXCLEAR);     // 清空串口缓冲区
#endif  // WINDOWS

}

// 发送数据
void SerialPort::send_data(const struct serial_transmit_data& data, int& fd, int& last_fd)
{
#ifdef LINUX
    // if (data.size != write(fd, data.raw_data, data.size))
    cout << "------------" << endl;
    cout << data.raw_data << endl;
    cout << data.size << endl;
    cout << "------------" << endl;
    int in_ = write(fd, data.raw_data, data.size);
    if (in_ != data.size)
    {
        cout << "!!! send data failure !!!" << fd << endl;
        restart_serial(fd, last_fd);       // 如果串口发送失败，则考虑重新启动串口
        cout << "restart fd" << fd << endl;
    }
    else
      	cout << "串口数据发送成功" << endl;
#endif // LINUX

#ifdef WINDOWS
    COMSTAT comstat;
    DWORD dwerrorflags;
    ClearCommError(hcom, &dwerrorflags, &comstat);

    DWORD dwBytesWrite = data.size;
    BOOL bWriteStat;
    /*
        * 参数1: 串口句柄
        * 参数2: 数据首地址
        * 参数3: 要发送的数据字节数
        * 参数4: DWORN*，用来接收返回成功发送的数据字节数
        * 参数5: NULL(同步发送)    |    OVERLAPPED*(异步发送)
    */
    bWriteStat = WriteFile(hcom, data.raw_data, dwBytesWrite, &dwBytesWrite, NULL);
    if (!bWriteStat)
        cout << "write failed..." << endl;
    else
        cout << "write successfully..." << endl;

    PurgeComm(hcom, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);     // 清空串口缓冲区
#endif // WINDOWS

}

// 接收数据
bool SerialPort::read_data(const struct serial_receive_data* data, int& mode, float& bulletspeed, float& buff_offset_x, float& buff_offset_y, int& col, int& fd, int& last_fd)
{
#ifdef LINUX
    float2uchar pitch_data_;
    float2uchar yaw_data_;
  
    /*
        * tcflush() 刷新（扔掉）输入缓存或输出缓存
        * TCIFLUSH  刷新输入队列
        * TCOFLUSH  刷新输出队列
        * TCIOFLUSH 刷新输入，输出队列
    */
    cout << "接收数据的fd: " << fd << endl;
    tcflush(fd, TCIFLUSH);      // 丢弃输入缓冲区中的旧数据
    unsigned char read_buffer[14];       // 缓冲区存储接收到的数据
    int  bytes_read = 0;        // 存储系统调用read()所读取的字节数

    bytes_read = read(fd, &read_buffer, 13);     // 读取数据
    cout << "接收数据的fd_2: " << fd << endl;
    // cout << "bytes_read: " << bytes_read;
    // for (int i = 0; i < bytes_read; i++) {
    //     cout << "buf " << i << ": " << read_buffer[i];
    // }
    // cout << endl;
    cout << "bytes_read" << bytes_read << endl;
    if (bytes_read == -1 || bytes_read == 0)    // 读取失败
    {
        // cout << "can not read!" << endl;
        // NOTICE("can not read!",3);
	cout << "串口读取失败" << endl;
        restart_serial(fd, last_fd);       // 重启串口
        success_ = false;
        return 0;
    }
    else
	cout << "串口读取成功" << endl;
    if (read_buffer[0] == data->head && read_buffer[12] == data->end)    // 头尾帧读取成功
    {
        // NOTICE("Get stm32 data sucessed!!!", 3);
        mode = int(read_buffer[1]);

	// buff_offset_x = 100.0;
        // buff_offset_y = 100.0;
 	col = int(read_buffer[2]);
 	bulletspeed = char(read_buffer[3]);
        pitch_data_.c[0] = read_buffer[4];
	pitch_data_.c[1] = read_buffer[5];
	pitch_data_.c[2] = read_buffer[6];
	pitch_data_.c[3] = read_buffer[7];
	yaw_data_.c[0] = read_buffer[8];
	yaw_data_.c[1] = read_buffer[9];
	yaw_data_.c[2] = read_buffer[10];
	yaw_data_.c[3] = read_buffer[11];

	buff_offset_x = pitch_data_.f;
        buff_offset_y = yaw_data_.f;

	// cout << "机器人mode: " << mode << endl;
	// cout << "机器人buff_offset_x: " << buff_offset_x << endl;	
	// cout << "机器人buff_offset_y: " << buff_offset_y << endl;
	// cout << "机器人bulletspeed: " << bulletspeed << endl;
	// cout << "机器人col: " << col << endl;
        success_ = true;
	read_succeed = true;
	
        return 1;
    }
    else
    {
	cout << "头尾失败" << endl;
	read_succeed = false;
    }

    success_ = false;
    return 0;
#endif // LINUX

#ifdef WINDOWS
    unsigned char read_buffer[7];       // 缓冲区存储接收到的数据
    DWORD wCount = 6;       // 成功读取的数据字节数
    BOOL bReadStat;
    /*
        * 参数1: 串口句柄
        * 参数2: 数据首地址
        * 参数3: 要读取的数据最大字节数
        * 参数4: DWORD*，用来接收返回成功读取的数据字节数
        * 参数5: NULL(同步发送)    |    OVERLAPPED*(异步发送)
    */
    bReadStat = ReadFile(hcom, read_buffer, wCount, &wCount, NULL);
    if (!bReadStat)
    {
        cout << "read failed..." << endl;
        success_ = false;
        return 0;
    }
    else
        cout << "read successfully..." << endl;

    if (read_buffer[0] == data->head && read_buffer[5] == data->end) 
    {
        mode = bool(read_buffer[1]);

        buff_offset_x = char(read_buffer[2]);
        buff_offset_y = char(read_buffer[3]);
        bulletspeed = char(read_buffer[4]);
        // cout << "x: " << buff_offset_x << " y:" << buff_offset_y << endl;
        // gimbal_data = float(short((read_buffer[4]<<8) | read_buffer[3]))/100.0f;
        // cout << gimbal_data<< endl;
        success_ = true;
        return 1;
    }

    success_ = false;
    return 0;
#endif // WINDOWS

}

// 重启串口
void SerialPort::restart_serial(int& fd, int& last_fd)
{
#ifdef LINUX
    // cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1 && last_fd != -1)
    {
        printf("open_port wait to open %s ... \n", file_name_);
        last_fd = fd;
        return;
    }
    else if (fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL, 0);
        printf("port is open %s... \n", file_name_);
        last_fd = fd;
    }
    else
    {
        last_fd = fd;
        return;
    }

    struct termios port_set;
    if (buadrate_ == 0)
    {
        // B115200 115200波特
        cfsetispeed(&port_set, B115200);   // 设置输入波特率
        cfsetospeed(&port_set, B115200);   // 设置输出波特率
    }
    else if (buadrate_ == 1) 
    {
        // B921600 921600波特
        cfsetispeed(&port_set, B921600);   // 设置输入波特率
        cfsetospeed(&port_set, B921600);   // 设置输出波特率
    }

    port_set.c_cflag &= ~PARENB;         //无奇偶校验
    port_set.c_cflag &= ~CSTOPB;         //停止位:1bit
    port_set.c_cflag &= ~CSIZE;          //清除数据位掩码
    port_set.c_cflag |=  CS8;

    /*
        * 参数1: fd为打开的终端文件描述符
        * 参数2: optional_actions用于控制修改起作用的时间
        *        TCSANOW：不等数据传输完毕就立即改变属性
        *        TCSADRAIN：等待所有数据传输结束才改变属性
        *        TCSAFLUSH：等待所有数据传输结束,清空输入输出缓冲区才改变属性
        * 参数3: 结构体termios_p中保存了要修改的参数
    */
    tcsetattr(fd, TCSANOW, &port_set);             // 将设置应用到串口
#endif // LINUX

#ifdef WINDOWS
    CloseHandle(hcom);
    hcom = CreateFileA(file_name_, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hcom == (HANDLE)-1)
    {
        cout << "serialport open failed..." << endl;
        return;
    }
    else
        cout << "serialport open successful..." << endl;

    if (!SetupComm(hcom, 1024, 1024))
    {
        cout << "set the buffer failed..." << endl;
        return;
    }

    DCB dcb;
    GetCommState(hcom, &dcb);
    dcb.BaudRate = buadrate_;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hcom, &dcb);

    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = 1000;
    timeouts.ReadTotalTimeoutConstant = 5000;
    timeouts.ReadTotalTimeoutMultiplier = 500;
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 500;
    SetCommTimeouts(hcom, &timeouts);

    PurgeComm(hcom, PURGE_TXCLEAR | PURGE_RXCLEAR);
#endif // WINDOWS

}

void SerialPort::close_port(int& fd) {
#ifdef LINUX
    close(fd);
#endif // LINUX


#ifdef WINDOWS
    CloseHandle(hcom);
#endif // WINDOWS

}

void serial_transmit_data::get_xy_data(int16_t x, int16_t y, int light_len) {
    size = 7;
    raw_data[0] = head;
    raw_data[size - 1] = end;
    raw_data[1] = x >> 8;
    raw_data[2] = x & 0xff;
    raw_data[3] = y >> 8;
    raw_data[4] = y & 0xff;
    raw_data[5] = light_len;
}
