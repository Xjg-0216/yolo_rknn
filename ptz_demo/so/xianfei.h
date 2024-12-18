#include <iostream>
#include <time.h>
#include <stdio.h>      //标准输入输出定义
#include <stdlib.h>     
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>  
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <typeinfo> 

using namespace std;

#define LOBYTE(w)           ((uint8_t)(((uint16_t)(w)) & 0xff))
#define HIBYTE(w)           ((uint8_t)((((uint16_t)(w)) >> 8) & 0xff))

#define MAKEWORD(a, b)      ((uint16_t)(((uint8_t)(((uint16_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint16_t)(b)) & 0xff))) << 8))
#define MAKELONG(a, b)		((uint32_t)(((uint16_t)(a)) | ((uint32_t)((uint16_t)(b))) << 16))

#define DEG2RAD                     0.0174532925f
#define RAD2DEG                     57.29577957855f
#define VERTICAL_VIEW_ANGEL_DEFAULT 90
#define ZOOM_VALUE_DEFAULT          1000


#pragma pack(push, 1) // 保存当前对齐状态，并设置新的对齐为1字节
struct ctrl_to_gbc_t
{
    uint8_t sync[2];        //同步头:0xA9 0x5B
	struct
	{
		uint8_t trig:3;     //命令触发计数变化则会重新执行相同命令
		uint8_t value:5;    //命令码[0:未定义 1:陀螺校准 2:启动云台 3:停止云台 4:手动操控 5:指点平移 6:目标追踪] 这些命令都是发一次，收到云台返回后，清零即可
	} cmd;
	struct
	{
		uint8_t :3;
		int8_t fl_sens:5;   //跟随灵敏度[-16~15]
	} aux;
	struct
	{
		uint8_t :3;
		uint8_t go_zero:1;  //数据变化触发云台回中
		uint8_t wk_mode:2;  //工作模式 0:跟随 1:锁定 2:FPVM(无法每轴单独设置,三轴必须同时设置为FPVM)
		uint8_t op_type:2;  //操控方式 0:角度 1:速率
		int16_t op_value;   //角度操控方式：实际角度=(op_value*0.01) deg
		                    //速率操控方式：实际速率=(op_value*0.1)/(倍数) deg/s
	} gbc[3];               //这些数据只有手动操作模式下才会起作用，0：滚转轴，1：俯仰轴 2：指向轴，机体坐标系。
	struct
	{
		uint8_t :7;
		uint8_t valid:1;    //飞控数据有效标志 [0:无效 1:有效]
		int16_t angle[3];   //飞控欧拉角 0.01deg 机体坐标系。0：滚转角[-180,+180) 1：俯仰角[-90,90] 2：指向角[-180,+180)
		int16_t accel[3];   //飞控加速度 0.01m/s^2 坐标系:北东上(NEU)，北东天为正。0：北向加速度，1：东向加速度，2：天向加速度
	} uav;					//以上是云台需要的飞控数据，不提供或提供错误的飞控欧拉角和加速度，会导致吊舱的俯仰角与滚转角在飞控做机动飞行时可能不准确。飞控数据无效时，数据可全填0。
	struct
	{
		uint32_t vert_fov1x:7;      //相机1倍时的垂直视场角[单位:deg]
		uint32_t zoom_value:24;     //相机放大倍数 [0.001倍]
		uint32_t target_valid:1;    //目标偏移角度有效标志 [0:无效 1:有效]
		float target_angle[2];      //目标偏移角度[坐标系:图像中心为原点右正下正 单位:deg].0：水平偏移角度，1：垂直偏移角度
	} cam;
	uint8_t crc[2];//CRC16校验
};
 
struct gbc_to_ctrl_t
{
    uint8_t sync[2];    //同步头:0xB5 0x9A
    uint8_t fw_ver;     //固件版本
    uint8_t hw_err;     //硬件错误
    uint8_t inv_flag:1; //倒置标志[0:正常 1:倒置]
    uint8_t gbc_stat:3; //云台状态[0:未定义 1:初始化 2:停止状态 3:保护状态 4:手动操控 5:指点平移 6:目标跟踪]
    uint8_t :4;
    struct
    {
        uint8_t stat:3; //命令状态[0:开始执行 1:执行成功 2:执行失败]
        uint8_t valu:5; //命令码回传
    } cmd;
    int16_t cam_rate[3]; //相机转速 0.1 deg/s
    int16_t cam_angl[3]; //相机角度 0.01 deg [-180,+180)
    int16_t mtr_angl[3]; //电机角度 0.01 deg [-180,+180),以上定义可参考word版协议文件。
    uint8_t crc[2];     //CRC16校验
};

#pragma pack(pop) // 设置新对齐方式为原对齐方式

//云台控制角度值结构，写入的
struct crtl_signal_t
{
    //角速率控制信号，适用于锁定，跟随，俯拍的指向控制，实际速率与倍数相关
    int16_t pitch_rate_signal;  //0.1deg/s
    int16_t yaw_rate_signal;    //0.1deg/s 
    
    //角度控制信号
    int16_t pitch_angle_signal;  //0.1deg [-180 180]°
    int16_t yaw_angle_signal;    //0.1deg [-180 180]°
};

//载台数据
struct carrier_info_t
{
    uint8_t carrier_status; //0:载台数据无效 1:载台数据有效

    float carrier_roll;      
    float carrier_pitch;
    float carrier_psi;

    float carrier_aacn;
    float carrier_aace;
    float carrier_aacu;

    double carrier_lon;
    double carrier_lat;
    float carrier_alt;
};


struct gbc_info_t
{
    uint8_t fw_ver;     //固件版本
    uint8_t hw_err;     //硬件错误
    uint8_t inv_flag:1; //倒置标志[0:正常 1:倒置]
    uint8_t gbc_stat:3; //云台状态[0:未定义 1:初始化 2:停止状态 3:保护状态 4:手动操控 5:指点平移 6:目标跟踪]
    int16_t cam_rate[3];//相机转速 0.1 deg/s
    int16_t cam_angl[3];//相机角度 0.01 deg [-180,+180)
    int16_t mtr_angl[3];//电机角度 0.01 deg [-180,+180)
};

