#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>

#define DEG2RAD                     0.0174532925f
#define RAD2DEG                     57.29577957855f
#define VERTICAL_VIEW_ANGEL_DEFAULT 90
#define ZOOM_VALUE_DEFAULT          1000

#pragma pack(push, 1) // 保存当前对齐状态，并设置新的对齐为1字节

// 云台控制发送结构体
struct ctrl_to_gbc_t {
    uint8_t sync[2];        //同步头:0xA9 0x5B
    struct {
        uint8_t trig:3;     //命令触发计数变化则会重新执行相同命令
        uint8_t value:4;    //命令码[0:未定义 1:陀螺校准 2:启动云台 3:停止云台 4:手动操控 5:指点平移 6:目标追踪] 这些命令都是发一次，收到云台返回后，清零即可
    } cmd;
    struct {
        uint8_t :3;
        int8_t fl_sens:5;   //跟随灵敏度[-16~15]
    } aux;
    struct {
        uint8_t :3;
        uint8_t go_zero:1;  //数据变化触发云台回中
        uint8_t wk_mode:2;  //工作模式 0:跟随 1:锁定 2:FPVM(无法每轴单独设置,三轴必须同时设置为FPVM)
        uint8_t op_type:2;  //操控方式
        int16_t op_value;   //角度或速率操控方式
    } gbc[3];               //这些数据只有手动操作模式下才会起作用，0：滚转轴，1：俯仰轴 2：指向轴
    struct{
        uint8_t :7;
        uint8_t valid:1;    //飞控数据有效标志 [0:无效 1:有效]
        int16_t angle[3];   //飞控欧拉角 0.01deg 机体坐标系。0：滚转角[-180,+180) 1：俯仰角[-90,90] 2：指向角[-180,+180)
        int16_t accel[3];   //飞控加速度 0.01m/s^2 坐标系:北东上(NEU)，北东天为正。0：北向加速度，1：东向加速度，2：天向加速度
    } uav;
    struct {
        uint32_t vert_fov1x:7;      //相机1倍时的垂直视场角
        uint32_t zoom_value:24;     //相机放大倍数
        uint32_t target_valid:1;    //目标偏移角度有效标志
        float target_angle[2];      //目标偏移角度
    } cam;
    uint8_t crc[2]; //CRC16校验
};

// 云台控制接收结构体
struct gbc_to_ctrl_t {
    uint8_t sync[2];    //同步头:0xB5 0x9A
    uint8_t fw_ver;     //固件版本
    uint8_t hw_err;     //硬件错误
    uint8_t inv_flag:1; //倒置标志
    uint8_t gbc_stat:3; //云台状态
    uint8_t :4;
    struct {
        uint8_t stat:3; //命令状态
        uint8_t valu:5; //命令码回传
    } cmd;
    int16_t cam_rate[3]; //相机转速
    int16_t cam_angl[3]; //相机角度
    int16_t mtr_angl[3]; //电机角度
    uint8_t crc[2];     //CRC16校验
};

#pragma pack(pop) // 恢复原对齐方式

// 云台控制信号, 写入的
struct crtl_signal_t {
    int16_t pitch_rate_signal;
    int16_t yaw_rate_signal;
    int16_t pitch_angle_signal;
    int16_t yaw_angle_signal;
};

// 载台信息
struct carrier_info_t {
    uint8_t carrier_status;
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

// 实时解析姿态结构体
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

class GimbalController {
private:
    int g_usartFd = -1; // 串口句柄，默认值为-1
    uint8_t gbc_buff[128];  // 云台数据缓存
    uint8_t ctrl_buff[128];  //要写入云台，控制角度的数据缓存
    carrier_info_t user_carrier;  // 载台数据
    crtl_signal_t ctrl_signal;    // 云台控制信号
    // gbc_info_t     gbc_info;         //实时云台姿态信息结构
    
public:
    GimbalController(); 
    ~GimbalController();

    int init_serial(); 

    uint16_t calculate_crc16(volatile uint8_t* ptr, uint8_t len); 

    int set_ctrl_follow_mode(uint8_t* output_buff); 

    int decode_gbc_data(uint8_t* input_buff, gbc_info_t* gbc_info); 

    int send_gimbal_control_command(int pitch, int yaw); 

    int read_gimbal_status(gbc_info_t* gbc_info); 
};

#endif // GIMBAL_CONTROLLER_H
