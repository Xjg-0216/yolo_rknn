#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* 文件控制定义 */
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;

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
        uint8_t value:5;    //命令码[0:未定义 1:陀螺校准 2:启动云台 3:停止云台 4:手动操控 5:指点平移 6:目标追踪] 
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
        uint8_t wk_mode:2;  //工作模式 0:跟随 1:锁定 2:FPVM
        uint8_t op_type:2;  //操控方式 0:角度 1:速率
        int16_t op_value;   //操控值
    } gbc[3];               //这三组分别是滚转轴，俯仰轴，指向轴
    struct
    {
        uint8_t :7;
        uint8_t valid:1;    //飞控数据有效标志 [0:无效 1:有效]
        int16_t angle[3];   //飞控欧拉角 0.01deg 机体坐标系
        int16_t accel[3];   //飞控加速度 0.01m/s^2 坐标系:北东上(NEU)
    } uav;                  //飞控数据
    struct
    {
        uint32_t vert_fov1x:7;      //相机1倍时的垂直视场角
        uint32_t zoom_value:24;     //相机放大倍数
        uint32_t target_valid:1;    //目标偏移角度有效标志
        float target_angle[2];      //目标偏移角度
    } cam;
    uint8_t crc[2]; //CRC16校验
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
    int16_t mtr_angl[3]; //电机角度 0.01 deg [-180,+180)
    uint8_t crc[2];     //CRC16校验
};
#pragma pack(pop) // 设置新对齐方式为原对齐方式

// 云台控制角度值结构，写入的
struct crtl_signal_t
{
    int16_t pitch_rate_signal;  //0.1deg/s
    int16_t yaw_rate_signal;    //0.1deg/s
    int16_t pitch_angle_signal;  //0.1deg [-180 180]°
    int16_t yaw_angle_signal;    //0.1deg [-180 180]°
};

// 载台数据
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

uint16_t CalculateCrc16(volatile uint8_t *ptr,uint8_t len)    
{   
	uint16_t crc;   
	uint8_t da;   
	uint16_t crc_ta[16]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7, 0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef, };   
	crc=0;   
	while(len--!=0)   
	{   
		da=crc>>12;
		crc<<=4;
		crc^=crc_ta[da^(*ptr>>4)];
		da=crc>>12;
		crc<<=4;
		crc^=crc_ta[da^(*ptr&0x0F)];
		ptr++;
	}
	return(crc);
}

//解析云台数据函数
int decode_gbc_data(uint8_t* input_buff, gbc_info_t* gbc_info)
{
    gbc_to_ctrl_t* p_gbc =(gbc_to_ctrl_t*)input_buff;

    if(NULL == input_buff || NULL == gbc_info) return -1;

    if(0!=CalculateCrc16(input_buff,sizeof(gbc_to_ctrl_t))) return -1;

    gbc_info->fw_ver = p_gbc->fw_ver;   //固件版本
    gbc_info->hw_err = p_gbc->hw_err;   //硬件错误值
    gbc_info->inv_flag=p_gbc->inv_flag; //倒置标志[0:正常 1:倒置]
    gbc_info->gbc_stat=p_gbc->gbc_stat; //云台状态[0:未定义 1:初始化 2:停止状态 3:保护状态 4:手动操控 5:指点平移 6:目标跟踪]
    
    gbc_info->cam_rate[0]=p_gbc->cam_rate[0];
    gbc_info->cam_rate[1]=p_gbc->cam_rate[1];
    gbc_info->cam_rate[2]=p_gbc->cam_rate[2];

    gbc_info->cam_angl[0]=p_gbc->cam_angl[0];
    gbc_info->cam_angl[1]=p_gbc->cam_angl[1];
    gbc_info->cam_angl[2]=p_gbc->cam_angl[2];

    gbc_info->mtr_angl[0]=p_gbc->mtr_angl[0];
    gbc_info->mtr_angl[1]=p_gbc->mtr_angl[1];
    gbc_info->mtr_angl[2]=p_gbc->mtr_angl[2];
    return 1;
}

class GimbalControl
{
private:
    uint8_t gbc_buff[128];  //云台姿态数据缓存


public:
    GimbalControl() 
    {
        g_usartFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY)


    }


    // 跟随模式，云台指向跟随载台运动
    int setCtrlFollowMode(const carrier_info_t &carrier_info, uint8_t *output_buff, const crtl_signal_t &ctrl_signal)
    {
        ctrl_to_gbc_t *ctrl_to_gbc = (ctrl_to_gbc_t *)output_buff;

        if (output_buff == NULL)
            return -1;

        ctrl_to_gbc->sync[0] = 0xA9;
        ctrl_to_gbc->sync[1] = 0x5B;

        ctrl_to_gbc->cmd.trig++;
        ctrl_to_gbc->cmd.value = 4;
        ctrl_to_gbc->aux.fl_sens = 0;

        // 设置云台控制参数
        ctrl_to_gbc->gbc[0].go_zero = 0;
        ctrl_to_gbc->gbc[0].wk_mode = 2;
        ctrl_to_gbc->gbc[0].op_type = 0;
        ctrl_to_gbc->gbc[0].op_value = ctrl_signal.yaw_angle_signal;

        ctrl_to_gbc->gbc[1].go_zero = 0;
        ctrl_to_gbc->gbc[1].wk_mode = 2;
        ctrl_to_gbc->gbc[1].op_type = 0;
        ctrl_to_gbc->gbc[1].op_value = ctrl_signal.pitch_angle_signal;

        ctrl_to_gbc->gbc[2].go_zero = 0;
        ctrl_to_gbc->gbc[2].wk_mode = 2;
        ctrl_to_gbc->gbc[2].op_type = 0;
        ctrl_to_gbc->gbc[2].op_value = 0;

        // 设置飞控数据
        ctrl_to_gbc->uav.valid = carrier_info.carrier_status;
        ctrl_to_gbc->uav.angle[0] = carrier_info.carrier_roll * 100;
        ctrl_to_gbc->uav.angle[1] = carrier_info.carrier_pitch * 100;
        ctrl_to_gbc->uav.angle[2] = carrier_info.carrier_psi * 100;
        ctrl_to_gbc->uav.accel[0] = carrier_info.carrier_aacn * 100;
        ctrl_to_gbc->uav.accel[1] = carrier_info.carrier_aace * 100;
        ctrl_to_gbc->uav.accel[2] = carrier_info.carrier_aacu * 100;

        // CRC校验
        uint16_t crc_value = CalculateCrc16(output_buff, sizeof(ctrl_to_gbc_t) - 2);
        ctrl_to_gbc->crc[0] = (uint8_t)(crc_value >> 8);
        ctrl_to_gbc->crc[1] = (uint8_t)(crc_value & 0xFF);

        return sizeof(ctrl_to_gbc_t);  // 返回包大小
    }

    // 解析云台反馈数据
    int decodeGbcData(uint8_t *input_buff, gbc_info_t &gbc_info)
    {
        gbc_to_ctrl_t *gbc_to_ctrl = (gbc_to_ctrl_t *)input_buff;
        if (gbc_to_ctrl->sync[0] == 0xB5 && gbc_to_ctrl->sync[1] == 0x9A)
        {
            gbc_info.fw_ver = gbc_to_ctrl->fw_ver;
            gbc_info.hw_err = gbc_to_ctrl->hw_err;
            gbc_info.inv_flag = gbc_to_ctrl->inv_flag;
            gbc_info.gbc_stat = gbc_to_ctrl->gbc_stat;
            memcpy(gbc_info.cam_rate, gbc_to_ctrl->cam_rate, sizeof(gbc_info.cam_rate));
            memcpy(gbc_info.cam_angl, gbc_to_ctrl->cam_angl, sizeof(gbc_info.cam_angl));
            memcpy(gbc_info.mtr_angl, gbc_to_ctrl->mtr_angl, sizeof(gbc_info.mtr_angl));

            return 1;  // 成功解析数据
        }
        return 0;  // 解析失败
    }
};
