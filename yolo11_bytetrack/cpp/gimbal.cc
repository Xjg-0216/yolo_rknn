#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* 文件控制定义 */
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include "gimbal.h"

using namespace std;

GimbalController::GimbalController() {
    user_carrier = {0};   
    ctrl_signal = {0};
    memset(gbc_buff, 0, sizeof(gbc_buff)); 
}

    // 串口初始化
int GimbalController::init_serial()  {
    g_usartFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_usartFd == -1){// 打开端口失败
        perror("open_port: Unable to open /dev/ttyUSB0");
        printf("ttyUSB0 err.\n");
        return -1;
    }
    else{
        printf("ttyUSB0 succeed.\n");
    }
    struct termios options={0};
    cfmakeraw(&options);
    options.c_cflag |= CREAD;
    cfsetspeed(&options, B115200);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_iflag &= ~INPCK;
    options.c_cflag &= ~CSTOPB;
    options.c_cc[VTIME]= 0;
    options.c_cc[VMIN] = 0;
    tcsetattr(g_usartFd, TCSANOW, &options); 
    return 0;
}

// 计算 CRC16 校验
uint16_t GimbalController::calculate_crc16(volatile uint8_t* ptr, uint8_t len) {
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

// 控制云台的跟随模式
int GimbalController::set_ctrl_follow_mode(carrier_info_t carrier_info, uint8_t* output_buff,crtl_signal_t ctrl_signal) {


    ctrl_to_gbc_t* ctrl_to_gbc = (ctrl_to_gbc_t*)output_buff;
    if (output_buff == NULL) return -1;

    ctrl_to_gbc->sync[0] = 0xA9; 
    ctrl_to_gbc->sync[1] = 0x5B; 

    ctrl_to_gbc->cmd.trig++;
    ctrl_to_gbc->cmd.value = 4;
    ctrl_to_gbc->aux.fl_sens = 0;
    
    //gbc[0-2],to 0：滚转轴，1：俯仰轴 2：指向轴，机体坐标系。
    ctrl_to_gbc->gbc[0].go_zero =0;
    ctrl_to_gbc->gbc[0].wk_mode =2;//工作模式 0:跟随 1:锁定 2:FPVM
    ctrl_to_gbc->gbc[0].op_type =0;//操控方式 0:角度 1:速率
    ctrl_to_gbc->gbc[0].op_value= ctrl_signal.yaw_angle_signal;


    ctrl_to_gbc->gbc[1].go_zero = 0; //数据变化触发云台回中
    ctrl_to_gbc->gbc[1].wk_mode = 2; //工作模式 0:跟随 1:锁定 2:FPVM
    ctrl_to_gbc->gbc[1].op_type = 0; //操控方式 0:角度 1:速率
    ctrl_to_gbc->gbc[1].op_value= ctrl_signal.pitch_angle_signal;
    
    ctrl_to_gbc->gbc[2].go_zero = 0; //数据变化触发云台回中
    ctrl_to_gbc->gbc[2].wk_mode = 2; //工作模式 0:跟随 1:锁定 2:FPVM
    ctrl_to_gbc->gbc[2].op_type = 0; //操控方式 0:角度 1:速率
    ctrl_to_gbc->gbc[2].op_value= 0;//\ctrl_signal.yaw_angle_signal;


    ctrl_to_gbc->uav.valid   = carrier_info.carrier_status;
    ctrl_to_gbc->uav.angle[0]= carrier_info.carrier_roll*100;
    ctrl_to_gbc->uav.angle[1]= carrier_info.carrier_pitch*100;
    ctrl_to_gbc->uav.angle[2]= carrier_info.carrier_psi*100;
    ctrl_to_gbc->uav.accel[0]= carrier_info.carrier_aacn*100;
    ctrl_to_gbc->uav.accel[1]= carrier_info.carrier_aace*100;
    ctrl_to_gbc->uav.accel[2]= carrier_info.carrier_aacu*100;


    ctrl_to_gbc->cam.vert_fov1x = VERTICAL_VIEW_ANGEL_DEFAULT;
    ctrl_to_gbc->cam.zoom_value = ZOOM_VALUE_DEFAULT;
    ctrl_to_gbc->cam.target_angle[0] =0;
    ctrl_to_gbc->cam.target_angle[1] =0;
    ctrl_to_gbc->cam.target_valid = 0;

    int16_t crc;
    crc = calculate_crc16(output_buff,sizeof(ctrl_to_gbc_t)-2);
    ctrl_to_gbc->crc[0] = crc>>8;
    ctrl_to_gbc->crc[1] = crc&0xFF;

    return 0;
}

//解析云台数据函数
int GimbalController::decode_gbc_data(uint8_t* input_buff, gbc_info_t* gbc_info)
{
    gbc_to_ctrl_t* p_gbc =(gbc_to_ctrl_t*)input_buff;

    if(NULL == input_buff || NULL == gbc_info) return -1;

    if(0!=calculate_crc16(input_buff,sizeof(gbc_to_ctrl_t))) return -1;

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

// 向云台发送控制
int GimbalController::send_gimbal_control_command(int pitch, int yaw) {
    //角度控制， 写入ctrl_signal
    ctrl_signal.pitch_angle_signal = pitch * 100;
    ctrl_signal.yaw_angle_signal = yaw * 100;
    int ret = set_ctrl_follow_mode(user_carrier, ctrl_buff, ctrl_signal);
    if (ret != -1)
    {
        write(g_usartFd, ctrl_buff, sizeof(ctrl_to_gbc_t));
        return 0;
    }
    else{
        return -1;
    }

}

// 读取云台的状态数据
int GimbalController::read_gimbal_status(gbc_info_t* gbc_info) {

    int ret = read(g_usartFd, gbc_buff, sizeof(gbc_to_ctrl_t));
    if (ret != -1) {
        //解析云台数据
        ret = decode_gbc_data(gbc_buff, gbc_info);
        if(ret==1)
        {
            // printf("固件版本=%d, hw_err=%d 倒置标志=%d 云台状态=%d\n", gbc_info.fw_ver, gbc_info.hw_err, gbc_info.inv_flag, gbc_info.gbc_stat);
            // printf("roll滚转轴角=%0.1f 俯仰=%0.1f  yaw指向=%0.1f\n", gbc_info.cam_angl[0]*0.01, gbc_info.cam_angl[1]*0.01, gbc_info.cam_angl[2]*0.01);
            return 0;
        }
    }
    return -1;
}

GimbalController::~GimbalController() {
    // 如果串口文件描述符已经打开，关闭它
    if (g_usartFd != -1) {
        close(g_usartFd);
        printf("串口已关闭。\n");
    }
}


