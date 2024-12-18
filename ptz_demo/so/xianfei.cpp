//example.cpp
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* 文件控制定义 */
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "xianfei.h"
#include "tty.h"

using namespace std;
int	g_comfd=-1;			//下位机com ,handle
float	g_pitch=0;                      //初始俯仰角度值

uint8_t         ctrl_buff[128];         //要写入云台，控制角度的数据缓存
uint8_t         gbc__buff[128];         //读出云台姿态数据缓存。。。

carrier_info_t  user_carrier={0};       //载台数据,初始值为0,表示无效
crtl_signal_t   ctrlSginal={0};         //控制云台角度的结构，初始值为0
gbc_info_t      gbcInfo={0};            //实时云台姿态信息结构

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

/*  跟随模式，云台指向跟随载台运动，输入载台姿态和北东天加速度，包数据地址，控制信号， 返回1数据可以发送给云台，其他数值数据不可使用*/
int set_ctrl_follow_mode(carrier_info_t carrier_info, uint8_t* output_buff,crtl_signal_t ctrl_signal)
{
    ctrl_to_gbc_t* ctrl_to_gbc = (ctrl_to_gbc_t* )output_buff;

    if(NULL == output_buff) return -1;

    ctrl_to_gbc->sync[0] = 0xA9; 
    ctrl_to_gbc->sync[1] = 0x5B; 

    ctrl_to_gbc->cmd.trig++;
    ctrl_to_gbc->cmd.value = 4;
    ctrl_to_gbc->aux.fl_sens = 0;
    
    //gbc[0-2],to 0：滚转轴，1：俯仰轴 2：指向轴，机体坐标系。
    ctrl_to_gbc->gbc[0].go_zero =ctrl_to_gbc->gbc[1].go_zero = 0; //数据变化触发云台回中
    ctrl_to_gbc->gbc[0].wk_mode =ctrl_to_gbc->gbc[1].wk_mode = 2; //工作模式 0:跟随 1:锁定 2:FPVM
    ctrl_to_gbc->gbc[0].op_type =ctrl_to_gbc->gbc[1].op_type = 0; //操控方式 0:角度 1:速率
    ctrl_to_gbc->gbc[0].op_value=ctrl_to_gbc->gbc[1].op_value= ctrl_signal.pitch_angle_signal;


    ctrl_to_gbc->gbc[2].go_zero = 0; //数据变化触发云台回中
    ctrl_to_gbc->gbc[2].wk_mode = 0; //工作模式 0:跟随 1:锁定 2:FPVM
    ctrl_to_gbc->gbc[2].op_type = 0; //操控方式 0:角度 1:速率
    ctrl_to_gbc->gbc[2].op_value= 0; //ctrl_signal.pitch_angle_signal;


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
    crc = CalculateCrc16(output_buff,sizeof(ctrl_to_gbc_t)-2);
    ctrl_to_gbc->crc[0] = crc>>8;
    ctrl_to_gbc->crc[1] = crc&0xFF;
    return 1;
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

    
extern "C"{

//初始化串口函数,这个函数在so 模式下，不能使用
int SerialInit2(void)
{
    g_comfd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_comfd == -1){// 打开端口失败
        perror("open_port: Unable to open /dev/ttyUSB0");
        printf("ttyUSB0 error.\n");
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
    tcsetattr(g_comfd, TCSANOW, &options);  
    return 0;
}

//获取云台姿态
void getGimbal(char* buf)
{
    //打开串口com................................................................................................................
    if(g_comfd==-1)
    {
        g_comfd = SerialInit();
        if(g_comfd==-1)
	{
		printf("open 串口 ttyUSB0 Error\n");
		return;
	}
	else
	{
		printf("打开 ttyUSB0 Success!\n");
        }
        //usleep(1000*200);							
    }
    
    //指点跟随角度控制        
    ctrlSginal.pitch_angle_signal=int(g_pitch) *100; //这个角度值要乘100,
    int ret = set_ctrl_follow_mode(user_carrier,ctrl_buff, ctrlSginal);
    if(-1!=ret)
    {                    
        ret=write(g_comfd, ctrl_buff, sizeof(ctrl_to_gbc_t));  //发送数据
        //printf("写入角度 ret=%d....\n", g_pitch);       
    }
        
    //接收数据
    ret=read(g_comfd, gbc__buff,sizeof(gbc_to_ctrl_t));      
    if(-1!=ret)
    {
        //解析云台数据
        ret = decode_gbc_data(gbc__buff, &gbcInfo);
        if(ret==1)
        {
           printf("\n\n\n");
           printf("固件版本=%d, hw_err=%d 倒置标志=%d 云台状态=%d\n", gbcInfo.fw_ver, gbcInfo.hw_err, gbcInfo.inv_flag, gbcInfo.gbc_stat);
           g_pitch =gbcInfo.cam_angl[1]*0.01;
           sprintf(buf,"%0.1f", g_pitch);
        }
    }
}

//设置角度值
void setAngle(float pitch)
{
    g_pitch =pitch;
    //打开串口com................................................................................................................
    if(g_comfd==-1)
    {
	g_comfd =SerialInit();
	if(g_comfd==-1)
	{
		printf("open 串口 ttyUSB0 Error\n");
		return;
	}
	else
	{
		printf("打开 ttyUSB0 Success!\n");		
        }
        
        //指点跟随角度控制        
        ctrlSginal.pitch_angle_signal=pitch *100; //这个角度值要乘100,
        int ret = set_ctrl_follow_mode(user_carrier,ctrl_buff, ctrlSginal);
        if(-1!=ret)
        {            
            write(g_comfd, ctrl_buff, sizeof(ctrl_to_gbc_t));  //发送数据
        }        
    }        
    else
    {
        //指点跟随角度控制        
        ctrlSginal.pitch_angle_signal=pitch *100; //这个角度值要乘100,
        int ret = set_ctrl_follow_mode(user_carrier,ctrl_buff, ctrlSginal);
        if(-1!=ret)
        {
            //发送数据
            write(g_comfd, ctrl_buff, sizeof(ctrl_to_gbc_t));  
        }
    }
    
    printf("setAngle sccess....\n");
    return;
}

//云台回中
void setCenter()//set pitch center
{
    //打开串口com................................................................................................................
    //write(g_comfd, (char*)pt, sizeof(pt));
    printf("setCenter sccess....\n");
}
}
