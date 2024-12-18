#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>  
#include     <sys/stat.h>   
#include     "string.h"
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include	<unistd.h>
#define 	FALSE  -1
#define 	TRUE   0

//int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
//int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,  19200,  9600, 4800, 2400, 1200,  300, };

int OpenDev(char *Dev);
void set_speed(int fd, int speed);

/*@brief   设置串口数据位，停止位和效验位,fd类型  int  打开的串口文件句柄,*@param  databits 类型  int 数据位   取值 为 7 或者8,@param  stopbits 类型  int 停止位   取值为 1 或者2,
*@param  parity  类型  int  效验类型 取值为N,E,O,,S*/
int set_Parity(int fd,int databits,int stopbits,int parity);
int SerialInit();

