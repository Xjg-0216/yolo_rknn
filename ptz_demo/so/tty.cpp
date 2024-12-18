#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>  
#include     <sys/stat.h>   
#include "string.h"
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <sys/stat.h>
#include <termios.h>

#define 	FALSE  -1
#define 	TRUE   0

int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,  19200,  9600, 4800, 2400, 1200,  300, };

#define BAUDRATE B115200 		///Baud rate : 115200
#define DEVICE "/dev/ttyUSB0"	//设置你的端口号
struct termios stNew;
struct termios stOld;

int OpenDev(char *Dev)
{
	int fd = open(Dev, O_RDWR | O_NOCTTY);         //| O_NOCTTY | O_NDELAY    
	if (-1 == fd)   
	{           
		perror("Can't Open Serial Port");
		return -1;      
	}   
	else    
		return fd;
}

void set_speed(int fd, int speed)
{
	int   i; 
	int   status; 
	struct termios   Opt;
	tcgetattr(fd, &Opt); 
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
	{ 
		if  (speed == name_arr[i]) {     
			tcflush(fd, TCIOFLUSH);     
			cfsetispeed(&Opt, speed_arr[i]);  
			cfsetospeed(&Opt, speed_arr[i]);   
			status = tcsetattr(fd, TCSANOW, &Opt);  
			if  (status != 0) {        
				perror("tcsetattr fd1");  
				return;     
			}    
			tcflush(fd,TCIOFLUSH);   
		}  
	}
}

/*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{ 
	struct termios options; 
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_oflag  &= ~OPOST;   /*Output*/
	if  ( tcgetattr( fd,&options)  !=  0) { 
		perror("SetupSerial 1");     
		return(FALSE);  
	}
	options.c_cflag &= ~CSIZE; 
	switch (databits) /*设置数据位数*/
	{   
	case 7:     
		options.c_cflag |= CS7; 
		break;
	case 8:     
		options.c_cflag |= CS8;
		break;   
	default:    
		fprintf(stderr,"Unsupported data size/n"); return (FALSE);  
	}
	switch (parity) 
	{   
	case 'n':
	case 'N':    
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
		break;  
	case 'o':   
	case 'O':     
		options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/  
		options.c_iflag |= INPCK;             /* Disnable parity checking */ 
		break;  
	case 'e':  
	case 'E':   
		options.c_cflag |= PARENB;     /* Enable parity */    
		options.c_cflag &= ~PARODD;   /* 转换为偶效验*/     
		options.c_iflag |= INPCK;       /* Disnable parity checking */
		break;
	case 'S': 
	case 's':  /*as no parity*/   
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;break;  
	default:   
		fprintf(stderr,"Unsupported parity/n");    
		return (FALSE);  
	}  
	/* 设置停止位*/  
	switch (stopbits)
	{   
	case 1:    
		options.c_cflag &= ~CSTOPB;  
		break;  
	case 2:    
		options.c_cflag |= CSTOPB;  
		break;
	default:    
		fprintf(stderr,"Unsupported stop bits/n");  
		return (FALSE); 
	} 
	/* Set input parity option */ 
	if (parity != 'n')   
		options.c_iflag |= INPCK; 
		
	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 150; 	/* 设置超时15 seconds*/   
	options.c_cc[VMIN] = 0; 	/* Update the options and do it NOW */
	if (tcsetattr(fd,TCSANOW,&options) != 0)   
	{ 
		perror("SetupSerial 3");   
		return (FALSE);  
	} 
	return (TRUE);  
}

//Open Port & Set Port
int SerialInit()
{
    int nFd = open(DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == nFd)
    {
        perror("Open Serial Port Error!\n");
        return -1;
    }

    if((fcntl(nFd, F_SETFL, 0))<0)
    {
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }

    if(tcgetattr(nFd, &stOld) != 0)
    {
        perror("tcgetattr error!\n");
        return -1;
    }

    stNew = stOld;
    cfmakeraw(&stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理

    //set speed
    cfsetispeed(&stNew, BAUDRATE);//115200
    cfsetospeed(&stNew, BAUDRATE);

    //set databits
    stNew.c_cflag |= (CLOCAL|CREAD);
    stNew.c_cflag &= ~CSIZE;
    stNew.c_cflag |= CS8;

    //set parity
    stNew.c_cflag &= ~PARENB;
    stNew.c_iflag &= ~INPCK;

    //set stopbits
    stNew.c_cflag &= ~CSTOPB;
    stNew.c_cc[VTIME]=0;    //指定所要读取字符的最小数量
    stNew.c_cc[VMIN]=0; 	//指定读取第一个字符的等待时间，时间的单位为n*100ms

    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    tcflush(nFd,TCIFLUSH);  //清空终端未完毕的输入/输出请求及数据。
    //options.c_cc[VTIME] = 150; 	/* 设置超时15 seconds*/   
    //options.c_cc[VMIN] = 0; 	/* Update the options and do it NOW */
	
    if(tcsetattr(nFd,TCSANOW,  &stNew) != 0 )
    {
        perror("tcsetattr Error!\n");
        return -1;
    }
    return nFd;
}

