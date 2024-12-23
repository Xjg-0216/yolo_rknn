#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>  // for sleep
#include "gimbal.h"
using namespace std;

int main(int argc, char* argv[]) {




    // 将命令行参数转换为整数
    int num1 = std::atoi(argv[1]); // argv[1] 是第一个用户输入参数
    int num2 = std::atoi(argv[2]); // argv[2] 是第二个用户输入参数

    // 创建 GimbalController 对象
    GimbalController gimbalController;

    // 初始化串口
    if (gimbalController.init_serial() == -1) {
        cout << "串口初始化失败！" << endl;
        return -1;
    }


    gbc_info_t gbc_info={0};
    for (;;)
    {
        int pitch = num1;
        int yaw = -num2; // 此代码中Pitch方向与实际方向相反
        printf("pitch= %d; yaw=%d\n", num1, num2);
        if (gimbalController.send_gimbal_control_command(pitch, yaw) == -1) {
            cout << "发送控制命令失败！" << endl;
            return -1;
        }
        if (gimbalController.read_gimbal_status(&gbc_info) == 0) {
        // 打印云台状态信息
        printf("固件版本=%d, hw_err=%d 倒置标志=%d 云台状态=%d\n", 
                gbc_info.fw_ver, gbc_info.hw_err, gbc_info.inv_flag, gbc_info.gbc_stat);
        printf("pitch=%0.1f yaw=%0.1f\n", 
                gbc_info.mtr_angl[0]*0.01, gbc_info.mtr_angl[1]*0.01);
        } else {
            cout << "读取云台状态失败！" << endl;
        }
        sleep(1);
        
    }
    return 0;
}
