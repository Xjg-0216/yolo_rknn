#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>  // for sleep
#include "gimbal.h"
using namespace std;

int main() {
    // 创建 GimbalController 对象
    GimbalController gimbalController;

    // 初始化串口
    if (gimbalController.init_serial() == -1) {
        cout << "串口初始化失败！" << endl;
        return -1;
    }

    // 发送控制命令，设置云台角度
    int pitch = 100;  // 设定俯仰角度 [-120, 120]
    int yaw = 50;    // 设定偏航角度 [-60 60]
    if (gimbalController.send_gimbal_control_command(pitch, yaw) == -1) {
        cout << "发送控制命令失败！" << endl;
        return -1;
    }

    gbc_info_t gbc_info={0};

    // 读取云台状态 10 次
    for (int i = 0; i < 10; i++) {
        if (gimbalController.read_gimbal_status(&gbc_info) == 0) {
            // 打印云台状态信息
            printf("固件版本=%d, hw_err=%d 倒置标志=%d 云台状态=%d\n", 
                   gbc_info.fw_ver, gbc_info.hw_err, gbc_info.inv_flag, gbc_info.gbc_stat);
            printf("roll滚转轴角=%0.1f 俯仰=%0.1f  yaw指向=%0.1f\n", 
                   gbc_info.cam_angl[0]*0.01, gbc_info.cam_angl[1]*0.01, gbc_info.cam_angl[2]*0.01);
        } else {
            cout << "读取云台状态失败！" << endl;
        }
        sleep(1);  // 延时 1 秒
    }
    // if (gimbalController.read_gimbal_status(&gbc_info) == 0) {
    // // 打印云台状态信息
    //     printf("固件版本=%d, hw_err=%d 倒置标志=%d 云台状态=%d\n", 
    //             gbc_info.fw_ver, gbc_info.hw_err, gbc_info.inv_flag, gbc_info.gbc_stat);
    //     printf("roll滚转轴角=%0.1f 俯仰=%0.1f  yaw指向=%0.1f\n", 
    //             gbc_info.cam_angl[0]*0.01, gbc_info.cam_angl[1]*0.01, gbc_info.cam_angl[2]*0.01);
    //     } else {
    //         cout << "读取云台状态失败！" << endl;
    //     }
    return 0;
}
