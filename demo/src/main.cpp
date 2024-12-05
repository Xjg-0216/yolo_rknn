#include <iostream>
#include <iomanip>

#include "uav_position.h"

int main() {
    // 初始化无人机数据和图像数据
    VectorXd UavData(7); // lat, lon, height, yaw, pitch, roll
    UavData << 38.1368980408, 116.2943725586, 88.4570007324, 1.827, -0.031, 0.157, 0;

    Vector3d PictureData(533.5, 371.5, 1);

    // 计算目标位置
    Vector2d target_position = UavPositioning::PixWorld(UavData, PictureData);

    // std::cout << "Target Position: Latitude = " << target_position(0) << ", Longitude = " << target_position(1) << std::endl;
    // 设置输出格式为固定小数点，并保留 8 位小数
    std::cout << std::fixed << std::setprecision(8)
              << "Target Position: Latitude = " << target_position(0)
              << ", Longitude = " << target_position(1) << std::endl;

    return 0;
}
