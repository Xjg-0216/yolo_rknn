#include <iostream>
#include <iomanip>

#include "uav_position.h"


// 哈弗辛公式计算两点间的距离（单位：公里）
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; // 地球半径，单位：公里

    // 将经纬度转换为弧度
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // 纬度和经度差值
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // 哈弗辛公式
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // 返回距离
    return R * c; // 返回公里
}

int main() {
    // 初始化无人机数据和图像数据
    VectorXd UavData(7); // lat, lon, height, yaw, pitch, roll
    UavData << 38.1368980408, 116.2943725586, 100, 0, 0, 0, 0;
    // 
    Vector3d PictureData(533.5, 371.5, 1);

    // 计算目标位置
    Vector2d target_position = UavPositioning::PixWorld(UavData, PictureData);

    // std::cout << "Target Position: Latitude = " << target_position(0) << ", Longitude = " << target_position(1) << std::endl;
    // 设置输出格式为固定小数点，并保留 8 位小数
    std::cout << std::fixed << std::setprecision(8)
              << "Target Position: Latitude = " << target_position(0)
              << ", Longitude = " << target_position(1) << std::endl;

    // 计算目标位置与初始位置之间的绝对距离
    double distance = haversine(UavData(0), UavData(1), target_position(0), target_position(1));

    // 输出距离（单位：公里）
    std::cout << "Distance between UAV and target: " << distance << " kilometers" << std::endl;

    return 0;
}
