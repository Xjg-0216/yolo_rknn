/*
向量法：直接计算视线偏角
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

// 像素点方向向量
Eigen::Vector3d PixToDirection(double box_x, double box_y, double img_width, double img_height, double fx, double fy) {
    double dx = (box_x - img_width / 2) / fx;
    double dy = (box_y - img_height / 2) / fy;
    return Eigen::Vector3d(dx, dy, 1.0).normalized();
}

// 欧拉角转旋转矩阵
Eigen::Matrix3d EulerToRotation(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(roll * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    return R;
}

// 计算目标偏角
std::pair<double, double> CalculateViewAngle(Eigen::Vector3d pix_dir, double t_az_pitch, double roll, double pitch, double yaw) {
    // Step 1: 机身-相机旋转矩阵
    Eigen::Matrix3d R_BODY_CAM = EulerToRotation(0, t_az_pitch, 0);

    // Step 2: NED-机身旋转矩阵
    Eigen::Matrix3d R_NED_BODY = EulerToRotation(roll, pitch, yaw);

    // Step 3: 转换像素方向到 NED 坐标系
    Eigen::Vector3d ned_dir = R_NED_BODY * R_BODY_CAM * pix_dir;

    // Step 4: 计算与NED视线偏角
    double tx = std::atan2(ned_dir.y(), ned_dir.z()); // 水平偏角
    double ty = std::atan2(-ned_dir.x(), ned_dir.z()); // 垂直偏角

    return {tx, ty};
}

int main() {
    // 参数
    double img_width = 640, img_height = 480;
    double fx = 640, fy = 480;
    double box_x = 400, box_y = 240; // 像素坐标
    double t_az_pitch = -15;         // 相机安装偏角
    double roll = 5, pitch = -10, yaw = 30; // 无人机姿态

    // 像素方向
    Eigen::Vector3d pix_dir = PixToDirection(box_x, box_y, img_width, img_height, fx, fy);

    // 计算视线偏角
    std::pair<double, double> result = CalculateViewAngle(pix_dir, t_az_pitch, roll, pitch, yaw);

    // 输出结果
    std::cout << "视线水平偏角 (rad): " << result.first << std::endl;
    std::cout << "视线垂直偏角 (rad): " << result.second << std::endl;

    return 0;
}
