/*
矩阵法：逐步旋转矩阵计算目标偏角
*/


#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>


// 像素到视线方向的旋转矩阵
Eigen::Matrix3d PixToAngle(double box_x, double box_y, double img_width, double img_height, double fx, double fy) {
    double dx = (box_x - img_width / 2) / fx;
    double dy = (box_y - img_height / 2) / fy;
    double dz = 1.0;

    // 构造像素旋转矩阵
    Eigen::Matrix3d R_pix;
    R_pix = Eigen::AngleAxisd(std::atan2(dy, dz), Eigen::Vector3d::UnitY()) * // 
            Eigen::AngleAxisd(std::atan2(dx, dz), Eigen::Vector3d::UnitZ());  
    std::cout << "R_pix = \n" << R_pix << std::endl;

    return R_pix;
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
std::pair<double, double> CalculateViewAngle(double box_x, double box_y, double img_width, double img_height,
                                             double fx, double fy, double t_az_pitch,
                                             double roll, double pitch, double yaw) {
    // Step 1: 像素旋转矩阵
    Eigen::Matrix3d R_pix = PixToAngle(box_x, box_y, img_width, img_height, fx, fy);

    // Step 2: 机身-相机旋转矩阵
    Eigen::Matrix3d R_BODY_CAM = EulerToRotation(0, t_az_pitch, 0);
    std::cout << "R_BODY_CAM = \n" << R_BODY_CAM << std::endl;
    // Step 3: NED-机身旋转矩阵
    Eigen::Matrix3d R_NED_BODY = EulerToRotation(roll, pitch, yaw);
    std::cout << "R_NED_BODY = \n" << R_NED_BODY << std::endl;
    // Step 4: 总旋转矩阵
    Eigen::Matrix3d R_total = R_NED_BODY * R_BODY_CAM * R_pix;
    std::cout << "R_total = \n" << R_total << std::endl;
    // Step 5: 从旋转矩阵提取视线偏角
    // double tx = std::atan2(R_total(1, 0), R_total(0, 0));  // 水平偏角 (Yaw)
    // double ty = std::atan2(-R_total(2, 0), std::sqrt(R_total(2, 1) * R_total(2, 1) + R_total(2, 2) * R_total(2, 2)));  // 垂直偏角 (Pitch)

    double sy = std::sqrt(R_total(0, 0) * R_total(0, 0) + R_total(1, 0) * R_total(1, 0));
    bool singular = sy < 1e-6; // 判断是否为奇异情况

    double roll1, pitch1, yaw1;

    if (!singular) {
        roll1 = std::atan2(R_total(2, 1), R_total(2, 2));  // X轴旋转（Roll）
        pitch1 = std::atan2(-R_total(2, 0), sy);           // Y轴旋转（Pitch）
        yaw1 = std::atan2(R_total(1, 0), R_total(0, 0));   // Z轴旋转（Yaw）
    } else {
        roll1 = std::atan2(-R_total(1, 2), R_total(1, 1));  // 当出现奇异情况时，直接计算roll
        pitch1 = std::atan2(-R_total(2, 0), sy);            // pitch
        yaw1 = 0;                                           // yaw 没有定义
    }

    return {yaw1, pitch1};
    // return {tx, ty};
}

int main() {
    // 参数
    double img_width = 640, img_height = 480;
    double fx = 640, fy = 480;
    double box_x = 320, box_y = 240; // 像素坐标
    double t_az_pitch = -15;         // 相机安装偏角
    double roll = 5, pitch = -10, yaw = 30; // 无人机姿态

    // 计算视线偏角
    std::pair<double, double> result = CalculateViewAngle(box_x, box_y, img_width, img_height, fx, fy, t_az_pitch, roll, pitch, yaw);

    // 输出结果
    std::cout << "视线水平偏角 (rad): " << result.first << std::endl;
    std::cout << "视线垂直偏角 (rad): " << result.second << std::endl;

    return 0;
}
