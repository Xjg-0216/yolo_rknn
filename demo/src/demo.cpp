#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

class DistanceEstimation {
public:
    DistanceEstimation() {
        // Camera image dimensions
        W = 640;
        H = 480;
        excel_path = "/home/neardi/Desktop/trash_detect/camera_parameters_ahd_2.xlsx";
        loadCameraParameters(excel_path);
    }

    void loadCameraParameters(const std::string& excel_path) {
        // Load camera intrinsic and extrinsic matrices from the Excel file
        // You can use your preferred method for reading Excel files in C++ here.
        // For simplicity, we'll just hardcode the values.
        // In real code, consider using a library like OpenCV or others to read Excel files.
        K << 640, 0.000, 320,
             0.000, 480, 240,
             0.000, 0.000, 1.000;

        // Set rotation matrix based on Euler angles (yaw, pitch, roll)
        double yaw = 0;   // Example yaw in radians
        double pitch = 0; // Example pitch in radians
        double roll = 0;  // Example roll in radians

        // Create rotation matrices for yaw, pitch, and roll
        Eigen::Matrix3d R_x, R_y, R_z;


        R_x << 1, 0, 0,
            0, std::cos(roll), std::sin(roll),
            0, -std::sin(roll), std::cos(roll);

        R_y << std::cos(pitch), 0, -std::sin(pitch),
                 0, 1, 0,
                 std::sin(pitch), 0, std::cos(pitch);

        R_z << std::cos(yaw), std::sin(yaw), 0,
                  -std::sin(yaw), std::cos(yaw), 0,
                  0, 0, 1;

        R =  R_x * R_y * R_z;

        // Set extrinsic matrix P (without translation part)
        P.setIdentity();
        P.topLeftCorner<3,3>() = R;
    }

    Eigen::Vector3d objectPointWorldPosition(double u1, double v1) {

        double fx = K(0, 0);
        double fy = K(1, 1);
        // Camera height
        double Height = 100;//摄像头安装高度

        // Camera tilt angles
        double angle_a = 75 * 3.1415926 / 180; //摄像头安装角度，俯仰角，向下为正
        double angle_b = std::atan((v1 - H / 2) / fy);
        double angle_c = angle_b + angle_a;

        double depth = (Height / std::sin(angle_c)) * std::cos(angle_b);

        Eigen::Matrix3d K_inv = K.inverse();
        Eigen::Matrix4d P_inv = P.inverse();
        
        Eigen::Vector3d point_c(u1, v1, 1);
        Eigen::Vector3d c_position = K_inv * (depth * point_c);
        Eigen::Vector4d c_position_homogeneous;
        c_position_homogeneous << c_position, 1.0;

        Eigen::Vector4d w_position = P_inv * c_position_homogeneous;
        return w_position.head(3);
    }

    Eigen::Vector3d distance(const std::vector<float>& kuang) {
        if (kuang.size()) {
            // int u = (kuang[0]+kuang[2])/2;
            // int v = (kuang[1]+kuang[3])/2;
            // int w = kuang[2]-kuang[0];
            // int h = kuang[3]-kuang[1];
            float u_c = kuang[0];
            float v_c = kuang[1];
            Eigen::Vector3d d1 = objectPointWorldPosition(u_c, v_c);
            // if (d1(0) <= 0) {
            //     d1.head(3).setZero();
            // }
            return d1.head(3);
        }

        return Eigen::Vector3d::Zero();
    }

private:
    int W, H;
    std::string excel_path;
    Eigen::Matrix3d K;
    Eigen::Matrix4d P;
    Eigen::Matrix3d R;
};

int main() {
    // 创建 DistanceEstimation 对象
    DistanceEstimation distanceEstimator;


    std::vector<float> kuang = {533.5, 371.5};  // 目标框坐标

    // 计算目标的世界坐标位置
    Eigen::Vector3d distance = distanceEstimator.distance(kuang);
    
    // 输出结果
    std::cout << "Estimated distance: " << distance.transpose() << std::endl;

    return 0;
}
