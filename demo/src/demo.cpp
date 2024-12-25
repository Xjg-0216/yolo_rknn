#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// 假设 RM 是你的自定义矩阵计算类，类似于 Python 中的 RM 类
namespace RM {
    // 将像素坐标转换为角度（俯仰角和偏航角）
    void PixToAngle(cv::Point2f boxx, int ImgWidth, int ImgHeight, float camerafy, float camerafz, float& z_Ry, float& y_Rz) {
        float cx = ImgWidth / 2.0f;
        float cy = ImgHeight / 2.0f;
        z_Ry = atan2((boxx.x - cx), camerafy);  // 偏航角（Yaw）
        y_Rz = atan2((boxx.y - cy), camerafz);  // 俯仰角（Pitch）
    }

    // 欧拉角转换为旋转矩阵
    void eulerAnglesToRotationMatrix(const std::vector<float>& eulerAngles, cv::Mat& R) {
        float roll = eulerAngles[0] * CV_PI / 180.0;
        float pitch = eulerAngles[1] * CV_PI / 180.0;
        float yaw = eulerAngles[2] * CV_PI / 180.0;

        cv::Mat R_x = (cv::Mat_<float>(3, 3) <<
            1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

        cv::Mat R_y = (cv::Mat_<float>(3, 3) <<
            cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));

        cv::Mat R_z = (cv::Mat_<float>(3, 3) <<
            cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1);

        R = R_z * R_y * R_x;  // 旋转矩阵 R = Rz * Ry * Rx
    }

    // 旋转矩阵转换为欧拉角
    void rotationMatrixToEulerAngles(const cv::Mat& R, float& roll, float& pitch, float& yaw) {
        pitch = asin(-R.at<float>(2, 0));
        if (cos(pitch) != 0) {
            roll = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
            yaw = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
        } else {
            roll = 0;
            yaw = atan2(-R.at<float>(0, 1), R.at<float>(1, 1));
        }

        // 将弧度转换为角度
        roll = roll * 180.0 / CV_PI;
        pitch = pitch * 180.0 / CV_PI;
        yaw = yaw * 180.0 / CV_PI;
    }
}

// 定义Air类表示无人机的姿态
class Air {
public:
    float roll;  // 滚转角
    float pitch; // 俯仰角
    float yaw;   // 偏航角

    Air(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};





// 获取视线角的函数
void GetVisultV(Air& air, cv::Point2f boxx) {
    // 图像的标定参数
    int ImgWidth = 640, ImgHeight = 480;
    float camerafy = 640.0f, camerafz = 480.0f;
    float t_az_pitch = -15.0f;

    // 第一步：根据图像上点计算其旋转角
    float z_Ry, y_Rz;
    RM::PixToAngle(boxx, ImgWidth, ImgHeight, camerafy, camerafz, z_Ry, y_Rz);

    // 生成像素旋转角的欧拉角
    std::vector<float> pix_eulerAngles = {0.0f, z_Ry, y_Rz};
    cv::Mat R_pix;
    RM::eulerAnglesToRotationMatrix(pix_eulerAngles, R_pix);

    // 第二步：计算机体到相机的旋转矩阵
    std::vector<float> BODY_CAM_eulerangle = {0.0f, t_az_pitch, 0.0f};
    cv::Mat R_BODY_CAM;
    RM::eulerAnglesToRotationMatrix(BODY_CAM_eulerangle, R_BODY_CAM);

    // 第三步：计算NED到机体的旋转矩阵
    float roll = air.roll * 180.0f / CV_PI;
    float pitch = air.pitch * 180.0f / CV_PI;
    float yaw = air.yaw * 180.0f / CV_PI;
    std::vector<float> NED_BODY_eulerangle = {roll, pitch, yaw};
    cv::Mat R_NED_BODY;
    RM::eulerAnglesToRotationMatrix(NED_BODY_eulerangle, R_NED_BODY);

    // 总旋转矩阵
    cv::Mat R_total = R_NED_BODY * R_BODY_CAM * R_pix;

    // 第四步：通过旋转矩阵获取旋转向量，并计算欧拉角
    float x_total, y_total, z_total;
    RM::rotationMatrixToEulerAngles(R_total, x_total, y_total, z_total);

    // 输出视线角
    float tx = z_total * CV_PI / 180.0f;  // 水平偏角
    float ty = y_total * CV_PI / 180.0f;  // 垂直偏角

    std::cout << "水平偏角 (tx): " << tx << " 弧度" << std::endl;
    std::cout << "垂直偏角 (ty): " << ty << " 弧度" << std::endl;
}

int main() {
    // 示例：创建一个Air对象，表示无人机的姿态
    Air air(10.0f, 5.0f, 15.0f); // roll, pitch, yaw
    cv::Point2f boxx(320.0f, 240.0f); // 目标的像素坐标
    GetVisultV(air, boxx); // 获取视线角
    return 0;
}