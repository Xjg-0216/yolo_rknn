#include "uav_position.h"
#include <cmath>

const double PI = 3.1415926;

// 旋转矩阵计算
Matrix3d UavPositioning::RMaix(double pitch, double yaw, double roll) {
    Matrix3d Rx;
    Rx << 1, 0, 0,
          0, cos(roll), sin(roll),
          0, -sin(roll), cos(roll);

    Matrix3d Ry;
    Ry << cos(pitch), 0, -sin(pitch),
          0, 1, 0,
          sin(pitch), 0, cos(pitch);

    Matrix3d Rz;
    Rz << cos(yaw), sin(yaw), 0,
          -sin(yaw), cos(yaw), 0,
          0, 0, 1;

    return Rx * Ry * Rz;
}

// 相机深度计算
double UavPositioning::Camera_depth(const Matrix3d& CameraUavRMatrix_inv, const Matrix3d& cameraMatrix_inv, 
                                    const Vector3d& imagePoint, double zConst, const Vector3d& tvec) {
    Vector3d tempMat = cameraMatrix_inv * imagePoint;
    Vector3d tempMat2 = CameraUavRMatrix_inv * tvec;
    double s = zConst - tempMat2(2);
    s /= tempMat(2);
    return s;
}

// 经纬度转换
Vector2d UavPositioning::DisConvertLonAndLat(const VectorXd& UavData, const Vector3d& wDisPoint) {
    double uav_longitude = UavData(1);
    double uav_latitude = UavData(0);

    double delta_longitude = wDisPoint(1) / (111320 * cos(uav_latitude * PI / 180));
    double delta_latitude = wDisPoint(0) / 110540;

    double target_lon = uav_longitude + delta_longitude;
    double target_lat = uav_latitude + delta_latitude;

    return Vector2d(target_lat, target_lon);
}

// 计算目标位置
Vector3d UavPositioning::WorldPosition(const VectorXd& UavData, const Vector3d& PictureData) {
    double fx = 640;
    double fy = 480;
    double cx = 320;
    double cy = 240;

    Matrix3d cameraMatrix;
    cameraMatrix << fx, 0, cx,
                    0, fy, cy,
                    0, 0, 1;

    Vector4d disCoeffs(-0.44622, 0.38641, 0.00675, 0.00608);

    Vector3d tvec(0, 0, -UavData(2));

    double camera_pitch_y = 75 * PI / 180;
    double camera_yaw_z = 15 * PI / 180;
    double camera_roll_x = 0 * PI / 180;
    Matrix3d CameraUavRMatrix = RMaix(camera_pitch_y, camera_yaw_z, camera_roll_x);

    double pitch = UavData(4);
    double yaw = UavData(3);
    double roll = UavData(5);
    Matrix3d UavRMatrix = RMaix(pitch, yaw, roll);
    Matrix3d UavRMatrix_inv = UavRMatrix.inverse();

    double zConst = 0;
    Matrix3d CameraUavRMatrix_inv = CameraUavRMatrix.inverse();
    Matrix3d cameraMatrix_inv = cameraMatrix.inverse();
    Vector3d imagePoint(320, 240, 1); // 图像中心点
    double s = Camera_depth(CameraUavRMatrix_inv, cameraMatrix_inv, imagePoint, zConst, tvec);

    Vector3d wDisPoint_ini = cameraMatrix_inv * PictureData * s;
    Vector3d wDisPoint = CameraUavRMatrix_inv * wDisPoint_ini;

    Vector3d world_position = UavRMatrix_inv * wDisPoint;

    double world_position_x = world_position(0) * UavData(2) / world_position(2);
    double world_position_y = world_position(1) * UavData(2) / world_position(2);
    double world_position_z = world_position(2);

    Vector3d world_position_end(world_position_x, world_position_y, world_position_z);

    return world_position_end;
}

// 计算目标经纬度
Vector2d UavPositioning::PixWorld(const VectorXd& UavData, const Vector3d& PictureData) {
    Vector3d world_position_end = WorldPosition(UavData, PictureData);
    return DisConvertLonAndLat(UavData, world_position_end);
}
