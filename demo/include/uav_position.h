#ifndef UAV_POSITION_H
#define UAV_POSITION_H

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

class UavPositioning {
public:
    UavPositioning();
    
    // 旋转矩阵计算
    static Matrix3d RMaix(double pitch, double yaw, double roll);

    // 计算相机深度
    static double Camera_depth(const Matrix3d& CameraUavRMatrix_inv, const Matrix3d& cameraMatrix_inv, 
                               const Vector3d& imagePoint, double zConst, const Vector3d& tvec);
    
    // 经纬度转换
    static Vector2d DisConvertLonAndLat(const VectorXd& UavData, const Vector3d& wDisPoint);
    
    // 从图像像素计算世界坐标
    static Vector2d PixWorld(const VectorXd& UavData, const Vector3d& PictureData);

private:
    // 计算目标位置的中间步骤
    static Vector3d WorldPosition(const VectorXd& UavData, const Vector3d& PictureData);
};

#endif // UAV_POSITION_H
