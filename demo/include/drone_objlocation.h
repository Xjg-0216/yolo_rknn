#ifndef __GEO_LOCATION_H__
#define __GEO_LOCATION_H__

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class DroneObjlocation
{
public:
    DroneObjlocation();
    DroneObjlocation(uint16_t img_width, uint16_t img_height, float fx, float fy, float cx, float cy);
    void set_parameter(uint16_t img_width, uint16_t img_height, float fx, float fy, float cx, float cy);
    std::map<std::string, std::vector<float>> get_target_location(std::vector<float> uv, float height, 
                                                                std::vector<float> euler_camera, 
                                                                std::vector<float> euler_drone, 
                                                                std::vector<float> position_drone);
    Eigen::Vector3d LatLonAlt2ECEF(Eigen::Vector3d pointLLA);
    Eigen::Vector3d ECEF2LatLonAlt(Eigen::Vector3d pointECEF);
    Eigen::Matrix3d CalRotation_NED2ECEF(Eigen::Vector3d pointLLA);
    
    Eigen::Matrix3d get_K();

private:
    uint16_t _img_width  = 1920;
    uint16_t _img_height = 1080;
    float _fx = 1000.0;
    float _fy = 1000.0;
    float _cx = 960;
    float _cy = 540;
    
    Eigen::Matrix3d K;
    double Rad_to_deg = 180.0 / M_PI;
};

#endif //__GEO_LOCATION_H__