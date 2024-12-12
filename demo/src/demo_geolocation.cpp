#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <vector>
#include "drone_objlocation.h"

using namespace std;

DroneObjlocation geo_location = DroneObjlocation();

void test01()
{
    std::map<std::string, std::vector<float>> result;

    std::vector<float> uv = {533, 371};
    float height = 88.4570007324;

    std::vector<float> euler_camera   = {0.0, 1.309, 0.0};
    std::vector<float> euler_drone    = {1.827, -0.031, 0.157};
    std::vector<float> position_drone = {38.1368980408, 116.2943725586, 88.4570007324};

    geo_location.set_parameter((uint16_t)640, (uint16_t)480, 640, 640, 300, 240); // img_width, img_height, fx, fy, cx, cy
    result = geo_location.get_target_location(uv, height, euler_camera, euler_drone, position_drone); 

    printf("p_d: %f, %f, %f\n", result["p_d"][0], result["p_d"][1], result["p_d"][2]);
    printf("p_e: %f, %f, %f\n", result["p_e"][0], result["p_e"][1], result["p_e"][2]);
    printf("gps: %f, %f, %f\n", result["gps"][0], result["gps"][1], result["gps"][2]);
    
}
int main(int argc, char *argv[])
{
    test01();
    return 0;
}