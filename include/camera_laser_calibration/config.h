//
// Created by m on 7/5/18.
//

#ifndef CAMERA_LIDAR_CALIBRATION_CONFIG_H
#define CAMERA_LIDAR_CALIBRATION_CONFIG_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <stdint.h>

using namespace std;
using namespace cv;


class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config()
    {}

public:
    ~Config();

    /*set a new config file*/
    static void setParameterFile(const std::string &filename);

    /*access the parameter values*/
    template<typename T>
    static T get(const std::string &key)
    {
        return T(Config::config_->file_[key]);
    }
};
#endif //CAMERA_LIDAR_CALIBRATION_CONFIG_H
