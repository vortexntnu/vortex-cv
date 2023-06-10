#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_detection/imgFilterConfig.h>

class FilterParams_rqt
{
public:
    FilterParams_rqt() {
        dynamic_reconfigure::Server<aruco_detection::imgFilterConfig>::CallbackType f;

        f = boost::bind(&FilterParams_rqt::callback, this, _1, _2);
        server.setCallback(f);
    }

    void callback(aruco_detection::imgFilterConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Reconfigure Request");
        configs = config;
    }   

    aruco_detection::imgFilterConfig configs;
private:
    dynamic_reconfigure::Server<aruco_detection::imgFilterConfig> server;
};