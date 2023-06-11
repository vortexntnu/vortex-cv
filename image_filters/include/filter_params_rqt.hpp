#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <image_filters/imgFilterConfig.h>

class FilterParams_rqt
{
public:
    FilterParams_rqt() {

        dynamic_reconfigure::Server<image_filters::imgFilterConfig>::CallbackType f;

        // Bind rqt callback to ros callback, idk
        f = boost::bind(&FilterParams_rqt::callback, this, _1, _2);
        server.setCallback(f);
    }

    void callback(image_filters::imgFilterConfig &config, uint32_t level) {
        ROS_INFO_STREAM("Reconfigure Request");
        configs = config;
    }   

    image_filters::imgFilterConfig configs;
private:
    dynamic_reconfigure::Server<image_filters::imgFilterConfig> server;
};