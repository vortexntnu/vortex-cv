#pragma once

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>

void writeIntsToFile(std::string filename, std::vector<int> ids);

std::string stampToString(const ros::Time &stamp, const std::string format = "%Y.%m.%d-%H.%M.%S");
