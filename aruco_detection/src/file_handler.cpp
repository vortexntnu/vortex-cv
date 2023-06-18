#include <file_handler.hpp>

void writeIntsToFile(std::string filename, std::vector<int> ids)
{
	std::ofstream outputFile(filename);
	if (outputFile.is_open()) {
		for (int val : ids) {
			outputFile << val << ", ";
		}
	}
	else
		ROS_WARN_STREAM("Unable to open file " << filename << ", try creating the missing directory");
	outputFile.close();
}

// Found here: https://answers.ros.org/question/340701/convert-ros-time-to-hours-minutes-seconds-string-in-system-timezone-in-c/
std::string stampToString(const ros::Time &stamp, const std::string format)
{
	const int output_size = 100;
	char output[output_size];
	std::time_t raw_time = static_cast<time_t>(stamp.sec);
	struct tm *timeinfo  = localtime(&raw_time);
	std::strftime(output, output_size, format.c_str(), timeinfo);
	std::stringstream ss;
	ss << std::setw(9) << std::setfill('0') << stamp.nsec;
	const size_t fractional_second_digits = 4;
	return std::string(output) + "." + ss.str().substr(0, fractional_second_digits);
}
