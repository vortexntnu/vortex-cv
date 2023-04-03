#include <file_handler.hpp>

void writeIntsToFile(std::string filename, std::vector<int> ids)
{
    std::ofstream outputFile(filename);
    if (outputFile.is_open())
    {
        for (int val : ids) 
        {
            outputFile << val << " ";
        }
    }
    else ROS_WARN_STREAM("Unable to open file " << filename);
    outputFile.close();
}
