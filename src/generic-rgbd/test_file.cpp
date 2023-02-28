#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");

    std::string model_color = "banana2_centered_46_88_vrt_fc.wrl";

    std::fstream file(model_color, std::ios::in);

    if (file.is_open())
    {
        // std::cout << "File opened";
        std::string replace = "  scale 1 1 1";
        std::string replace_with = "  scale 10 10 10";
        std::string line;
        std::vector<std::string> lines;

        while (std::getline(file, line))
        {
            std::cout << line << std::endl;

            std::string::size_type pos = 0;

            while ((pos = line.find(replace, pos)) != std::string::npos)
            {
                line.replace(pos, line.size(), replace_with);
                pos += replace_with.size();
            }

            lines.push_back(line);
        }

        file.close();
        std::ofstream output_file("test.wrl");
        if (output_file.is_open())
        {
            for (const auto &i : lines)
            {

                output_file << i << std::endl;
            }
        }
        output_file.close();
        // file.open(model_color, std::ios::out);
    }
    else
    {
        std::cout << "Error opening file" << model_color;
    }
}