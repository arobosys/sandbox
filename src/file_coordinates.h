#ifndef __FILE_COORDINATES__
#define __FILE_COORDINATES__

/**
* Class for reading coordinates in format N x y angle
* N - decimal number of single coordinate
*/

//#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <unistd.h>


struct POINT
{
double x;
double y;
double ang;
};

class FileCoordinates
{
    public:    
    FileCoordinates();
    ~FileCoordinates();
    bool GetFileContent(std::string &s_file, std::stringstream &s_stream_buffer);
    void ReadCoordinatesFromStringStream(std::stringstream &s_string_stream, std::vector<POINT> &vec_points);
    bool GetCoordinatesVectorFromFile(std::string &s_file, std::vector<POINT> &v_points);


};

#endif
