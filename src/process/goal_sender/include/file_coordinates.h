#ifndef __FILE_COORDINATES__
#define __FILE_COORDINATES__

/**
\file
\brief Header File
Class for reading coordinates in format N x y angle
* N - decimal number of single coordinate
*/

//#include <ros/ros.h>
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
u_long num;
};

class FileCoordinates
{
    public:
    FileCoordinates();
    ~FileCoordinates();
    /*!
    Get Content to file to std::stringstream
    \param[in] file name to read coordinates
    \param[out] std::stringstream
    */
    bool GetFileContent(std::string &s_file, std::stringstream &s_stream_buffer);
    /*!
    Convert coordinates to vector<POINT> - 3 double params
    \param[in] std::stringstream
    \param[out] vector<POINT> contain 3 double coordinates
    */
    void ReadCoordinatesFromStringStream(std::stringstream &s_string_stream, std::vector<POINT> &vec_points);
    /*!
    Get vector<POINT> from file name
    \param[in] file name
    \param[out] vector<POINT> contain 3 double coordinates
    */
    bool GetCoordinatesVectorFromFile(std::string &s_file, std::vector<POINT> &v_points);


};

#endif
