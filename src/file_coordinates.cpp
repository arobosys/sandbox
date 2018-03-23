#include "file_coordinates.h"

FileCoordinates::FileCoordinates()
{
}
FileCoordinates::~FileCoordinates()
{
}

bool FileCoordinates::GetFileContent(std::string &s_file, std::stringstream &s_stream_buffer)
{
    std::ifstream t_file;
    t_file.open(s_file);
    if(!t_file) 
    {
	std::cout << "File " << s_file.c_str() << " is not exists " << std::endl;
       return false;
    }
    s_stream_buffer << t_file.rdbuf();
    t_file.close();
    return true;
}

void FileCoordinates::ReadCoordinatesFromStringStream(std::stringstream &s_string_stream, std::vector<POINT> &vec_points)
{
    std::string line;
    std::vector<std::string> internal;
    std::string tok;

    while(getline(s_string_stream, tok, '\n')) 
    {

       std::stringstream s_param(tok);
       std::string s_;
       std::vector<std::string> v_vals;
       while(getline(s_param, s_, ' '))
       {
           std::cout << s_.c_str() << std::endl;
           v_vals.push_back(s_);

           POINT p;
           if(v_vals.size() > 3)
           {
               p.ang = atof(v_vals[3].c_str());
               p.y = atof(v_vals[2].c_str());
               p.x = atof(v_vals[1].c_str());
               vec_points.push_back(p);
           }
	}
    }
}

bool FileCoordinates::GetCoordinatesVectorFromFile(std::string &s_file, std::vector<POINT> &v_points)
{
    std::stringstream s_stream;
    if(!GetFileContent(s_file, s_stream)) return false;
    ReadCoordinatesFromStringStream(s_stream, v_points);
    return true;
}