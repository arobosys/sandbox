#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <tf/tf.h>
//#include <geometry_msgs/Quaternion.h>
//#include "Diode.h"


//struct Diod
//{
//int16_t r;
//int16_t g;
//int16_t b;
//};




struct Pointline
	{

	int id;
				
	float x; 				
	float y;
	float w; 

	int r;
	int g;
	int b;
	};

//char line[100];

int main(int argc, char** argv)
{

std::ifstream ifs{"pointers.txt"};

 
    std::vector<Pointline> vec_d;
    Pointline p;


    while(ifs >> p.id >> p.x >> p.y >> p.w >> p.r >> p.g >> p.b)
    {

	vec_d.push_back(p);
    }

    for(int i=0;i<vec_d.size();i++)
    {
	std::cout << vec_d[i].x<< " " << vec_d[i].y << " " <<vec_d[i].w << " " << vec_d[i].g << std::endl;


	};

}

