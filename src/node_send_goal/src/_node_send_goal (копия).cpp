#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <tf/tf.h>
//#include <geometry_msgs/Quaternion.h>
#include "Diode.h"


struct Diod
{
int16_t r;
int16_t g;
int16_t b;
};




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

Pointline GetPoint(char* str)
{
  Pointline p;
    int probel_coun = 0;
   // int id;
    int r, g, b;
    char s_digit[50] = {0};
    char idk[10];
    double x = 0, y = 0, w = 0;
    for(int i=0;i<strlen(str);i++)
    {

	
     if(probel_coun > 0) s_digit[strlen(s_digit)] = str[i];


       if(probel_coun == 1)
	{
		for(int k=0;(k<i);k++)
		{
			idk[k]=str[k];
		}
             p.id = atoi(idk);
	     std::cout << p.id << std::endl;
		//return p;
		
     	}

      

       if(str[i] == ' ')
       {
//		std::cout << s_digit << std::endl;
           probel_coun++;

           if(probel_coun == 2) p.x = atof(s_digit);
 

           if(probel_coun == 3) p.y = atof(s_digit);
	   if(probel_coun == 4) p.w = atof(s_digit);

	   if(probel_coun == 5) p.r = atoi(s_digit);
	   if(probel_coun == 6) p.g = atoi(s_digit);
	   // if(probel_coun == 7) p.b = atoi(s_digit);

//std::cout << s_digit << std::endl;

            memset(s_digit,0,50);
            continue;
       }

    }
  	   p.b = atoi(s_digit);

//	std::cout << s_digit << std::endl;
	return p;

}

int main(int argc, char** argv)
{
char c;
std::ifstream f;//("pointers.txt"); // подключаем файл 
f.open("pointers.txt");
std::string line;
getline(f,line); 
//std::cout << line << std::endl;
Pointline ggg = GetPoint((char*)line.c_str());

ros::init(argc, argv, "another_send_goal");
  ros::NodeHandle n;

  ros::Publisher led_pub_;
  malish::Diode led_msg_;
  ros::Publisher another_send_goal = n.advertise<malish::led>("another_send_goal", 1000);
  led_pub_ = n_.advertise<malish::led>("/led", 10);




 //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the move_base action server to come up");

std::cout << "proverka vivoda" << ggg.id << ' ' << ggg.x << ' ' << ggg.y << ' ' << ggg.w << ' ' << ggg.r << ' ' << ggg.g << ' '<< ggg.b << std::endl;


	};



