


#include <ros/ros.h>                                   //

#include <move_base_msgs/MoveBaseAction.h>             //подключаемые в примере файлы
#include <actionlib/client/simple_action_client.h>     //

#include <iostream> // подключение библиотеки записи, чтения в поток
#include <fstream>  // подключение библиотеки работы с файлами
#include <string>   // подключение библиотеки работы со строками

//using namespace std // подключение пространства имён std (достаточное, но не эффективное решение)

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




#define Trajectory "pointers.txt"

/*
 подключаемый файл с точками траектории движения, формата:

PointName x y w

где:
 
PointName - имя 
x         - абсцисса 
y         - ордината 
w         - угол ориентации 

*/


struct Pointline
{

	int Pointid;
	char PointName[50]; 				
	float x; 				
	float y;
	float w;	
};



Pointline GetPoint(char* str)
{
  Pointline p;
 int probel_coun = 0;
    char s_digit[20] = {0};
    double x = 0, y = 0, ang = 0;
    for(int i=0;i<strlen(str);i++)
    {
     if(probel_coun > 0) s_digit[strlen(s_digit)] = str[i];


       if(str[i] == ' ')
       {
//		std::cout << s_digit << std::endl;
           probel_coun++;
           if(probel_coun == 2) p.x = atof(s_digit);
           if(probel_coun == 3) p.y = atof(s_digit);
            memset(s_digit,0,20);
            continue;
       }

    }
    p.w = atof(s_digit);
	std::cout << s_digit << std::endl;
return p;
}









int main(int argc, char** argv){        // точка входа в программу
ros::init(argc, argv, "Pinchuk_node");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
   }








/*
fstream trajectory(Trajectory)

if (!trajectory)                       // проверка наличия файла с координатами точек траектории
    {                                      
        cout << "File not found\n";    
    }

*/
char c;
std::ifstream f;//("pointers.txt"); // подключаем файл 
f.open("pointers.txt");
std::string line;

while(getline (f,line))
{
//    std::cout << line << std::endl;
Pointline ggg = GetPoint((char*)line.c_str()); //преобр

std::cout << "proverka vivoda" << ggg.x << " " << ggg.y << " " << ggg.w << std::endl;




//блок передачи Goal

  move_base_msgs::MoveBaseGoal goal;

 

//  for (int i = 1; i <= 4; i++)

  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = ggg.x;
  goal.target_pose.pose.position.y = ggg.y;

/*
				//  реализация кватернионов (если она нужна)


double radians = ggg.w * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;
*/

  goal.target_pose.pose.orientation.w = ggg.w;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCEEDED");
  else
    ROS_INFO("UNSUCCEEDED");




}
//while( (c = getc(f)) != EOF)
//    putchar(c);







/*

// подсчёт количества строк в файле

    char *str = new char [1024];
    int NumberStr = 0;
    while (!trajectory.eof())
    {
        trajectory.getline(NumberStr, 1024, '\n');
        NumberStr++;
    }
    trajectory.close();
    delete str;
       
}
*/
























//int NumberStr = 5;
//
//
//
//Pointline trajectoryPoints[NumberStr];  //создание объекта типа Pointline 
/*





		

//coordinatefline(char *str, trajectoryPoints &p)      
char *str = "1 Myst 0.332 0.23 1.31";


    int probel_coun = 0;
    char s_digit[20] = {0};
    double x = 0, y = 0, ang = 0;
    for(int i=0;i<strlen(str);i++)
    {
       if(str[i] == ' ')
       {
           probel_coun++;
           if(probel_coun == 3) x = atof(s_digit);
           if(probel_coun == 4) y = atof(s_digit);
            memset(s_digit,0,20);
            continue;
       }
       if(probel_coun > 1) s_digit[strlen(s_digit)] = str[i];
    }
    w = atof(s_digit);
//    std::cout << x <<  "ow " << y << " " << ang << std::endl;


8/
  }

fstream trajectory(Trajectory)

if (!trajectory)                       // проверка наличия файла с координатами точек траектории
    {                                      
        cout << "File not found\n";    
    }



for(int = 1;i <= NumberStr; i++)
{
coordinatefline(trajectory.getline,trajectoryPoints[i])

*/

  return 0;
}







