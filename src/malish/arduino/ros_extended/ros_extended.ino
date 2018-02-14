/*!
 * \file ros_extended.ino
 *
 * Arduino - ROS bridge for sensors control.
 *
 * \author Georgy Ostroumov ostroumov.gr@arobosys.com
 */

#include <Ultrasonic.h>

#include <ros.h>
#include <malish/Diode.h>
#include <malish/Sonar.h>

int buzz = 6;

int led1 = 11;
int led2 = 12;
int led3 = 8;

bool dio1 = false;
bool dio2 = false;
bool dio3 = false;

int echoPin = 10;
int trigPin = 9;
Ultrasonic ultrasonic(trigPin, echoPin);

void callback( const malish::Diode& data){
    dio1 = data.dio1;
    dio2 = data.dio2;
    dio3 = data.dio3;
}

ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;
ros::Subscriber<malish::Diode> sub("/led", &callback);

malish::Sonar son;
ros::Publisher pub("/sonars", &son);

void setup() {
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
}

void loop()
{
    int dist = ultrasonic.Ranging(CM);

    if (dist < 80)
    {
        int frequency = map(dist, 0, 80, 3500, 4500);
        tone(buzz, frequency, 10);
    }

    if(dio1){digitalWrite(led1, HIGH);}
    else {digitalWrite(led1, LOW);}

    if(dio2){digitalWrite(led2, HIGH);}
    else {digitalWrite(led2, LOW);}

    if(dio3){digitalWrite(led3, HIGH);}
    else {digitalWrite(led3, LOW);}

    son.son1 = dist;
    son.son2 = 0;
    son.son3 = 0;
    son.son4 = 0;
    son.timestamp = nh.now();

    pub.publish(&son);
    nh.spinOnce();

    delay(10);
}
