/*!
 * \file ros_extended.ino
 *
 * Arduino - ROS bridge for sensors control.
 *
 * \authors Georgy Ostroumov ostroumov.gr@arobosys.com
 */

#include <Ultrasonic.h>

#include <ros.h>
#include <malish/Diode.h>
#include <malish/Sonar.h>
#include <malish/AmperkaImu.h>
#include <sensor_msgs/ChannelFloat32.h>

// IMU dependencies
#include <sensor_msgs/Imu.h>
// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>

#define G 9.80665F
#define imuFrame "imu"

int buzz = 6;

int led1 = 12;
int led2 = 11;
int led3 = 8;

bool dio1 = false;
bool dio2 = false;
bool dio3 = false;

/*
 * Set pins for sonars.
 */
int echoPin = 10;
int trigPin = 9;
Ultrasonic ultrasonic(trigPin, echoPin);

// Creates object to work with accelerometer.
Accelerometer accel;
// Creates object to work with gyroscope.
Gyroscope gyro;

/*
 * Activate diodes callback function (enable/disable).
 */
void callback( const malish::Diode& data){
    dio1 = data.dio1;
    dio2 = data.dio2;
    dio3 = data.dio3;
}

ros::NodeHandle <ArduinoHardware, 2, 2, 80, 105> nh;
ros::Subscriber<malish::Diode> sub("/led", &callback);

malish::Sonar son;
ros::Publisher pub("/sonars", &son);

uint32_t seq_imu = 0;
malish::AmperkaImu imuMsg;
ros::Publisher imuPublisher("/arduino/imu", &imuMsg);

void sendImu() {
    // Считываем данные с акселерометра в единицах G.
    accel.readGXYZ(&imuMsg.linear_acceleration.x, &imuMsg.linear_acceleration.y, &imuMsg.linear_acceleration.z);
    imuMsg.linear_acceleration.x *= -G;
    imuMsg.linear_acceleration.y *= -G;
    imuMsg.linear_acceleration.z *= G;
    // Считываем данные с акселерометра в радианах в секунду.
    gyro.readRadPerSecXYZ(&imuMsg.angular_velocity.x, &imuMsg.angular_velocity.y, &imuMsg.angular_velocity.z);
    imuMsg.angular_velocity.x *= 1;
    imuMsg.angular_velocity.y *= -1;
    imuMsg.angular_velocity.z *= -1;

    imuMsg.timestamp = nh.now();
    imuPublisher.publish(&imuMsg);
}

void setup() {
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    nh.advertise(imuPublisher);
    // Accelerometer initialization.
    accel.begin();
    // Gyroscope initialization.
    gyro.begin();

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
    sendImu();
    nh.spinOnce();


    delay(10);
}
