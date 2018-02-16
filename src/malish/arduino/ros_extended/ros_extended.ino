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

// Max active distance of sonars.
static const int max_son_dist = 80;

// Pin for audio device.
int buzz = 6;
// Pins for leds, debug.
int led1 = 12;
int led2 = 11;
int led3 = 8;

bool dio1 = false;
bool dio2 = false;
bool dio3 = false;

/*
 * Set pins for sonars.
 */
int trigPin[4] = {30, 32, 34, 36};
int echoPin[4] = {31, 33, 35, 37};
Ultrasonic ultrasonic1(trigPin[0], echoPin[0]);
Ultrasonic ultrasonic2(trigPin[1], echoPin[1]);
Ultrasonic ultrasonic3(trigPin[2], echoPin[2]);
Ultrasonic ultrasonic4(trigPin[3], echoPin[3]);

// Creates object to work with accelerometer.
Accelerometer accel;
// Creates object to work with gyroscope.
Gyroscope gyro;

/*
 * Activate diodes callback function (enable/disable).
 */
void callback( const malish::Diode& data){
    if (data.dio1&&data.dio2){
       dio1 = false;
       dio2 = false;
    }
    else{
      dio1 = data.dio1;
      dio2 = data.dio2;
    }
    dio3 = data.dio3;
}

ros::NodeHandle nh;
ros::Subscriber<malish::Diode> sub("/led", &callback);

malish::Sonar son;
ros::Publisher pub("/sonars", &son);

// Publisher for IMU.
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

void sonar_loop() {
    int dist1 = ultrasonic1.Ranging(CM);
    int dist2 = ultrasonic2.Ranging(CM);
    int dist3 = ultrasonic3.Ranging(CM);
    int dist4 = ultrasonic4.Ranging(CM);

    if (dist1 < max_son_dist)
    {
        int frequency = map(dist1, 0, 80, 3500, 4500);
        tone(buzz, frequency, 10);
    }


    if (dist2 < max_son_dist)
    {
        int frequency = map(dist2, 0, 80, 3500, 4500);
        tone(buzz, frequency, 10);
    }

    
    if (dist3 < max_son_dist)
    {
        int frequency = map(dist3, 0, 80, 3500, 4500);
        tone(buzz, frequency, 10);
    }

    if (dist4 < max_son_dist)
    {
        int frequency = map(dist4, 0, 80, 3500, 4500);
        tone(buzz, frequency, 10);
    }

    if(dio1){digitalWrite(led1, HIGH);}
    else {digitalWrite(led1, LOW);}

    if(dio2){digitalWrite(led2, HIGH);}
    else {digitalWrite(led2, LOW);}

    if(dio3){digitalWrite(led3, HIGH);}
    else {digitalWrite(led3, LOW);}

    son.sonLeft = dist1;
    son.sonRight = dist2;
    son.sonFront = dist3;
    son.sonRear = dist4;
    son.timestamp = nh.now();
    pub.publish(&son);
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
    sonar_loop();
    sendImu();
    nh.spinOnce();

    delay(10);
    
}
