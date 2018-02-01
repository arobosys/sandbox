/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

//#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

#include <MotorWheel.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>


irqISR(irq1, isr1);
MotorWheel wheel1(3, 2, 4, 5, &irq1);

irqISR(irq2, isr2);
MotorWheel wheel2(11, 12, 14, 15, &irq2);

irqISR(irq3, isr3);
MotorWheel wheel3(9, 8, 16, 17, &irq3);

irqISR(irq4, isr4);
MotorWheel wheel4(10, 7, 18, 19, &irq4);

Omni4WD Omni(&wheel1, &wheel2, &wheel3, &wheel4);

void goAhead(unsigned int speedMMPS) {
  if (Omni.getCarStat() != Omni4WD::STAT_ADVANCE) Omni.setCarSlow2Stop(300);
  Omni.setCarAdvance(0);
  Omni.setCarSpeedMMPS(speedMMPS, 300);
}


ros::NodeHandle  nh;

//Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  goAhead(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("/omni/command", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  //servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  //delay(1);
}
