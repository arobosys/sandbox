#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
//#include <geometry_msgs/Twist.h>
#include <malish/NewTwist.h>
#include <malish/ArduOdom.h>

#include <MotorWheel.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>

#define RPMtoW (6.2831853f/60.f/REDUCTION_RATIO)

bool ledflag;

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


void turnLeft(unsigned int speedMMPS){
    if(Omni.getCarStat()!=Omni4WD::STAT_LEFT) Omni.setCarSlow2Stop(300);
        Omni.setCarLeft(0);
        Omni.setCarSpeedMMPS(speedMMPS, 300);
}

void turnRight(unsigned int speedMMPS){
    if(Omni.getCarStat()!=Omni4WD::STAT_RIGHT) Omni.setCarSlow2Stop(300);
        Omni.setCarRight(0);
        Omni.setCarSpeedMMPS(speedMMPS, 300);
}

void rotateRight(unsigned int speedMMPS){
    if(Omni.getCarStat()!=Omni4WD::STAT_ROTATERIGHT) Omni.setCarSlow2Stop(300);
        Omni.setCarRotateRight(0);
        Omni.setCarSpeedMMPS(speedMMPS, 300);
}

void rotateLeft(unsigned int speedMMPS){
    if(Omni.getCarStat()!=Omni4WD::STAT_ROTATELEFT) Omni.setCarSlow2Stop(300);
        Omni.setCarRotateLeft(0);
        Omni.setCarSpeedMMPS(speedMMPS, 300);
}

void allStop(unsigned int speedMMPS){
    if(Omni.getCarStat()!=Omni4WD::STAT_STOP) Omni.setCarSlow2Stop(300);
        Omni.setCarStop();
}

ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;

int linear_vel;
float orient, angle_vel;

void servo_cb( const malish::NewTwist& msg){
  //LED switcher
  if (ledflag)  
  {
    digitalWrite(LED_BUILTIN, HIGH); 
    //goAhead(msg.data); 
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    //turnLeft(msg.data);
  }
  
  linear_vel = msg.linear_vel;//= sqrt(msg.linear.x^2 + msg.linear.y^2);
  orient = msg.orient;//= atan2(msg.linear.y, msg.linear.x);
  angle_vel = msg.angle_vel;//= msg.angular.z;
  
  Omni.setCarMove(linear_vel, orient, angle_vel);
  
  ledflag = !ledflag; 
}


ros::Subscriber<malish::NewTwist> sub("/twist/command", servo_cb);

int vfl, vfr, vrl, vrr;
malish::ArduOdom odom_msg;

ros::Publisher odom_pub("ardu_odom", &odom_msg);


void setup(){
  
  ledflag = true;
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(odom_pub);
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
  Omni.PIDEnable(0.35,0.02,0,10);
  //Omni.PIDEnable(2.0,1.0,0,10);//PID enable
  pinMode(LED_BUILTIN, OUTPUT);
  
}

void loop(){

	odom_msg.wfl = (float)(RPMtoW) * wheel1.getSpeedRPM();
	odom_msg.wrl = (float)(RPMtoW) * wheel2.getSpeedRPM();
	odom_msg.wrr = (float)(RPMtoW) * wheel3.getSpeedRPM();
	odom_msg.wfr = (float)(RPMtoW) * wheel4.getSpeedRPM();
	odom_msg.timestamp = nh.now();
	odom_pub.publish(&odom_msg);
	
  	nh.spinOnce();
  	Omni.PIDRegulate();

}
