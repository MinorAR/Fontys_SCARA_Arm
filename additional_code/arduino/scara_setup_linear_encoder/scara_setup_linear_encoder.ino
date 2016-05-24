#include <ArduinoHardware.h>
#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <Encoder.h>

//Function declarations
void refCb();
void resCb(const std_msgs::Bool& msg);

//Pins
Encoder lin(18, 19);
const byte refPin = 20;
const byte lowerLimitPin = 22;
const byte upperLimitPin = 23;

//globals
volatile bool calibrated = false;
ros::NodeHandle nh;

std_msgs::Float32 val_msg;
ros::Publisher val_pub("/scara_setup/linear_encoder/value", &val_msg);

std_msgs::Bool cal_msg;
ros::Publisher cal_pub("/scara_setup/linear_encoder/calibrated", &cal_msg);

std_msgs::Bool lol_msg;
ros::Publisher lol_pub("/scara_setup/linear_encoder/lower_limit", &lol_msg);

std_msgs::Bool upl_msg;
ros::Publisher upl_pub("/scara_setup/linear_encoder/upper_limit", &upl_msg);

ros::Subscriber<std_msgs::Bool> res_sub("/scara_setup/linear_encoder/reset", &resCb);

void setup() {
  //Set up pins
  pinMode(refPin, INPUT);
  pinMode(lowerLimitPin, INPUT);
  pinMode(upperLimitPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(refPin), refCb, RISING);
  
  //Set up publisher
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(val_pub);
  nh.advertise(cal_pub);
  nh.advertise(lol_pub);
  nh.advertise(upl_pub);
  nh.subscribe(res_sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  val_msg.data = (lin.read() / 1000000.0);
  val_pub.publish(&val_msg);

  cal_msg.data = calibrated;
  cal_pub.publish(&cal_msg);

  lol_msg.data = digitalRead(lowerLimitPin);
  lol_pub.publish(&lol_msg);

  upl_msg.data = digitalRead(upperLimitPin);
  upl_pub.publish(&upl_msg);
  
  nh.spinOnce();
  delay(20);
}

void refCb()
{
  long encPos = lin.read();
  long steps = encPos / 10000; //calculate amount of refmarks up to now (lower)
  
  long lower = steps * 10000; //expected count lower
  long upper = (steps + 1) * 10000; //expected count upper
  
  long difLow = encPos - lower;
  long difUp = upper - encPos;

  if(difLow > difUp){
    lin.write(upper);
  }
  else{
    lin.write(lower);
  }

  calibrated = true;
}

void resCb(const std_msgs::Bool& msg)
{
  lin.write(0);
  calibrated = false;
}

