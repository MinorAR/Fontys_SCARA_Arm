#include <ArduinoHardware.h>
#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <Encoder.h>

//Function declarations
void refCb();
void resCb(const std_msgs::Empty& msg);

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

std_msgs::String val_shoulder_msg;
ros::Publisher val_shoulder_pub("/scara_setup/shoulder/value", &val_shoulder_msg);

std_msgs::Float32 val_elbow_msg;
ros::Publisher val_elbow_pub("/scara_setup/elbow/value", &val_elbow_msg);

std_msgs::Float32 val_wrist_msg;
ros::Publisher val_wrist_pub("/scara_setup/wrist/value", &val_wrist_msg);

std_msgs::Float32 load_msg;
ros::Publisher load_pub("/scara_setup/arduino_load", &load_msg);

std_msgs::Bool cal_msg;
ros::Publisher cal_pub("/scara_setup/linear_encoder/calibrated", &cal_msg);

std_msgs::Bool lol_msg;
ros::Publisher lol_pub("/scara_setup/linear_encoder/lower_limit", &lol_msg);

std_msgs::Bool upl_msg;
ros::Publisher upl_pub("/scara_setup/linear_encoder/upper_limit", &upl_msg);

ros::Subscriber<std_msgs::Empty> res_sub("/scara_setup/linear_encoder/reset", &resCb);

void setup() {
  //Set up pins
  pinMode(refPin, INPUT);
  pinMode(lowerLimitPin, INPUT);
  pinMode(upperLimitPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(refPin), refCb, RISING);
  
  //Set up publisher
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(val_shoulder_pub);
  nh.advertise(val_elbow_pub);
  nh.advertise(val_wrist_pub);
  nh.advertise(load_pub);
  nh.advertise(val_pub);
  nh.advertise(cal_pub);
  nh.advertise(lol_pub);
  nh.advertise(upl_pub);
  nh.subscribe(res_sub);

  Serial2.begin(9600);
  Serial2.setTimeout(200);
}

const unsigned long lt = 50; //desired loop time
unsigned long tdiff = 0;
char test_str[10];

void loop() {
  unsigned long t1 = millis();

  Serial2.write("1");
  String str = Serial2.readStringUntil('\n');
  //int val = str.toInt();
  //val_shoulder_msg.data = (float)val;
  str.trim();
  str.toCharArray(test_str, 10);
  val_shoulder_msg.data = test_str;
  val_shoulder_pub.publish(&val_shoulder_msg);

  /*Serial2.write("2");
  str = Serial2.readStringUntil('\n');
  val = str.toInt();
  val_elbow_msg.data = val;
  val_elbow_pub.publish(&val_elbow_msg);*/

  /*Serial2.write("3");
  str = Serial2.readStringUntil('\n');
  val = str.toInt();
  val_wrist_msg.data = val;
  val_wrist_pub.publish(&val_wrist_msg);*/

  load_msg.data = (tdiff * 100.0) / lt;
  //load_msg.data = tdiff;
  load_pub.publish(&load_msg);
  
  val_msg.data = (lin.read() / 1000000.0);
  val_pub.publish(&val_msg);

  cal_msg.data = calibrated;
  cal_pub.publish(&cal_msg);

  lol_msg.data = digitalRead(lowerLimitPin);
  lol_pub.publish(&lol_msg);

  upl_msg.data = digitalRead(upperLimitPin);
  upl_pub.publish(&upl_msg);
  
  nh.spinOnce();

  tdiff = millis() - t1;
  
  if(tdiff < lt){
    delay(lt - tdiff);
  }
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

void resCb(const std_msgs::Empty& msg)
{
  lin.write(0);
  calibrated = false;
}

