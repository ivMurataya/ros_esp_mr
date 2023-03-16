#include <ros.h>
#include <std_msgs/Int16.h>

const int enc_r = 18;
const int enc_l = 5;

int count_r = 0;
int count_l = 0;

ros::NodeHandle nh;
std_msgs::Int16 enc_r_msg;
std_msgs::Int16 enc_l_msg;
ros::Publisher right_enc ("encr_r_values", &enc_r_msg );
ros::Publisher left_enc ("encr_l_values", &enc_l_msg);

void setup() {
   // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(right_enc);
  nh.advertise(left_enc);
  pin_definition();
}

void loop() {
  // put your main code here, to run repeatedly:
  right_enc.publish(&enc_r_msg);
  left_enc.publish(&enc_l_msg);
  nh.spinOnce();
  delay(50);
}

void pin_definition(){
  pinMode(enc_r,INPUT);
  pinMode(enc_l, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_r),Update_encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_l),Update_encL,CHANGE);
}

void Update_encR(){
  enc_r_msg.data= count_r ++;
}
void Update_encL(){
  enc_l_msg.data= count_l ++;
}
