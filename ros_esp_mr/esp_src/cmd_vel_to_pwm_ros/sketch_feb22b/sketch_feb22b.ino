                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
/*
 
*/




#include <ros.h>
#include <geometry_msgs/Twist.h>


ros::NodeHandle  nh;

const uint8_t R_PWM = 12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;

const uint8_t L_PWM = 33;
const uint8_t L_BACK = 25;
const uint8_t L_FORW = 26; 

const uint8_t channel_L = 0;
const uint8_t channel_R = 1;

float left_wheel;
float right_wheel;


void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){
    nh.loginfo("hola");
    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z)/2;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z)/2;
    direction();
    speed();
    if (velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
        nh.loginfo("stop");
    }


}


ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );

void setup(){
    nh.initNode();
    nh.subscribe(sub);
     //Serial.begin(115200);
    pin_def();
    stop();
    //Serial.println("Get Ready");
    delay(2000);
}

void pin_def(){
  
    const int freq = 5000;
    const int res = 8;
    
    pinMode(L_PWM, OUTPUT);
    pinMode(L_FORW, OUTPUT);
    pinMode(L_BACK, OUTPUT);
    pinMode (R_PWM,OUTPUT);
    pinMode (R_FORW, OUTPUT);
    pinMode (R_BACK, OUTPUT);

    ledcSetup(channel_R, freq, res);
    ledcSetup(channel_L , freq,res);

    ledcAttachPin (R_PWM, channel_R);
    ledcAttachPin (L_PWM, channel_L);
    
}


void loop(){
 
    nh.spinOnce();
    
}

void direction (){
    digitalWrite(L_FORW,left_wheel >0 );
    digitalWrite(L_BACK,left_wheel <0);
    digitalWrite(R_FORW,right_wheel >0);
    digitalWrite(R_BACK,right_wheel <0);
}

void speed(){
    ledcWrite(channel_R,200);
    ledcWrite(channel_L,200);
}
   
void stop()
{
  
    ledcWrite(channel_R,0);
    ledcWrite(channel_L,0);
}
