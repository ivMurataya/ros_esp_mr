
/*
#define motor_r_pwm
#define motor_r_a
#define motor_r_b

#define motor_l_b
#define motor_l_pwm
#define motor_l_a
*/
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

void setup(){
    Serial.begin(115200);
    pin_def();
    stop();
    Serial.println("Get Ready");
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

void cmdVel_to_pwm(float x , float z){
    left_wheel = (x-z)/2;
    right_wheel = (x+z)/2;
    direction();
    speed();
    if (x== 0.0 & z ==0.0){
        stop();
    }
    Serial.print(left_wheel);
    Serial.print(" / ");
    Serial.println(right_wheel);
}

void loop(){
    Serial.println("Forward");
    cmdVel_to_pwm(0.5,0.0);
    delay(2000);
    Serial.println("Back");
    cmdVel_to_pwm(-0.5,0.0);
    delay(2000);
    Serial.println("left");
    cmdVel_to_pwm(0.0,1.0);
    delay(2000);
    Serial.println("Right");
    cmdVel_to_pwm(0.0,-1.0);

    
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
   
void stop(){
    ledcWrite(channel_R,0);
    ledcWrite(channel_L,0);
}
