/*
 * CONVENCION PARA MOTORES. RF = R(IGHT)F(ORWARD) ES EL MOTOR DERECHO DELANTERO
 * 
 * CONTROL PARA RELE DE 2 CANALES (INVERSO)
 * 
 */
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <digitalWriteFast.h>
#include <PID_v1.h>
/*
 * FREQUENCY VARIATOR DRIVERS
 * CONVENCION PARA MOTORES. RF = R(IGHT)F(ORWARD) ES EL MOTOR DERECHO DELANTERO
 */
const int PWM_RF = 11;                        //0V-5V a 0V-10V en IBT2(BTS7960)
const int ONOFF_RF = 7;                      //K2: 
const int CWCCW_RF = 22;
const int PWM_LF = 10;                        //0V-5V a 0V-10V en IBT2(BTS7960)
const int ONOFF_LF = 6;                      //K2: 
const int CWCCW_LF = 23;
const int PWM_RB = 9;                        //0V-5V a 0V-10V en IBT2(BTS7960)
const int ONOFF_RB = 5;                      //K2: 
const int CWCCW_RB = 24;
const int PWM_LB = 8;                        //0V-5V a 0V-10V en IBT2(BTS7960)
const int ONOFF_LB = 4;                      //K2: 
const int CWCCW_LB = 25;
float auxiliar;

float new_cmd_vel_timer = 0.0;
bool OnOff = false;
float Hz = 0.0;
int CWCCW_R = 1;
int CWCCW_L = 1;
unsigned long timer = 0;
const float scale = 5.1;                      //0Hz-50Hz to 0V-5V, scale = 5V/50Hz = 0.1V/Hz, 0-255PWM, 255PWM/50Hz = 5 PWM/Hz
//MOTORGO_FVD
int VEL_RF_PWM = 0;
int DIR_RF = 1;
int VEL_LF_PWM = 0;
int DIR_LF = 1;
int VEL_RB_PWM = 0;
int DIR_RB = 1;
int VEL_LB_PWM = 0;
int DIR_LB = 1;
bool stop_req = false;
int sum_error = 0;
int vel_PWM_joystickScale = 254;
//GEOMETRY DATA
const float gearTrain = 0.42857; 
const float wheel_diameter = 0.63;
const float track_width = 1.0;

int pwmControl = 0;
int controlMode = 0;
int pwmFR = 0;
int pwmFL = 0;
int pwmBR = 0;
int pwmBL = 0;
int DIR_R = 0;
int DIR_L = 0;
//PID
double SetpointFollower, InputFollower, OutputFollower;
double aggKp=57, aggKi=902, aggKd=9.5;
//double aggKp=2, aggKi=5, aggKd=0;
float velRFpwm;
float velLFpwm;
float velRBpwm;
float velLBpwm;
PID followerPID(&InputFollower, &OutputFollower, &SetpointFollower, aggKp, aggKi, aggKd, DIRECT); 
   
/*
 * CALLBACKS
 */
ros::NodeHandle nh;                           //declaro el ROS nodehandle
void OnOff_Cb(const std_msgs::Bool& ONOFF){
  OnOff = ONOFF.data;
}
void messageCb(const geometry_msgs::Twist& msg){     //vector velocidad
  pwmFR = msg.linear.x;
  pwmFL = msg.linear.y;
  pwmBR = msg.linear.z;
  pwmBL = msg.angular.x;
  DIR_R = msg.angular.y;
  DIR_L = msg.angular.z;
}

void emergency_stop_statusCb(const std_msgs::Bool& msg) {
  stop_req = msg.data;
  //new_goal = true; 
}

//TOPICOS MSGS
geometry_msgs::Vector3 motorRF;
geometry_msgs::Vector3 motorLF;
geometry_msgs::Vector3 motorRB;
geometry_msgs::Vector3 motorLB;
std_msgs::Bool joystick;
std_msgs::String ControlSignal;
std_msgs::Int16 inputfollower;
std_msgs::Int16 pwm_pid;
std_msgs::Float32 Pose;
std_msgs::Bool OnOff_status;
//TOPICOS
//ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb);
ros::Subscriber<std_msgs::Bool> sub_emergency_stop("/emergency_stop_status", &emergency_stop_statusCb);
ros::Subscriber<geometry_msgs::Twist> sub_vel("/wheel_pwm", &messageCb);
ros::Subscriber<std_msgs::Bool> OnOff_req("OnOff_cmd", &OnOff_Cb);

ros::Publisher OnOff_status_pub("/OnOff_cmd_status", &OnOff_status);
ros::Publisher MotorRF("/motorRF", &motorRF);      //On=True=StartMotor   ;   Off=False=StopMotor
ros::Publisher MotorLF("/motorLF", &motorLF);      //On=True=StartMotor   ;   Off=False=StopMotor
ros::Publisher MotorRB("/motorRB", &motorRB);      //On=True=StartMotor   ;   Off=False=StopMotor
ros::Publisher MotorLB("/motorLB", &motorLB);      //On=True=StartMotor   ;   Off=False=StopMotor
ros::Publisher Joystick("/joystick", &joystick);      //On=True=StartMotor   ;   Off=False=StopMotor
ros::Publisher controlManguera("/Control", &ControlSignal);
ros::Publisher pwmPID("/pwmPID", &pwm_pid);
//ros::Publisher posePub("/mansoPose", &pose);


void setup(){
  TCCR1B = TCCR1B & B11111000 | B00000001;              // D11 D12 for PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;          // D9 D10 for PWM frequency of 31372.55 Hz
  TCCR4B = TCCR4B & B11111000 | B00000001;   // D6 D7 D8 for PWM frequency of 31372.55 Hz   // for PWM frequency of 31372.55 Hz
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.subscribe(OnOff_req);
  nh.advertise(MotorRF);
  nh.advertise(MotorLF);
  nh.advertise(MotorRB);
  nh.advertise(MotorLB);
  nh.advertise(OnOff_status_pub);
  nh.advertise(pwmPID);

  pinMode(PWM_RF, OUTPUT);
  pinMode(ONOFF_RF, OUTPUT);
  pinMode(CWCCW_RF, OUTPUT);
  digitalWrite (ONOFF_RF, HIGH);
  digitalWrite (CWCCW_RF, HIGH);
  motorGo_FVD(0, 0, 0);
  
  pinMode(PWM_LF, OUTPUT);
  pinMode(ONOFF_LF, OUTPUT);
  pinMode(CWCCW_LF, OUTPUT);
  digitalWrite (ONOFF_LF, HIGH);
  digitalWrite (CWCCW_LF, HIGH);
  motorGo_FVD(1, 0, 0);
  
  pinMode(PWM_RB, OUTPUT);
  pinMode(ONOFF_RB, OUTPUT);
  pinMode(CWCCW_RB, OUTPUT);
  digitalWrite (ONOFF_RB, HIGH);
  digitalWrite (CWCCW_RB, HIGH);
  motorGo_FVD(2, 0, 0);

  pinMode(PWM_LB, OUTPUT);
  pinMode(ONOFF_LB, OUTPUT);
  pinMode(CWCCW_LB, OUTPUT);
  digitalWrite (ONOFF_LB, HIGH);
  digitalWrite (CWCCW_LB, HIGH);
  motorGo_FVD(3, 0, 0);
  
  followerPID.SetOutputLimits(-150, 150);
  followerPID.SetMode(AUTOMATIC);
  followerPID.SetSampleTime(100);
  SetpointFollower = 320;
  SetupFVD();
}
void loop(){
  if ((millis()-timer) > 100 && stop_req == false){
    if (OnOff == true){
      digitalWrite (ONOFF_RF, LOW);
      digitalWrite (ONOFF_LF, LOW);
      digitalWrite (ONOFF_RB, LOW);
      digitalWrite (ONOFF_LB, LOW);
      OnOff_status.data = true;

    }
    else if (OnOff == false){
      digitalWrite (ONOFF_RF, HIGH);
      digitalWrite (ONOFF_LF, HIGH);
      digitalWrite (ONOFF_RB, HIGH);
      digitalWrite (ONOFF_LB, HIGH);
      OnOff_status.data = false;
    }
      VEL_RF_PWM = pwmFR;
      VEL_LF_PWM = pwmFL;
      VEL_RB_PWM = pwmBR;
      VEL_LB_PWM = pwmBL;
      motorGoDIR();
      motorGo_FVD(0, DIR_RF, VEL_RF_PWM);
      motorGo_FVD(1, DIR_LF, VEL_LF_PWM); 
      motorGo_FVD(2, DIR_RB, VEL_RB_PWM);
      motorGo_FVD(3, DIR_LB, VEL_LB_PWM);
      motorPub();
      OnOff_status_pub.publish(&OnOff_status);
      nh.spinOnce(); 
      timer = millis();    
    }
  else if((millis()-timer) > 100 && stop_req == true){
    motorGo_FVD(0, DIR_RF, 0);
    motorGo_FVD(1, DIR_LF, 0); 
    motorGo_FVD(2, DIR_RB, 0);
    motorGo_FVD(3, DIR_LB, 0);
    digitalWrite (ONOFF_RF, HIGH);
    digitalWrite (ONOFF_LF, HIGH);
    digitalWrite (ONOFF_RB, HIGH);
    digitalWrite (ONOFF_LB, HIGH);
    OnOff_status.data = false;
    OnOff_status_pub.publish(&OnOff_status);

  }
  nh.spinOnce();
}
void SetupFVD(){
  analogWrite (PWM_RF, 0);
  digitalWrite (ONOFF_RF, LOW);
  digitalWrite (CWCCW_RF, LOW);
  CWCCW_R = 1;
  analogWrite (PWM_LF, 0);
  digitalWrite (ONOFF_LF, LOW);
  digitalWrite (CWCCW_LF, LOW);
  CWCCW_L = 1;
  analogWrite (PWM_RB, 0);
  digitalWrite (ONOFF_RB, LOW);
  digitalWrite (CWCCW_RB, LOW);
  analogWrite (PWM_LB, 0);
  digitalWrite (ONOFF_LB, LOW);
  digitalWrite (CWCCW_LB, LOW);
}
void motorGoDIR (){
  //if (DIR_R == 0 && DIR_L == 0){
    if(DIR_R == 1){
      DIR_RF = 2;
      CWCCW_R = 2;
      DIR_RB = DIR_RF;
    }
    else if (DIR_R == 2){
      DIR_RF = 1;
      CWCCW_R = 1;
      DIR_RB = DIR_RF;
    }
    DIR_RB = DIR_RF;
    if(DIR_L == 1){
      DIR_LF = 2;
      CWCCW_L = 2;
      DIR_LB = 1;
    }
    else if (DIR_L == 2){
      DIR_LF = 1;
      CWCCW_L = 1;
      DIR_LB = 2;
    }
  //}
  /*else if (DIR_R == 0 || DIR_L == 0){
    DIR_RF = 0;
    DIR_RB = DIR_RF;
    DIR_LF = 0;
    DIR_LB = DIR_LF;
  }*/
}
void motorGo_FVD(uint8_t motor, uint8_t direct, uint8_t pwm){         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
  if(motor == 0 ){
    if (direct == 0){
      digitalWrite (CWCCW_RF, HIGH); 
      analogWrite(PWM_RF, 0);
      CWCCW_R = 0;  
    }
    else if(direct == 1){
      digitalWrite (CWCCW_RF, HIGH); 
      analogWrite(PWM_RF, pwm);
      CWCCW_R = 1;
    }
    else if(direct == 2){
      digitalWrite (CWCCW_RF, LOW);
      analogWrite(PWM_RF, pwm);
      CWCCW_R = 2;      
    }
  }
  if(motor == 1 ){
    if (direct == 0){
      digitalWrite (CWCCW_LF, HIGH); 
      analogWrite(PWM_LF, 0);
      CWCCW_L = 0;  
    }
    else if(direct == 1){
      digitalWrite (CWCCW_LF, HIGH); 
      analogWrite(PWM_LF, pwm);
      CWCCW_L = 1;
    }
    else if(direct == 2){
      digitalWrite (CWCCW_LF, LOW);
      analogWrite(PWM_LF, pwm);
      CWCCW_L = 2;      
    }
  }
  if(motor == 2 ){
    if (direct == 0){
      digitalWrite (CWCCW_RB, HIGH); 
      analogWrite(PWM_RB, 0);
    }
    else if(direct == 1){
      digitalWrite (CWCCW_RB, HIGH); 
      analogWrite(PWM_RB, pwm);
    }
    else if(direct == 2){
      digitalWrite (CWCCW_RB, LOW);
      analogWrite(PWM_RB, pwm);
    }
  }
  if(motor == 3 ){
    if (direct == 0){
      digitalWrite (CWCCW_LB, HIGH); 
      analogWrite(PWM_LB, 0);
    }
    else if(direct == 1){
      digitalWrite (CWCCW_LB, HIGH); 
      analogWrite(PWM_LB, pwm);
      CWCCW_L = 1;
    }
    else if(direct == 2){
      digitalWrite (CWCCW_LB, LOW);
      analogWrite(PWM_LB, pwm);
    }
  }
}
void motorPub(){
  if (OnOff == true){
    motorRF.x = 1;
    motorLF.x = 1;
    motorRB.x = 1;
    motorLB.x = 1;
  }
  else if (OnOff == false){
    motorRF.x = 2;
    motorLF.x = 2;
    motorRB.x = 2;
    motorLB.x = 2;
  }
  motorRF.y = DIR_RF;
  motorLF.y = CWCCW_L;
  motorRB.y = DIR_RB;
  motorLB.y = CWCCW_L;

  MotorRF.publish( &motorRF );
  MotorLF.publish( &motorLF );
  MotorRB.publish( &motorRB );
  MotorLB.publish( &motorLB );
}
void OnOffFollower(){
  /*if (Input == 1.0){  
        VEL_RF_PWM = 75 + pwmControl;
        VEL_LF_PWM = 75;
        VEL_RB_PWM = VEL_RF_PWM;
        VEL_LB_PWM = VEL_LF_PWM;
        ControlSignal.data = "compensarDesvDerecha";
      }
      else if (Input == -1.0){
        VEL_RF_PWM = 75;
        VEL_LF_PWM = 75 + pwmControl;
        VEL_RB_PWM = VEL_RF_PWM;
        VEL_LB_PWM = VEL_LF_PWM;
        ControlSignal.data = "compensarDesvIzq";
      }
      else if(Input == 0.0){
        VEL_RF_PWM = 75;
        VEL_LF_PWM = 75;
        VEL_RB_PWM = VEL_RF_PWM;
        VEL_LB_PWM = VEL_LF_PWM;
        ControlSignal.data = "Alineado";
      }
      else if(Input == 2.0){
        VEL_RF_PWM = 0;
        VEL_LF_PWM = 0;
        VEL_RB_PWM = 0;
        VEL_LB_PWM = 0;
        ControlSignal.data = "Stop";
      }*/
}
