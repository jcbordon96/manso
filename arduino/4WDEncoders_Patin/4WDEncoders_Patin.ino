/*
 * CONVENCION PARA MOTORES. RF = R(IGHT)F(ORWARD) ES EL MOTOR DERECHO DELANTERO
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <digitalWriteFast.h>
#include <PID_v1.h>
#include <PinChangeInt.h>

/*
 * ENCODERS
 */
#define RFMotor 0
#define LFMotor 1
#define RBMotor 2
#define LBMotor 3
//ENCODER MOTOR "RF"
#define c_RightFrontEncoderInterruptA 5
#define c_RightFrontEncoderInterruptB 4
#define c_RightFrontEncoderPinA 18
#define c_RightFrontEncoderPinB 19
#define RightFrontEncoderIsReversed
volatile bool _RightFrontEncoderASet;
volatile bool _RightFrontEncoderBSet;
volatile bool _RightFrontEncoderAPrev;
volatile bool _RightFrontEncoderBPrev;
volatile long _RightFrontEncoderTicks = 0;
long _RightFrontEncoderTicks_aux = 0.0;
float sum_RightFront_encoder_ticks = 0.0;
float rev_RightFront_wheel = 0.0;
float rps_RF = 0.0;
float rps_RFL = 0.0;
float vel_RF = 0.0;
float rps_req_RF = 0.0;
float rps_req_RF_last = 0.0;
float rps_req_RF_cmd = 0.0;
//ENCODER MOTOR "LF"
#define c_LeftFrontEncoderInterruptA 0
#define c_LeftFrontEncoderInterruptB 1
#define c_LeftFrontEncoderPinA 2
#define c_LeftFrontEncoderPinB 3
#define LeftFrontEncoderIsReversed
volatile bool _LeftFrontEncoderASet;
volatile bool _LeftFrontEncoderBSet;
volatile bool _LeftFrontEncoderAPrev;
volatile bool _LeftFrontEncoderBPrev;
volatile long _LeftFrontEncoderTicks = 0.0;
long _LeftFrontEncoderTicks_aux = 0.0;
float sum_LeftFront_encoder_ticks = 0.0;
float rev_LeftFront_wheel = 0.0;
float rps_LF = 0.0;
float rps_LFL = 0.0;
float vel_LF = 0.0;
float rps_req_LF = 0.0;
float rps_req_LF_last = 0.0;
float rps_req_LF_cmd = 0.0;
//ENCODER MOTOR "RB"
#define c_RightBackEncoderPinA A12
#define c_RightBackEncoderPinB A13
#define RightBackEncoderIsReversed
volatile bool _RightBackEncoderASet;
volatile bool _RightBackEncoderBSet;
volatile bool _RightBackEncoderAPrev;
volatile bool _RightBackEncoderBPrev;
volatile long _RightBackEncoderTicks = 0;
long _RightBackEncoderTicks_aux = 0;
float sum_RightBack_encoder_ticks = 0.0;
float rev_RightBack_wheel = 0.0;
float rps_RB = 0.0;
float rps_RBL = 0.0;
float vel_RB = 0.0;
float rps_req_RB = 0.0;
float rps_req_RB_last = 0.0;
float rps_req_RB_cmd = 0.0;
//ENCODER MOTOR "LB"
#define c_LeftBackEncoderPinA A15
#define c_LeftBackEncoderPinB A14
#define LeftBackEncoderIsReversed
volatile bool _LeftBackEncoderASet;
volatile bool _LeftBackEncoderBSet;
volatile bool _LeftBackEncoderAPrev;
volatile bool _LeftBackEncoderBPrev;
volatile long _LeftBackEncoderTicks = 0;
long _LeftBackEncoderTicks_aux = 0;
float sum_LeftBack_encoder_ticks = 0.0;
float rev_LeftBack_wheel = 0.0;
float rps_LB = 0.0;
float rps_LBL = 0.0;
float vel_LB = 0.0;
float rps_req_LB = 0.0;
float rps_req_LB_last = 0.0;
float rps_req_LB_cmd = 0.0;
//encoderArm
#define c_ArmYEncoderInterruptA 2
#define c_ArmYEncoderInterruptB 3
#define c_ArmYEncoderPinA 20
#define c_ArmYEncoderPinB 21
#define ArmYEncoderIsReversed
volatile bool _ArmYEncoderASet;
volatile bool _ArmYEncoderBSet;
volatile bool _ArmYEncoderAPrev;
volatile bool _ArmYEncoderBPrev;
volatile long _ArmYEncoderTicks = 0;
long _ArmYEncoderTicks_aux = 0;
volatile double arm_pose = 0;
const int ARM_RIGHT = 2;
const int ARM_LEFT = 1;

#define c_StopInterrupt A8

float auxiliar;
unsigned long new_cmd_vel_timer = 0.0;
unsigned long timer = 0;
unsigned long timer_encoder = 0;
unsigned long delta_timer = 0;
unsigned long pub_pose_timer = 0;
unsigned long timeout_stuck_timer = 0;
unsigned int timeout_stuck = 500;
bool is_stuck = false;
bool to_zero = false;
bool warn_stuck = false;
bool is_warned = false;
bool filterReq = false;
volatile bool stop_req = false;
volatile bool hard_stop = false;
float rps_limit = 0.25;
bool accel_control = true;
int sum_error = 0;
//GEOMETRY DATA

const float wheel_diameter = 0.63;
const float track_width = 1.6;
//PID
double SetpointRF, SetpointLF,  InputRF, InputLF,  OutputRF, OutputLF, SetpointRB, SetpointLB, InputRB, InputLB, OutputRB, OutputLB;;
double aggKp=65, aggKi=500, aggKd=15;
int VEL_RF_PWM = 0;
float velRFpwm = 0.0;
int VEL_LF_PWM = 0;
float velLFpwm = 0.0;
int VEL_RB_PWM = 0;
float velRBpwm = 0.0;
int VEL_LB_PWM = 0;
float velLBpwm = 0.0;
int DIR_R = 0;
int DIR_L = 0;
PID RFPID(&InputRF, &OutputRF, &SetpointRF, aggKp, aggKi, aggKd, DIRECT); 
PID LFPID(&InputLF, &OutputLF, &SetpointLF, aggKp, aggKi, aggKd, DIRECT); 
PID RBPID(&InputRB, &OutputRB, &SetpointRB, aggKp, aggKi, aggKd, DIRECT); 
PID LBPID(&InputLB, &OutputLB, &SetpointLB, aggKp, aggKi, aggKd, DIRECT);

/*
 * TOOL: PATIN
 */
//EndStop
#define c_ToolEndStopInterrupt A9
#define c_LeftEndStopInterrupt A11
#define c_RightEndStopInterrupt A10
volatile bool arm_is_left = false;
volatile bool arm_is_right = false;
volatile bool tool_is_ok = false;
//TOOL: Patin
bool stop_flag = false;
int DIR = 1;
const int MOTOR_ARM_RPWM_PIN = 12;
const int MOTOR_ARM_LPWM_PIN = 11;
const int MOTOR_TOOL_RPWM_PIN = 10;
const int MOTOR_TOOL_LPWM_PIN = 9;
bool timerClock = false;
bool left_flag = false;
bool right_flag = false;
volatile bool arm_tool = false;
bool firstsetup = false;
int pwm_arm = 100;
int pwm_tool = 0;
float distanceToZone = 0;
const float precision = 15;//0.05;
bool right_to_left = false;
bool startTool = false;
int countRev = 0;
unsigned long timerTool = 0;
bool manual_cmd = false;
bool move_is_ok = false;
int i = 0;
bool reset = false;
int cmd_flag = 0;
const float middle = 89;//0.1;
const float left = 25;//0.1;
const float right = 151;//0.1;
unsigned long timeZone = 0;
unsigned long timerZone = 2500;
bool timerZoneGoal = false;
bool okGoal_1 = false;
bool okGoal_2 = false;
bool okGoal_3 = false;
bool firstEndStopLeft = true;

//V2
//mangueraCb
int controlMode = 0;

float Pose = 0.0;
//follower cb
int InputFollowerCb = 320;
//PID
double SetpointFollower, InputFollower, OutputFollower;
double aggKpF=57, aggKiF=902, aggKdF=9.5;
//double aggKp=2, aggKi=5, aggKd=0;
float pwmControl = 0.0;
PID followerPID(&InputFollower, &OutputFollower, &SetpointFollower, aggKpF, aggKiF, aggKdF, DIRECT); 
 
/*
 * CALLBACKS
 */
void messageCb(const geometry_msgs::Twist& msg){
  double x = msg.linear.x;
  auxiliar = msg.linear.x;
  double z = msg.angular.z;
  new_cmd_vel_timer = millis();

  if( is_stuck ){
    rps_req_RF = 0;
    rps_req_LF = 0;
    rps_req_RB = 0;
    rps_req_LB = 0; 
    if( x == 0 && z == 0){
      to_zero = true;
    } 
    else{
      if( to_zero == true){
        is_stuck = false;
        to_zero = false;
        warn_stuck = false;
        is_warned = false;
      }
      
    }
  }
  else{
    rps_req_RF = x/(3.1416*wheel_diameter) + z*track_width/(wheel_diameter*3.1416*2);
    rps_req_LF = x/(3.1416*wheel_diameter) - z*track_width/(wheel_diameter*3.1416*2);
    rps_req_RB = rps_req_RF;
    rps_req_LB = rps_req_LF;  
  }
  
}
void manualCb(const std_msgs::Bool& msg){
  manual_cmd = msg.data;
}
void cvArm_Cb(const std_msgs::Int8& msg) {
    double y =msg.data;
    stop_flag = false;
    if(y == 1){
      cmd_flag = 1;
    }
    else if(y == 2){
      cmd_flag = 2;
    }
    else if(y == 3){
      cmd_flag = 3;
    }
    else if (y == 4){
      cmd_flag = 4;
    }
    else if (y == -1){
      cmd_flag = -1;
      stop_flag = true;
    }
}
void cvTool_Cb(const std_msgs::Int8& msg) {
    startTool = msg.data;
    stop_flag = false;
}

void stopCb(const std_msgs::Bool& msg){
  if(hard_stop == false){
    stop_req = msg.data;
  }
}

ros::NodeHandle nh;
//TOPICOS MSGS
geometry_msgs::Twist encoder;
geometry_msgs::Twist pwm;
//tool:patin
std_msgs::Bool endStopRight;
std_msgs::Bool endStopLeft;
std_msgs::Bool endStopTool;
std_msgs::Bool emergency_stop;
std_msgs::Float32 distancetozone;
std_msgs::String armDiagnostics;
std_msgs::Float32 armPose;
std_msgs::Float32 Timer;
//V2
std_msgs::Int16 pwm_pid;
std_msgs::Float32 pose;
std_msgs::Int16 stuck;
// geometry_msgs::Vector3 Velo;

//TOPICOS
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &messageCb);
ros::Subscriber<std_msgs::Bool> sub_stop("stop", &stopCb);
ros::Publisher pwm_pub("/wheel_pwm", &pwm);

ros::Publisher emergency_stop_status_pub("emergency_stop_status", &emergency_stop);//
ros::Publisher distanceToZone_pub("distanceToZone", &distancetozone);//
ros::Publisher armDiagnostics_pub("armDiagnostics", &armDiagnostics);//
ros::Publisher armPose_pub("armPose", &armPose);
ros::Subscriber<std_msgs::Bool> sub_manual("manual_cmd", &manualCb);//
ros::Subscriber<std_msgs::Bool> sub_cvTool("cvTool_cmd", &cvTool_Cb);
ros::Subscriber<std_msgs::Int8> sub_cvArm("cvArm_cmd", &cvArm_Cb);
ros::Publisher endstopRight_pub("EndStopRight", &endStopRight);
ros::Publisher endstopLeft_pub("EndStopLeft", &endStopLeft);
ros::Publisher endstopTool_pub("EndStopTool", &endStopLeft);
ros::Publisher stuck_pub("wheel_stuck", &stuck);
ros::Publisher encoder_pub("wheel_encoders", &encoder);
// ros::Publisher velo_pub("velo", &Velo);

//V2

ros::Publisher pwmPID("/pwmPID", &pwm_pid);//
ros::Publisher pose_pub("/mansoPose", &pose);

void setup(){
  nh.initNode();
  nh.subscribe(sub_vel);
  // nh.advertise(velo_pub);
  nh.advertise(stuck_pub);
  nh.advertise(encoder_pub);
  nh.advertise(pwm_pub);
  nh.advertise(pose_pub);
  nh.advertise(endstopRight_pub);
  nh.advertise(endstopLeft_pub);
  nh.advertise(endstopTool_pub);
  nh.advertise(distanceToZone_pub);
  nh.advertise(armDiagnostics_pub);
  nh.advertise(armPose_pub);
  nh.subscribe(sub_manual);
  nh.subscribe(sub_cvArm);
  nh.subscribe(sub_cvTool);
  //V2
  nh.advertise(pwmPID);
  nh.advertise(emergency_stop_status_pub);
  //V3

  RFPID.SetMode(AUTOMATIC);
  RFPID.SetSampleTime(100);
  LFPID.SetMode(AUTOMATIC);
  LFPID.SetSampleTime(100);
  RBPID.SetMode(AUTOMATIC);
  RBPID.SetSampleTime(100);
  LBPID.SetMode(AUTOMATIC);
  LBPID.SetSampleTime(100);
  
  pinMode(c_RightFrontEncoderPinA, INPUT_PULLUP);      
  pinMode(c_RightFrontEncoderPinB, INPUT_PULLUP);
  attachInterrupt(c_RightFrontEncoderInterruptA, HandleRightFrontMotorInterruptA, CHANGE);
  attachInterrupt(c_RightFrontEncoderInterruptB, HandleRightFrontMotorInterruptB, CHANGE);
  pinMode(c_LeftFrontEncoderPinA, INPUT_PULLUP);      
  pinMode(c_LeftFrontEncoderPinB, INPUT_PULLUP);
  attachInterrupt(c_LeftFrontEncoderInterruptA, HandleLeftFrontMotorInterruptA, CHANGE);
  attachInterrupt(c_LeftFrontEncoderInterruptB, HandleLeftFrontMotorInterruptB, CHANGE);
  pinMode(c_RightBackEncoderPinA, INPUT_PULLUP);      
  pinMode(c_RightBackEncoderPinB, INPUT_PULLUP);
  attachPinChangeInterrupt(c_RightBackEncoderPinA, HandleRightBackMotorInterruptA, CHANGE);
  attachPinChangeInterrupt(c_RightBackEncoderPinB, HandleRightBackMotorInterruptB, CHANGE);
  pinMode(c_LeftBackEncoderPinA, INPUT_PULLUP);      
  pinMode(c_LeftBackEncoderPinB, INPUT_PULLUP);
  attachPinChangeInterrupt(c_LeftBackEncoderPinA, HandleLeftBackMotorInterruptA, CHANGE);
  attachPinChangeInterrupt(c_LeftBackEncoderPinB, HandleLeftBackMotorInterruptB, CHANGE);

  //TOOL:PATIN
  pinMode(c_ToolEndStopInterrupt, INPUT_PULLUP); 
  pinMode(c_LeftEndStopInterrupt, INPUT_PULLUP); 
  pinMode(c_RightEndStopInterrupt, INPUT_PULLUP);
  pinMode(c_StopInterrupt, INPUT_PULLUP);
  attachPinChangeInterrupt (c_ToolEndStopInterrupt, HandleToolEndStopInterrupt, CHANGE);
  attachPinChangeInterrupt (c_LeftEndStopInterrupt, HandleLeftEndStopInterrupt, CHANGE);
  attachPinChangeInterrupt (c_RightEndStopInterrupt, HandleRightEndStopInterrupt, CHANGE);
  attachPinChangeInterrupt (c_StopInterrupt, HandleStopInterrupt, CHANGE);
  
  pinMode(MOTOR_ARM_RPWM_PIN, OUTPUT);
  pinMode(MOTOR_ARM_LPWM_PIN, OUTPUT);
  pinMode(MOTOR_TOOL_RPWM_PIN, OUTPUT);
  pinMode(MOTOR_TOOL_LPWM_PIN, OUTPUT);

  pinMode(c_ArmYEncoderPinA, INPUT_PULLUP);      
  pinMode(c_ArmYEncoderPinB, INPUT_PULLUP);     
  attachInterrupt(c_ArmYEncoderInterruptA, HandleArmYMotorInterruptA, CHANGE);
  attachInterrupt(c_ArmYEncoderInterruptB, HandleArmYMotorInterruptB, CHANGE);

  tool_is_ok = !digitalRead(c_ToolEndStopInterrupt);
  arm_is_right = !digitalRead(c_RightEndStopInterrupt);
  arm_is_left = !digitalRead(c_LeftEndStopInterrupt);
  armSetup();

  //V2
  followerPID.SetOutputLimits(-150, 150);
  followerPID.SetMode(AUTOMATIC);
  followerPID.SetSampleTime(100);
  SetpointFollower = 320;
}
void loop(){
  PubArmPose();
  if(millis() - new_cmd_vel_timer > 1000){// Hombre a tierra, seguro por desconexion
    rps_req_RF = 0;           
    rps_req_LF = 0;
    rps_req_RB = 0;           
    rps_req_LB = 0;
    nh.spinOnce();
  }
  if ((millis()-timer) > 100){
    nh.spinOnce();
    PubArmPose();
    motorGoDIR();
    encoderMotor(RFMotor);
    encoderMotor(LFMotor);
    encoderMotor(RBMotor);
    encoderMotor(LBMotor);
    timer_encoder = millis();
    encoderFilters();
    encoderPub();
    // veloPub();
    if(stop_req == false){

      rps_req_RF_cmd = rps_req_RF;
      rps_req_LF_cmd = rps_req_LF;
      rps_req_RB_cmd = rps_req_RB;
      rps_req_LB_cmd = rps_req_LB;
      accelerationRamp();
      PIDloop();
      pwmPub();

      
      timer = millis();
      /*
      * TOOL: PATIN
      */
      //PubArmPose();
      endStopRight.data = arm_is_right;
      endstopRight_pub.publish ( &endStopRight );
      endStopLeft.data = arm_is_left;
      endstopLeft_pub.publish ( &endStopLeft );
      endStopTool.data = tool_is_ok;
      endstopTool_pub.publish ( &endStopTool );
      if(arm_is_left == true){
        _ArmYEncoderTicks =  0;
        pwm_tool = 2;
      }
      if(arm_is_right == true){
        _ArmYEncoderTicks =  177; 
        pwm_tool = 1;
      }
      CV();
    //armCommand();
      stop_req = digitalRead(c_StopInterrupt);
      hard_stop = digitalRead(c_StopInterrupt);
      emergency_stop.data = stop_req;
      emergency_stop_status_pub.publish( &emergency_stop);
      nh.spinOnce();
    }
    else{
      stop_req = digitalRead(c_StopInterrupt);
      hard_stop = digitalRead(c_StopInterrupt);
      emergency_stop.data = stop_req;
      emergency_stop_status_pub.publish( &emergency_stop);
    }
  
  }
}
void motorGoDIR (){
  if(rps_req_RF>0){
    DIR_R = 2;
  }
  else if (rps_req_RF<0){
    DIR_R = 1;
  }
  if(rps_req_LF>0){
    DIR_L = 1;
  }
  else if (rps_req_LF<0){
    DIR_L = 2;
  }
}
void encoderMotor(uint8_t encoder){
  if(encoder == 0){
    sum_RightFront_encoder_ticks =+ _RightFrontEncoderTicks;
    rev_RightFront_wheel = sum_RightFront_encoder_ticks/2400;
    delta_timer =  millis()-timer_encoder;
    rps_RF = ((_RightFrontEncoderTicks/2400.0*1000)/(delta_timer))*15/35;
    _RightFrontEncoderTicks = 0;
  }
  if(encoder == 1){
    sum_LeftFront_encoder_ticks =+ _LeftFrontEncoderTicks;
    rev_LeftFront_wheel = sum_LeftFront_encoder_ticks/2400;
    delta_timer =  millis()-timer_encoder;
    rps_LF = (((_LeftFrontEncoderTicks/2400.0)*1000)/(delta_timer))*15/35;
    _LeftFrontEncoderTicks = 0;
  }
  if(encoder == 2){
    sum_RightBack_encoder_ticks =+ _RightBackEncoderTicks;
    rev_RightBack_wheel = sum_RightBack_encoder_ticks/2400;
    delta_timer =  millis()-timer_encoder;
    rps_RB = (((_RightBackEncoderTicks/2400.0)*1000)/(delta_timer))*15/35;
    _RightBackEncoderTicks = 0;
  }
  if(encoder == 3){
    sum_LeftBack_encoder_ticks =+ _LeftBackEncoderTicks;
    rev_LeftBack_wheel = sum_LeftBack_encoder_ticks/2400;
    delta_timer =  millis()-timer_encoder;
    rps_LB = (((_LeftBackEncoderTicks/2400.0)*1000)/(delta_timer))*15/35;
    _LeftBackEncoderTicks = 0;
  }
}
void encoderFilters (){
  if( abs(rps_RF) < rps_limit ){
    rps_RFL = rps_RF; 
  }
  else{
    rps_RF = rps_RFL;
    sum_error = sum_error + 1;
  }
  if( abs(rps_LF) < rps_limit ){
    rps_LFL = rps_LF; 
  }
  else{
    rps_LF = rps_LFL;
    sum_error = sum_error + 1;
  }
  if( abs(rps_RB) < rps_limit ){
    rps_RBL = rps_RB; 
  }
  else{
    rps_RB = rps_RBL;
    sum_error = sum_error + 1;
  }
  if( abs(rps_LB) < rps_limit ){
    rps_LBL = rps_LB; 
  }
  else{
    rps_LB = rps_LBL;
    sum_error = sum_error + 1;
  }
}
void encoderPub(){
  vel_RF = rps_RF*3.1416*wheel_diameter;
  vel_LF = rps_LF*3.1416*wheel_diameter;
  vel_RB = rps_RB*3.1416*wheel_diameter;
  vel_LB = rps_LB*3.1416*wheel_diameter;
  encoder.linear.x = vel_RF;
  encoder.linear.y = vel_LF;
  encoder.linear.z = vel_RB;
  encoder.angular.x = vel_LB;
  encoder_pub.publish( &encoder );  
}
// void veloPub(){
//   Velo.x = vel_RB;
//   Velo.y = abs(vel_LB);
//   velo_pub.publish ( &Velo );
// }
void accelerationRamp(){
  if(abs(rps_req_RF-rps_req_RF_last) > 0.2 && accel_control == true){
    rps_req_RF_cmd = rps_req_RF_last + ((rps_req_RF - rps_req_RF_last)/abs(rps_req_RF - rps_req_RF_last))*0.2;   
  }
  rps_req_RF_last = rps_req_RF_cmd;
  if(abs(rps_req_LF-rps_req_LF_last) > 0.2 && accel_control == true){
    rps_req_LF_cmd = rps_req_LF_last + ((rps_req_LF - rps_req_LF_last)/abs(rps_req_LF - rps_req_LF_last))*0.2;   
  }
  rps_req_LF_last = rps_req_LF_cmd;
  if(abs(rps_req_RB-rps_req_RB_last) > 0.2 && accel_control == true){
    rps_req_RB_cmd = rps_req_RB_last + ((rps_req_RB - rps_req_RB_last)/abs(rps_req_RB - rps_req_RB_last))*0.2;   
  }
  rps_req_RB_last = rps_req_RB_cmd;
  if(abs(rps_req_LB-rps_req_LB_last) > 0.2 && accel_control == true){
    rps_req_LB_cmd = rps_req_LB_last + ((rps_req_LB - rps_req_LB_last)/abs(rps_req_LB - rps_req_LB_last))*0.2;   
  }
  rps_req_LB_last = rps_req_LB_cmd;
}
void PIDloop(){
  InputRF = abs(rps_RF);
  SetpointRF = abs(rps_req_RF_cmd);
  RFPID.SetTunings(aggKp, aggKi, aggKd);
  RFPID.Compute();
  velRFpwm = OutputRF;
  VEL_RF_PWM = round(velRFpwm);

  InputLF = abs(rps_LF);
  SetpointLF = abs(rps_req_LF_cmd);
  LFPID.SetTunings(aggKp, aggKi, aggKd);
  LFPID.Compute();
  velLFpwm = OutputLF;
  VEL_LF_PWM = round(velLFpwm);
  
  InputRB = abs(rps_RB);
  SetpointRB = abs(rps_req_RB_cmd);
  RBPID.SetTunings(aggKp, aggKi, aggKd);
  RBPID.Compute();
  velRBpwm = OutputRB;
  VEL_RB_PWM = round(velRBpwm);
  
  InputLB = abs(rps_LB);
  SetpointLB = abs(rps_req_LB_cmd);
  LBPID.SetTunings(aggKp, aggKi, aggKd);
  LBPID.Compute();
  velLBpwm = OutputLB;
  VEL_LB_PWM = round(velLBpwm);
}
void pwmPub(){

  if( vel_RF != 0 && vel_RB != 0 && vel_LF != 0 && vel_LB != 0){
    warn_stuck = false;
    is_warned = false;
    stuck.data = 0;
  }
  if(abs(vel_RF) == 0 && abs(VEL_RF_PWM) > 0){
    warn_stuck = true;
    stuck.data += 1000;
  }
  else if(abs(vel_RB) == 0 && abs(VEL_RB_PWM) > 0 ){
    warn_stuck = true;
    stuck.data += 100;
  }
  else if(abs(vel_LF) == 0 && abs(VEL_LF_PWM) > 0 ){
    warn_stuck = true;
    stuck.data += 10;
  }
  else if(abs(vel_LB) == 0 && abs(VEL_LB_PWM) > 0){
    warn_stuck = true;
    stuck.data += 1;
  }
  if(warn_stuck && is_warned == false){
    timeout_stuck_timer = millis();
    is_warned = true;
  }
  if( is_warned && (millis() - timeout_stuck_timer) > timeout_stuck){
    is_stuck = true;
  }
  if(is_stuck){
    pwm.linear.x = 0;
    pwm.linear.y = 0;
    pwm.linear.z = 0;
    pwm.angular.x = 0;
    pwm.angular.y = DIR_R;
    pwm.angular.z = DIR_L;   
  }
  else{
    pwm.linear.x = VEL_RF_PWM;
    pwm.linear.y = VEL_LF_PWM;
    pwm.linear.z = VEL_RB_PWM;
    pwm.angular.x = VEL_LB_PWM;
    pwm.angular.y = DIR_R;
    pwm.angular.z = DIR_L;
  }
  stuck_pub.publish( &stuck );
  pwm_pub.publish( &pwm );
}
/*
 * TOOL: PATIN
 */
void PubArmPose(){
  armPose.data = _ArmYEncoderTicks;//(_ArmYEncoderTicks/2400.0)*(2*3.1416)*LenghtArm;
  armPose_pub.publish(&armPose);
  nh.spinOnce();
}
void armSetup(){
  while(arm_is_left == false && stop_flag == false){
    if(millis()- timer > 100){
      timer = millis();
      motorGo(0, ARM_LEFT, 150);
      armDiagnostics.data =  " Setting Arm...";
      armDiagnostics_pub.publish( &armDiagnostics );
      nh.spinOnce();
    }
  }
  DIR = ARM_RIGHT;
  motorGo(0, DIR, 0);
  armDiagnostics.data = "Arm In Position";
  armDiagnostics_pub.publish ( &armDiagnostics );
  _ArmYEncoderTicks = 0;
  arm_pose = 0.0;
  //motorH.data = arm_is_left;
  //motorH_pub.publish(&motorH);
  while(tool_is_ok == false && stop_flag == false){
    armDiagnostics.data =  " Setting Tool...";
    armDiagnostics_pub.publish( &armDiagnostics );
    motorGo(1, ARM_LEFT, 100);
    nh.spinOnce();
  }
  motorGo(1, ARM_LEFT, 0);
  armDiagnostics.data = "Tool In Position";
  armDiagnostics_pub.publish ( &armDiagnostics );
  nh.spinOnce();
  firstEndStopLeft = true;
}
void armCommand(){
  //Mover a la izquierda
  if(cmd_flag == 1){
    pwm_tool = 1;
    while(arm_is_left == false && pwm_tool == 1 && stop_flag == false){ 
      motorGo(0, pwm_tool, 150);
      PubArmPose();
    }
  motorGo(0,0,0);
  cmd_flag = 0;
  }
  //Mover al medio
  if(cmd_flag == 2){
    move_is_ok = false;
    while(move_is_ok == false && stop_flag == false){
      MoveArm(middle);
      PubArmPose();
    }
    cmd_flag = 0;
  }
  //Mover a la derecha
  if(cmd_flag == 3){
    pwm_tool = 2;
    while(arm_is_right == false && pwm_tool == 2 && stop_flag == false){ 
      motorGo(0, pwm_tool, 150);
      PubArmPose();
    }
    motorGo(0,0,0);
    cmd_flag = 0;
  }
  //Accionar el sacayuyo
  if(cmd_flag == 4){
    tool();
    cmd_flag = 0;
  }
  if (cmd_flag == 0){
    motorGo(0,0,0);
  }
}
void CV(){
  //Mover a la izquierda
  if(cmd_flag == 1){
    pwm_tool = 1;
    if (firstEndStopLeft == true){
      MoveArm(left);
      firstEndStopLeft = false;
    }
    if(arm_is_left == false && pwm_tool == 1 && stop_flag == false && firstEndStopLeft == false){ 
      move_is_ok = false;
      if(move_is_ok == false && stop_flag == false){
        MoveArm(left);
        //PubArmPose();
      }
    }
  //motorGo(0,0,0);
  //cmd_flag = 0;
  }
  //Mover al medio
  if(cmd_flag == 2){
    move_is_ok = false;
    if(move_is_ok == false && stop_flag == false){
      MoveArm(middle);
      //PubArmPose();
    }
    //cmd_flag = 0;
  }
  //Mover a la derecha
  if(cmd_flag == 3){
    move_is_ok = false;
    pwm_tool = 2;
    if(arm_is_right == false && pwm_tool == 2 && stop_flag == false){ 
      if(move_is_ok == false && stop_flag == false){
        MoveArm(right);
        //PubArmPose();
      }
    }
    //motorGo(0,0,0);
    //cmd_flag = 0;
  }
  //Accionar el sacayuyo
  if(startTool == true){
    tool();
    //cmd_flag = 0;
  }
  if (cmd_flag == -1){
    motorGo(0,0,0);
    startTool == false;
  }
}
void MoveArm(int ticks){
  motorGo(1, 0, 0);
  if (timerClock == false){
    arm_pose = _ArmYEncoderTicks;//(_ArmYEncoderTicks/2400.0)*(2*3.1416)*LenghtArm;
    distanceToZone = ticks - arm_pose;
    distancetozone.data = distanceToZone;
    distanceToZone_pub.publish( &distancetozone );
    if(abs(distanceToZone) <= precision){
      motorGo(0, ARM_RIGHT, 0);
      timerClock = true;
      timeZone = millis();
      move_is_ok = true;
    }
    else{
      if(distanceToZone > 0){
        DIR = 2;
        motorGo(0, 2, 80);
        armDiagnostics.data = " Arm moving to RIGHT";
        armDiagnostics_pub.publish( &armDiagnostics );
      }
      else if(distanceToZone < 0){
        DIR = 1;
        motorGo(0, 1, 80);
        armDiagnostics.data = " Arm moving to LEFT";
        armDiagnostics_pub.publish( &armDiagnostics );
      }
    }
    //DIRmotorGo();
    //motorGo(0, DIR, 80);
  }
  if (timerClock == true){
    nh.spinOnce();
    Timer.data = millis() - timeZone;
    //timer_pub.publish( &Timer );
    if ( millis() - timeZone < timerZone ){
      armDiagnostics.data = "Arm in Position";
      armDiagnostics_pub.publish( &armDiagnostics );
      //startTool = true;
    }
    if ( millis() - timeZone > timerZone ){
      armDiagnostics.data = "timer end, move to next goal";
      armDiagnostics_pub.publish( &armDiagnostics );
      okGoal_1 = true;
      timerClock = false;
      //startTool = true;
    }
  }
  nh.spinOnce();
}  
void tool(){
  if(startTool == true){
    motorGo(0, 0, 0);
    while (tool_is_ok == true){
      armDiagnostics.data = " patin ON 1";
      armDiagnostics_pub.publish ( &armDiagnostics );
      motorGo(1, ARM_LEFT, 254);
      PubArmPose();
    }
    while (tool_is_ok == false){
      motorGo(1, ARM_LEFT, 254);
      armDiagnostics.data = " patin ON 2";
      armDiagnostics_pub.publish ( &armDiagnostics );
      PubArmPose();
      //startTool = false;
      PubArmPose();
    }
    if (tool_is_ok == true){
      motorGo(1, ARM_LEFT, 0);
      armDiagnostics.data = " patin OFF";
      armDiagnostics_pub.publish ( &armDiagnostics );
      PubArmPose();
    }
  /*while (millis() - timerTool < 2000){
    armDiagnostics.data = " timer";
    armDiagnostics_pub.publish ( &armDiagnostics );
    nh.spinOnce();
  }*/
  startTool = false;
  }
}
void DIRmotorGo(){
  if(distanceToZone > 0){
    DIR = 2;
    armDiagnostics.data = " Arm moving to RIGHT";
    armDiagnostics_pub.publish( &armDiagnostics );
  }
  else if(distanceToZone < 0){
    DIR = 1;
    armDiagnostics.data = " Arm moving to LEFT";
    armDiagnostics_pub.publish( &armDiagnostics );
  }
}
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm){         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
  if(motor == 0){
    if(direct == 1){
      analogWrite(MOTOR_ARM_LPWM_PIN, 0); 
      analogWrite(MOTOR_ARM_RPWM_PIN, pwm);
    }
    else if(direct == 2){
      analogWrite(MOTOR_ARM_RPWM_PIN, 0); 
      analogWrite(MOTOR_ARM_LPWM_PIN, pwm);      
    }
    else{
      analogWrite(MOTOR_ARM_LPWM_PIN, 0); 
      analogWrite(MOTOR_ARM_RPWM_PIN, 0);            
    }
  }
  if(motor == 1){
    if(direct == 1){
      analogWrite(MOTOR_TOOL_LPWM_PIN, 0); 
      analogWrite(MOTOR_TOOL_RPWM_PIN, pwm);
    }
    else if(direct == 2){
      analogWrite(MOTOR_TOOL_RPWM_PIN, 0); 
      analogWrite(MOTOR_TOOL_LPWM_PIN, pwm);      
    }
    else{
      analogWrite(MOTOR_TOOL_LPWM_PIN, 0); 
      analogWrite(MOTOR_TOOL_RPWM_PIN, 0);            
    }
  }
}
/*
 * ENCODERS
 */
void HandleRightFrontMotorInterruptA(){
  _RightFrontEncoderBSet = digitalReadFast(c_RightFrontEncoderPinB);
  _RightFrontEncoderASet = digitalReadFast(c_RightFrontEncoderPinA);
  
  _RightFrontEncoderTicks+=RightFrontParseEncoder();
  
  _RightFrontEncoderAPrev = _RightFrontEncoderASet;
  _RightFrontEncoderBPrev = _RightFrontEncoderBSet;
}
void HandleRightFrontMotorInterruptB(){
  // Test transition;
  _RightFrontEncoderBSet = digitalReadFast(c_RightFrontEncoderPinB);
  _RightFrontEncoderASet = digitalReadFast(c_RightFrontEncoderPinA);
  
  _RightFrontEncoderTicks+=RightFrontParseEncoder();
  
  _RightFrontEncoderAPrev = _RightFrontEncoderASet;
  _RightFrontEncoderBPrev = _RightFrontEncoderBSet;
}
int RightFrontParseEncoder(){
  if(_RightFrontEncoderAPrev && _RightFrontEncoderBPrev){
    if(!_RightFrontEncoderASet && _RightFrontEncoderBSet) return 1;
    if(_RightFrontEncoderASet && !_RightFrontEncoderBSet) return -1;
  }else if(!_RightFrontEncoderAPrev && _RightFrontEncoderBPrev){
    if(!_RightFrontEncoderASet && !_RightFrontEncoderBSet) return 1;
    if(_RightFrontEncoderASet && _RightFrontEncoderBSet) return -1;
  }else if(!_RightFrontEncoderAPrev && !_RightFrontEncoderBPrev){
    if(_RightFrontEncoderASet && !_RightFrontEncoderBSet) return 1;
    if(!_RightFrontEncoderASet && _RightFrontEncoderBSet) return -1;
  }else if(_RightFrontEncoderAPrev && !_RightFrontEncoderBPrev){
    if(_RightFrontEncoderASet && _RightFrontEncoderBSet) return 1;
    if(!_RightFrontEncoderASet && !_RightFrontEncoderBSet) return -1;
  }
}
void HandleLeftFrontMotorInterruptA(){
  _LeftFrontEncoderBSet = digitalReadFast(c_LeftFrontEncoderPinB);
  _LeftFrontEncoderASet = digitalReadFast(c_LeftFrontEncoderPinA);
  
  _LeftFrontEncoderTicks+=LeftFrontParseEncoder();
  
  _LeftFrontEncoderAPrev = _LeftFrontEncoderASet;
  _LeftFrontEncoderBPrev = _LeftFrontEncoderBSet;
}
// Interrupt service routines for the Left motor's quadrature encoder
void HandleLeftFrontMotorInterruptB(){
  // Test transition;
  _LeftFrontEncoderBSet = digitalReadFast(c_LeftFrontEncoderPinB);
  _LeftFrontEncoderASet = digitalReadFast(c_LeftFrontEncoderPinA);
  
  _LeftFrontEncoderTicks+=LeftFrontParseEncoder();
  
  _LeftFrontEncoderAPrev = _LeftFrontEncoderASet;
  _LeftFrontEncoderBPrev = _LeftFrontEncoderBSet;
}
int LeftFrontParseEncoder(){
  if(_LeftFrontEncoderAPrev && _LeftFrontEncoderBPrev){
    if(!_LeftFrontEncoderASet && _LeftFrontEncoderBSet) return 1;
    if(_LeftFrontEncoderASet && !_LeftFrontEncoderBSet) return -1;
  }else if(!_LeftFrontEncoderAPrev && _LeftFrontEncoderBPrev){
    if(!_LeftFrontEncoderASet && !_LeftFrontEncoderBSet) return 1;
    if(_LeftFrontEncoderASet && _LeftFrontEncoderBSet) return -1;
  }else if(!_LeftFrontEncoderAPrev && !_LeftFrontEncoderBPrev){
    if(_LeftFrontEncoderASet && !_LeftFrontEncoderBSet) return 1;
    if(!_LeftFrontEncoderASet && _LeftFrontEncoderBSet) return -1;
  }else if(_LeftFrontEncoderAPrev && !_LeftFrontEncoderBPrev){
    if(_LeftFrontEncoderASet && _LeftFrontEncoderBSet) return 1;
    if(!_LeftFrontEncoderASet && !_LeftFrontEncoderBSet) return -1;
  }
}
void HandleRightBackMotorInterruptA(){
  _RightBackEncoderBSet = digitalReadFast(c_RightBackEncoderPinB);
  _RightBackEncoderASet = digitalReadFast(c_RightBackEncoderPinA);
  
  _RightBackEncoderTicks+=RightBackParseEncoder();
  
  _RightBackEncoderAPrev = _RightBackEncoderASet;
  _RightBackEncoderBPrev = _RightBackEncoderBSet;
}
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightBackMotorInterruptB(){
  // Test transition;
  _RightBackEncoderBSet = digitalReadFast(c_RightBackEncoderPinB);
  _RightBackEncoderASet = digitalReadFast(c_RightBackEncoderPinA);
  
  _RightBackEncoderTicks+=RightBackParseEncoder();
  
  _RightBackEncoderAPrev = _RightBackEncoderASet;
  _RightBackEncoderBPrev = _RightBackEncoderBSet;
}
int RightBackParseEncoder(){
  if(_RightBackEncoderAPrev && _RightBackEncoderBPrev){
    if(!_RightBackEncoderASet && _RightBackEncoderBSet) return 1;
    if(_RightBackEncoderASet && !_RightBackEncoderBSet) return -1;
  }else if(!_RightBackEncoderAPrev && _RightBackEncoderBPrev){
    if(!_RightBackEncoderASet && !_RightBackEncoderBSet) return 1;
    if(_RightBackEncoderASet && _RightBackEncoderBSet) return -1;
  }else if(!_RightBackEncoderAPrev && !_RightBackEncoderBPrev){
    if(_RightBackEncoderASet && !_RightBackEncoderBSet) return 1;
    if(!_RightBackEncoderASet && _RightBackEncoderBSet) return -1;
  }else if(_RightBackEncoderAPrev && !_RightBackEncoderBPrev){
    if(_RightBackEncoderASet && _RightBackEncoderBSet) return 1;
    if(!_RightBackEncoderASet && !_RightBackEncoderBSet) return -1;
  }
}
//----------------------
void HandleLeftBackMotorInterruptA(){
  _LeftBackEncoderBSet = digitalReadFast(c_LeftBackEncoderPinB);
  _LeftBackEncoderASet = digitalReadFast(c_LeftBackEncoderPinA);
  
  _LeftBackEncoderTicks+=LeftBackParseEncoder();
  
  _LeftBackEncoderAPrev = _LeftBackEncoderASet;
  _LeftBackEncoderBPrev = _LeftBackEncoderBSet;
}
// Interrupt service routines for the Left motor's quadrature encoder
void HandleLeftBackMotorInterruptB(){
  // Test transition;
  _LeftBackEncoderBSet = digitalReadFast(c_LeftBackEncoderPinB);
  _LeftBackEncoderASet = digitalReadFast(c_LeftBackEncoderPinA);
  
  _LeftBackEncoderTicks+=LeftBackParseEncoder();
  
  _LeftBackEncoderAPrev = _LeftBackEncoderASet;
  _LeftBackEncoderBPrev = _LeftBackEncoderBSet;
}
int LeftBackParseEncoder(){
  if(_LeftBackEncoderAPrev && _LeftBackEncoderBPrev){
    if(!_LeftBackEncoderASet && _LeftBackEncoderBSet) return 1;
    if(_LeftBackEncoderASet && !_LeftBackEncoderBSet) return -1;
  }else if(!_LeftBackEncoderAPrev && _LeftBackEncoderBPrev){
    if(!_LeftBackEncoderASet && !_LeftBackEncoderBSet) return 1;
    if(_LeftBackEncoderASet && _LeftBackEncoderBSet) return -1;
  }else if(!_LeftBackEncoderAPrev && !_LeftBackEncoderBPrev){
    if(_LeftBackEncoderASet && !_LeftBackEncoderBSet) return 1;
    if(!_LeftBackEncoderASet && _LeftBackEncoderBSet) return -1;
  }else if(_LeftBackEncoderAPrev && !_LeftBackEncoderBPrev){
    if(_LeftBackEncoderASet && _LeftBackEncoderBSet) return 1;
    if(!_LeftBackEncoderASet && !_LeftBackEncoderBSet) return -1;
  }
}
//TOOL: PATIN
void HandleArmYMotorInterruptA(){
  _ArmYEncoderBSet = digitalReadFast(c_ArmYEncoderPinB);
  _ArmYEncoderASet = digitalReadFast(c_ArmYEncoderPinA);
  
  _ArmYEncoderTicks+=ArmYParseEncoder();
  
  _ArmYEncoderAPrev = _ArmYEncoderASet;
  _ArmYEncoderBPrev = _ArmYEncoderBSet;
}
void HandleArmYMotorInterruptB(){
  _ArmYEncoderBSet = digitalReadFast(c_ArmYEncoderPinB);
  _ArmYEncoderASet = digitalReadFast(c_ArmYEncoderPinA);
  
  _ArmYEncoderTicks+=ArmYParseEncoder();
  
  _ArmYEncoderAPrev = _ArmYEncoderASet;
  _ArmYEncoderBPrev = _ArmYEncoderBSet;
}
int ArmYParseEncoder(){
  if(_ArmYEncoderAPrev && _ArmYEncoderBPrev){
    if(!_ArmYEncoderASet && _ArmYEncoderBSet) return 1;
    if(_ArmYEncoderASet && !_ArmYEncoderBSet) return -1;
  }else if(!_ArmYEncoderAPrev && _ArmYEncoderBPrev){
    if(!_ArmYEncoderASet && !_ArmYEncoderBSet) return 1;
    if(_ArmYEncoderASet && _ArmYEncoderBSet) return -1;
  }else if(!_ArmYEncoderAPrev && !_ArmYEncoderBPrev){
    if(_ArmYEncoderASet && !_ArmYEncoderBSet) return 1;
    if(!_ArmYEncoderASet && _ArmYEncoderBSet) return -1;
  }else if(_ArmYEncoderAPrev && !_ArmYEncoderBPrev){
    if(_ArmYEncoderASet && _ArmYEncoderBSet) return 1;
    if(!_ArmYEncoderASet && !_ArmYEncoderBSet) return -1;
  }
}
void HandleToolEndStopInterrupt(){
  tool_is_ok = !digitalRead(c_ToolEndStopInterrupt);
}
void HandleRightEndStopInterrupt(){
  arm_is_right = !digitalRead(c_RightEndStopInterrupt);
  if(arm_is_right == true){
    motorGo(0, ARM_LEFT, 0);
  }
}
void HandleLeftEndStopInterrupt(){
  arm_is_left = !digitalRead(c_LeftEndStopInterrupt);
  if(arm_is_left == true){
    motorGo(0, ARM_RIGHT, 0);
  }
}

void HandleStopInterrupt(){
  stop_req = digitalRead(c_StopInterrupt);
  hard_stop = digitalRead(c_StopInterrupt);
}
