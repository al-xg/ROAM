// PinChangeIntExample, version 1.1 Sun Jan 15 06:24:19 CST 2012
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

#include "PinChangeInt.h"

uint8_t latest_interrupted_pin;
unsigned long report_time;
unsigned long work_time;
unsigned long tmp_time;
unsigned int tmmpa;
unsigned int tmmpb;

volatile uint16_t interrupt_count[100]={
  0}; // 100 possible arduino pins
volatile uint16_t interrupt_start[100]={
  0}; // 100 possible arduino pins
unsigned long timestart=0;
//int16_t
// unThrottleInStart = TCNT1;

#include "Wire.h"

//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
const byte FrontI2C=0x70;
const byte RearI2C=0x71;
//byte LeftI2C=0x72;
//byte RightI2C=0x73;
//const byte SensorAddress[6];

//The Sensor ranging command has a value of 0x51
const byte RangeCommand=0x51;

//Commands the sensors to take a range reading
void takeRangeReading(byte SensorAddress){
  Wire.beginTransmission(SensorAddress);             //Start addressing 
  Wire.write(RangeCommand);                          //send range command 
  Wire.endTransmission();
}    

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication. 
word requestRange(byte SensorAddress){ 
  Wire.requestFrom(SensorAddress, byte(2));
  if(Wire.available() >= 2){                //Sensor responded with the two bytes 
    byte HighByte = Wire.read();            //Read the high byte back 
    byte LowByte = Wire.read();             //Read the low byte back 
    word Range = word(HighByte, LowByte);   //Make a 16-bit word out of the two bytes for the range 
    return Range; 
  }
  else return word(0); //Else nothing was received, return 0
}


//Meguno Link
char channelName[ ] = "debug";


#include "PID_v1.h"

//Define Variables we'll be connecting to
double Setpoint_right, right_sonar, Output_right;
double Setpoint_front, front_sonar, Output_front;
double Setpoint_left, left_sonar, Output_left;
double Setpoint_rear, rear_sonar, Output_rear;
const double PIDSampleTime=100; //interval in ms
const double safe_distance=90; //value in cm
const double OutMax=600; //value in ms
const double P=7;
const double I=3;
const double D=0;

//Specify the links and initial tuning parameters
PID PID_right(&right_sonar, &Output_right, &Setpoint_right,P,I,D, DIRECT);
PID PID_front(&front_sonar, &Output_front, &Setpoint_front,P,I,D, DIRECT);
PID PID_left(&left_sonar, &Output_left, &Setpoint_left,P,I,D, DIRECT);
PID PID_rear(&rear_sonar, &Output_rear, &Setpoint_rear,P,I,D, DIRECT);


#include "RCArduinoFastLib.h"

// Assign your channel out pins
#define chanel1_OUT_PIN PIN6
#define chanel2_OUT_PIN PIN5
#define chanel3_OUT_PIN PIN4
#define chanel4_OUT_PIN PIN3
//#define chanel5_OUT_PIN PIN2

// Assign servo indexes
#define chanel1_INDEX 0
#define chanel2_INDEX 1
#define chanel3_INDEX 2
#define chanel4_INDEX 3
#define SERVO_FRAME_SPACE 4

//PWM Sonars
int trigger= 13;
//int piezzo=12;

void Sonar_pulse(){
  digitalWrite(trigger,LOW);
  digitalWrite(trigger,HIGH);
  delayMicroseconds(20);
  digitalWrite(trigger,LOW);
}

//RC Inputs/outputs
double pitch_in=1500;
double roll_in=1500;
double throttle_in=900;
double mode_switch=900;
double aux1=900;
double aux2=900;
int compd_pitch, compd_roll, compd_throttle;

int ConstrainPWM(int PWM_out, int MinPW, int MaxPW){
    if (PWM_out<MinPW) PWM_out=MinPW;
    if (PWM_out>MaxPW) PWM_out=MaxPW;
   return PWM_out;
}

#include "SatelliteReceiver.h"
SatelliteReceiver Rx;

void GetSpektrum(){
  Rx.getFrame();
  //roll_in=Rx.getAile();
  //pitch_in=Rx.getElev();
  //throttle_in=Rx.getThro();
  //yaw_in=Rx.getRudd();
  //mode_switch=Rx.getFlap();
  //aux1= Rx.getGear();
    aux2= Rx.getAux2();
}


void quicfunc() {
  if(1==PCintPort::pinState){
    interrupt_start[PCintPort::arduinoPin]=TCNT1;
  }
  else{
    if (interrupt_start[PCintPort::arduinoPin]<TCNT1){
      interrupt_count[PCintPort::arduinoPin]=(TCNT1-interrupt_start[PCintPort::arduinoPin])>>1;
    }
    else {
      tmmpa=interrupt_start[PCintPort::arduinoPin]+32768;
      tmmpb=TCNT1+32768;
      interrupt_count[PCintPort::arduinoPin]=(tmmpb-tmmpa)>>1;
    }
  }
}

void PCpin(int pin){
  pinMode(pin,INPUT);
  digitalWrite(pin, LOW);
  PCintPort::attachInterrupt(pin, &quicfunc, CHANGE);
}


void setup() {
  Serial.begin(57600);
  //Serial1.begin(115200); //Spektrum serial
  
  pinMode(trigger,OUTPUT);   //Trigger pin for PWM Sonars
  digitalWrite(trigger,LOW);

  //Mega sensors
    //PCpin(A8);//62
    //PCpin(A9);
    //PCpin(A10);
    //PCpin(A11);
    //PCpin(A12);
    //PCpin(A13);
    //PCpin(A14);
    //PCpin(A15);//69

  //Micro Sensors
    PCpin(11);
    //PCpin(10);
    PCpin(9);
    //PCpin(8);

  //RC inputs
    //PCpin(50);
    //PCpin(51);
    //PCpin(52);
    //PCpin(53);
  
  //Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
  Wire.begin();
  takeRangeReading(FrontI2C);
  takeRangeReading(RearI2C);
  
  //Set up PWM outputs
  pinMode(chanel1_OUT_PIN,OUTPUT);
  pinMode(chanel2_OUT_PIN,OUTPUT);
  pinMode(chanel3_OUT_PIN,OUTPUT);
  pinMode(chanel4_OUT_PIN,OUTPUT);

  CRCArduinoFastServos::attach(chanel1_INDEX,chanel1_OUT_PIN);
  CRCArduinoFastServos::attach(chanel2_INDEX,chanel2_OUT_PIN);
  CRCArduinoFastServos::attach(chanel3_INDEX,chanel3_OUT_PIN);
  CRCArduinoFastServos::attach(chanel4_INDEX,chanel4_OUT_PIN);

  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);
  CRCArduinoFastServos::begin();
  

  //Initialise PID loops
  Setpoint_right= Setpoint_front = Setpoint_left = Setpoint_rear = safe_distance;
  
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(PIDSampleTime);
  PID_right.SetOutputLimits(0,OutMax);
  
  PID_front.SetMode(AUTOMATIC);
  PID_front.SetSampleTime(PIDSampleTime);
  PID_front.SetOutputLimits(0,OutMax);
 
  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(PIDSampleTime);
  PID_left.SetOutputLimits(0,OutMax);
  
  PID_rear.SetMode(AUTOMATIC);
  PID_rear.SetSampleTime(PIDSampleTime);
  PID_rear.SetOutputLimits(0,OutMax);
  //pinMode(piezzo,OUTPUT); //Buzzer for PID output, find unused pin

  
  //Intialise loop timers
  report_time	= millis();
  work_time	= millis();
}

void report(){
  report_time = millis();

  Serial.print(F("{TIMEPLOT:PID|data|Output_right|T|"));
  Serial.print(Output_right);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|Output_left|T|"));
  Serial.print(Output_left);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|safe_distance|T|"));
  Serial.print(safe_distance);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|RightSonar|T|"));
  Serial.print(right_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|FrontSonar|T|"));
  Serial.print(front_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|RearSonar|T|"));
  Serial.print(rear_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|LeftSonar|T|"));
  Serial.print(left_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|AileronOut|T|"));
  Serial.print(compd_roll);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|Aileron|T|"));
  Serial.print(roll_in);
  Serial.println(F("}"));

  /*Serial.print(F("{TIMEPLOT:PID|data|Aux1|T|"));
   Serial.print(aux1);
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:PIDsettings|data|Kd|T|"));
   Serial.print(PID_right.GetKd());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:PIDsettings|data|Ki|T|"));
   Serial.print(PID_right.GetKi());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:PIDsettings|data|Kp|T|"));
   Serial.print(PID_right.GetKp());
   Serial.println(F("}"));
   */
  /*Serial.print(F("{TIMEPLOT:Variables|data|millis()|T|"));
   Serial.print(millis());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:Variables|data|tmmpa|T|"));
   Serial.print(tmmpa);
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:Variables|data|tmmpb|T|"));
   Serial.print(tmmpb);
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:Variables|data|tmp_time|T|"));
   Serial.print(tmp_time);
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:Variables|data|TCNT1|T|"));
   Serial.print(TCNT1);
   Serial.print(F("}"));
   
   
   Serial.print("{MESSAGE:");
   Serial.print(channelName);
   Serial.print("|data|");
   Serial.print(millis());
   Serial.print(",");
   Serial.print(TCNT1);
   Serial.print(",");
   Serial.print(right_sonar);
   Serial.println("}");
   
   */
  //Serial.print(" \n");

}

void workloop(){
  work_time = millis();

  //Refresh sensor readings
    front_sonar= requestRange(FrontI2C); //read I2C sonar range, Value in cm
    rear_sonar= requestRange(RearI2C); //read I2C sonar range, Value in cm
    left_sonar= (interrupt_count[11])/58; //read PWM sonar range, value in cm
    right_sonar= (interrupt_count[9])/58; //read PWM sonar range, value in cm
    
  //Refresh RC inputs  
    //GetSpektrum();  
     // roll_in= (interrupt_count[51]);
    //pitch_in= (interrupt_count[50]);
    //throttle_in= (interrupt_count[52]);
     // aux1= (interrupt_count[53]);

  //Run PID loops
    PID_right.Compute();
    PID_front.Compute();
    PID_left.Compute();
    PID_rear.Compute();

  //Send trigger pulse to sonars
    Sonar_pulse();
    takeRangeReading(FrontI2C);
    takeRangeReading(RearI2C);
    

  //Do we want obstacle avoidance on?
     if(aux1>1400){
         compd_roll=(roll_in-int(Output_right)+int(Output_left));
       //compd_pitch=(pitch_in-int(Output_rear)+int(Output_front)); //remember to check the direction of pitch before testing
         compd_roll=ConstrainPWM(compd_roll,1100,1950);
         compd_pitch=ConstrainPWM(compd_pitch,1100,1950);
     }
    else {
      compd_roll=roll_in;
      compd_pitch=pitch_in;
    }

  //buzzer
     /*
     if (aux2>1600){
       if(right_sonar<(Setpoint_right+20)) analogWrite(piezzo,Output_right);
       else analogWrite(piezzo,0);
     }
     else analogWrite(piezzo,0);
     */

  //CRCArduinoFastServos::writeMicroseconds(chanel1_INDEX,compd_pitch);
    CRCArduinoFastServos::writeMicroseconds(chanel2_INDEX,compd_roll);
  //CRCArduinoFastServos::writeMicroseconds(chanel3_INDEX,compd_throttle);
  //CRCArduinoFastServos::writeMicroseconds(chanel4_INDEX,mode_switch);
}


void loop() {
  tmp_time=millis();
  
  if (tmp_time  >report_time+ 100){
    report();
  } 
  if (tmp_time  >work_time + PIDSampleTime){
    workloop();
  }
    
 }





















