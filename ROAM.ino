// PinChangeIntExample, version 1.1 Sun Jan 15 06:24:19 CST 2012
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

#include <PinChangeInt.h>



uint8_t latest_interrupted_pin;
unsigned long report_time;
unsigned long work_time;
unsigned long tmp_time;
unsigned int tmmpa;
unsigned int tmmpb;

char channelName[ ] = "debug";

volatile uint16_t interrupt_count[100]={
  0}; // 100 possible arduino pins
volatile uint16_t interrupt_start[100]={
  0}; // 100 possible arduino pins
unsigned long timestart=0;
//int16_t
// unThrottleInStart = TCNT1;



#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint_right, right_sonar, Output_right;
double Setpoint_front, left_sonar, Output_front;
double Setpoint_left, front_sonar, Output_left;
double Setpoint_rear, rear_sonar, Output_rear;
double PIDSampleTime=100; //interval in ms
double safe_distance=90; //value in cm
double OutMax=600; //value in ms

double P=7;
double  I=3;
double  D=0;



//Specify the links and initial tuning parameters
PID myPID_right(&right_sonar, &Output_right, &Setpoint_right,P,I,D, DIRECT);
PID myPID_front(&front_sonar, &Output_front, &Setpoint_front,P,I,D, DIRECT);
PID myPID_left(&left_sonar, &Output_left, &Setpoint_left,P,I,D, DIRECT);
PID myPID_rear(&rear_sonar, &Output_rear, &Setpoint_rear,P,I,D, DIRECT);

#include <RCArduinoFastLib.h>

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

double pitch_in=1500;
double roll_in=1500;
double throttle_in=900;
double mode_switch=900;
int compd_pitch, compd_roll, compd_throttle;

int trigger= 13;
int piezzo=2;
//int green=11;
//int blue=2;
//int red=10;

void quicfunc() {
  //latest_interrupted_pin=PCintPort::arduinoPin;
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

void Sonar_pulse(){
  digitalWrite(trigger,LOW);
  digitalWrite(trigger,HIGH);
  delayMicroseconds(20);
  digitalWrite(trigger,LOW);
}

void setup() {
  pinMode(13,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(13, LOW);
  //pinMode(PIN2, INPUT);

  //sensors
  PCpin(A8);//62
  //PCpin(A9);
  PCpin(A10);
  //PCpin(A11);
  //PCpin(A12);
  //PCpin(A13);
  //PCpin(A14);
  //PCpin(A15);//69

  //inputs
  //PCpin(50);
  PCpin(51);
  //PCpin(52);
  PCpin(53);
  //PCpin(10);
  //PCpin(11);


  Serial.begin(57600);
  Serial.println("---------------------------------------");

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

  Setpoint_right= Setpoint_front = Setpoint_left = Setpoint_rear = safe_distance;

  //turn the PIDs on
  myPID_right.SetMode(AUTOMATIC);
  //myPID_front.SetMode(AUTOMATIC);
  //myPID_left.SetMode(AUTOMATIC);
  //myPID_rear.SetMode(AUTOMATIC);
  report_time	= millis();
  work_time	= millis();
  myPID_right.SetSampleTime(PIDSampleTime);
  myPID_left.SetSampleTime(PIDSampleTime);
  myPID_right.SetOutputLimits(0,OutMax);
  myPID_left.SetOutputLimits(0,OutMax);
}

void report(){
  report_time	= millis();

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

  Serial.print(F("{TIMEPLOT:PID|data|LeftSonar|T|"));
  Serial.print(left_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|AileronOut|T|"));
  Serial.print(compd_roll);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|Aileron|T|"));
  Serial.print(roll_in);
  Serial.println(F("}"));

  /*Serial.print(F("{TIMEPLOT:PID|data|Mode Switch|T|"));
   Serial.print(mode_switch);
   Serial.println(F("}"));

   Serial.print(F("{TIMEPLOT:PIDsettings|data|Kd|T|"));
   Serial.print(myPID_right.GetKd());
   Serial.println(F("}"));

   Serial.print(F("{TIMEPLOT:PIDsettings|data|Ki|T|"));
   Serial.print(myPID_right.GetKi());
   Serial.println(F("}"));

   Serial.print(F("{TIMEPLOT:PIDsettings|data|Kp|T|"));
   Serial.print(myPID_right.GetKp());
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
  work_time	= millis();
  
  right_sonar= (interrupt_count[62])/58; //value in cm
  //front_sonar= (interrupt_count[63])/58; //value in cm
  left_sonar= (interrupt_count[64])/58; //value in cm
  //rear_sonar= (interrupt_count[65])/58; //value in cm

  //pitch_in= (interrupt_count[50]);
  roll_in= (interrupt_count[51]);
  //throttle_in= (interrupt_count[52]);
  mode_switch= (interrupt_count[53]);

  myPID_right.Compute();
  //myPID_front.Compute();
  myPID_left.Compute();
  //myPID_rear.Compute();

  //Send trigger pulse to sonars
  Sonar_pulse();

  if(mode_switch>1400){
    compd_roll=(roll_in-int(Output_right)+int(Output_left));
    
    //Cap PW out
    if (compd_roll<1100) compd_roll=1100;
    if (compd_roll>1950) compd_roll=1950;
    
    /* //buzzer
    if(right_sonar<(Setpoint_right+20)) analogWrite(piezzo,Output_right);
    else analogWrite(piezzo,0);*/
  }
    
  else {
    compd_roll=roll_in;
    //analogWrite(piezzo,0);
  }

  //CRCArduinoFastServos::writeMicroseconds(chanel1_INDEX,compd_pitch);
  CRCArduinoFastServos::writeMicroseconds(chanel2_INDEX,compd_roll);
  //CRCArduinoFastServos::writeMicroseconds(chanel3_INDEX,compd_throttle);
  //CRCArduinoFastServos::writeMicroseconds(chanel4_INDEX,interrupt_count[63]);
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




















