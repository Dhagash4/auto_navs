#include <Messenger.h>
#include <digitalWriteFast.h>
#include <AutoPID.h>


Messenger Messenger_Handler = Messenger();

//Encoder Pins

//Left
#define left_encoder_apin 2
#define left_encoder_bpin 4
#define left_encoder_interrupt 0

#define LeftEncoderIsReversed
volatile float Left_Enc_previousticks = 0;
volatile float Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;
long copy_Left_Encoder_Ticks = 0;

//Right

#define right_encoder_apin 3
#define right_encoder_bpin 5
#define right_encoder_interrupt 1 

#define RightEncoderIsReversed
volatile float Right_Enc_previousticks = 0;
volatile float Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;
long copy_Right_Encoder_Ticks = 0;


//motorpins

//Left

#define left_motor_pow_pin 8
#define left_motor_dir_pin 12

//Right
#define right_motor_pow_pin 9
#define right_motor_dir_pin 13

//Reset Pin

#define RESET_PIN 7

//AutoPID Object and Variables

double lefMotRPM, lefMotSetRPM, lefOutPWM, lefWheRPM, lefMotAbsRPM;  // directly use for AutoPID function
double rigMotRPM, rigMotSetRPM, rigOutPWM, rigWheRPM, rigMotAbsRPM;

#define lefOutPWMMax 150
#define lefOutPWMMin 13
#define lefKP 0.0276
#define lefKD 0.035
#define lefKI 0.0743

#define rigOutPWMMax 150
#define rigOutPWMMin 13
#define rigKP 0.0276
#define rigKD 0.035
#define rigKI 0.0743
//-------------Starting PID Objects-----------------------------------
AutoPID lefMotPID(&lefMotAbsRPM, &lefMotSetRPM, &lefOutPWM, lefOutPWMMin, lefOutPWMMax, lefKP, lefKI, lefKD);
AutoPID rigMotPID(&rigMotAbsRPM, &rigMotSetRPM, &rigOutPWM, rigOutPWMMin, rigOutPWMMax, rigKP, rigKI, rigKD);

float N = 0.0;    // Used for getting RPMs to set
const float radWhe = 0.0975; // metres
const float wheBas = 0.70;  // metres
const float pi = 3.1415;
const float geaRat = 22;

//Time 

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

unsigned long TT;
int frequency = 30;

float lefCurTime=0, rigCurTime=0;


float lefPreTim = 0, rigPreTim = 0;
float current_time = 0;

//Twist from ROS
float linear = 0.0;
float angular = 0.0;
float ang = 0.0;

int lefMotDirInd = 1, rigMotDirInd = 1; // 1 means forward rotation of a wheel
int preLefMotDirInd = 1, preRigMotDirInd = 1;

void lefMotRPMCal() // It will calculate real time RPM for left motor
{ // No. of count difference in a time interval
  // divided by the time taken in the time interval in Minutes
  // There are 1024 ticks in 1 revolution
  // Gear ratio is 22:1
  //if (Left_Encoder_Ticks != Left_Enc_previousticks) {
  noInterrupts();
  copy_Left_Encoder_Ticks = Left_Encoder_Ticks;
  lefCurTime = micros();
  interrupts();
  lefMotRPM = (60 * 1000000 * (copy_Left_Encoder_Ticks - Left_Enc_previousticks) / (lefCurTime - lefPreTim)) / 1024;
  //lefMotRPM = (60 * 1000000 * (Left_Encoder_Ticks - Left_Enc_previousticks) / (TT)) / 1024;
  lefMotAbsRPM = abs(lefMotRPM);
  lefWheRPM = lefMotRPM / geaRat; //
  //}
  
  lefPreTim = lefCurTime;
  //this is the time when some of the counts will be lost
  //before executing next line since both lines have a small time interval
  //delay for running
  Left_Enc_previousticks = copy_Left_Encoder_Ticks;
}

void rigMotRPMCal() // It will calculate real time RPM for right motor
{
  //if (Right_Encoder_Ticks != Right_Enc_previousticks) {
  
  noInterrupts();
  copy_Right_Encoder_Ticks = Right_Encoder_Ticks;
  rigCurTime = micros();
  interrupts();
  rigMotRPM = (60 * 1000000 * (copy_Right_Encoder_Ticks - Right_Enc_previousticks) / (rigCurTime - rigPreTim)) / 1024;
  //rigMotRPM = (60 * 1000000 * (Left_Encoder_Ticks - Left_Enc_previousticks) / (TT)) / 1024;
  rigMotAbsRPM = abs(rigMotRPM);
  rigWheRPM = rigMotRPM / geaRat;
  
   
  rigPreTim = rigCurTime;
  Right_Enc_previousticks = copy_Right_Encoder_Ticks;
}

void forMov()
{
  digitalWrite(left_motor_dir_pin, LOW);
  digitalWrite(right_motor_dir_pin, HIGH);
  analogWrite(left_motor_pow_pin, lefOutPWM);
  analogWrite(right_motor_pow_pin, rigOutPWM);
  Serial.println(" Forward Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
  

}
void bacMov()
{
  digitalWrite(left_motor_dir_pin, HIGH);
  digitalWrite(right_motor_dir_pin, LOW);
  analogWrite(left_motor_pow_pin, lefOutPWM);
  analogWrite(right_motor_pow_pin, rigOutPWM);
  Serial.println(" Backward Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}
void lefMov()
{
  digitalWrite(right_motor_dir_pin, HIGH);
  digitalWrite(left_motor_dir_pin, HIGH);
  analogWrite(left_motor_pow_pin, lefOutPWM);
  analogWrite(right_motor_pow_pin, rigOutPWM);
  Serial.println(" left Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}
void rigMov()
{
  digitalWrite(right_motor_dir_pin, LOW);
  digitalWrite(left_motor_dir_pin, LOW);
  analogWrite(left_motor_pow_pin, lefOutPWM);
  analogWrite(right_motor_pow_pin, rigOutPWM);
  Serial.println(" right Called");
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
}

void stopMov()
{
  lefOutPWM=0;
  rigOutPWM=0;
  digitalWrite(right_motor_dir_pin, LOW);
  digitalWrite(left_motor_dir_pin, LOW);
  analogWrite(left_motor_pow_pin, lefOutPWM);
  analogWrite(right_motor_pow_pin, rigOutPWM);
  preLefMotDirInd = lefMotDirInd;
  preRigMotDirInd = rigMotDirInd;
  lefMotPID.stop();
  rigMotPID.stop();
  }


void setup() {

  //Reset Pin
  //
  //digitalWrite(RESET_PIN,HIGH);
  //delay(200);
  //pinMode(RESET_PIN,OUTPUT);
  //delay(200);
  
    Serial.begin(9600);

  //delay(200);
  //Serial.println("Reset Done");
 //Reset Done
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  //Setup Encoders
  //Left Encoder
  pinMode(left_encoder_apin, INPUT); 
  digitalWrite(left_encoder_apin, HIGH);// Turn on pullup resistor
  pinMode(left_encoder_bpin, INPUT); 
  digitalWrite(left_encoder_bpin, HIGH);// Turn on pullup resistor
  lefPreTim = micros();
  attachInterrupt(left_encoder_interrupt,ISRLefEncCou , RISING);
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Right Encoder
  pinMode(right_encoder_apin, INPUT);
  digitalWrite(right_encoder_apin, HIGH); // Turn on pullup resistor
  pinMode(right_encoder_bpin, INPUT);
  digitalWrite(right_encoder_bpin, HIGH);// Turn on pullup resistor
  
  rigPreTim = micros();
  attachInterrupt(right_encoder_interrupt,ISRRigEncCou, RISING);
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //SetupMotors
  //Left Motor
  pinMode(left_motor_pow_pin, OUTPUT);
  pinMode(left_motor_dir_pin,OUTPUT);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Right Motor 
  pinMode(right_motor_pow_pin, OUTPUT);
  pinMode(right_motor_dir_pin,OUTPUT);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //Setup Messenger
  Messenger_Handler.attach(OnMessageCompleted);

  //AutoPID

  
  lefMotPID.setTimeStep(20);  // Calculations will be done and
  rigMotPID.setTimeStep(20);  //variables will be changed after this time has passed
  

  TT = micros();
  
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{

   Read_From_Serial();
  
  Serial.print("r"); //RPM
  Serial.print("\t");
  Serial.print(long(lefMotRPM));
  Serial.print("\t");
  Serial.print(long(rigMotRPM));
  Serial.print("\n");

  Serial.print("a"); //SetRPM
  Serial.print("\t");
  Serial.print(long(lefMotSetRPM));
  Serial.print("\t");
  Serial.print(long(rigMotSetRPM));
  Serial.print("\n");

   Serial.print("p"); //SetRPM
   Serial.print("\t");
   Serial.print(long(lefOutPWM));
   Serial.print("\t");
   Serial.print(long(rigOutPWM));
   Serial.print("\n");

  
   //Update_Time();

   
  
   //Update_Motors();
  
   Update_Encoders();

   //Serial.print("Started The code");
   while(micros()-TT<1000000/frequency){
    }
   TT = micros();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Read_From_Serial()
{
   while(Serial.available())
    {
       int data = Serial.read();
       Messenger_Handler.process(data);
  
    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OnMessageCompleted()
{
  
  char reset[] = "r";
  char set_speed[] = "s";
  
  
  if(Messenger_Handler.checkString("r"))
  {
    
      
  //     Reset();
    
  }
  if(Messenger_Handler.checkString("s"))
  {

     Set_Speed();

    if(linear == 0 && ang == 0){
  // Vehicle at stall
    lefMotSetRPM = 0;
    rigMotSetRPM = 0;
    lefMotDirInd = 0;
    rigMotDirInd = 0;
  
  }

  if(abs(linear)>0 && ang == 0)
  {
  // Straight line motion, also a turn of radius infinity
    N = 22*60 * linear / (2 * pi * radWhe);
    lefMotSetRPM = abs(N);
    rigMotSetRPM = abs(N);
    if(N>=0){
    lefMotDirInd = 1;
    rigMotDirInd = 1;
    }
    else 
    {
    lefMotDirInd = -1;
    rigMotDirInd = -1;
    }
  }

  if(linear == 0 && abs(ang)>0)
  {
  // Turn of radius zero
    N = 22*60 * ang * wheBas / (4 * pi * radWhe);
    lefMotSetRPM = abs(N);
    rigMotSetRPM = abs(N);
    if(N>=0)
    {
    lefMotDirInd = -1;
    rigMotDirInd = 1;
    }
    else 
    {
    lefMotDirInd = 1;
    rigMotDirInd = -1;
    }
  }

  if(abs(linear)>0 && abs(ang)>0)
  {
  // Here vehicle would go for a turn of a non zero radius
    float L = linear/ang; // radius of turn
    float Nl = 0.0 , Nr = 0.0;
 
    Nl = 22*60*(L-wheBas/2.0)*ang/(2*pi*radWhe);
    Nr = 22*60*(L+wheBas/2.0)*ang/(2*pi*radWhe);
    lefMotSetRPM = abs(Nr);
    rigMotSetRPM = abs(Nl);
    lefMotDirInd = Nl / abs(Nl);
    rigMotDirInd = Nr / abs(Nr);

  }


   lefMotRPMCal();
   rigMotRPMCal();
   
   lefMotPID.run(); //call every loop, updates automatically at certain time interval
   rigMotPID.run();

   

   if (lefMotDirInd == preLefMotDirInd && rigMotDirInd == preRigMotDirInd) {
    // follow the code
    // This if...else... ensures that whenever wheel change their turning direction,
    // the wheels first come to stop
  } else {
    analogWrite(right_motor_pow_pin, 0);
    analogWrite(left_motor_pow_pin, 0);
    lefMotPID.stop();
    rigMotPID.stop();
    //Serial.println(" Stop Called ");
    //delay(200); // Assuming 0.2 sec for complete halt of motor.
  }

  //-------Which function to call acoording to direction------------
  // indicator variables. The last two conditions of assingment
  // ensures that wheels are not stopped if the function is called again------
  if (lefMotDirInd == 1) {
    if (rigMotDirInd == 1) {
      forMov();     // movement starts here
      
    } else {
      rigMov();     // movement starts here
      
    }
  }

  if (lefMotDirInd == -1) {
    if (rigMotDirInd == -1) {
      bacMov();     // movement starts here
      
    } else {
      lefMov();     // movement starts here
      
    }
  }
  if (lefMotDirInd == 0 && rigMotDirInd == 0) {
    
    
    stopMov();
    
    
    

    
    }

     
     return;   
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void Reset()
{
 
  
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  
 
  
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ISRLefEncCou()
{
  
  LeftEncoderBSet = digitalReadFast(left_encoder_bpin);   // read the input pin
 
  
  #ifdef LeftEncoderIsReversed
    Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
  #else
    Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
  #endif
 
  //Left_Encoder_Ticks++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ISRRigEncCou()
{
  
 
  RightEncoderBSet = digitalReadFast(right_encoder_bpin);   // read the input pin

  #ifdef RightEncoderIsReversed
    Right_Encoder_Ticks -= RightEncoderBSet ? -1 : +1;
  #else
    Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
  #endif
  
  //Right_Encoder_Ticks++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Update_Encoders()
{
 
  Serial.print("e");
  Serial.print("\t");
  Serial.print(long(Left_Encoder_Ticks));
  Serial.print("\t");
  Serial.print(long(Right_Encoder_Ticks));
  Serial.print("\n");
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Set_Speed()
{
    
  linear = Messenger_Handler.readLong() ;
  //linear = linear/10 ;
  linear = linear * 0.1;
  angular = Messenger_Handler.readLong();
  ang = angular/10;
  
  Serial.print("i");
  Serial.print("\t");
  Serial.print(long(linear*10));
  Serial.print("\t");
  Serial.print(long(ang*10));
  Serial.print("\n");
  
  
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Update_Motors()
{
  
  //moveRightMotor(lefOutPWM);
  //moveLeftMotor(rigOutPWM);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(linear);
  Serial.print("\t");
  Serial.print(angular);  
  Serial.print("\n");


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void Update_Time()
{
  
      
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
      MicrosecsSinceLastUpdate = 10 - LastUpdateMicrosecs + CurrentMicrosecs;

    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
 
  
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveRightMotor(float rightspeedVal)
{
  if (rightspeedVal>0)
  {
       
 digitalWrite(right_motor_dir_pin, HIGH);
 analogWrite(right_motor_pow_pin,rightspeedVal);
    
  }
  else if(rightspeedVal<0)
  {
 digitalWrite(right_motor_dir_pin, LOW);
 analogWrite(right_motor_pow_pin,abs(rightspeedVal));
 
  }
  
  else if(rightspeedVal == 0)
  {
 analogWrite(right_motor_pow_pin,0);  
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void moveLeftMotor(float leftspeedVal)
{
 if (leftspeedVal > 0)
  {
digitalWrite(left_motor_dir_pin, LOW);
analogWrite(left_motor_pow_pin,leftspeedVal);
  }
  
  else if(leftspeedVal < 0)
  {
 digitalWrite(left_motor_dir_pin, HIGH);
 analogWrite(left_motor_pow_pin,leftspeedVal);
  }
  
  else if(leftspeedVal == 0)
  {

 analogWrite(left_motor_pow_pin,0);
  
   }    
}

////////////////////////////////////////////////////////////////////////////END OF CODE////////////////////////////////////////////////////////////////////////////////////////////////////
