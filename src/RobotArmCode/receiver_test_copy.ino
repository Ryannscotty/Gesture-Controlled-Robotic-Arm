// Receiver side bluetooth configurion 
#include <SoftwareSerial.h>
char flexread;
char MPU_read;
 const int numchars = 3;
float RX_Flex[3];
float RX_MPU[3];
static int indx =0;
static int indx2 = 0;
boolean newData = false;
char Buffer[32];
char MPUBuffer[32];
int RobotStartButton;
int rxPin = 13;
int txPin = 12;
void RX2(void);
int pushButton = 27;
int buttonState=1;
int buttonRead = digitalRead(pushButton);
SoftwareSerial mySerial(8,9);
SoftwareSerial mySerial2(10,11);
void buttonRX(void);
int PWM_TEST_BLED = 29;


 // 1st motor driver
int EN1 = 7; 
int jointArmLeftMotor1 = 8; // left motor joint arm(pos terminal)
int jointArmLeftMotor2 = 9;  //left motor joint arm(neg terminal)
int EN2 = 22;
int jointArmRightMotor1 = 2;// right motor joint arm(pos terminal)
int jointArmRightMotor2 = 3;//right motor joint arm(neg terminal)
// 2nd motor driver 
int EN3 = 31; 
int basemotor1 = 4; // base motor
int basemotor2 = 5;  //base motor 2
int EN4 = 22;

typedef enum{
  STARTSTATE=1,
  RUNNINGSTATE=2
  
} RoboStates;

typedef struct{
  float ThumbFinger;
  float IndexFinger;
  float MiddleFinger;
  double mpuXaxis;
  double mpuYaxis;
  double mpuZaxis;
  //RoboStates RobotStartButton;
} ROBOTARM;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  pinMode(PWM_TEST_BLED,OUTPUT);
   pinMode(EN1,OUTPUT);
   pinMode(jointArmLeftMotor1,OUTPUT);
   pinMode(jointArmLeftMotor2,OUTPUT);
   pinMode(EN2,OUTPUT);
   pinMode(jointArmRightMotor1,OUTPUT);
   pinMode(jointArmRightMotor2,OUTPUT);
   pinMode(pushButton,INPUT);
   pinMode(EN3,OUTPUT);
   pinMode(basemotor1,OUTPUT);
   pinMode(basemotor2,OUTPUT);
   pinMode(EN4,OUTPUT);
   
}

void loop() {
flexSensorRX();
delay(500);
mpuRX();
delay(500);
//buttonRX();
delay(500);
RobotRun2();
delay(500);
}


void buttonRX(void){
  /*
  if(Serial1.available() > 0){
    int buttonreader = Serial1.read();
    if(buttonreader == 1){
      RobotStartButton = 1;
      Serial.println(buttonreader);
    }
    */
  

  if((buttonState == 1) && (buttonRead==0)){
    RobotStartButton = 1;
    Serial.println("startstate");
  }
  
}

void flexSensorRX(void){
char endmaker = '\n';
   
  while(Serial1.available() > 0){
      flexread = Serial1.readBytesUntil(',',Buffer,5);
      RX_Flex[flexread] = '\0';
    RX_Flex[indx] = atof(Buffer);
        indx++;
    if(indx >= 3){
      indx = 0;
    }
   
    if( RX_Flex[indx] == '\n')
    {
   
      indx =0;
    }
  Serial.print("voltage 1: ");
  Serial.println(RX_Flex[0]);
  Serial.print("voltage 2: ");
  Serial.println( (RX_Flex[1]));
  Serial.print("voltage 3: ");
  Serial.println( (RX_Flex[2]));
  Serial.println();
  delay(10);
  
}
}

void mpuRX(void){
    
  while(Serial2.available() > 0){
      MPU_read = Serial2.readBytesUntil(',',MPUBuffer,9);
      //MPU_read = Serial2.readBytes(MPUBuffer,7);
      RX_MPU[MPU_read] = '\0';
    RX_MPU[indx2] = atof(MPUBuffer);
        indx2++;
    if(indx2 >= 3){
      indx2 = 0;
    }
   
    if( RX_MPU[indx2] == '\n')
    {
   
      indx2 =0;
    }
  Serial.print("XANG: ");
  Serial.println(RX_MPU[0]);
  Serial.print("YANG: ");
  Serial.println( (RX_MPU[1]));
  Serial.print("ZANG: ");
  Serial.println( (RX_MPU[2]));
  Serial.println();
  delay(10);
  }
  
}


  /*  
  void RobotRun(void){
    RoboStates RobotState=RobotStartButton;
    ROBOTARM *robotPointer;
    robotPointer = malloc(sizeof(ROBOTARM));
    robotPointer->ThumbFinger = RX_Flex[0];
    robotPointer->IndexFinger = RX_Flex[1];
    robotPointer->MiddleFinger = RX_Flex[2];
    robotPointer->mpuXaxis = RX_MPU[0];
    robotPointer->mpuYaxis = RX_MPU[1];
    robotPointer->mpuZaxis = RX_MPU[2];
switch(RobotState){
  case (STARTSTATE):
      /// do push button to wake up  robot arm into running mode
          digitalWrite(PWM_TEST_BLED,HIGH);
          RobotState=RUNNINGSTATE;
     break;
  case(RUNNINGSTATE):
         if(robotPointer->ThumbFinger >= 3.50){
          // use the claw function
         }
         else if(robotPointer->IndexFinger >= 5.00){
          JointArmForward();
          delay(50);
         }
         else if((robotPointer->IndexFinger <= 4.25) && (robotPointer->IndexFinger >= 3.75)){
          JointArmBackwrd();
          delay(50);
         }
         else{
          JointArmMotorStop();
          delay(50);
         }
       
    break;  
  default:
      RobotState=STARTSTATE;
      digitalWrite(PWM_TEST_BLED,LOW);
     break;
     
  }
  
  }
*/
  void JointArmForward(void){
    digitalWrite(EN1,HIGH);
    analogWrite(jointArmLeftMotor1,0);
    analogWrite(jointArmLeftMotor2,225);
    digitalWrite(EN2,HIGH);
    analogWrite(jointArmRightMotor1,255);
    analogWrite(jointArmRightMotor2,0);
    
  }

  void JointArmBackwrd(void){
    digitalWrite(EN1,HIGH);
    analogWrite(jointArmLeftMotor1,255);
    analogWrite(jointArmLeftMotor2,0);
    digitalWrite(EN2,HIGH);
    analogWrite(jointArmRightMotor1,0);
    analogWrite(jointArmRightMotor2,255);
  }
void JointArmMotorStop(void){
  digitalWrite(EN1,HIGH);
    analogWrite(jointArmLeftMotor1,0);
    analogWrite(jointArmLeftMotor2,0);
    digitalWrite(EN2,HIGH);
    analogWrite(jointArmRightMotor1,0);
    analogWrite(jointArmRightMotor2,0);
}



void RobotRun2(void){
  RoboStates RobotState=RobotStartButton;
    ROBOTARM *robotPointer;
    robotPointer = malloc(sizeof(ROBOTARM));
    robotPointer->ThumbFinger = RX_Flex[0];
    robotPointer->IndexFinger = RX_Flex[1];
    robotPointer->MiddleFinger = RX_Flex[2];
    robotPointer->mpuXaxis = RX_MPU[0];
    robotPointer->mpuYaxis = RX_MPU[1];
    robotPointer->mpuZaxis = RX_MPU[2];

 if(robotPointer->ThumbFinger >= 3.50){
          // use the claw function
    Serial.println("Actuate Gripper!");
    delay(50);
          }
 else if(robotPointer->IndexFinger > 4.00){
           Serial.println("Index Finger Bend");
          JointArmForward();
          delay(50);
          
         }
 else if((robotPointer->IndexFinger <= 4.00) && (robotPointer->IndexFinger >= 3.45)){
        Serial.println("Index Finger release");
          JointArmBackwrd();
          delay(50);
        
         }
  else if((robotPointer->mpuXaxis >310) && ((robotPointer->mpuZaxis >= 180) &&(robotPointer->mpuZaxis <= 200))){
           Serial.println("Rotate Base CCW");
           baseCounterClockWise();
           delay(50);
  }
 else if(((robotPointer->mpuXaxis >= 10) &&(robotPointer->mpuXaxis <= 70)) && ((robotPointer->mpuZaxis > 0) &&(robotPointer->mpuZaxis <= 12))){
            Serial.println("Rotate Base CW");
              baseClockWise();
             delay(50);
  }
 else{
          JointArmMotorStop();
          BaseMotorStop();
          delay(50);
         }
    
}

 void baseClockWise(void){
    digitalWrite(EN3,HIGH);
    analogWrite(basemotor1,0);
    analogWrite(basemotor2,100);
    
    
  }

void baseCounterClockWise(void){
    digitalWrite(EN3,HIGH);
    analogWrite(basemotor1,100);
    analogWrite(basemotor2,0);
    
    
  }
  void BaseMotorStop(void){
    digitalWrite(EN3,HIGH);
    analogWrite(basemotor1,0);
    analogWrite(basemotor2,0);
  }
