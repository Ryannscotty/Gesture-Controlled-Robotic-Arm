#include "BluetoothSerial.h"
//#include <SoftwareSerial.h>
#include <Wire.h>
#include <HardwareSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//HardwareSerial SerialPort(2); // UART2
//SoftwareSerial BT(8,9); // RX pin on 12 and TX on pin 13

String MACadd = "98:D3:11:FC:3D:72";
uint8_t address[6] = {0x98, 0x0D,0x11,0xFC,0x3D,0x72};
String name = "HC-06";
char *pin = "1234";
bool connected;

#define HardwareRX 16
int redLed = 13;
int whiteLed = 12;
int yellowLed = 14;

int pushButton=23;
int buttonRead;
int buttonState =1;
int flexSensor1 = 34;
int flexSensor2 = 35;
int flexSensor3 = 32;
int flexHolder[3];
static float voltage1;
static float voltage2;
static float voltage3;
static float voltTX[3];
boolean flag1 = true;
const int MPU_addr = 0x68;
int minVal = 265;
int maxVal = 402;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double x;
double y;
double z;
static double MPU[3];

int RXpin = 16;
int TXpin = 17;
int RXpin2 = 5;
int TXpin2 = 18;
void setup() {
 Serial.begin(115200);
 Serial2.begin(115200,SERIAL_8N1,RXpin,TXpin);
 Serial1.begin(115200,SERIAL_8N1,RXpin2,TXpin2);
 //BT.begin(115200);
MPU_Setup();
pinMode(pushButton,INPUT);
pinMode(redLed,OUTPUT);
pinMode(whiteLed,OUTPUT);
pinMode(yellowLed,OUTPUT);
delay(100);
 // this is the config for onboard bluetooth module for esp32
 /*
 SerialBT.setPin(pin);
 connected = SerialBT.connect(name);
 
 if (connected){
  Serial.println("connected Succesfully!");
 }
 else{
  while(!SerialBT.connected(10000)){
    Serial.println("Failed to connect.");
  }
 }
 // for disconnection
if (SerialBT.disconnect()){
  Serial.println(" Disconnected succesfully");
}
SerialBT.connect();
*/
}

void loop() {
buttonRead =digitalRead(pushButton);
flexRead();
delay(500);
flexSensorTX();
delay(500);
MPU_Read();
delay(500);
MPU_TX();
delay(500);

 if((buttonState == 1) && (buttonRead==0)){
  //flag1 = false;
  Serial1.println(1);
  Serial.println(1);
  delay(50);
  //flag1=true;
  digitalWrite(redLed,HIGH);
  digitalWrite(whiteLed,HIGH);
  digitalWrite(yellowLed,HIGH);
  delay(250);
  digitalWrite(redLed,LOW);
  digitalWrite(whiteLed,LOW);
  digitalWrite(yellowLed,LOW);
  delay(250);
  digitalWrite(redLed,HIGH);
  digitalWrite(whiteLed,HIGH);
  digitalWrite(yellowLed,HIGH);
  delay(250);
  digitalWrite(redLed,LOW);
  digitalWrite(whiteLed,LOW);
  digitalWrite(yellowLed,LOW);
  delay(250);
  digitalWrite(redLed,HIGH);
  digitalWrite(whiteLed,HIGH);
  digitalWrite(yellowLed,HIGH);
  delay(250);
  digitalWrite(redLed,LOW);
  digitalWrite(whiteLed,LOW);
  digitalWrite(yellowLed,LOW);
  delay(250);
}
}

void flexSensorTX(void){
voltTX[0] = (voltage1);
voltTX[1] = (voltage2);
voltTX[2] = (voltage3);
for(byte i = 0; i < sizeof(voltTX) / sizeof(voltTX[0]); i++){
  Serial.print("voltages: ");
 Serial.println(voltTX[i],2);
 Serial.println();
 Serial2.print(voltTX[i],2);
 Serial2.print(',');
  delay(30);
  }
if( voltage1 > 3.50 || voltage2 > 3.50 || voltage3 > 3.50){
  digitalWrite(redLed,HIGH);
  digitalWrite(whiteLed,HIGH);
  digitalWrite(yellowLed,HIGH);
}
else{
  digitalWrite(redLed,LOW);
  digitalWrite(whiteLed,LOW);
  digitalWrite(yellowLed,LOW);
}
}


void flexRead(void){
flexHolder[0] = analogRead(flexSensor1);
flexHolder[1] = analogRead(flexSensor2);
flexHolder[2] = analogRead(flexSensor3);
 
voltage1 = (flexHolder[0] * 5.0) / 4095.0;
voltage2 = (flexHolder[1] * 5.0) / 4095.0;
voltage3 = (flexHolder[2] * 5.0) / 4095.0;
//Serial.println("voltage1: " +String( voltage1));
//Serial.println("voltage2: " +String(voltage2));
//Serial.println("voltage3: " +String(voltage3));
//Serial.println();
  //delay(10);
}


void MPU_Setup(void){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}


void MPU_Read(void){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI) + 4;
  y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  Serial.print("AngleX = ");
  Serial.println(x);
  Serial.print("AngleY = ");
  Serial.println(y);
  Serial.print("AngleZ = ");
  Serial.println(z);
  Serial.println();
  //Serial.print("float AngleX = ");
  //Serial.println(float(x));
  //Serial.print("float AngleY = ");
  //Serial.println(float(y));
  //Serial.print("float AngleZ = ");
  //Serial.println(float(z));
  delay(30);
}


void MPU_TX(void){
  MPU[0]=x;
  MPU[1]=y;
  MPU[2]=z;
for(byte i = 0; i < sizeof(MPU) / sizeof(MPU[0]); i++){
Serial1.println(MPU[i],2);
Serial1.print(',');
Serial.println(MPU[i],2);
  delay(10);
  }
}
