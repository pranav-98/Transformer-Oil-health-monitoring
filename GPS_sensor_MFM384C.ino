#include <ModbusMaster.h>
#include <String.h>
#include <SoftwareSerial.h>

#include<Wire.h>
#define Addr 0x28
SoftwareSerial mySerial(9, 10);
const int trigPin = 11;
const int echoPin = 12;
long duration;
int dist;
float cTemp;
float humidity;

#define MAX485_DE      3
#define MAX485_RE_NEG  2
float p9,num3;
// instantiate ModbusMaster object

int meterIndex = 0;
    int mIndex = 0;


struct MeterParam{
  uint32_t data;
  char slNumber[8];
};ModbusMaster node;



MeterParam meters[32];

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


void setup()
{
  mySerial.begin(9600);               // the GPRS baud rate  
  Serial.begin(9600);    // the GPRS baud rate
  Wire.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);    
  delay(1000);

    node.begin(1, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}
 


bool state = true;

void loop()
{
  uint8_t result;
 uint16_t data[32];
 
  result = node.readInputRegisters(0x00, 60);
  if (result == node.ku8MBSuccess)
  {
   
    Serial.print("V1: ");
    uint16_t rP1 = node.getResponseBuffer(0x00);
    uint16_t rP2 = node.getResponseBuffer(0x01);


unsigned int data[2]={rP1,rP2};
float num;
memcpy(&num,data,4);
Serial.println(num);

    Serial.print("V2: ");
    uint16_t rP3 = node.getResponseBuffer(0x02);
    uint16_t rP4 = node.getResponseBuffer(0x03);


unsigned int data1[2]={rP3,rP4};
float num1;
memcpy(&num1,data1,4);
Serial.println(num1);


    Serial.print("V3: ");
    uint16_t rP5 = node.getResponseBuffer(0x04);
    uint16_t rP6 = node.getResponseBuffer(0x05);


unsigned int data3[2]={rP5,rP6};
float num2;
memcpy(&num2,data3,4);
Serial.println(num2);


      Serial.print("Average voltage: ");
    uint16_t rP7 = node.getResponseBuffer(0x06);
    uint16_t rP8 = node.getResponseBuffer(0x07);


unsigned int data4[2]={rP7,rP8};
memcpy(&num3,data4,4);
Serial.println(num3);

//--------------------------------------------------------------------------


     Serial.print(" C1: ");
    uint16_t pf11 = node.getResponseBuffer(0x10);
    uint16_t pf12 = node.getResponseBuffer(0x11);


unsigned int pff6[2]={pf11,pf12};
float p6;
memcpy(&p6,pff6,4);
Serial.println(p6);


     Serial.print(" C2: ");
    uint16_t pf13 = node.getResponseBuffer(0x12);
    uint16_t pf14 = node.getResponseBuffer(0x13);


unsigned int pff7[2]={pf13,pf14};
float p7;
memcpy(&p7,pff7,4);
Serial.println(p7);


     Serial.print(" C3: ");
    uint16_t pf15 = node.getResponseBuffer(0x14);
    uint16_t pf16 = node.getResponseBuffer(0x15);


unsigned int pff8[2]={pf15,pf16};
float p8;
memcpy(&p8,pff8,4);
Serial.println(p8);



     Serial.print("Average Current: ");
    uint16_t pf17 = node.getResponseBuffer(0x16);
    uint16_t pf18 = node.getResponseBuffer(0x17);


unsigned int pff9[2]={pf17,pf18};
memcpy(&p9,pff9,4);
Serial.println(p9);
 
}

  unsigned int data5[4];
Wire.beginTransmission(Addr);  
Wire.write(0x80);  
Wire.endTransmission();  
delay(300);    
Wire.requestFrom(Addr, 4);
if(Wire.available() == 4)  
{
data5[0] = Wire.read();    
data5[1] = Wire.read();    
data5[2] = Wire.read();    
data5[3] = Wire.read();        
humidity = (((data5[0] & 0x3F) * 256.0) +  data5[1]) * (100.0 / 16383.0);    
cTemp = (((data5[2] * 256.0) + (data5[3] & 0xFC)) / 4) * (165.0 / 16383.0) - 40;    
     
Serial.print("Relative Humidity : ");    
Serial.print(humidity);    
Serial.println(" %RH");    
Serial.print("Temperature in Celsius : ");    
Serial.print(cTemp);    
Serial.println(" C");  
 
}  
delay(1000);
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
dist= duration*0.034/2;
Serial.print("Distance: "); // Prints string "Distance" on the LCD
Serial.print(dist); // Prints the distance value from the sensor
Serial.print(" cm");

delay(1000);
Send2Pachube();
if (mySerial.available())
    Serial.write(mySerial.read());
}

void Send2Pachube()
{
  mySerial.println("AT");
  delay(1000);

  mySerial.println("AT+CPIN?");
  delay(1000);

  mySerial.println("AT+CREG?");
  delay(1000);

  mySerial.println("AT+CGATT?");
  delay(1000);

  mySerial.println("AT+CIPSHUT");
  delay(1000);

  mySerial.println("AT+CIPSTATUS");
  delay(2000);

  mySerial.println("AT+CIPMUX=0");
  delay(2000);
 
  ShowSerialData();
 
  mySerial.println("AT+CSTT=\"imis/internet\"");//start task and setting the APN,
  delay(1000);
 
  ShowSerialData();
 
  mySerial.println("AT+CIICR");//bring up wireless connection
  delay(3000);
 
  ShowSerialData();
 
  mySerial.println("AT+CIFSR");//get local IP adress
  delay(2000);
 
  ShowSerialData();
 
  mySerial.println("AT+CIPSPRT=0");
  delay(3000);
 
  ShowSerialData();
 
  mySerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//start up the connection
  delay(6000);
 
  ShowSerialData();
 
  mySerial.println("AT+CIPSEND");//begin send data to remote server
  delay(4000);
  ShowSerialData();
 
 
 
  String str="GET http://api.thingspeak.com/update?api_key=05MBR681ECI4YASZ&field1="+String(cTemp)+"&field2="+ String(humidity)+"&field3=" +String(dist)+"&field4="+String(p9)+"&field5="+ String(num3);
  mySerial.println(str);//begin send data to remote server
  delay(4000);
  ShowSerialData();

  mySerial.println((char)26);//sending
  delay(5000);//waitting for reply, important! the time is base on the condition of internet
  mySerial.println();
 
  ShowSerialData();
 
  mySerial.println("AT+CIPSHUT");//close the connection
  delay(100);
  ShowSerialData();
}
void ShowSerialData()
{
  while(mySerial.available()!=0)
    Serial.write(mySerial.read());
}
