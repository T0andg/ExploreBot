#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "PCF8575.h"
#include "MQ135.h" 
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
Adafruit_PWMServoDriver SG90 = Adafruit_PWMServoDriver();
#define TCAADDR 0x70
#define PIN_MQ135 A7
MQ135 mq135 = MQ135(PIN_MQ135);
PCF8575 pcf8575(0x27);
int cb1 = A0;
int cb2 = A3;
int previousGoc1 = -1;
int previousGoc2 = -1;
static const int RXPin = A2, TXPin = A1;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
#define RX_PIN P6
#define TX_PIN P7
int goc;

const byte add[][6] = {"00001", "00002"};
unsigned long previousMillis = 0;
const long interval = 10;  
Adafruit_AHTX0 aht;
#define SERVOMIN  150 
#define SERVOMAX  600

int topulse(int goc) // Chuyen goc thanh xung
{
  int xung = map(goc, 0, 180, SERVOMIN, SERVOMAX);
  return xung;
}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(TCAADDR);  
  Wire.write(1 << bus);          
  Wire.endTransmission();
}
 

#define enA 3  //in1
#define in1 2 //enA
#define in2 4
#define in3 5
#define in4 7 //enB
#define enB 6 //in4
RF24 radio(8, 9); // CE, CSN
int  xAxis, yAxis, xBxis, yBxis, ServoVal1, ServoVal2;
int MQ, As1, As2;
float lattitude, longitude, ppm, humidity, temp, T, H, V, K;
float nhietdo = 0.0, doam = 0.0;
float kinhdo = 0.0, vido = 0.0;
struct PacketData
{
  int xAxis;
  int yAxis;
  int xBxis;
  int yBxis;

};
PacketData receiverData;

struct DuLieu 
{
  float nhietdo;
  float doam;
  float ppm;
  float kinhdo;
  float vido;
};
DuLieu data2;

int motorSpeedA = 0;
int motorSpeedB = 0;

void setup()
{
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);   
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);   
  pinMode(in4, OUTPUT);   
  pinMode(cb1, INPUT);
  pinMode(cb2, INPUT);
  pcf8575.pinMode(P5, OUTPUT);
  pcf8575.pinMode(P4, OUTPUT);
  pcf8575.pinMode(P3, OUTPUT);
  pcf8575.pinMode(P2, OUTPUT);
  pcf8575.pinMode(P1, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();
  SG90.begin();
  SG90.setOscillatorFrequency(27000000);
  SG90.setPWMFreq(60);
  pcf8575.begin();
  radio.begin();
  radio.openWritingPipe(add[0]); //00001
  radio.openReadingPipe(1, add[1]); //00002
  radio.startListening(); //Bat dau nhan
  ss.begin(GPSBaud);

  TCA9548A(7);
  if (!aht.begin()) {
    Serial.println("Không thể khởi tạo cảm biến AHT10!");
    while (1); // Dừng chương trình nếu khởi tạo thất bại
  }
}

void AHT10(){
  TCA9548A(7);
  sensors_event_t humidity, temp;

  if (!aht.getEvent(&humidity, &temp)) {
    Serial.println("Không thể đọc dữ liệu từ cảm biến AHT10!");
    return;
  }

  nhietdo = temp.temperature;
  doam = humidity.relative_humidity;
}

void Servo(){
  TCA9548A(4);
  ServoVal1 = int(yBxis);
  goc = map(ServoVal1, 0, 1023, 0, 180);
  if (goc != previousGoc1) {
    SG90.setPWM(0, 0, topulse(goc));
    previousGoc1 = goc; // Cập nhật giá trị góc trước đó
  }
  ServoVal2 = int(xBxis);
  goc = map(ServoVal2, 0, 1023, 180, 0);
  if (goc != previousGoc2) {
    SG90.setPWM(1, 0, topulse(goc));
    previousGoc2 = goc; // Cập nhật giá trị góc trước đó
  }
  delay(10);
}

void GPS(){
  if (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      if (gps.location.isValid())
      {
        vido = (gps.location.lat(), 6);
        kinhdo = (gps.location.lng(), 6);
      }
    }
  }
}
void Led(){
  TCA9548A(2);
  As1 = analogRead(cb1);
  As2 = analogRead(cb2);
  if ((As1 > 700) && (As2 > 700)){
    pcf8575.digitalWrite(P5, HIGH);
    pcf8575.digitalWrite(P4, HIGH);
    pcf8575.digitalWrite(P3, HIGH);
    pcf8575.digitalWrite(P2, HIGH);
    pcf8575.digitalWrite(P1, HIGH);
  } 
  else {
    pcf8575.digitalWrite(P5, LOW);
    pcf8575.digitalWrite(P4, LOW);
    pcf8575.digitalWrite(P3, LOW);
    pcf8575.digitalWrite(P2, LOW);
    pcf8575.digitalWrite(P1, LOW);
  }
}

void loop()
{

  AHT10();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Servo();  
  }

  if(radio.available())
  {
    radio.read(&receiverData, sizeof(PacketData));        
    xAxis = receiverData.xAxis;
    yAxis = receiverData.yAxis;
    xBxis = receiverData.xBxis;
    yBxis = receiverData.yBxis;

    if (xAxis < 470) {
      // Motor A Di lui
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor B Di lui
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      //Chuyển giá trị xung của Y-Asix sang giá trị tốc độ 
      motorSpeedA = map(xAxis, 470, 0, 0, 255);
      motorSpeedB = map(xAxis, 470, 0, 0, 255);
    }
    else if (xAxis > 550) {
      // Motor A đi thẳng
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor B đi thẳng
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      motorSpeedA = map(xAxis, 550, 1023, 0, 255);
      motorSpeedB = map(xAxis, 550, 1023, 0, 255);
    }
    else {
      motorSpeedA = 0;
      motorSpeedB = 0;
    }
    // y-axis quay trái, quay phải
    if (yAxis < 470) {
      int xMapped = map(yAxis, 470, 0, 0, 255);
      xMapped = xMapped * 0.65;
      motorSpeedA = motorSpeedA + xMapped;
      motorSpeedB = motorSpeedB - xMapped;
      if (motorSpeedA < 0) {
        motorSpeedA = 0;
      }
      if (motorSpeedB > 255) {
        motorSpeedB = 255;
      }
    }
    if (yAxis > 550) {
      int xMapped = map(yAxis, 550, 1023, 0, 255);
      xMapped = xMapped * 0.65;
      motorSpeedA = motorSpeedA - xMapped;
      motorSpeedB = motorSpeedB + xMapped;
      if (motorSpeedA > 255) {
        motorSpeedA = 255;
      }
      if (motorSpeedB < 0) {
        motorSpeedB = 0;
      }
    }
    if (motorSpeedA < 70) {
      motorSpeedA = 0;
    }
    if (motorSpeedB < 70) {
      motorSpeedB = 0;
    }
    motorSpeedA = motorSpeedA * 0.3;
    motorSpeedB = motorSpeedB * 0.3;
    analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
    analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
    radio.stopListening();  //Bat dau truyen 
    Nongdo();
    AHT10();
    Led();
    T = nhietdo;
    H = doam;
    V = vido;
    K = kinhdo;
    data2.ppm = mq135.getPPM();
    data2.nhietdo = T;
    data2.doam = H;
    data2.kinhdo = K;
    data2.vido = V;
    radio.write(&data2, sizeof(DuLieu));
    radio.startListening();
  }

}

void Nongdo(){
  float ppm = mq135.getPPM();
}
