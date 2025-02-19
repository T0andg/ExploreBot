#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
RF24 radio(9, 8); // CE, CSN

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte add[][6] = {"00001", "00002"};
String xAxis, yAxis, xBxis, yBxis;
float ppm, nhietdo, doam, kinhdo, vido;
struct PacketData 
{
  int xAxis;
  int yAxis;
  int xBxis;
  int yBxis;    
 
};
PacketData data;

struct DuLieu
{
  float nhietdo;
  float doam;
  float ppm;
  float kinhdo;
  float vido;
};
DuLieu receidata2;

void setup() {
  Serial.begin(9600);
  radio.begin();
  SPI.begin();
  radio.openWritingPipe(add[1]); //00002
  radio.openReadingPipe(1, add[0]); //00001
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();    //Bat dau che do gui

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }

}

void loop()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  radio.stopListening();
  data.xAxis = analogRead(A1);
  data.yAxis = analogRead(A0);
  data.xBxis = analogRead(A2);
  data.yBxis = analogRead(A3);  
  radio.write(&data, sizeof(PacketData));
  Serial.print("xA = ");
  Serial.print(data.xAxis);
  Serial.print(", yA = ");
  Serial.println(data.yAxis);
  Serial.print("xB = ");
  Serial.print(data.xBxis);
  Serial.print(", yB = ");
  Serial.println(data.yBxis);
  radio.startListening();

  delay(5);
  if (radio.available()) {
    radio.read(&receidata2, sizeof(DuLieu));
    nhietdo = receidata2.nhietdo;
    doam = receidata2.doam;
    ppm = receidata2.ppm;
    kinhdo = receidata2.kinhdo;
    vido = receidata2.vido;
    display.clearDisplay();
    display.print("Chat luong Oxy: ");
    display.println(ppm);
    display.print("Nhiet do: ");
    display.print(nhietdo);
    display.println(" do C");
    display.print("Do am: ");
    display.print(doam);
    display.println("%");
    display.print("Kinh do: ");
    display.println(kinhdo);
    display.print("Vi do: ");
    display.println(vido);
    display.display();

    delay(100);
    radio.stopListening();
  }
}

