#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mcp_can.h>

//For CAN Bus
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); 
#define PE1  0xF048
#define PE2  0xF148
#define PE3  0xF248
#define PE4  0xF348
#define PE5  0xF448
#define PE6  0xF548
#define PE7  0xF648
#define PE8  0xF748
#define PE9  0xF848
#define PE10 0xF948
#define PE11 0xFA48
#define PE12 0xFB48
#define PE13 0xFC48
#define PE14 0xFD48
#define PE15 0xFE48
#define PE16 0xFF48

// If using software SPI (the default case):
#define OLED_MOSI   9     //Data
#define OLED_CLK    8     //Clk
#define OLED_DC    5//11  //DC/SA0
#define OLED_CS    6//13  //CS
#define OLED_RESET 12     //Rst
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// For Inputs
#define rsPIN A5
#define shiftUPIN A3
#define shiftDPIN A2
#define rPotPIN A1
#define rpmPOTmin 0
#define rpmPOTmax 1023
#define rpmPOTmid (rpmPOTmax+rpmPOTmin)/2

// For Shift Register
#define latchPIN    4
#define clockPIN    7
#define dataPIN     3
#define pinQ0       1
#define pinQ1       2
#define pinQ2       4
#define pinQ3       8
#define pinQ4      16
#define pinQ5      32
#define pinQ6      64
#define pinQ7     128
int SR1 = 0, SR2 = 0, SR3 = 0, SR4 = 0, SR5 = 0;

//For Shift Lights
#define rpmBlink 50; // RPM Flash timing (ms)
#define sRPM1    6800
#define sRPM2    7400
#define sRPM3    8000
#define sRPM4    8600
#define sRPM5    9200
#define sRPM6    9800
#define sRPM7   10400
#define sRPM8   11000
#define sRPM9   11600
#define sRPM10  12000
#define sRPM11  12200
#define sRPM12  12300
#define sLED1   pinQ3 //SR2
#define sLED2   pinQ2 //SR2
#define sLED3   pinQ1 //SR2
#define sLED4   pinQ0 //SR2
#define sLED5   pinQ7 //SR1
#define sLED6   pinQ6 //SR1
#define sLED7   pinQ5 //SR1
#define sLED8   pinQ4 //SR1
#define sLED9   pinQ3 //SR1
#define sLED10  pinQ2 //SR1
#define sLED11  pinQ1 //SR1
#define sLED12  pinQ0 //SR1

//For Warning Lights
#define led1R  pinQ4 //SR2
#define led1B  pinQ5 //SR2
#define led1G  pinQ6 //SR2
#define led2R  pinQ7 //SR2
#define led2B  pinQ0 //SR3
#define led2G  pinQ1 //SR3
#define led3R  pinQ2 //SR3
#define led3B  pinQ3 //SR3
#define led3G  pinQ4 //SR3
#define led4R  pinQ5 //SR3
#define led4B  pinQ6 //SR3
#define led4G  pinQ7 //SR3
#define led5R  pinQ0 //SR4
#define led5B  pinQ1 //SR4
#define led5G  pinQ2 //SR4
#define led6R  pinQ3 //SR4
#define led6B  pinQ4 //SR4
#define led6G  pinQ5 //SR4
#define engTempLow 60
#define engTempMedLow 70
#define engTempMedHigh 100
#define engTempHigh 110
#define engTempLowBlink 500
#define engTempHighBlink 50
#define batVoltMed 12.5
#define batVoltLow 10
//#define oilPresLow ** need to get these values as function of RPM
//#define oilPresHigh ** need to get these values as function of RPM

//For 7 Segment
#define ssTL pinQ2 //SR5
#define ssTR pinQ4 //SR5
#define ssBL pinQ0 //SR5
#define ssBR pinQ6 //SR5
#define ssTC pinQ3 //SR5
#define ssCC pinQ1 //SR5
#define ssBC pinQ7 //SR5
#define ssDP pinQ5 //SR5

//For Rotary Switch 
#define RS1H 20
#define RS2L 130
#define RS2H 155
#define RS3L 160
#define RS3H 185
#define RS4L 190
#define RS4H 215
#define RS5L 240
#define RS5H 270
#define RS6L 330
#define RS6H 350
#define RS7L 470
#define RS7H 525
#define RS8L 1000

#define runStartup 1

static const unsigned char PROGMEM QFSAELogo[] = {
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000001, B11111111, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11111111, B10000000, B00000000,
  B00000000, B00000000, B11111111, B11111000, B00000000, B00000000, B00000000, B00000011, B00000000, B00000000, B00000000, B00000000, B00011111, B11111111, B00000000, B00000000,
  B00000000, B00000000, B01100000, B00011100, B00000000, B00000000, B00000000, B00011111, B11100000, B00000000, B00000000, B00000000, B00111000, B00000110, B00000000, B00000000,
  B00000000, B00000000, B00110000, B00000111, B11111111, B11111111, B11111111, B11111100, B11111111, B11111111, B11111111, B11111111, B11100000, B00001100, B00000000, B00000000,
  B00000000, B00000000, B00011000, B00000011, B11111111, B11111111, B11111111, B11111000, B00111111, B11111111, B11111111, B11111111, B11000000, B00011000, B00000000, B00000000,
  B00000000, B00000000, B00001100, B00000000, B00000000, B00000000, B00000000, B01100000, B00011100, B00000000, B00000000, B00000000, B00000000, B00110000, B00000000, B00000000,
  B00000000, B00000000, B00000110, B00000000, B00000000, B00000000, B00000001, B11000000, B00001110, B00000000, B00000000, B00000000, B00000000, B01100000, B00000000, B00000000,
  B00000000, B00000000, B00000011, B11111111, B11111111, B10000000, B00000001, B10000000, B00000111, B00000000, B00000011, B11111111, B11111111, B11000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00001111, B11111111, B11111111, B11111111, B10000000, B00000011, B11111111, B11111111, B11111111, B11110000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B00000000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000110, B00000000, B00000001, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001110, B00000000, B00000000, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011100, B00000000, B00000000, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011100, B00000000, B00000000, B01100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00011111, B11111111, B11111111, B11000000, B00000000, B00000000, B00111000, B00000000, B00000000, B01110000, B00000000, B00000000, B00000111, B11111111, B11111111, B11100000,
  B01111111, B11111111, B11111111, B11111111, B00000000, B00000000, B01110000, B00000000, B00000000, B00111000, B00000000, B00000001, B11111111, B11111111, B11111111, B11111000,
  B11111100, B00000000, B00000011, B11111111, B11000000, B00000000, B11100000, B00000000, B00000000, B00011100, B00000000, B00001111, B11111111, B00000000, B00000000, B11111100,
  B11110000, B00000000, B00000000, B00000011, B11100000, B00000000, B11000000, B00000111, B11000000, B00001110, B00000000, B00001111, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000000, B11000000, B00011111, B11110000, B00001110, B00000000, B00011110, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000001, B11000000, B00111100, B01111000, B00000110, B00000000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100000, B00000011, B00000111, B11110000, B00011111, B10000011, B00000000, B00011000, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100111, B11111111, B11111111, B11100000, B00001111, B11111111, B11111111, B10011000, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11111111, B11111111, B00001111, B11000000, B00000111, B11000001, B11111111, B11111100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00000000, B11111011, B10000000, B00000011, B11111110, B00000000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B00001111, B10000111, B00000000, B00000001, B11000111, B11100000, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100000, B11111000, B00001110, B00000000, B00000000, B11100000, B00111100, B00011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11100111, B10000000, B00011100, B00000000, B00000000, B01110000, B00000111, B10011100, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B11101100, B00000000, B00011100, B00000000, B00000000, B01110000, B00000000, B11011110, B00000000, B00000000, B00000000, B00111100,
  B11110000, B00000000, B00000000, B00000000, B01100000, B00000000, B00111000, B00000000, B00000000, B00111000, B00000000, B00011000, B00000000, B00000000, B00000000, B00111100,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00111000,
  B01110000, B00011111, B11111110, B00111111, B11111110, B01111111, B11110001, B11000000, B00001100, B11000000, B00001011, B00000000, B00000000, B00010000, B00000000, B00111000,
  B01111000, B00011111, B11111111, B01111111, B11111111, B01111111, B11111101, B11100000, B00011101, B11000000, B00011011, B00000100, B00000000, B00111000, B00000000, B01111000,
  B01111111, B00001111, B11111111, B01110000, B00000111, B01111111, B11111101, B11110000, B00111101, B11000000, B00011011, B00000110, B00000000, B00011100, B00000111, B11111000,
  B00111111, B00000000, B00000000, B01100000, B00000011, B00000000, B00011101, B11111000, B01111101, B11000000, B00011011, B00011111, B00000000, B00001110, B00000111, B11110000,
  B00000000, B00011111, B11111111, B01100000, B00000011, B01111111, B11111001, B11011111, B11101101, B11000000, B00011011, B00001110, B00000000, B00000111, B00000000, B00000000,
  B00000000, B00011111, B11111111, B01100000, B00000011, B01111111, B11111101, B11001111, B11001101, B11000000, B00011011, B00000100, B00000001, B11111111, B10000000, B00000000,
  B00000000, B00011000, B00000000, B01110000, B00000111, B01100000, B00001101, B11000111, B10001101, B11000000, B00011011, B00000000, B00000011, B11111111, B11000000, B00000000,
  B00000000, B00011000, B00000000, B01111111, B11111111, B01100000, B00001101, B11000000, B00001101, B11111111, B11111011, B11111111, B11100111, B00000000, B11100000, B00000000,
  B00000000, B00011000, B00000000, B00111111, B11111110, B01100000, B00001101, B11000000, B00001100, B11111111, B11111011, B11111111, B11101110, B00000000, B01110000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000100, B00000000, B00100000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00011111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000,
  B00000000, B00000000, B00000000
};


#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup()   { 
  Serial.begin(115200);
  // Initialize CAN Bus
  while (CAN_OK != CAN.begin(CAN_250KBPS))
  {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
     
  // Initialize Shift Registers
  pinMode(latchPIN,OUTPUT);
  pinMode(clockPIN,OUTPUT);
  pinMode(dataPIN,OUTPUT);

  // Initialize OLED
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextColor(WHITE);
  
  // Fun Startup Sequence
  if (runStartup > 0){
    digitalWrite(latchPIN,LOW);
      shiftOut(dataPIN,clockPIN,MSBFIRST,B11111111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,B11111111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,B11111111);
      shiftOut(dataPIN,clockPIN,MSBFIRST,B11111111);
    digitalWrite(latchPIN,HIGH);
    
    display.clearDisplay();
    dispLogo();
    delay(500);
    int loadBarW = 0;
    for (int loadBarX = 0; loadBarX < 128; loadBarX += 2){
      loadBarW=random(10)+2;
      display.drawRect(loadBarX,59,loadBarW,2,WHITE);
      display.display();
    }
    delay(100);
  }

  //Reset Display
  display.clearDisplay();
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,B00000000);
    shiftOut(dataPIN,clockPIN,MSBFIRST,B00000000);
    shiftOut(dataPIN,clockPIN,MSBFIRST,B00000000);
    shiftOut(dataPIN,clockPIN,MSBFIRST,B00000000);
  digitalWrite(latchPIN,HIGH);
  delay(200);
}

// CAN Bus Value Variables
int rpm = 0;
float avgWheelSpeed = 0;
float engTemp = 0;
float engTempLast = 0;
int updateEngTemp = 0;
float batVolt = 0;
float batVoltLast = 0;
int updateBatVolt = 0;
float oilPres = 0;
float oilPresLast = 0;
int updateOilPres = 0;
float tps = 0;
float tpsLast = 0;
int updateTPS = 0;

// Other Variables
float rsSwitch = 0;
int rpmFlash = 0;
int count = 0;
int gearNum = 0;
float rpmPOT;
int switchMode = 0;
int rpmBlinkTime = 0;
int sLightFlash = 1;
int engTempBlinkTime = 0;
int engTempFlash = 1;

//For CAN Bus
unsigned char len = 0;
unsigned char buf[8];
unsigned int canId;

void loop() {
  rsSwitch = analogRead(rsPIN);
  rpmPOT = analogRead(rPotPIN);
  SR1 = 0;
  SR2 = 0;
  SR3 = 0;
  SR4 = 0;
  SR5 = 0;

  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      canId = CAN.getCanId();
      
      if (canId == PE1){
        //RPM
        rpm = buf[1]*256 + buf[0];

        //TPS
        tps = buf[3]*256 + buf[2];
        if (tps > 32767)
          tps -= 65536;
        tps = tps*0.1;
        if (tps != tpsLast)
          updateTPS = 1;
        tpsLast = tps;
      }
      else if (canId == PE6){
        //Battery Voltage
        batVolt = buf[1]*256 + buf[0];
        if (batVolt > 32767)
          batVolt -= 65536;
        batVolt=batVolt*0.01;
        if (batVolt != batVoltLast)
          updateBatVolt = 1;
        batVoltLast = batVolt;

        //Engine Temp
        engTemp = buf[5]*256 + buf[4];
        if (engTemp > 32767)
          engTemp -= 65536;
        engTemp=engTemp*0.1;
        if (engTemp != engTempLast)
          updateEngTemp = 1;
        engTempLast = engTemp;
      }
      else if (canId == PE12){
        avgWheelSpeed = buf[1]*256 + buf[0];
      }
    }

  gear();
  warningLights();
  shiftLights();
  
  digitalWrite(latchPIN,LOW);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR5);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR4);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR3);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR2);
    shiftOut(dataPIN,clockPIN,MSBFIRST,SR1);
  digitalWrite(latchPIN,HIGH);
  //Serial.println(rsSwitch);
  
  if (rsSwitch<RS1H && switchMode != 1){
    switchMode = 1;
    dispLogo();
  }
  else if (rsSwitch>RS2L && rsSwitch<RS2H && (switchMode != 2 || updateEngTemp == 1)){
    switchMode = 2;
    updateEngTemp = 0;
    dispEngTemp();
  }
  else if (rsSwitch>RS3L && rsSwitch<RS3H && (switchMode != 3 || updateBatVolt == 1)){
    switchMode = 3;
    updateBatVolt = 0;
    dispBatVolt();
  }
  else if (rsSwitch>RS4L && rsSwitch<RS4H && (switchMode != 4 || updateOilPres == 1)){
    switchMode = 4;
    updateOilPres = 0;
    dispOilPres();
  }
  else if (rsSwitch>RS5L && rsSwitch<RS5H && switchMode != 5){
    switchMode = 5;
    dispA();
  }
  else if (rsSwitch>RS6L && rsSwitch<RS6H && switchMode != 6){
    switchMode = 6;
    dispGTFO();
  }
  else if (rsSwitch>RS7L && rsSwitch<RS7H && switchMode != 7){
    switchMode = 7;
    dispPIT();
  }
  else if (rsSwitch>RS8L && (switchMode != 8 || updateTPS == 1)){
    updateTPS = 0;
    switchMode = 8;
    dispTPS();
  }
  else if (switchMode != 9){
    switchMode = 9;
    dispERROR();
  }

  if (count>25)
    count = 0;
  count++;
}

void shiftLights(void){
  if (rpm > sRPM12){
    if (millis() - rpmFlash > rpmBlinkTime){
      rpmBlinkTime = millis();
      sLightFlash = sLightFlash*-1;
    }
    if (sLightFlash == 1){
      SR2 += sLED1+sLED2+sLED3+sLED4;
      SR1 += sLED5+sLED6+sLED7+sLED8+sLED9+sLED10+sLED11+sLED12;
    }
  }
  else if (rpm > sRPM11){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6+sLED7+sLED8+sLED9+sLED10+sLED11;
  }
  else if (rpm > sRPM10){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6+sLED7+sLED8+sLED9+sLED10;
  }
  else if (rpm > sRPM9){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6+sLED7+sLED8+sLED9;
  }
  else if (rpm > sRPM8){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6+sLED7+sLED8;
  }
  else if (rpm > sRPM7){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6+sLED7;
  }
  else if (rpm > sRPM6){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5+sLED6;
  }
  else if (rpm > sRPM5){
    SR2 += sLED1+sLED2+sLED3+sLED4;
    SR1 += sLED5;
  }
  else if (rpm > sRPM4){
    SR2 += sLED1+sLED2+sLED3+sLED4;
  }
  else if (rpm > sRPM3){
    SR2 += sLED1+sLED2+sLED3;
  }
  else if (rpm > sRPM2){
    SR2 += sLED1+sLED2;
  }
  else if (rpm > sRPM1){
    SR2 += sLED1;
  }
}

void warningLights(void){
/* 
L|C|S
1|R|2
1|B|2
1|G|2
2|R|2
2|B|3
2|G|3
3|R|3
3|B|3
3|G|3
4|R|3
4|B|3
4|G|3
5|R|4
5|B|4
5|G|4
6|R|4
6|B|4
6|G|4
*/
  // RPM
  if (rpm > sRPM9)
    SR2 += led1B;
  else if (rpm > sRPM5)
    SR2 += led1G;
  else if (rpm > sRPM1)
    SR2 += led1R;

  //Engine Temp
  if (engTemp < engTempLow){
    if (millis() - engTempLowBlink > engTempBlinkTime){
      engTempBlinkTime = millis();
      engTempFlash = engTempFlash*-1;
    }
    if (engTempFlash == 1){
      SR3 += led2B;
    }
  }
  else if(engTemp < engTempMedLow)
    SR3 += led2B;
  else if(engTemp > engTempHigh)
    if (millis() - engTempHighBlink > engTempBlinkTime){
      engTempBlinkTime = millis();
      engTempFlash = engTempFlash*-1;
    }
    if (engTempFlash == 1){
      SR2 += led2R;
    }
  else if(engTemp > engTempMedHigh)
    SR2 += led2R;
  else
    SR3 += led2G;

  //Battery Voltage
  if (batVolt < batVoltLow)
    SR3 += led3R;
  else if(batVolt < batVoltMed)
    SR3 += led3B;
  else
    SR3 += led3G;

  if (count > 20){
    SR3 += led4R+led3G;
    SR4 += led6R+led6G+led6B;
  }
  else if (count > 15){
    SR3 += led3R;
    SR4 += led5R+led5G+led5B+led6B;
  }
  else if (count > 10){
    SR3 += led4R+led4G+led4B;
    SR4 += led5B+led6G;
  }
  else if (count > 5){
    SR3 += led4B+led3R+led3G+led3B;
    SR4 += led5G+led6R;
  }
  else{
    SR3 += led4G+led3B;
    SR4 += led5R;
  }
}

void gear(void){
  if (gearNum == 1)
    SR5 += ssTL+ssBL;
  else if (gearNum == 2)
    SR5 += ssTC+ssTR+ssCC+ssBL+ssBC;
  else if (gearNum == 3)
    SR5 += ssTC+ssTR+ssCC+ssBR+ssBC;
  else if (gearNum == 4)
    SR5 += ssTR+ssCC+ssTL+ssBR;
  else if (gearNum == 5)
    SR5 += ssTC+ssTL+ssCC+ssBR+ssBC;
  else if (gearNum == 6)
    SR5 += ssTL+ssBL+ssCC+ssBR+ssBC;
  else
    SR5 += ssTL+ssBL+ssTR+ssBR+ssBC+ssTC;
}

void dispLogo(void){
  display.clearDisplay();
  display.drawBitmap(0, 7, QFSAELogo, 128, 50, WHITE);
  display.display();
}

void dispEngTemp(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Eng Temp");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(engTemp,0);
  display.display();
}

void dispBatVolt(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Bat Volt");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(batVolt,1);
  display.display();
}

void dispOilPres(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Oil Pres");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println(oilPres,0);
  display.display();
}

void dispRPM(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("RPM");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(4);
  if (rpm < 10)
    display.setCursor(96,25);
  else if (rpm < 100)
    display.setCursor(72,25);
  else if (rpm < 1000)
    display.setCursor(48,25);
  else if (rpm < 10000)
    display.setCursor(24,25);
  else
    display.setCursor(0,25);
  display.println(rpm);
  display.display();
}

void dispA(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Open A");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println("A");
  display.display();
}

void dispB(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Open B");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.println("B");
  display.display();
}

void dispTPS(void){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("TPS");
  display.drawLine(0,18,128,18,WHITE);
  display.setTextSize(5);
  display.setCursor(0,25);
  display.print(tps);
  display.println("%");
  display.display();
}

void dispERROR(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(22,14);
  display.println("ERR");
  display.display();
}

void dispGTFO(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(7,14);
  display.println("GTFO");
  display.display();
}

void dispPIT(void){
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(22,14);
  display.println("PIT");
  display.display();
}

