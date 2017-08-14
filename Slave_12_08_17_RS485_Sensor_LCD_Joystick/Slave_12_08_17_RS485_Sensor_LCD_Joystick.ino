/* PROJECT: SLAVE/RS485/MODBUS RTU
   AUTHOR: DAVIP MANH
   DADE: 12/08/2017
   VERSION: 1.0
*/

//***************** Begin RS485 *****************//
#include <ModbusRtu.h>

#define ID    2
#define baud  115200
Modbus slave(ID, 0, 8); // (uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin); pin 8: control RS485 module

// Data array for modbus network sharing
#define NumberData  7
uint16_t au16data[NumberData];
//***************** End RS485 *****************//

//***************** Begin Sensor DHT *****************//
#include "DHT.h"

#define DHT1PIN   A0    // Pin name for Sensor 1
#define DHT2PIN   A1    // Pin name for Sensor 2
#define DHTTYPE   DHT21 // DHT 21 (AM2301)
#define timeread  100  // time read data of sensors

DHT dht1(DHT1PIN, DHTTYPE);  //Declare for Sensor 1
DHT dht2(DHT2PIN, DHTTYPE);  //Declare for Sensor 2

float h1, t1, f1, h2, t2, f2;
signed int _h1, _t1, _f1, _h2, _t2, _f2; // Convert from float to int
unsigned long tempms;
int sensor;
//***************** End Sensor DHT *****************//

//***************** Begin LCD *****************//
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
unsigned long delayclr, out;
//***************** End LCD *****************//

//***************** Begin Joystick *****************//
#define AX      A7
#define AY      A6
#define SW      A5

#define Press   1
#define Left    2
#define Right   3
#define Up      4
#define Down    5


//#define JSSW  !digitalRead(SW)
bool seen = 1;
int mode = 0, dp = 0, dp1 = 1, dp2 = 1, dp3 = 1;
//***************** End Joystick *****************//

//***************** Declare Subroutines *****************//
void io_poll();
void clearlcd(int t);
void outmode(int t);
void displaydp(String text1, String text2);
int readjs();
//***************** Declare Subroutines *****************//

void setup() {
  //pinMode Joystick
  pinMode(SW, INPUT_PULLUP);

  // Start communication RS485
  slave.begin(baud);

  // Start read Sensor
  dht1.begin();
  dht2.begin();
  tempms = millis();

  // Start LCD 16x02
  lcd.begin(16, 2);
  lcd.setCursor(1, 0);
  lcd.print("Measure T & H");
  lcd.setCursor(0, 1);
  lcd.print("RS485-Modbus RTU");
  //delay(1000);
  lcd.clear();
  delayclr = millis();
}

void loop() {
  if (readjs() > 1)
  {
    seen = !seen;
    delay(300);
  }
  if (readjs() == Press) mode = 1;
  if (mode == 0)
  {
    if (millis() > tempms)
    {
      //Read Sensor 1
      h1 = dht1.readHumidity();
      t1 = dht1.readTemperature();
      f1 = dht1.readTemperature(true);

      //Read Sensor 1
      h2 = dht2.readHumidity();
      t2 = dht2.readTemperature();
      f2 = dht2.readTemperature(true);

      //Check if any reads failed and exit early (to try again).
      if (isnan(h1) || isnan(t1) || isnan(f1))  bitWrite( au16data[6], 0, 1);
      else bitWrite( au16data[6], 0, 0);
      if (isnan(h2) || isnan(t2) || isnan(f2))  bitWrite( au16data[6], 1, 1);
      else bitWrite( au16data[6], 1, 0);

      // Convert value to int
      _h1 = h1 * 100;
      _t1 = t1 * 100;
      _f1 = f1 * 100;

      _h2 = h2 * 100;
      _t2 = t2 * 100;
      _f2 = f2 * 100;
      if (seen)
      {
        if (isnan(h1) || isnan(t1) || isnan(f1))
        {
          lcd.setCursor(4, 0);
          lcd.print("Sensor 1");
          lcd.setCursor(0, 1);
          lcd.print("   No Device!   ");
        }
        else
        {
          clearlcd(3000);
          lcd.setCursor(4, 0);
          lcd.print("Sensor 1");
          lcd.setCursor(0, 1);
          lcd.print(t1);
          lcd.write(223); // Icon 0 - 1101 1111 - 0xDF
          lcd.print("C");
          lcd.setCursor(9, 1);
          lcd.print(h1);
          lcd.print("%");
        }
      }
      else
      {
        if (isnan(h2) || isnan(t2) || isnan(f2))
        {
          lcd.setCursor(4, 0);
          lcd.print("Sensor 2");
          lcd.setCursor(0, 1);
          lcd.print("   No Device!   ");
        }
        else
        {
          clearlcd(3000);
          lcd.setCursor(4, 0);
          lcd.print("Sensor 2");
          lcd.setCursor(0, 1);
          lcd.print(t2);
          lcd.write(223); // Icon 0 - 1101 1111 - 0xDF
          lcd.print("C");
          lcd.setCursor(9, 1);
          lcd.print(h2);
          lcd.print("%");
        }
      }
      tempms = millis() + timeread;
    }
    //Poll messages RS485
    slave.poll( au16data, NumberData );
    io_poll();
  }
  else if (mode == 1)
  {
    if (readjs() == Right)
    {
      dp++;
      out = millis();
      if (dp > 5) dp = 0;
    }
    else if (readjs() == Left)
    {
      dp--;
      out = millis();
      if (dp < 0) dp = 5;
    }
    switch (dp) {
      case 0:
        displaydp("    Setting   ", "Delta T1 =      ");
        break;
      case 1:
        displaydp("    Setting   ", "Delta H1 =      ");
        break;
      case 2:
        displaydp("    Setting   ", "Delta T2 =      ");
        break;
      case 3:
        displaydp("    Setting   ", "Delta H2 =      ");
        break;
      case 4:
        displaydp("    Setting   ", "TimeRL12 =      ");
        break;
      case 5:
        displaydp("    Setting   ", "TimeRL34 =      ");
        break;
      default:
        break;
    }
  }
}

void io_poll() {
  // get digital inputs -> au16data[6]
  //bitWrite( au16data[6], 0, ss1);

  // set digital outputs -> au16data[1]
  //  digitalWrite( 6, bitRead( au16data[1], 0 ));

  // set analog outputs
  //  analogWrite( 10, au16data[2] );

  // read analog inputs - Maximum value is 2^15 -1 = 32767
  au16data[0] = _h1;
  au16data[1] = _t1;
  au16data[2] = _f1;

  au16data[3] = _h2;
  au16data[4] = _t2;
  au16data[5] = _f2;

  // diagnose communication
  //au16data[6] = slave.getInCnt();
  //au16data[7] = slave.getOutCnt();
  //au16data[8] = slave.getErrCnt();
}

void clearlcd(int t)
{
  if (millis() > delayclr)
  {
    lcd.clear();
    delayclr = millis() + t;
  }
}
void outmode(int t)
{
  if (millis() > (out + t))
  {
    mode = 0;
    out = millis();
  }
}

void displaydp(String text1, String text2)
{
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.setCursor(15, 0);
  lcd.print(dp);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  outmode(10000);
}

int readjs()
{
  int readAX = analogRead(AX);
  int readAY = analogRead(AY);
  bool readSW = digitalRead(SW);
  if (readAX < 330) return Left;
  else if (readAX > 410) return Right;
  if (readAY < 330) return Up;
  else if (readAY > 410) return Down;
  if (!readSW) return Press;
  else return 0;
}

