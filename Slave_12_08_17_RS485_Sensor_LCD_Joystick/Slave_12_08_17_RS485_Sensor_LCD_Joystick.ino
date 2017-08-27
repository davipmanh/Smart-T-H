/* PROJECT: SLAVE/RS485/MODBUS RTU
   AUTHOR: DAVIP MANH
   DADE: 20/08/2017
   VERSION: 1.0
*/

//***************** Begin RS485 *****************//
#include <ModbusRtu.h>

#define ID    2
#define baud  115200
Modbus slave(ID, 0, 8); // (uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin); pin 8: control RS485 module

// Data array for modbus network sharing
#define NumberData  16
uint16_t au16data[NumberData];
//***************** End RS485 *****************//

//***************** Begin Sensor DHT *****************//
#include "DHT.h"

#define DHT1PIN   A0    // Pin name for Sensor 1
#define DHT2PIN   A1    // Pin name for Sensor 2
#define DHTTYPE   DHT21 // DHT 21 (AM2301)
#define timeread  100   // time read data of sensors

#define delta_t    1   // delta value for temperature calibration
#define delta_h    0.1   // delta value for humidity calibration
#define delta_rl1  1000  // delta value for timer relay 1 calibration
#define delta_rl2  1000  // delta value for timer relay 2 calibration
#define delta_up   0.5   // delta value for upper calibration
#define delta_low  0.5   // delta value for lower calibration

DHT dht1(DHT1PIN, DHTTYPE);  //Declare for Sensor 1
DHT dht2(DHT2PIN, DHTTYPE);  //Declare for Sensor 2

float h1, t1, f1, h2, t2, f2;
signed int _h1, _t1, _f1, _h2, _t2, _f2; // Convert from float to int
unsigned long tempms;
int sensor;
float dt1 = 0.0, dh1 =  0.0, dt2 =  0.0, dh2 =  0.0, up1 = 40, low1 = 25, up2 = 80, low2 = 50;
unsigned int time1 = 10000, time2 = 10000;
//***************** End Sensor DHT *****************//

//***************** Begin LCD *****************//
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
unsigned long delayclr, out, timeChange, timeBlink;
int change = -1;
int counter;
bool edit = 0;
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
bool seen = 1, old_seen = 1;
int mode = 0, dp = 0, dp1 = 1, dp2 = 1, dp3 = 1;
//***************** End Joystick *****************//

//***************** Begin Declare Relay *****************//
#define RL1      9
#define RL2      10
#define RL3      11
#define RL4      12

unsigned long timeOnRL1, timeOnRL2, timeOnRL3, timeOnRL4;
byte flag = 0;
//***************** End Declare Relay *****************//

//***************** Declare Subroutines *****************//
void io_poll();
void clearlcd(int t);
void outmode(int t);
void displaydp(String text1, String text2, float value);
int readjs();
void blinkCursor(int x, int y);
void UpdateValue(float x, float t);

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
  timeChange = millis();
  timeBlink = millis();

  //Setup Relay
  pinMode(RL1, OUTPUT);
  pinMode(RL2, OUTPUT);
  pinMode(RL3, OUTPUT);
  pinMode(RL4, OUTPUT);

  digitalWrite(RL1, HIGH);
  digitalWrite(RL2, HIGH);
  digitalWrite(RL3, HIGH);
  digitalWrite(RL4, HIGH);

  timeOnRL1 = millis();
  timeOnRL2 = millis();
  timeOnRL3 = millis();
  timeOnRL4 = millis();
}

void loop() {
  if (readjs() == Right || readjs() == Left)
  {
    if (millis() > timeChange + 400)
    {
      seen ^= 1;
      lcd.clear();
      timeChange = millis();
    }
  }

  if (readjs() == Press)
  {
    if (millis() > timeChange + 400)
    {
      mode ^= 1;
      lcd.clear();
      timeChange = millis();
    }
  }
  if (mode == 0)
  {
    if (millis() > tempms)
    {
      //Read Sensor 1
      h1 = dht1.readHumidity();
      t1 = dht1.readTemperature();
      //f1 = dht1.readTemperature(true);

      //Read Sensor 1
      h2 = dht2.readHumidity();
      t2 = dht2.readTemperature();
      //f2 = dht2.readTemperature(true);

      //Check if any reads failed and exit early (to try again).
      if (isnan(h1) || isnan(t1) || isnan(f1))  bitWrite( au16data[6], 0, 1);
      else bitWrite( au16data[6], 0, 0);
      if (isnan(h2) || isnan(t2) || isnan(f2))  bitWrite( au16data[6], 1, 1);
      else bitWrite( au16data[6], 1, 0);

      // Convert value to int
      _h1 = (h1 + dh1) * 100;
      _t1 = (t1 + dt1) * 100;
      //_f1 = f1 * 100;

      _h2 = (h2 + dh2) * 100;
      _t2 = (t2 + dt2) * 100;
      //_f2 = f2 * 100;
      if (seen)
      {
        if (isnan(h1) || isnan(t1) || isnan(f1))
        {
          lcd.setCursor(0, 0);
          lcd.print("    Sensor 1    ");
          lcd.setCursor(0, 1);
          lcd.print("   No Device!   ");
        }
        else
        {
          //clearlcd(3000);
          lcd.setCursor(0, 0);
          lcd.print("    Sensor 1    ");
          lcd.setCursor(0, 1);
          lcd.print(t1 + dt1);
          lcd.write(223); // Icon 0 - 1101 1111 - 0xDF
          lcd.print("C  ");
          lcd.setCursor(9, 1);
          lcd.print(h1 + dh1);
          lcd.print("%  ");
        }
      }
      else
      {
        if (isnan(h2) || isnan(t2) || isnan(f2))
        {
          lcd.setCursor(0, 0);
          lcd.print("    Sensor 2    ");
          lcd.setCursor(0, 1);
          lcd.print("   No Device!   ");
        }
        else
        {
          //clearlcd(3000);
          lcd.setCursor(0, 0);
          lcd.print("    Sensor 2    ");
          lcd.setCursor(0, 1);
          lcd.print(t2 + dt2);
          lcd.write(223); // Icon 0 - 1101 1111 - 0xDF
          lcd.print("C  ");
          lcd.setCursor(9, 1);
          lcd.print(h2 + dh2);
          lcd.print("%  ");
        }
      }
      tempms = millis() + timeread;
    }

    if ((t1 + dt1) >= up1)
    {
      if (millis() > (timeOnRL1 + time1))
      {
        if ((t1 + dt1) >= up1)
        {
          digitalWrite(RL1, LOW);
          flag |= B0001;
          timeOnRL1 = millis();
        }
      }
    }
    else
    {
      if ((t1 + dt1) <= low1)
      {
        digitalWrite(RL1, HIGH);
        flag &= B1110;
      }
      timeOnRL1 = millis();
    }

    if ((t2 + dt2) >= up1)
    {
      if (millis() > (timeOnRL3 + time1))
      {
        if ((t2 + dt2) >= up1)
        {
          digitalWrite(RL3, LOW);
          flag |= B0100;
          timeOnRL3 = millis();
        }
      }
    }
    else
    {
      if ((t2 + dt2) <= low1)
      {
        digitalWrite(RL3, HIGH);
        flag &= B1011;
      }
      timeOnRL3 = millis();
    }

    if ((h1 + dh1) >= up2)
    {
      if (millis() > (timeOnRL2 + time2))
      {
        if ((h1 + dh1) >= up2)
        {
          digitalWrite(RL2, LOW);
          flag |= B0010;
          timeOnRL2 = millis();
        }
      }
    }
    else
    {
      if ((h1 + dh1) <= low2)
      {
        digitalWrite(RL2, HIGH);
        flag &= B1101;
      }
      timeOnRL2 = millis();
    }

    if ((h2 + dh2) >= up2)
    {
      if (millis() > (timeOnRL4 + time2))
      {
        if ((h2 + dh2) >= up2)
        {
          digitalWrite(RL4, LOW);
          flag |= B1000;
          timeOnRL4 = millis();
        }
      }
    }
    else
    {
      if ((h2 + dh2) <= low2)
      {
        digitalWrite(RL4, HIGH);
        flag &= B0111;
      }
      timeOnRL4 = millis();
    }

    //Poll messages RS485
    slave.poll( au16data, NumberData );
    io_poll();
  }
  else if (mode == 1)
  {
    if (readjs() == Right)
    {
      if (millis() > timeChange + 220)
      {
        dp++;
        lcd.clear();
        timeChange = millis();
      }
      out = millis();
      if (dp > 10) dp = 0;
    }
    else if (readjs() == Left)
    {
      if (millis() > timeChange + 210)
      {
        dp--;
        lcd.clear();
        timeChange = millis();
      }
      out = millis();
      if (dp < 0) dp = 10;
    }
    switch (dp) {
      case 0:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            dt1 += delta_t;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            dt1 -= delta_t;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Delta T1 = ", dt1);
        break;
      case 1:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            dh1 += delta_h;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            dh1 -= delta_h;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Delta H1 = ", dh1);
        break;
      case 2:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            dt2 += delta_t;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            dt2 -= delta_t;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Delta T2 = ", dt2);
        break;
      case 3:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            dh2 += delta_h;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            dh2 -= delta_h;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Delta H2 = ", dh2);
        break;
      case 4:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            time1 += delta_rl1;
            lcd.clear();
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            time1 -= delta_rl1;
            lcd.clear();
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "TimeRL T = ", time1 / 1000);
        break;
      case 5:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            time2 += delta_rl2;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            time2 -= delta_rl2;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "TimeRL H = ", time2 / 1000);
        break;
      case 6:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            up1 += delta_up;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            up1 -= delta_up;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Upper 1 <= ", up1);
        break;
      case 7:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            low1 += delta_low;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            low1 -= delta_low;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Lower 1 >= ", low1);
        break;
      case 8:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            up2 += delta_up;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            up2 -= delta_up;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Upper 2 <= ", up2);
        break;
      case 9:
        if (readjs() == Up)
        {
          if (millis() > timeChange + 200)
          {
            low2 += delta_low;
            timeChange = millis();
            out = millis();
          }
        }
        else if (readjs() == Down)
        {
          if (millis() > timeChange + 200)
          {
            low2 -= delta_low;
            timeChange = millis();
            out = millis();
          }
        }
        displaydp("    Setting   ", "Lower 2 >= ", low2);
        break;
      case 10:
        lcd.setCursor(0, 0);
        lcd.print(" Information ");
        lcd.setCursor(14, 0);
        if (dp < 10)
          lcd.print(" ");
        lcd.print(dp);
        lcd.setCursor(0, 1);
        lcd.print(" 090.294.68.24  ");
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
  au16data[0] = _t1;
  au16data[1] = _h1;
  //au16data[2] = _f1;

  au16data[2] = _t2;
  au16data[3] = _h2;
  //  bitWrite( au16data[4], 0, digitalRead(RL1));
  //  bitWrite( au16data[4], 1, digitalRead(RL2));
  //  bitWrite( au16data[4], 2, digitalRead(RL3));
  //  bitWrite( au16data[4], 3, digitalRead(RL4));
  au16data[4] = flag;
  bool x = au16data[5];
  if (x)
  {
    dt1 = au16data[6];
    dh1 = au16data[7];
    dt2 = au16data[8];
    dh2 = au16data[9];
    time1 = au16data[10];
    time2 = au16data[11];
    up1 = au16data[12];
    low1 = au16data[13];
    up2 = au16data[14];
    low2 = au16data[15];
  }
  else
  {
    au16data[6] = dt1;
    au16data[7] = dh1 ;
    au16data[8] = dt2 ;
    au16data[9] = dh2;
    au16data[10] = time1 ;
    au16data[11] = time2;
    au16data[12] = up1;
    au16data[13] = low1;
    au16data[14] = up2;
    au16data[15] = low2;
  }
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

void displaydp(String text1, String text2, float value)
{
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.setCursor(14, 0);
  if (dp < 10)
    lcd.print(" ");
  lcd.print(dp);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.setCursor(11, 1);
  lcd.print(value);
  outmode(10000);
}

int readjs()
{
  int readAX = analogRead(AX);
  int readAY = analogRead(AY);
  bool readSW = digitalRead(SW);
  if (readAX < 360) return Left;
  else if (readAX > 390) return Right;
  if (readAY < 330) return Up;
  else if (readAY > 410) return Down;
  if (!readSW) return Press;
  else return 0;
}

void blinkCursor(int x, int y)
{
  if (millis() > timeBlink)
  {
    lcd.setCursor(x, y);
    lcd.cursor();
    timeBlink = millis() + 500;
  }
  else
    lcd.noCursor();
}

void UpdateValue(float x, float t)
{
  if (readjs() == Up) x += t;
  else if (readjs() == Down)  x -= t;
}
