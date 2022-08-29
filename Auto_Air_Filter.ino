 //******************************
 //*Project by Brandon of Honest Brothers on YouTube
 //*
 //*Abstract: Read value of PM1,PM2.5 and PM10 air quality. Report this to the Serial Bus. Read the values into a buffer to average them. Turn on a relay if the PM2.5 values are at or below 12 μg/m3.
 //*
 //*The pins can change since we used software serial.
 //*The RX pin on the sensor connects to pin 11 on the Arduino. Sensor RX connects to Arduino TX. We're not using this in this project. 
 //*The TX pin on the sensor connects to pin 10 on the Arduino. Sensor TX connects to Arduino RX.
 //*
 //*Attribution
 //*Particle Meter:
 //*https://wiki.dfrobot.com/PM2.5_laser_dust_sensor_SKU_SEN0177
 //*
 //*LiquidCrystal i2c:
 //*https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/
 //*
 //*Arduino Smoothing:
 //*https://www.arduino.cc/en/Tutorial/BuiltInExamples/Smoothing
 //******************************
 
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display. If LCD is not working, check the link above. There is some code to ttry to change the i2c address that might be needed.

//If you are not getting readings, try commenting the line below this out, and uncommenting the line two below this. Also make sure the TX of the sensor is going to the RX of the Arduino.
#define LENG 31   //PMS5003 buffer read length is 32 bytes. 0x42 is the first byte in the buffer, 0x42+31 bytes is 32 bytes. 
//#define LENG 23   //PMS3003 buffer read length is 24 bytes. 0x42 is the first byte in the buffer, 0x42+23 is 24 bytes.

unsigned char buf[LENG];

//int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
//int PM10Value=0;         //define PM10 value of the air detector module

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average=0;                // the average


SoftwareSerial PMSerial(10, 11); // RX, TX we only need rx

void setup()
{
  PMSerial.begin(9600);
  PMSerial.setTimeout(1500);
  Serial.begin(9600);

  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  pinMode(0, OUTPUT);
}

void loop()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if(PMSerial.find(0x42)){
    PMSerial.readBytes(buf,LENG);

    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        //PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        //PM10Value=transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }

  static unsigned long OledTimer=millis();
    if (millis() - OledTimer >=1000)
    {
      OledTimer=millis();

//      Serial.print("PM 1.0: ");
//      Serial.print(PM01Value);
//      Serial.println("  ug/m3");

      Serial.print("PM 2.5: ");
      Serial.print(PM2_5Value);
      Serial.println("  ug/m3");
      lcd.setCursor(0,0);   //Set cursor to character 0 on line 0
      lcd.print("PM2.5 :" + String(PM2_5Value) + "(ug/m3)");

//      Serial.print("PM 10: ");
//      Serial.print(PM10Value);
//      Serial.println("  ug/m3");
      
      // Print Average
      Serial.print("PM2.5 avg: ");
      Serial.print(average);
      Serial.println(" ug/m3");
      Serial.println(" ");
      lcd.setCursor(0,1);   //Set cursor to character 0 on line 1
      lcd.print("AVG   :" + String(average) + "(ug/m3)");
      delay(1000);
    }
/*Averaging Code*/
      // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = PM2_5Value;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

/*Turn on/off the relay*/
  if (average > 12){
    digitalWrite(0, HIGH);// This turns the relay on when PM2.5 is above the recommended safety level of 12. 
  }
  else if (average < 12){
    digitalWrite(0, LOW); // This will turn the fan off when the PM2.5 average is below 12. The average prevents the fan from turning on and off a bunch.
  }
  else {
    lcd.setCursor(0,0);   //Set cursor to character 0 on line 0
    lcd.print("Shiz is broken!"); //This shouldn't happen, but if it does, we want the screen to notify us there is something weird going on. 
  }
}

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;

  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

//int transmitPM01(unsigned char *thebuf)
//{
//  int PM01Val;
//  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
//  return PM01Val;
//}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }

//transmit PM Value to PC
//int transmitPM10(unsigned char *thebuf)
//{
//  int PM10Val;
//  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
//  return PM10Val;
//}
