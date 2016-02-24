#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

#define LED_PIN       7
#define N_PIXELS      4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

const int rx=9;
const int tx=10;
SoftwareSerial mySerial(rx, tx);

byte r, g, b;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() 
{
  strip.begin();
  strip.setPixelColor(0,strip.Color(255,0,0));
  strip.setPixelColor(1,strip.Color(255,0,0));
  strip.setPixelColor(2,strip.Color(255,0,0));
  strip.setPixelColor(3 ,strip.Color(255,0,0));
  strip.show();
  
  pinMode(rx,INPUT);
  pinMode(tx,OUTPUT);
  mySerial.begin(38400);
  mySerial.println("Setting interrupts");
  mySerial.println("LSM raw read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    mySerial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  mySerial.println("Found LSM9DS0 9DOF");
  mySerial.println("");
  mySerial.println("");
  delay(500);
  strip.setPixelColor(0,strip.Color(0,0,0));
  strip.setPixelColor(1,strip.Color(0,0,0));
  strip.setPixelColor(2,strip.Color(0,0,0));
  strip.setPixelColor(3 ,strip.Color(0,0,0));
  strip.show();
  delay(100);
}

void loop() 
{
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  mySerial.print("Accel X: "); mySerial.print(accel.acceleration.x); mySerial.print(" ");
  mySerial.print("  \tY: "); mySerial.print(accel.acceleration.y);       mySerial.print(" ");
  mySerial.print("  \tZ: "); mySerial.print(accel.acceleration.z);     mySerial.println("  \tm/s^2");

  float x = accel.acceleration.x*16;
  float y = accel.acceleration.y*16;
  float z = accel.acceleration.z*16;
  r = ceil(abs(x));
  g = ceil(abs(y));
  b = ceil(abs(z));

  mySerial.print("Color: "); mySerial.print(r); mySerial.print(" ");
  mySerial.print(g); mySerial.print(" "); mySerial.println(b);
  uint32_t color = strip.Color(r,g,b);
  
  strip.setPixelColor(0, color); // Moderately bright green color.
  strip.setPixelColor(1, color); // Moderately bright green color.
  strip.setPixelColor(2, color); // Moderately bright green color.
  strip.setPixelColor(3, color); // Moderately bright green color.
  /*strip.setBrightness(20 + 20*(int)sqrt(lsm.accelData.x * lsm.accelData.x 
                              + lsm.accelData.y * lsm.accelData.y
                              + lsm.accelData.z * lsm.accelData.z));*/
  strip.show();
}
