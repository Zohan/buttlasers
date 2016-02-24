#include "TinyWireM.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Neopixel.h"

#define N_PIXELS  4  // Number of pixels you are using
#define LED_PIN    1  // NeoPixel LED strand is connected to GPIO #1 / D1
Adafruit_NeoPixel  strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

byte rColor, gColor, bColor;

void setup() 
{
  //Serial.begin(9600);
  strip.begin();
  strip.setPixelColor(0,strip.Color(255,0,0));
  strip.setPixelColor(1,strip.Color(255,0,0));
  strip.setPixelColor(2,strip.Color(255,0,0));
  strip.setPixelColor(3 ,strip.Color(255,0,0));
  strip.show();
  
  
  // Try to initialise and warn if we couldn't detect the chip
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  strip.setPixelColor(0 ,strip.Color(0,255,0));
  strip.show();
  delay(250);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  strip.setPixelColor(0,strip.Color(0,0,255));
  strip.show();
  delay(250);
  
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
    strip.setPixelColor(0,strip.Color(0,255,0));
    strip.show();
  }
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
  rColor = abs(lsm.accelData.x)*8;
  gColor = abs(lsm.accelData.y)*8;
  bColor = abs(lsm.accelData.z)*8;

  uint32_t color = strip.Color(rColor,gColor,bColor);
  
  strip.setPixelColor(0, color); // Moderately bright green color.
  strip.setPixelColor(1, color); // Moderately bright green color.
  strip.setPixelColor(2, color); // Moderately bright green color.
  strip.setPixelColor(3, color); // Moderately bright green color.
  /*strip.setBrightness(20 + 20*(int)sqrt(lsm.accelData.x * lsm.accelData.x 
                              + lsm.accelData.y * lsm.accelData.y
                              + lsm.accelData.z * lsm.accelData.z));*/
  strip.show();
}
