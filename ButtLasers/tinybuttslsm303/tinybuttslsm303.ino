#include "TinyWireM.h"
#include "Adafruit_LSM303_U.h"
#include "Adafruit_Neopixel.h"

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
#define N_PIXELS  4  // Number of pixels you are using
#define LED_PIN    1  // NeoPixel LED strand is connected to GPIO #1 / D1
Adafruit_NeoPixel  strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

byte rColor, gColor, bColor;

void setup() 
{
  //Serial.begin(9600);
  strip.begin();
  strip.setPixelColor(0,strip.Color(255,0,0));
  strip.show();
  // Try to initialise and warn if we couldn't detect the chip
  if (!accel.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
  delay(100);
}

void loop() 
{
  sensors_event_t event; 
  accel.getEvent(&event);
  rColor = abs(event.acceleration.x)*8;
  gColor = abs(event.acceleration.y)*8;
  bColor = abs(event.acceleration.z)*8;

  uint32_t color = strip.Color(rColor,gColor,bColor);
  
  strip.setPixelColor(0, color); // Moderately bright green color.
  strip.setPixelColor(1, color); // Moderately bright green color.
  strip.setPixelColor(2, color); // Moderately bright green color.
  strip.setPixelColor(3, color); // Moderately bright green color.
  strip.show();
  //strip.setBrightness(120);
  /*strip.setBrightness((int)sqrt(event.acceleration.x * event.acceleration.x 
                              + event.acceleration.y * event.acceleration.y
                              + event.acceleration.z * event.acceleration.z));*/
}
