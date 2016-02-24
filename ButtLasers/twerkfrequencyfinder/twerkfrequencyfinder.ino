#include <SPI.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

#define PIN            7
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      4

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

sensors_event_t accel, mag, gyro, temp;

//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally
int timer[10];//sstorage for timing of events
int slope[10];//storage for slope of events
unsigned int totalTimer;//used to calculate period
unsigned int period;//storage for period of wave
byte index = 0;//current storage index
float frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for decided whether you have a match
byte noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 3;//slope tolerance- adjust this if you need
int timerTol = 10;//timer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 30;//raise if you have a very noisy signal

const int rx=9;
const int tx=10;
SoftwareSerial mySerial(rx, tx);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  
  mySerial.println(F("------------------------------------"));
  mySerial.print  (F("Sensor:       ")); mySerial.println(accel.name);
  mySerial.print  (F("Driver Ver:   ")); mySerial.println(accel.version);
  mySerial.print  (F("Unique ID:    ")); mySerial.println(accel.sensor_id);
  mySerial.print  (F("Max Value:    ")); mySerial.print(accel.max_value); mySerial.println(F(" m/s^2"));
  mySerial.print  (F("Min Value:    ")); mySerial.print(accel.min_value); mySerial.println(F(" m/s^2"));
  mySerial.print  (F("Resolution:   ")); mySerial.print(accel.resolution); mySerial.println(F(" m/s^2"));  
  mySerial.println(F("------------------------------------"));
  mySerial.println(F(""));

  mySerial.println(F("------------------------------------"));
  mySerial.print  (F("Sensor:       ")); mySerial.println(mag.name);
  mySerial.print  (F("Driver Ver:   ")); mySerial.println(mag.version);
  mySerial.print  (F("Unique ID:    ")); mySerial.println(mag.sensor_id);
  mySerial.print  (F("Max Value:    ")); mySerial.print(mag.max_value); mySerial.println(F(" uT"));
  mySerial.print  (F("Min Value:    ")); mySerial.print(mag.min_value); mySerial.println(F(" uT"));
  mySerial.print  (F("Resolution:   ")); mySerial.print(mag.resolution); mySerial.println(F(" uT"));  
  mySerial.println(F("------------------------------------"));
  mySerial.println(F(""));

  mySerial.println(F("------------------------------------"));
  mySerial.print  (F("Sensor:       ")); mySerial.println(gyro.name);
  mySerial.print  (F("Driver Ver:   ")); mySerial.println(gyro.version);
  mySerial.print  (F("Unique ID:    ")); mySerial.println(gyro.sensor_id);
  mySerial.print  (F("Max Value:    ")); mySerial.print(gyro.max_value); mySerial.println(F(" rad/s"));
  mySerial.print  (F("Min Value:    ")); mySerial.print(gyro.min_value); mySerial.println(F(" rad/s"));
  mySerial.print  (F("Resolution:   ")); mySerial.print(gyro.resolution); mySerial.println(F(" rad/s"));  
  mySerial.println(F("------------------------------------"));
  mySerial.println(F(""));

  mySerial.println(F("------------------------------------"));
  mySerial.print  (F("Sensor:       ")); mySerial.println(temp.name);
  mySerial.print  (F("Driver Ver:   ")); mySerial.println(temp.version);
  mySerial.print  (F("Unique ID:    ")); mySerial.println(temp.sensor_id);
  mySerial.print  (F("Max Value:    ")); mySerial.print(temp.max_value); mySerial.println(F(" C"));
  mySerial.print  (F("Min Value:    ")); mySerial.print(temp.min_value); mySerial.println(F(" C"));
  mySerial.print  (F("Resolution:   ")); mySerial.print(temp.resolution); mySerial.println(F(" C"));  
  mySerial.println(F("------------------------------------"));
  mySerial.println(F(""));
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
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

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

byte rColor, gColor, bColor;

void setup(void) 
{
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setPixelColor(0, pixels.Color(0,150,0)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  
  pinMode(rx,INPUT);
  pinMode(tx,OUTPUT);
  mySerial.begin(38400);
  mySerial.println("Setting interrupts");
  mySerial.println(F("LSM9DS0 9DOF Sensor Test")); mySerial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    mySerial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  mySerial.println(F("Found LSM9DS0 9DOF"));
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  rColor = 0;
  gColor = 0;
  bColor = 0;
  
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1563;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (0 << CS10);    // 256 prescaler
  TCCR1B |= (0 << CS11);    // 256 prescaler
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  
  mySerial.println("Starting");
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  prevData = newData;//store previous value
  newData = accel.acceleration.x;//get value from A0
  if (prevData < 127 && newData >=127){//if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope-maxSlope)<slopeTol){//if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){//new max slope just reset
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0]-timer[index])<timerTol && abs(slope[0]-newSlope)<slopeTol){//if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i=0;i<index;i++){
          totalTimer+=timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
      }
      else{//crossing midpoint but not match
        index++;//increment index
        if (index > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope){//if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else{//slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch>9){
        reset();
      }
    }
  }
    
  if (newData == 0 || newData == 1023){//if clipping
    PORTB |= B00100000;//set pin 13 high- turn on clipping indicator led
    clipping = 1;//currently clipping
  }
  
  time++;//increment timer at rate of 38.5kHz
  
  ampTimer++;//increment amplitude timer
  if (abs(127-ADCH)>maxAmp){
    maxAmp = abs(127-ADCH);
  }
  if (ampTimer==1000){
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}

void reset(){//clea out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void loop(void) 
{  
  /* Get a new sensor event */ 

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  if(rColor + accel.acceleration.x < 0) {
    rColor = 0;
  } else if(rColor + accel.acceleration.x > 255){
    rColor = 255;
  } else {
    rColor += accel.acceleration.x;
  }
  if(gColor + accel.acceleration.y < 0) {
    gColor = 0;
  } else if(gColor + accel.acceleration.y > 255) {
    gColor = 255;
  } else {
    gColor += accel.acceleration.y;
  }
  if(bColor + accel.acceleration.z < 0) {
    bColor = 0;
  } else if(bColor + accel.acceleration.z > 255){
    bColor = 255;
  } else {
    bColor += accel.acceleration.z;
  }
  
  rColor = abs(accel.acceleration.x)*11;
  gColor = abs(accel.acceleration.y)*11;
  bColor = abs(accel.acceleration.z)*11;
  
  pixels.setPixelColor(0, pixels.Color(rColor,gColor,bColor)); // Moderately bright green color.
  pixels.setPixelColor(1, pixels.Color(rColor,gColor,bColor)); // Moderately bright green color.
  pixels.setPixelColor(2, pixels.Color(rColor,gColor,bColor)); // Moderately bright green color.
  pixels.setPixelColor(3, pixels.Color(rColor,gColor,bColor)); // Moderately bright green color.
  pixels.show();

  // print out accelleration data
  //mySerial.print("Accel X: "); mySerial.print(accel.acceleration.x); mySerial.print(" ");
  //mySerial.print("  \tY: "); mySerial.print(accel.acceleration.y);       mySerial.print(" ");
  //mySerial.print("  \tZ: "); mySerial.print(accel.acceleration.z);     mySerial.println("  \tm/s^2");
  mySerial.print(period);
    mySerial.println(" hz");
  if (checkMaxAmp>ampThreshold){
    frequency = 38462/float(period);//calculate frequency timer rate/period
  
    //print results
    mySerial.print(frequency);
    mySerial.println(" hz");
  }
  delay(50);
}
