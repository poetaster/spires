/*
  Spires is copyright 2024, Mark Washeim blueprint@poetaster.de
  GPLv3



*/
#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <util/crc16.h>
#include "AD9833.h"
#include <EncoderButton.h>
#include <DigiPotX9Cxxx.h>

DigiPot pot(A0, A1, 4);

bool debug = true;
// encoder
// the a and b + the button pin large encoders are 6,5,4
EncoderButton eb1(2, 3, 5);


/* these come from rampart bytebeats */
int encoder_pos_last = 0;
long encoder_delta = 0;
int enc_offset = 1; // changes direction
int enc_delta; // which direction
//for program switching
int prog = 1;
int bank = 1;
int pb1 = 1;
int pb1total = 8;
int pb2 = 1;
int pb2total = 28;
int pb3 = 1;
int pb3total = 20;

int numProg = 8;

#include "encoder.h"


/* AUDIO */
const float ContToFreq[63] PROGMEM = {29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00,
                                      58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00,
                                      116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00,
                                      233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00,
                                      466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00,
                                      932.33, 987.77, 1046.50
                                     };
//  each device needs its own select pin.
AD9833 AD[1] =
{
  AD9833(8),
  //AD9833(9)
};  //  4 devices.

// AD9833 communication pins
#define GEN_FSYNC1  8                       // Chip select pin for AD9833 1
#define GEN_FSYNC2  9                       // Chip select pin for AD9833 2
#define GEN_CLK     13                      // CLK and DATA pins are shared with multiple AD9833.
#define GEN_DATA    11
// AD9833 Waveform Module
const int SINE = 0x2000;                    // Define AD9833's waveform register value.
const int SQUARE = 0x2028;                  // When we update the frequency, we need to
const int TRIANGLE = 0x2002;                // define the waveform when we end writing.
const float refFreq = 25000000.0;           // On-board crystal reference frequency
float freq_init1 = 440.0;                    // Initial frequency for Generator 1
float freq_init2 = 442.0;                    // Initial frequency for Generator 2
float freq_target1 = 440.0;                 // Target frequency for Generator 1
float freq_target2 = 442.0;                 // Target frequency for Generator 2
float freq_offset = 1; //4 / 3;

float volume = 1000;  // volume for our machine

// Table with note symbols, used for display
const char *IndexToNote[] = {"C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-"};

// Tables with frequencies from scales around the world
#include "scales.h"

#define BASE_NOTE_FREQUENCY  16.3516 // C-0, Core of note/cent offset calculations, keep it accurate
// Definition of generated frequency range
#define MIN_GENERATED_FREQ  30.0 // Do not change to lower if you like your speakers
#define MAX_GENERATED_FREQ  1000.0
// Definition of binaural generator setting modes
#define MODE_FREQ_FREQ    0
#define MODE_FREQ_OFFSET  1
#define MODE_NOTE_OFFSET  2
#define MODE_SAVE         3

signed char note_target = 47; //A-4, 440Hz
#define NOTE_CALCULATION_OFFSET 10 //base note for calculation is C-0, but note table starts from A#0 

float noteIndex;

/* END AUDIO */
// The number of sensors in your system.
const uint8_t sensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {6, 7};

int led = 9; // for the vactrol
//  analogReference(DEFAULT);  // 5v

VL53L0X sensors[sensorCount];

// variables for runtime control
bool up;
bool cont = true;
bool continuous = false;
float lastVol = 99;
int currentDepth = 9;
bool flutter = false;

void setup()
{


  // setup d for audio volume
  //analogReference(DEFAULT);  // 5v
  //pinMode(led, OUTPUT);
  //analogReference(DEFAULT);
  //analogReference(INTERNAL2V56);
  //pinMode(4, ANALOG);
  // from audio
  //pinMode(A0, OUTPUT);
  //pinMode(A1, OUTPUT);
  // Define pins function
  //pinMode(GEN_FSYNC1, OUTPUT);                      // GEN_FSYNC1
  //pinMode(GEN_FSYNC2, OUTPUT);                      // GEN_FSYNC2


  SPI.begin();
  delay(50);

  //while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  if (debug) {


    Serial.println(__FILE__);
    Serial.print("AD9833_LIB_VERSION: ");
    Serial.println(AD9833_LIB_VERSION);
    Serial.println();

  }

  // start amp first
  //AMP.begin(10, 14);
  //Serial.println(AMP.getVolume(2));

  AD[0].begin();
  //AD[1].begin();
  //  A major chord
  AD[0].setWave(AD9833_TRIANGLE);
  //AD[1].setWave(AD9833_SINE);
  AD[0].setFrequency(240.00, 0);     //  A
  //AD[1].setFrequency(242.00, 0);     //  C#
  // END from audio



  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left atSerial.println(freq_init1);
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(50);
    //sensors[i].setMeasurementTimingBudget(70000); //  adjust this value to move to slower note slurs/jumps

  }


  // setup for the encoder with button
  // Link the event(s) to your function
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  eb1.setLongPressHandler(onEb1LongPress, true);
  eb1.setEncoderPressedHandler(onEb1PressTurn);

  //sensors[1].setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  // sensors[1].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //sensors[1].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  //
  sensors[0].setMeasurementTimingBudget(70000);
  sensors[1].setMeasurementTimingBudget(70000);
  Serial.println(sensors[0].readRangeSingleMillimeters());
  //analogWrite(4, 255);



  if (debug) {
    Serial.println("end setup");
  }
  pot.set(99);


}

int steps = 4;
int currentVolume = 99;
int bottom;

void loop()
{

  float temp1;
  int temp2;
  float voltemp;

  if (sensors[0].timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }

  volume = sensors[0].readRangeSingleMillimeters();




  //Tremello(); //


  //analogWrite(led, volume);
  //analogWrite(4, volume); // D4 is the dac on the LGT8F
  //Serial.println(volume);


  // not used since we're using an offset for both osc
  //freq_target2 = pgm_read_float(&IndexToFreq[map(sensors[0].readRangeContinuousMillimeters(), 10, 1300, 0, 31)]) ; // freq_init2;
  //Serial.println(temp1);
  //temp2 = freq_target2;
  //temp2 = int(map(sensors[1].readRangeContinuousMillimeters(), 0, 1300, 31, 0));
  freq_target2 = sensors[1].readRangeSingleMillimeters();
  if (freq_target2  < 1300 ) {
    if ( ! continuous ) {
      //AD[0].setWave(AD9833_TRIANGLE);
      temp2 = int(map(freq_target2, 50, 1300, 28, 0));
    } else {
      //AD[0].setWave(AD9833_SINE);
      temp2 = map(freq_target2, 50, 1300, 780, 420);
      //temp2 = map(freq_target2, 10, 1300, 58, 18);
      //temp2 = pgm_read_float( &ContToFreq[temp2 ]);
    }
    //if (debug) Serial.print("freq:");
    //if (debug )  Serial.println(temp2);
  }

  switch (prog) {
    case 1:
      temp1 = pgm_read_float( &IndexToFreq[temp2 ]);
      break;
    case 2:
      temp1 = pgm_read_float( &BayatiToFreq[temp2 ]);
      break;
    case 3:
      temp1 = pgm_read_float( &nawaAtharToFreq[temp2]);
      break;
    case 4:
      temp1 = pgm_read_float( &farafahzaToFreq[temp2 ]);
      break;
    case 5:
      temp1 = pgm_read_float( &nikrizToFreq[temp2 ]);
      break;
    case 6:
      temp1 = pgm_read_float( &phrygianToFreq[temp2 ]);
      break;
    case 7:
      temp1 = pgm_read_float( &romaMinorToFreq[temp2 ]);
      break;
    case 8:
      temp1 = pgm_read_float( &partch1ToFreq[temp2 ]);
      break;
  }


  // here we either glide up or down
  if ( temp1 < 1600  && temp1 > 30 && continuous == false) {
    if (freq_init1 > temp1) {
      cont = GlideFreq(freq_init1, temp1, false);

    } else if (freq_init1 < temp1) {
      cont = GlideFreq(freq_init1, temp1, true);
    }
    freq_init1 = temp1;
  } else if (temp2 < 800  && temp2 > 100 && continuous == true ) { //&& abs(freq_init1 - temp2) > 5 ) {
    if (freq_init1 > temp2) {
      cont = GlideContinuous(freq_init1, temp2, false);

    } else if (freq_init1 < temp2) {
      cont = GlideContinuous(freq_init1, temp2, true);
    }
    freq_init1 = temp2;
  }
  while (cont == false) {
    ; //nop
  }
  if (volume < 8190.00) {
    //if (debug )    Serial.println(volume);
    // alwasy adjust volume as fast as possible and don't continue loop until it's finished.
    volume = map(volume, 50, 1300, 99, 25);

    if (volume > 99) volume = 99;
    if (volume < 25 ) volume = 25;

    //if (debug ) Serial.print(" - ");

    currentDepth = (int)volume;
    bottom = 99 - currentDepth;
    if (! flutter && volume != lastVol) {
      if (debug ) Serial.println(volume);
      if (debug ) Serial.println(lastVol);
      flutter = GlideVolume( volume , lastVol );
      
    }
    lastVol = volume;
  }

  eb1.update(); // respond to encoder/button


}
// END LOOP

// Function to glide notes up/down
bool GlideFreq(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from + 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);
  }
  return true;
}
// Function to glide notes up/down
bool GlideContinuous(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from + 0.1;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 0.1;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);
  }
  return true;
}



// Function to glide volume up/down
bool GlideVolume(float from, float too) {
  flutter = true;
  //make sure we complete the glides before the loop proceeds
  float ft = from;

  if (from < too) {
    while (ft < too) {
      //analogWrite(4, from);
      ft = ft + 1;
      //MCP.setValue(from);
      pot.increase(1);

    }

  } else {
    while (ft > too) {
      //analogWrite(4, from);
      ft = ft - 1;
      //MCP.setValue(from);
      pot.decrease(1);


    }
  }
  return false;
}
// Function to glide volume up/down
void Tremello() {

  //make sure we complete the glides before the loop proceeds
  if (currentVolume < bottom ) {
    currentVolume = currentVolume + 1;
    pot.increase(1);
  } else {
    currentVolume = currentVolume - 1;
    pot.decrease(1);
    ;
  }

  if (debug )    Serial.println( currentVolume);
}
