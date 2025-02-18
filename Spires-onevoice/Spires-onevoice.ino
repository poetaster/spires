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

// # define ENCODER_DO_NOT_USE_INTERRUPTS
#include <EncoderButton.h>

bool debug = false;
// variables for runtime control
bool up;
bool cont = true;
volatile float lastvol = 255;
unsigned int lastPos;
bool continuous = false;
bool sine = true;

int led = 2; // for display led

unsigned int cSpeed = 200000;
//  each device needs its own select pin.
AD9833 AD[1] =
{
  AD9833(8),
  //AD9833(9)
};  //  4 devices.

// The number of sensors in your system.
const uint8_t sensorCount = 1; // original used two. switched to ldr for volume
// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {7};

VL53L0X sensors[sensorCount];


// encoder
// the a and b + the button pin large encoders are 6,5,4
EncoderButton eb1(3, 6, 4);


/* these come from rampart bytebeats */
int encoder_pos_last = 0;
long encoder_delta = 0;
int enc_offset = 1; // changes direction
int enc_delta; // which direction
//for program switching
int prog = 0;
int bank = 0;
int pb1 = 0;
int pb1total = 6;
int pb2 = 0;
int pb2total = 6;
int pb3 = 0;
int pb3total = 6;

int numProg = 18;
int numBank = 3;

#include "encoder.h"


/* AUDIO */
const float ContToFreq[63] PROGMEM = {29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00,
                                      58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00,
                                      116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00,
                                      233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00,
                                      466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00,
                                      932.33, 987.77, 1046.50
                                     };

// AD9833 communication pins
#define GEN_FSYNC1  8                       // Chip select pin for AD9833 1
#define GEN_FSYNC2  9                       // Chip select pin for AD9833 2
#define GEN_CLK     12                      // CLK and DATA pins are shared with multiple AD9833.
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

float volume = 255;  // volume for our machine

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



//  analogReference(DEFAULT);  // 5v

void setup()
{


  // setup led for audio volume
  //analogReference(DEFAULT);  // 5v
  pinMode(led, OUTPUT);
  // DAC stuff, not used
  //analogReference(DEFAULT);
  //analogReference(INTERNAL2V56);
  //pinMode(4, ANALOG);
  // from audio

  SPI.begin();
  delay(50);

  //while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  Serial.println(__FILE__);
  Serial.print("AD9833_LIB_VERSION: ");
  Serial.println(AD9833_LIB_VERSION);
  Serial.println();

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

  //can be also read from eeprom
  freq_target1 = freq_init1;
  freq_target2 = freq_init2;

  //noteIndex = FreqToNote(freq_target1);

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
    sensors[i].setSignalRateLimit(0.25);
    sensors[i].startContinuous(50);
    sensors[i].setMeasurementTimingBudget(cSpeed); //  adjust this value to move to slower note slurs/jumps
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    //sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    //sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  }


  // setup for the encoder with button
  // Link the event(s) to your function
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  eb1.setLongPressHandler(onEb1LongPress, true);
  eb1.setEncoderPressedHandler(onEb1PressTurn);

  //sensors[0].setSignalRateLimit(0.15);

  //sensors[0].setMeasurementTimingBudget(500000);
  lastPos = sensors[0].readRangeContinuousMillimeters();
  if (debug) Serial.println(lastPos);
  //analogWrite(4, 255);
  continuous = false;
}


void loop()
{

  float temp1;
  int temp2;
  
  //freq_target1 = sensors[0].readRangeContinuousMillimeters(); //freq_init1;
  if (sensors[0].timeoutOccurred()) {
    if (debug) Serial.print(" TIMEOUT");
  }

  //analogWrite(4, volume); // D4 is the dac on the LGT8F
  // readRangeContinuousMillimeters
  freq_target2 = sensors[0].readRangeContinuousMillimeters();
  
  if (freq_target2  < 700 ) {
    if (debug) Serial.println(freq_target2);

    if ( ! continuous ) {
      if (abs(lastPos - freq_target2) > 15) { // NOT sure
        temp2 = int(map(freq_target2, 10, 700, 28, 0));
      }
      lastPos = freq_target2;
    } else {
      temp2 = map(freq_target2, 10, 700, 680, 40);
      lastPos = freq_target2;

    }
  }

  switch (bank) {
    case 0:
      switch (pb1) {
        case 0:
          temp1 = pgm_read_float( &Nahawand[temp2 ]);
          break;
        case 1:
          temp1 = pgm_read_float( &Bayati[temp2 ]);
          break;
        case 2:
          temp1 = pgm_read_float( &NawaAthar[temp2]);
          break;
        case 3:
          temp1 = pgm_read_float( &Farafahza[temp2 ]);
          break;
        case 4:
          temp1 = pgm_read_float( &Nikriz[temp2 ]);
          break;
        case 5:
          temp1 = pgm_read_float( &Bhairav[temp2 ]);
          break;
      }
      break;
    case 1:
      switch (pb2) {
        case 0:
          temp1 = pgm_read_float( &PhrygianFreq[temp2 ]);
          break;
        case 1:
          temp1 = pgm_read_float( &RomanMinor[temp2 ]);
          break;
        case 2:
          temp1 = pgm_read_float( &NeapolitanMinor[temp2 ]);
          break;
        case 3:
          temp1 = pgm_read_float( &MelodicMinor[temp2 ]);
          break;
        case 4:
         temp1 = pgm_read_float( &Spanish[temp2 ]);  
          break;
        case 5:
          temp1 = pgm_read_float( &LydianMinor[temp2 ]); 
          break;
      }
      break;
    case 2:
      switch (pb3) {
        case 0:
          temp1 = pgm_read_float( &Suznak[temp2 ]);
          break;
        case 1:
          temp1 = pgm_read_float( &Zamzam[temp2 ]);
          break;
        case 2:
          temp1 = pgm_read_float( &KijazKarKurd[temp2 ]); 
          break;
        case 3:
          temp1 = pgm_read_float( &RomanianMinor[temp2 ]);
          break;
        case 4:
          temp1 = pgm_read_float( &Enigmatic[temp2 ]);
          break;
        case 5:
          temp1 = pgm_read_float( Partch1[temp2 ]);
          break;
      }
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
  } else if ( temp2 < 700 && temp2 > 80 && continuous == true ) { //&& abs(freq_init1 - temp2) > 5 ) {
    //temp2 = pgm_read_float( &ContToFreq[temp2 ] ); //ContToFreq
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

  eb1.update(); // respond to encoder/button
  //delay(10);
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
      from = from + 0.5;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 0.5;

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
      from = from + 0.07;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 0.07;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);
  }
  return true;
}

// Function to glide volume up/down
bool GlideVolume(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  if (up) {
    while (from < too) {
      analogWrite(4, from);
      from = from + 1;

    }

  } else {
    while (from > too) {
      analogWrite(4, from);
      from = from - 1;

    }
  }
  lastvol = too;
  return true;
}
