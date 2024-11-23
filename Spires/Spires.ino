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

bool debug = false;
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
int pb1total = 5;
int pb2 = 1;
int pb2total = 28;
int pb3 = 1;
int pb3total = 20;

int numProg = 5;

#include "encoder.h"


/* AUDIO */

//  each device needs its own select pin.
AD9833 AD[2] =
{
  AD9833(8),
  AD9833(9)
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
float freq_offset = 2.0;

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
// The number of sensors in your system.
const uint8_t sensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { 6, 7 };

int led = 10; // for the vactrol
//  analogReference(DEFAULT);  // 5v
VL53L0X sensors[sensorCount];

void setup()
{


  // setup led for audio volume
  //analogReference(DEFAULT);  // 5v
  pinMode(led, OUTPUT);
  analogReference(DEFAULT);
  pinMode(4, ANALOG);
  // from audio

  // Define pins function
  //pinMode(GEN_FSYNC1, OUTPUT);                      // GEN_FSYNC1
  //pinMode(GEN_FSYNC2, OUTPUT);                      // GEN_FSYNC2

  SPI.begin();
  delay(50);

  AD[0].begin();
  AD[1].begin();
  //  A major chord
  AD[0].setWave(AD9833_TRIANGLE);
  AD[1].setWave(AD9833_TRIANGLE);
  AD[0].setFrequency(440.00, 0);     //  A
  AD[1].setFrequency(554.37, 0);     //  C#

  /* syntherjack, for reference
    Set both AD9833 CS pins to high (don't accept data)
    digitalWrite(GEN_FSYNC1, HIGH);
    digitalWrite(GEN_FSYNC2, HIGH);
    AD9833reset(GEN_FSYNC1);                                   // Reset AD9833 module after power-up.
    delay(50);
    AD9833init(freq_init1, SQUARE, GEN_FSYNC1);                  // Set the frequency and Sine Wave output

    AD9833reset(GEN_FSYNC2);                                   // Reset AD9833 module after power-up.
    delay(50);
    AD9833init(freq_init2, SQUARE, GEN_FSYNC2);                  // Set the frequency and Sine Wave output
  */
  //can be also readed from eeprom
  freq_target1 = freq_init1;
  freq_target2 = freq_init2;

  //noteIndex = FreqToNote(freq_target1);

  // END from audio

  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  Serial.println(__FILE__);
  Serial.print("AD9833_LIB_VERSION: ");
  Serial.println(AD9833_LIB_VERSION);
  Serial.println();

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
    sensors[i].setMeasurementTimingBudget(80000); //  adjust this value to move to slower note slurs/jumps

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
  //sensors[1].setMeasurementTimingBudget(500000);

}
bool up;
bool cont = true;
float lastvol = 150;

void loop()
{
  /* test sensors
     for (uint8_t i = 0; i < sensorCount; i++)
    {
    Serial.print(sensors[i].readRangeContinuousMillimeters()*0.2);
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print('\t');
    }
  */

  eb1.update(); // respond to encoder/button

  float temp1;
  float temp2;

  //freq_target1 = sensors[0].readRangeContinuousMillimeters(); //freq_init1;

  volume = map(sensors[0].readRangeSingleMillimeters(), 100, 1000, 255, 0);
  if (volume > 255 ) {
    volume = 255;
  } else if (volume < 0) {
    volume = 0;
  }
  //analogWrite(led, volume);
  analogWrite(4, volume); // D4 is the dac on the LGT8F
  //Serial.println(volume);


  // not used since we're using an offset for both osc
  //freq_target2 = pgm_read_float(&IndexToFreq[map(sensors[0].readRangeContinuousMillimeters(), 10, 1300, 0, 31)]) ; // freq_init2;
  //Serial.println(temp1);
  //temp2 = freq_target2;

  switch (prog) {
    case 1:
      temp1 = pgm_read_float( &IndexToFreq[map(sensors[1].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);
      break;
    case 2:
      temp1 = pgm_read_float( &BayatiToFreq[map(sensors[1].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);
      break;
    case 3:
      temp1 = pgm_read_float( &nawaAtharToFreq[map(sensors[1].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);
      break;
    case 4:
      temp1 = pgm_read_float( &farafahzaToFreq[map(sensors[1].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);
      break;
    case 5:
      temp1 = pgm_read_float( &nikrizToFreq[map(sensors[1].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);
      break;
  }


  // here we either glide up or down
  if ( temp1 < 1000  && temp1 > 50) {
    if (freq_init1 > temp1) {
      cont = GlideFreq(freq_init1, temp1, false);

    } else if (freq_init1 < temp1) {
      cont = GlideFreq(freq_init1, temp1, true);
    }
    freq_init1 = temp1;
  }
  while (cont == false) {
    ; //nop
  }


}
// END LOOP

// Function to glide notes up/down
bool GlideFreq(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      AD[1].setFrequency(from * freq_offset);
      from = from + 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      AD[1].setFrequency(from * freq_offset);
      from = from - 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    AD[1].setFrequency(from * freq_offset);
  }
  return true;
}

// Function to glide volume up/down
bool GlideVolume(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  if (up) {
    while (from < too) {
      from = from + 1;

    }

  } else {
    while (from > too) {

      from = from - 1;

    }
  }
  return true;
}




/* from synther jack just for reference as unused */

/* Function converting frequency to offset from BASE_NOTE_FREQUENCY
  float FreqToNote(float frequency) {
  float x = (frequency / BASE_NOTE_FREQUENCY);
  float y = 12.0 * log(x) / log(2.0);
  return y;
  }*/

// AD9833 related functions
/* AD9833 documentation advises a 'Reset' on first applying power.
  void AD9833reset(int syncpin) {
  WriteRegister(0x100, syncpin);   // Write '1' to AD9833 Control register bit D8.
  delay(10);
  }
*/

/* Set the frequency and waveform registers in the selected via syncpin AD9833
  void AD9833init(float frequency, int waveform, int syncpin) {
  long freq_word = (frequency * pow(2, 28)) / refFreq;

  int MSB = (int)((freq_word & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  int LSB = (int)(freq_word & 0x3FFF);

  //Set control bits 15 ande 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000;

  WriteRegister(0x2100, syncpin);               // Allow 28 bits to be loaded into a frequency register in two consecutive writes and reset internal registers to 0
  WriteRegister(LSB, syncpin);                  // Write lower 14 bits to AD9833 registers
  WriteRegister(MSB, syncpin);                  // Write upper 14 bits to AD9833 registers
  WriteRegister(0xC000, syncpin);               // Set phase register
  WriteRegister(waveform, syncpin);             // Exit & Reset to SINE
  }
*/

/* Set the frequency registers in the AD9833.
  void AD9833set(float frequency, int syncpin) {

  long freq_word = (frequency * pow(2, 28)) / refFreq;

  int MSB = (int)((freq_word & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  int LSB = (int)(freq_word & 0x3FFF);

  // Set control bits 15 ande 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000;

  // Set frequency registers without reseting or changing phase to avoid clicking
  WriteRegister(SQUARE, syncpin);               // Allow 28 bits to be loaded into a frequency register in two consecutive writes
  WriteRegister(LSB, syncpin);                  // Write lower 14 bits to AD9833 registers
  WriteRegister(MSB, syncpin);                  // Write upper 14 bits to AD9833 registers
  }
*/
/* Write to AD9833 register
  void WriteRegister(int dat, int syncpin) {
  // Display and AD9833 use different SPI MODES so it has to be set for the AD9833 here.
  SPI.setDataMode(SPI_MODE2);

  digitalWrite(syncpin, LOW);           // Set FSYNC low before writing to AD9833 registers
  delayMicroseconds(10);              // Give AD9833 time to get ready to receive data.

  SPI.transfer(highByte(dat));        // Each AD9833 register is 32 bits wide and each 16
  SPI.transfer(lowByte(dat));         // bits has to be transferred as 2 x 8-bit bytes.

  digitalWrite(syncpin, HIGH);          //Write done. Set FSYNC high
  }
*/
