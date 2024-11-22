/*
  This example shows how to set up and read multiple VL53L1X sensors connected to
  the same I2C bus. Each sensor needs to have its XSHUT pin connected to a
  different Arduino pin, and you should change sensorCount and the xshutPins array
  below to match your setup.

  For more information, see ST's application note AN4846 ("Using multiple VL53L0X
  in a single design"). The principles described there apply to the VL53L1X as
  well.
*/
#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <util/crc16.h>
#include "AD9833.h"

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

float volume = 255;  // volume for our machine

// Table with note symbols, used for display
const char *IndexToNote[] = {"C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-"};

// Table with note frequencies, from A#0 to C-6, used for freq <-> note conversion
/*
 * const float IndexToFreq[63] PROGMEM = {29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00,
                                       58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00,
                                       116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00,
                                       233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00,
                                       466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00,
                                       932.33, 987.77, 1046.50
                                      };
                                      */
//nahawand raga up and down
const float IndexToFreq[32] PROGMEM = {
  65.40639132515, 73.416191979318, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 123.47082531372,
  116.54094037925,
  //65.40639132515, 73.416191979318, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925,
  130.8127826503, 146.83238395864, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 246.94165062743, 
  //130.8127826503, 146.83238395864, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 233.08188075851, 
  233.08188075851,
  261.6255653006, 293.66476791727, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 493.88330125487, 
  //261.6255653006, 293.66476791727, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 466.16376151701,
  466.16376151701, 
  523.2511306012, 587.32953583454, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 987.76660250974, 
  //523.2511306012, 587.32953583454, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 932.32752303403,
  932.32752303403,
  
};

//nahawand raga up and down
const float BayatiToFreq[32] PROGMEM = {
69.295657744138, 77.781745930394, 82.406889228065, 92.498605677695,
65.40639132515, 71.326175505781, 77.781745930466, 87.30705785815, 97.998858995279, 103.82617439479, 116.54094037925,
130.8127826503, 142.65235101156, 155.56349186093, 174.6141157163, 195.99771799056, 207.65234878959, 233.08188075851,
261.6255653006, 285.30470202312, 311.12698372187, 349.2282314326, 391.99543598112, 415.30469757918, 466.16376151701,
523.2511306012, 570.60940404625, 622.25396744373, 698.4564628652, 783.99087196223, 830.60939515835, 932.32752303403,
};


const float nawaAtharToFreq[32] PROGMEM = {
73.41619197925, 77.781745930394, 82.406889228065, 97.998858995188,
103.82617439499, 116.54094037947, 123.47082531395, 146.8323839585, 155.56349186079, 164.81377845613, 195.99771799038,
207.65234878997, 233.08188075894, 246.94165062789, 293.664767917, 311.12698372158, 329.62755691226, 391.99543598075,
415.30469757995, 466.16376151787, 493.88330125578, 587.329535834, 622.25396744316, 659.25511382452, 783.99087196151,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 830.60939515835,
};

const float farafahzaToFreq[32] PROGMEM = {
65.406391325119, 69.29565774417, 77.78174593043, 87.30705785811, 92.498605677738, 103.82617439475 ,
116.54094037952, 130.81278265024, 138.59131548834, 155.56349186086, 174.61411571622, 184.99721135548, 207.65234878949 ,
233.08188075904, 261.62556530048, 277.18263097668, 311.12698372172, 349.22823143244, 369.99442271095, 415.30469757899 ,
466.16376151809, 523.25113060096, 554.36526195336, 622.25396744344, 698.45646286488, 739.9888454219, 830.60939515797 ,
932.32752303618, 1046.5022612019, 1108.7305239067, 1244.5079348869,
};

const float nikrizToFreq[32] PROGMEM = {
65.40639132515, 73.416191979318, 77.781745930466, 92.49860567778, 97.998858995279, 109.99999999977, 116.54094037925,
130.8127826503, 146.83238395864, 155.56349186093, 184.99721135556, 195.99771799056, 219.99999999954, 233.08188075851,
261.6255653006, 293.66476791727, 311.12698372187, 369.99442271112, 391.99543598112, 439.99999999908, 466.16376151701,
523.2511306012, 587.32953583454, 622.25396744373, 739.98884542224, 783.99087196223, 879.99999999817, 932.32752303403,
1046.5022612024, 1174.6590716691, 1244.5079348875, 1479.9776908445,
};
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
  analogReference(INTERNAL2V56);
  pinMode(DAC0, ANALOG);
  // from audio

  // Define pins function
  //pinMode(GEN_FSYNC1, OUTPUT);                      // GEN_FSYNC1
  //pinMode(GEN_FSYNC2, OUTPUT);                      // GEN_FSYNC2

  SPI.begin();
  delay(50);

  AD[0].begin();
  AD[1].begin();
  // Set both AD9833 CS pins to high (don't accept data)
  //digitalWrite(GEN_FSYNC1, HIGH);
  //digitalWrite(GEN_FSYNC2, HIGH);

  //  A major chord
  AD[0].setWave(AD9833_TRIANGLE);
  AD[1].setWave(AD9833_TRIANGLE);
  AD[0].setFrequency(440.00, 0);     //  A
  AD[1].setFrequency(554.37, 0);     //  C#


  //AD9833reset(GEN_FSYNC1);                                   // Reset AD9833 module after power-up.
  //delay(50);
  //AD9833init(freq_init1, SQUARE, GEN_FSYNC1);                  // Set the frequency and Sine Wave output

  //AD9833reset(GEN_FSYNC2);                                   // Reset AD9833 module after power-up.
  //delay(50);
  //AD9833init(freq_init2, SQUARE, GEN_FSYNC2);                  // Set the frequency and Sine Wave output

  //can be also readed from eeprom
  freq_target1 = freq_init1;
  freq_target2 = freq_init2;

  noteIndex = FreqToNote(freq_target1);

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
    sensors[i].setMeasurementTimingBudget(80000);

  }



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
  /*for (uint8_t i = 0; i < sensorCount; i++)
    {
    Serial.print(sensors[i].readRangeContinuousMillimeters()*0.2);
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print('\t');
    }*/
  float temp1;
  float temp2;

  //freq_target1 = sensors[0].readRangeContinuousMillimeters(); //freq_init1;

  volume = map(sensors[1].readRangeSingleMillimeters(), 110, 1400, 0, 255);
  if (volume > 255 ) {
    volume = 255;
  }
  analogWrite(led, int(volume));
  
  Serial.println(volume);
  //Serial.println();

  //freq_target2 = pgm_read_float(&IndexToFreq[map(sensors[0].readRangeContinuousMillimeters(), 10, 1300, 0, 31)]) ; // freq_init2;

  temp1 = pgm_read_float(&IndexToFreq[map(sensors[0].readRangeContinuousMillimeters(), 10, 1300, 31, 0) ]);

  //Serial.println(temp1);
  temp2 = freq_target2;

  
  if ( temp1 < 1000  && temp1 > 50) {
    
    if (freq_init1 > temp1) {
      cont = GlideFreq(freq_init1, temp1, false);

    } else if (freq_init1 < temp1) {
      cont = GlideFreq(freq_init1, temp1, true);
    }
    
    //AD[0].setFrequency(temp1);
    //AD9833set( sensors[0].readRangeContinuousMillimeters() , GEN_FSYNC1);

    freq_init1 = temp1;

  }
  while (cont == false) {
    ; //nop
  }
  //Serial.println(freq_init1);



  //delay(5);
  //Serial.println();
}
// Function to glide notes up/down
bool GlideFreq(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;
  
  
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      AD[1].setFrequency(from*2);
      from = from + 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    
  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      AD[1].setFrequency(from*2);
      from = from - 0.3;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    AD[1].setFrequency(from*2);
  }
  return true;
}

// Function converting frequency to offset from BASE_NOTE_FREQUENCY
float FreqToNote(float frequency) {
  float x = (frequency / BASE_NOTE_FREQUENCY);
  float y = 12.0 * log(x) / log(2.0);
  return y;
}

// AD9833 related functions
// AD9833 documentation advises a 'Reset' on first applying power.
void AD9833reset(int syncpin) {
  WriteRegister(0x100, syncpin);   // Write '1' to AD9833 Control register bit D8.
  delay(10);
}

// Set the frequency and waveform registers in the selected via syncpin AD9833
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

// Set the frequency registers in the AD9833.
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

// Write to AD9833 register
void WriteRegister(int dat, int syncpin) {
  // Display and AD9833 use different SPI MODES so it has to be set for the AD9833 here.
  SPI.setDataMode(SPI_MODE2);

  digitalWrite(syncpin, LOW);           // Set FSYNC low before writing to AD9833 registers
  delayMicroseconds(10);              // Give AD9833 time to get ready to receive data.

  SPI.transfer(highByte(dat));        // Each AD9833 register is 32 bits wide and each 16
  SPI.transfer(lowByte(dat));         // bits has to be transferred as 2 x 8-bit bytes.

  digitalWrite(syncpin, HIGH);          //Write done. Set FSYNC high
}
