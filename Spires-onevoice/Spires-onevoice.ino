/*
  Spires is copyright 2024, Mark Washeim blueprint@poetaster.de
  GPLv3



*/
#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MIDI.h>
#include <Midier.h>
#include <math.h>
#include <util/crc16.h>
#include "AD9833.h"

// # define ENCODER_DO_NOT_USE_INTERRUPTS
#include <EncoderButton.h>

// some midier setup

MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);


const midier::Degree scaleDegrees[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
const midier::Note roots[] = {
  midier::Note::C, midier::Note::D,  midier::Note::E, midier::Note::F, midier::Note::G, midier::Note::A, midier::Note::B
};

midier::Mode mode;
int scaleRoot = 0; // start at c, yawn.

int currentMode[21];

void initScales() {
  // iterate over all root notes
  const midier::Mode mode = midier::Mode::Ionian;
  for (auto note : roots)
  {
    // play the major chord
    makeScale(note, mode);
  }
}
void makeScale(midier::Note root, midier::Mode mode) {

  // the root note of the scale
  const midier::Note scaleRoot = root;

  // we are playing ionian mode which is the major scale
  // if you are not familiar with modes, just know that "ionian" is the major scale and "aeolian" is the minor scale
  //const midier::Mode mode = midier::Mode::Ionian;


  for (midier::Degree scaleDegree : scaleDegrees)
  {
    // find out the interval to be added to the root note for this chord degree and chord quality
    const midier::Interval interval = midier::scale::interval(mode, scaleDegree);

    // calculate the note of this chord degree
    const midier::Note note = scaleRoot + interval;
    currentMode[ scaleDegree - 1 ] = midier::midi::number(note, 2);


  }
  //Serial.println();
}


bool debug = false;
// variables for runtime control
bool up;
bool cont = true;
volatile float lastvol = 255;
unsigned int lastPos;
bool continuous = false;
bool sine = true;

// freq to midi
static const uint32_t midi_note_at_zero_volts = 12;
static const float semitones_per_octave = 12.0f;
static const float volts_per_semitone = 1.0f / semitones_per_octave;
static const float a4_frequency = 440.0f;
static const uint32_t a4_midi_note = 69;

int led = 2; // for display led
// default works fine


/*
  Converts a note frequency to the closest corresponding MIDI
  note number.
  Adapted from https://gist.github.com/francoisgeorgy/d155f4aa5e8bd767504c2b43e1ba2902
*/
uint32_t frequency_to_midi_note(float frequency) {
  float note = 69.0f + logf(frequency / a4_frequency) / logf(2.0f) * semitones_per_octave;
  return ceil(note);
}

float midi_note_to_frequency(uint32_t midi_note) {
  float semitones_away_from_a4 = (float)(midi_note) - (float)(a4_midi_note);
  return powf(2.0f, semitones_away_from_a4 / semitones_per_octave) * a4_frequency;
}

// need to send note offs :)
void allOff() {
  for (int i = 28; i < 90; i++) {
    if ( ! debug) MIDI.sendNoteOff(i, 0, 1);
  }
}

int midiChannel = 1;  // Define which MIDI channel to transmit on (1 to 16).


/* signal generator setup */

unsigned int cSpeed = 200000;
//  each device needs its own select pin.
AD9833 AD[1] =
{
  AD9833(8),
  //AD9833(9)
};  //  4 devices.

/* TOF sensor setup */

// The number of sensors in your system.
const uint8_t sensorCount = 1; // original used two. switched to ldr for volume
// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {7};

VL53L0X sensors[sensorCount];


// encoder setup
// the a and b + the button pin large encoders are 6,5,4
EncoderButton eb1(3, 6, 4);


/* these come from rampart bytebeats */
int encoder_pos_last = 0;
long encoder_delta = 0;
int enc_offset = 1; // changes direction
int enc_delta; // which direction

// program switching ugh, todo
int prog = 0;
int bank = 0;
int pb1 = 0;
int pb1total = 7;
int pb2 = 0;
int pb2total = 6;
int pb3 = 0;
int pb3total = 6;
int pb4 = 0;
int pb4total = 10;
int pb5 = 0;
int pb5total = 6;
int pb6 = 0;
int pb6total = 7;

int numProg = 41;
int numBank = 6;

// we have to slow it down :)
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 95; // this is the frequency change loop delay

int lastNote;

#include "encoder.h"


/* AUDIO */
const float ContToFreq[63] PROGMEM = {29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91, 55.00,
                                      58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00,
                                      116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, 220.00,
                                      233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00,
                                      466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00,
                                      932.33, 987.77, 1046.50
                                     };



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



void setup()
{

  pinMode(led, OUTPUT);
  SPI.begin();
  delay(50);
  //while (!Serial) {}

  if (debug == false) {
    MIDI.begin(MIDI_CHANNEL_OMNI);//MIDI_CHANNEL_OMNI);
  }
  if (debug) Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // start amp first not using this
  //AMP.begin(10, 14);
  // start the signal generator
  AD[0].begin();
  AD[0].setWave(AD9833_SINE);
  AD[0].setFrequency(240.00, 0);     //  A

  //can be also read from eeprom
  freq_target1 = freq_init1;
  freq_target2 = freq_init2;

  // start the TOF sensor

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
    //sensors[i].setSignalRateLimit(0.25);
    sensors[i].startContinuous(50);
    sensors[i].setMeasurementTimingBudget(cSpeed); //  adjust this value to move to slower note slurs/jumps
    // increase laser pulse periods (defaults are 14 and 10 PCLKs) 18/14
    //sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    //sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);

  }


  // setup for the encoder with button
  // Link the event(s) to your function
  eb1.setClickHandler(onEb1Clicked);
  eb1.setEncoderHandler(onEb1Encoder);
  eb1.setLongPressHandler(onEb1LongPress, true);
  eb1.setEncoderPressedHandler(onEb1PressTurn);

  lastPos = sensors[0].readRangeContinuousMillimeters();
  if (debug) Serial.println(lastPos);
  // we start in scale mode
  continuous = false;
  startMillis = millis();
  //midi note offs
  //allOff();
}


void loop()
{

  float temp1;
  int temp2;

  eb1.update(); // respond to encoder/button

  //freq_target1 = sensors[0].readRangeContinuousMillimeters(); //freq_init1;
  if (sensors[0].timeoutOccurred()) {
    if (debug) Serial.print(" TIMEOUT");
  }

  currentMillis = millis();
  // slow everything down a bit :)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {

    // readRangeContinuousMillimeters
    freq_target2 = sensors[0].readRangeContinuousMillimeters();

    if (freq_target2  < 700 ) {
      if ( ! continuous ) {
        if (abs(lastPos - freq_target2) > 3) { // NOT sure
          if (bank == 3) {
            // we have some pentatanic scales :*)
            temp2 = int(map(freq_target2, 0, 700, 17, 0));

          } else if (bank == 5) {
            // and some hexatonics
            temp2 = int(map(freq_target2, 0, 700, 23, 0));
          }
          else if (bank == 0) {
            // and we keep range smaller for trad modes.
            temp2 = int(map(freq_target2, 0, 700, 20, 0));
          }
          else {
            temp2 = int(map(freq_target2, 0, 700, 28, 0));
          }


        }
        lastPos = freq_target2;
      } else {
        if (abs(lastPos - freq_target2) > 3) { // NOT sure
          //temp2 = map(freq_target2, 0, 700, 660, 120);
          temp2 = map(freq_target2, 0, 700, 54, 7);
        }
        lastPos = freq_target2;

      }
    }
    // this is clumsy. it does have the nice advantage of a short view of the scales
    switch (bank) {
      case 0:
        switch (pb1) {
          case 0:
            mode = midier::Mode::Ionian;
            break;
          case 1:
            mode = midier::Mode::Dorian;
            break;
          case 2:
            mode = midier::Mode::Phrygian;
            break;
          case 3:
            mode = midier::Mode::Lydian;
            break;
          case 4:
            mode = midier::Mode::Mixolydian;
            break;
          case 5:
            mode = midier::Mode::Aeolian;
            break;
          case 6:
            mode = midier::Mode::Locrian;
            break;
        }
        makeScale( roots[scaleRoot], mode);
        temp1 = midi_note_to_frequency(currentMode[temp2]);
        if (debug) {
          /*
          Serial.print("scaleRoot & degree ");
          Serial.print((char)roots[scaleRoot]);
          Serial.print(" " );
          Serial.println(currentMode[temp2]);
          */
        }
        break;
      case 1:
        switch (pb2) {
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
            temp1 = pgm_read_float( &Partch1[temp2 ]);
            break;
        }
        break;
      case 3:
        switch (pb4) {
          case 0:
            temp1 = pgm_read_float( &Pelog[temp2 ]);
            break;
          case 1:
            temp1 = pgm_read_float( &Zhi[temp2 ]);
            break;
          case 2:
            temp1 = pgm_read_float( &Yu[temp2 ]);
            break;
          case 3:
            temp1 = pgm_read_float( &MajPent[temp2 ]);
            break;
          case 4:
            temp1 = pgm_read_float( &Kumoi[temp2 ]);
            break;
          case 5:
            temp1 = pgm_read_float( &Jiao[temp2 ]);
            break;
          case 6:
            temp1 = pgm_read_float( &Prometheus[temp2 ]);
            break;
          case 7:
            temp1 = pgm_read_float( &Chinese[temp2 ]);
            break;
          case 8:
            temp1 = pgm_read_float( &Gong[temp2 ]);
            break;
          case 9:
            temp1 = pgm_read_float( &Egypt[temp2 ]);
            break;
        }
        break;
      case 4:
        switch (pb5) {
          case 0:
            temp1 = pgm_read_float( &MajorPentatonic[temp2 ]);
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
      case 5:
        switch (pb6) {
          case 0:
            temp1 = pgm_read_float( &HexDorian[temp2 ]);
            break;
          case 1:
            temp1 = pgm_read_float( &HexAeolian[temp2 ]);
            break;
          case 2:
            temp1 = pgm_read_float( &HexSus[temp2 ]);
            break;
          case 3:
            temp1 = pgm_read_float( &HexPhrygian[temp2 ]);
            break;
          case 4:
            temp1 = pgm_read_float( &HexMaj6[temp2 ]);
            break;
          case 5:
            temp1 = pgm_read_float( &HexMaj7[temp2 ]);
            break;
          case 6:
            temp1 = pgm_read_float( &PhrygianFreq[temp2 ]);
            break;
        }
        break;


    }

    // the frequency is used now to glide up or down

    if ( temp1 < 800  && temp1 > 30 && continuous == false) {
      if (freq_init1 > temp1) {
        cont = GlideFreq(freq_init1, temp1, false);

      } else if (freq_init1 < temp1) {
        cont = GlideFreq(freq_init1, temp1, true);
      }
      freq_init1 = temp1;
    } else if ( continuous == true ) { //&& abs(freq_init1 - temp2) > 5 ) {
      temp2 = pgm_read_float( &ContToFreq[temp2 ] ); //ContToFreq
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
    startMillis = currentMillis;  //IMPORTANT to save the start time .
  }



}
// END LOOP

// Function to glide notes up/down
bool GlideFreq(float from, float too, bool up) {
  //make sure we complete the glides before the loop proceeds
  cont = false;

  // send noteoff on new note
  if ( ! debug ) {
      MIDI.sendNoteOff(frequency_to_midi_note(lastNote), 0, midiChannel);
      //now send note on
      MIDI.sendNoteOn(frequency_to_midi_note(too), 127, midiChannel);
  }
  lastNote = too;

  //if (debug) Serial.println( frequency_to_midi_note(too));
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from + 1;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 1;

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
    // send noteoff on new note
  if ( ! debug ) {
      MIDI.sendNoteOff(frequency_to_midi_note(lastNote), 0, midiChannel);
      //now send note on
      MIDI.sendNoteOn(frequency_to_midi_note(too), 127, midiChannel);
  }
  lastNote = too;
  
  if (up) {
    while (from < too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from + 1;//0.06;

    }
    // complete since while may exit early
    AD[0].setFrequency(too);
    //AD[1].setFrequency(too * freq_offset);

  } else {
    while (from > too) {
      AD[0].setFrequency(from);
      //AD[1].setFrequency(from * freq_offset);
      from = from - 1;//0.06;

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
