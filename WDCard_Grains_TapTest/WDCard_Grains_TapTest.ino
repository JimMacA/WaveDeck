/*
WDCard_Grains_TapTest
from WDCard_Grains
Tiny mods to test the Piezos.  Diag LED flashes when Piezos are not in state 0.
mods tagged 20261021
20261022 Adding E-Ink
The trick will be figuring out how to update display without causing trouble.
At the moment, the update freezes a processor for a second.

Modified for WaveDeckCard

2025-08-03
Jim MacArthur, Harvard University
Use Philhower core as the library.
Follow the instructions for installing Arduino IDE and Philhower library found here:
https://learn.adafruit.com/adafruit-feather-rp2040-pico/arduino-ide-setup
except that you will select DatanoiseTV PicoADK v2 as the board
Details on Philhower library, esp I2S, are here:
https://arduino-pico.readthedocs.io/en/latest/i2s.html

WaveDeckGrains is a granular processor inspired by Mutable Instrument's Clouds.
It digitizes audio and stores it into a 4-second delay buffer,
then generates "grains" of sound, ranging in size from 1024 to 16384 samples long.
The user can control:
The position of the grains in the 4-second delay buffer
The size of grains being generated
The playback speed (pitch) of the grains
The rate at which new grains are generated, aka the density.

Incoming audio can be frozen, allowing the user to explore the 4-second buffer by scrubbing the position control.


POT1 = Size   Size
POT2 = Position.  Posn
POT3 = Density  Dens
POT4 = Pitch   Ptch

SW6 Freezes the audio in the buffer


To do:
At the moment, left out is full wet and right out is full dry.
Still need sw3 to add an alt mode wet dry mix
which goes to full wet in freeze mode
use the 1/4 sine table as the wet/dry map

Save/recall? Possible with this HW, but didn't implement yet
feedback: didn't implement.
Stereo spread:
Gillet implements stereo spread in granular_sample_player.h
Stereo spread parameter gets multiplied by random number to great a pan variable for each grain

*/

//#define TAPTEST

#include "PDLS_Common.h"

// Board
Board_EXT myBoard = boardRaspberryPiPico_RP2040_EXT3;  //still used?

// Driver
#include "Pervasive_Wide_Small.h"
Pervasive_Wide_Small myDriver(eScreen_EPD_206_KS_0E, boardRaspberryPiPico_RP2040);

// Screen
#include "PDLS_Basic.h"
Screen_EPD myScreen(&myDriver);

// Checks
#if (SCREEN_EPD_RELEASE < 1000)
#error Required SCREEN_EPD_RELEASE 1000
#endif // SCREEN_EPD_RELEASE

// Fonts
uint8_t fontSmall, fontMedium, fontLarge, fontVery;

#include "resources.h"  //Gillet's collection of tables, unmodified
#include <I2S.h>
#include <Wire.h>  //the prox sensor IC is controlled by I2C

// Create the I2S port using a PIO state machine
I2S i2s(OUTPUT);
I2S i2sIn(INPUT);
const int sampleRate = 32000;

// GPIO pin numbers
#define pBCLK 23
//#define pWS (pBCLK+1)  //output I2S frame sync assumed
#define pDOUT 25   //output I2S data
#define INBCLK 21  //input I2S bit clock.  Input frame sync myst be GPIO22
#define INDOUT 20  //input I2S data
#define NMUTE 19   //DAC !mute signal

//ADC mux select lines go to bottom board
#define SEL0 17
#define SEL1 18
#define DIAGLED 16
#define SWA 2 // muxed switch A
#define SWB 3 // muxed switch B

//These variables monitor and debounce the pushbutton switches

bool SW1On;
bool SW1Button;
bool OldSW1Button;
uint16_t SW1Debounce;

bool SW2On;
bool SW2Button;
bool OldSW2Button;
uint16_t SW2Debounce;

bool SW3On;
bool SW3Button;
bool OldSW3Button;
uint16_t SW3Debounce;

bool SW4On;
bool SW4Button;
bool OldSW4Button;
uint16_t SW4Debounce;

bool SW5On;
bool SW5Button;
bool OldSW5Button;
uint16_t SW5Debounce;

bool SW6On;
bool SW6Button;
bool OldSW6Button;
uint16_t SW6Debounce;

bool SW7On;
bool SW7Button;
bool OldSW7Button;
uint16_t SW7Debounce;

bool SW8On;
bool SW8Button;
bool OldSW8Button;
uint16_t SW8Debounce;

uint16_t FreezeMode = 0;

uint16_t MuxSel = 0;   //cycles through A/D multiplexer
uint16_t Pot1Val = 0;  //Pot2 reading, normalized to 15 bits
float Pot1Avg = 0;  //Pot2 reading, low-pass filtered
uint16_t Pot2Val = 0;  //Pot3 reading, normalized to 15 bits
float Pot2Avg = 0;  //Pot3 reading, low-pass filtered
uint16_t Pot3Val = 0;  //Pot4 reading, normalized to 15 bits
float Pot3Avg = 0;  //Pot4 reading, low-pass filtered
uint16_t Pot4Val = 0;  //Pot 1 reading, normalized to 15 bits
float Pot4Avg = 0;  //Pot1 reading, low-pass filtered
#define POTFILTERCOEFF 0.01f

//these keep track of time.  We use millis for things like switch debouncing
unsigned long previousMillis = 0;     
unsigned long currentMillis = 0;      //current value of Millis, so I don't have to keep asking
unsigned long DebounceTimeStamp = 0;  //last time a button was pressed.
#define DEBOUNCEMSEC 50               //debouncer time constant in msec

uint32_t Time;                    //increments every sample

uint16_t Rect1Size = 10;
uint16_t Rect2Size = 10;
uint16_t Rect3Size = 10;
uint16_t Rect4Size = 10;

uint32_t PreviousMicros;          //use microseconds to control timing of launching new grains
                                  //(We can't use Time, because it stops in freeze mode)

int32_t InLeft, InRight;   //left and right channel audio from ADC
float fInLeft, fInRight;    //floating point versions of audio input...
float fOutLeft, fOutRight;  //...and output
int32_t LSample, RSample;  //interim left and right channel audio sample

uint16_t StartSetup1 = 0;  // proc 1 sets this to sync proc 2.  Necessary for I2S startup

float Density;  //controls how often new grains are created.  0.0 is never, increases to 1.0 non-linearly
//float Overlap;
float GrainIntervalF = 500000.0f;  //scaled reciprocal of density is generation interval in microseconds
uint32_t GrainInterval = 500000;  //cast into an int.

uint32_t randomNumber;  //32-bit random number generated every time we try to sow a grain.  Used in randomizing parameters.

float Compressor = 1.0f;  //simple reactive compressor tries to keep output from clipping

#define DELAY1MASK 0x1FFFF  //used to do modulo math on audio delay
int16_t Delay1[131072];  //audio data stored in array of int16s
#define DELAY1SIZEF 131072.0f
#define DELAY1SIZE 131072



//The table that holds the envelope that shapes the grain amplitudes is stored in lut_window in resources.cc
//It is 4096 elements long, but only covers the front half of the envelope.  The whole envelope is 8192 long.
//#define WINDOWTABLEMASK 0x0FFF  // lut_window is 4096 samples long, but it's half of the window
#define WINDOWSIZE 8192.0f     
#define WINDOWHALFSIZE 4096.0f

float GrainSize;  //size of each grain is derived from a lookup table, controlled by POT3
float EnvelopeSpeed = 1.0f; //Size gets converted into speed of traversing the envelope window
float fGrainSum;  //sample that is the summation of all of the currently generated grains

#define NUMBEROFGRAINS 16  //algorithm fails at 24

struct Grain {
  uint32_t GrnPtr;  //the read pointer of the grain in delay memory
  float GrnPtrF;    //the floating point version of that pointer
  float GrnPtrResidue;  //the fractional amount of the pointer
  int32_t GrnPtrInterpTo; //always GrnPtr + 1.  We interpolate from GrnPtr to GrnPtrInterpTo
  float GrnOffset;  //offset from the end of the delay memory, derived from the Position parameter
  float GrnPitch;  //the amount that the grain pointer is incremented every sample, derived from the Pitch parameter
  float Envelope;  //current value of the grain's amplitude envelope
  int32_t EnvPtr = 0;  //read pointer in the envelope lookup table
  float EnvPtrF = 0;   //the floating point version of that pointer
  float EnvPtrResidue;         //the fractional amount of the pointer
  int32_t EnvPtrInterpTo = 0;  //integer part plus 1
  float EnvSpeed = 1.0f; //EnvelopeSpeed copied into each grain
  bool EnvOn;  //1 means grain is active
} grains[NUMBEROFGRAINS];


float GrainPitch = 0.0f;  //sets the playback rate for the grains -- derived from Pitch param

float Position;  //Position param sets the starting location in audio buffer for grains

bool SowGrain;  //used to indicate whether a grain is successfully sown

float TempF;  //temporary float


//WaveDeck has a pair of piezoelectric tap sensors, which are handles with these structures

struct Piezo
{
  uint32_t Accum;   //a DC offset baseline, created when we initialize the piezo
  uint16_t Thresh;  //accum plus an amount.  Piezo needs to go over this to activate
  uint16_t Peak;   //highest value recorded by the piezo since the start of the tap
  uint16_t OldPeak;  //previous highest value
  uint16_t State;  //used by the piezo processing state machine
  uint16_t Volume;   //indicates how hard the piezo was tapped.  Range is 0 to 255
  uint16_t VolCorrection;  //corrects for difference between piezo sensors
  uint32_t Timeout;  //used to control peak detection and retrigger timing
  uint16_t ADCChan;  //ADC channel that piezo sensor is attached to
} Piezo1,Piezo2;


#define I2C_ADDRESS 0x50  //address of FDC1004 prox sensor
uint16_t I2CReadWord;
uint16_t CapValue1;  //capacitance values read by FDC1004
uint16_t CapOffset1;
uint16_t CapValue2;
uint16_t CapOffset2;


void setup() //proc0 initialization code
{

  pinMode(DIAGLED, OUTPUT);
  digitalWrite(DIAGLED, 0);
  pinMode(NMUTE, OUTPUT);
  digitalWrite(NMUTE, 0);  //init low to reset the display

  pinMode(SEL0, OUTPUT);    //SEL0 and SEL1 select which analog input the multiplexer is looking at 
  digitalWrite(SEL0, 0);
  pinMode(SEL1, OUTPUT);
  digitalWrite(SEL1, 0);
  pinMode(SWA, INPUT);
  pinMode(SWB, INPUT);

    SPI.setSCK(6);
    SPI.setMOSI(7);

    hV_HAL_begin();

    hV_HAL_Serial_crlf();
    hV_HAL_log(LEVEL_INFO, __FILE__);
    hV_HAL_log(LEVEL_INFO, __DATE__ " " __TIME__);
    hV_HAL_Serial_crlf();

    // Screen
    myScreen.begin();

    // Fonts
#if (FONT_MODE == USE_FONT_TERMINAL)

    fontSmall = Font_Terminal6x8;
    fontMedium = Font_Terminal8x12;
    fontLarge = Font_Terminal12x16;
    fontVery = Font_Terminal16x24;

#else // FONT_MODE

    fontSmall = myScreen.addFont(Font_DejaVuSans12);
    fontSmall -= ((fontSmall > 0) ? 1 : 0);
    fontMedium = myScreen.addFont(Font_DejaVuSans16);
    fontMedium -= ((fontMedium > 0) ? 1 : 0);
    fontLarge = myScreen.addFont(Font_DejaVuSans24);
    fontLarge -= ((fontLarge > 0) ? 1 : 0);
    fontVery = myScreen.addFont(Font_DejaVuMono48);
    fontVery -= ((fontVery > 0) ? 1 : 0);

#endif // FONT_MODE

    myScreen.setOrientation(2);

    hV_HAL_log(LEVEL_INFO, "Regenerate");
    myScreen.regenerate();

    myScreen.clear();

    myScreen.selectFont(fontMedium);
//    myScreen.gText(0, 0, "SizePosDensPtch");
    myScreen.gText(0, 0, "Size");
    myScreen.gText(34,0, "Pos");
    myScreen.gText(61,0, "Dens");
    myScreen.gText(96,0, "Ptch");

    myScreen.selectFont(fontLarge);
    myScreen.gText(15, 80, "GRANULAR");
    myScreen.gText(10, 100, "PROCESSOR");

    myScreen.selectFont(fontMedium);
    myScreen.gText(0, 190, "Tap to hold");
    myScreen.gText(0, 205, " prev 4 seconds.");
    myScreen.gText(0, 220, "Tap again");
    myScreen.gText(0, 235, "  to resume --->");

    myScreen.flush();



  WaveDeckI2C_begin();  //start interface to prox sensor

  Piezo1.ADCChan = A0;          //set the correct ADC channels for the two piezo sensors
  Piezo1.VolCorrection = 256;
  InitPiezo(&Piezo1);           //this determines the DC offset error of the piezo

  Piezo2.ADCChan = A1;
  Piezo2.VolCorrection = 256;  
  InitPiezo(&Piezo2);


  delay(100);           //give prox sensor time to initialize
  I2CReadWord = ReadWordI2C(0x02);
  CapOffset1 = I2CReadWord;  //this is the baseline capacitance, which is why you need to keep your hands away here
  I2CReadWord = ReadWordI2C(0x00);
  CapOffset2 = I2CReadWord;

  //Initialize the I2S interface to the audio D/A converter.

  i2s.setBCLK(pBCLK);
  i2s.setDATA(pDOUT);
  i2s.setBitsPerSample(32);
  i2s.setBuffers(4, 64);  //number of buffers, size of each buffer.

  delayMicroseconds(10000);  //making sure proc 1 is ready
  StartSetup1 = 1;          //tell proc1 to start up its I2S interface

  // start I2S at the sample rate with 16-bits per sample
  if (!i2s.begin(sampleRate)) {
    while (1)  //i2s didn't initialize properly so stay here flashing the LED
    {
      digitalWrite(DIAGLED, HIGH);  //flash an error indicator
      delay(200);
      digitalWrite(DIAGLED, LOW);
      delay(200);
    }
  }

  digitalWrite(NMUTE, 1);  //unmute the audio D/A converter

  previousMillis = millis();     
  DebounceTimeStamp = previousMillis;

  PreviousMicros = micros();
}


//InitPiezo accumulates 256 readings of a piezo, to create a baseline and a threshold
void InitPiezo(struct Piezo *PiezoN)
{
  PiezoN->Accum = 0;   
  for(int i = 0; i<256; i++)  PiezoN->Accum += analogRead(PiezoN->ADCChan);

  PiezoN->Thresh = uint16_t(PiezoN->Accum >> 8) + 40;

  PiezoN->State = 0;
}



void setup1() //proc1 initialization code
{
    //Initialize the I2S interface to the audio A/D converter.
  i2sIn.setBCLK(INBCLK);
  i2sIn.setDATA(INDOUT);
  i2sIn.setBitsPerSample(32);
  i2sIn.setBuffers(4, 64);  //number of buffers, size of each buffer.

  while (StartSetup1 == 0);  //wait for proc0 to to be ready

  // start I2S at the sample rate with
  if (!i2sIn.begin(sampleRate)) 
  {
    while (1);  //i2s didn't initialize properly so stay here flashing the LED
  }
}


void loop()  //proc 0 handles the user interface
{
//20261021
#ifdef TAPTEST
  if(Piezo1.State || Piezo2.State) digitalWrite(DIAGLED, 1);
  else digitalWrite(DIAGLED, 0);
#endif



//if Density is below some threshold,
//we won't use GrainInterval to sow seeds.
//but instead we will use MIDI and tap triggers

  if(Density < 0.05f) PreviousMicros = micros();  //this guarantees that GrainInterval will be ignored

  if((micros() - PreviousMicros) >= GrainInterval) //if it's been more than a GrainInterval since we tried to sow a grain
  {
    PreviousMicros += GrainInterval;
    Try2SowGrain();
  }

  currentMillis = millis(); //used for switch debouncing, so we don't have to keep checking millis

  
  //every time through the loop we increment the multiplexer on the bottom board, which selects another
  //potentiometer (pot) to digitize.
  switch (MuxSel) 
  {
    case 0:
    //ADCA(3) = J7-2
    //ADCB(2) = POT1
    //SWA = TACT5 = SW9 = switch8
    //SWB = TACT1 = SW3
      Pot1Val = (analogRead(A2) << 5);  //normalize 10-bit ADC to 0-32767 range
      Pot1Avg = (Pot1Avg * (1.00f - POTFILTERCOEFF)) + ((float(Pot1Val) * POTFILTERCOEFF)); //low-pass filter of POT1

//This code creates a push-on, push-off function for SW8, the freeze button
      OldSW8Button = SW8Button;
      SW8Button = digitalRead(SWA);
      if (SW8Button != OldSW8Button)  //SW6 changed?
      {
        if (currentMillis - DebounceTimeStamp > DEBOUNCEMSEC)  //ignore event if it's within debounce window
        {
          if (!SW8Button)  //For these switches, 0 = on.  Button is pressed.  Ignore releases
          {
            if (FreezeMode) 
            {
              FreezeMode = 0;
#ifndef TAPTEST
           digitalWrite(DIAGLED, 0);  //20261021  
#endif
            } 
            else 
            {
              FreezeMode = 1;
#ifndef TAPTEST
            digitalWrite(DIAGLED, 1);   //20261021 
#endif
            }
          }
        }
        DebounceTimeStamp = currentMillis;
      }
      break;

    case 1:
    //ADCA(3) = POT3
    //ADCB(2) = J6-2
    //SWA = TACT6 = SW5 
    //SWB = TACT2 = SW6

      Pot3Val = analogRead(A3) << 5;  //normalize 10-bit ADC to 0-32767 range
      Pot3Avg = (Pot3Avg * (1.00f - POTFILTERCOEFF)) + ((float(Pot3Val) * POTFILTERCOEFF)); //low-pass filter of POT3
      Density = Pot3Avg / 32768.0f;  //range now between 0.0 and 1.0
     
      GrainIntervalF = 1.0f - Density;
      GrainIntervalF = GrainIntervalF * GrainIntervalF * GrainIntervalF;  //shaping the curve

      //right prox sensor controls how much randomness is added to GrainInterval
      I2CReadWord = ReadWordI2C(0x02);       //read capacitance from sensor
      if(I2CReadWord > CapOffset1) CapValue1 = I2CReadWord - CapOffset1;
      else CapValue1 = 0;

//comment out the following line to remove the prox randomizer to GrainInterval      
//      GrainIntervalF += float(CapValue1) * float(randomNumber & 0x000000FF) * 0.000001f;
      GrainInterval = uint32_t(GrainIntervalF * 300000.0);
      break;

    case 2:
    //ADCA(3) = J7-1
    //ADCB(2) = J6-1
    //SWA = TACT7 = SW2
    //SWB = TACT3 = SW8


//move this to another case to follow pot?
      GrainPitch = lut_quantized_pitch[uint16_t(Pot4Avg) >> 5] * 0.20f;  //table is 1024 elements long. scaled to a bit more than 2 8va
      if (FreezeMode) GrainPitch += 1.0f; //Time isn't increasing in Freeze mode, so we need to add 1.0

      Position = Pot2Avg * 4.0;  //Position now in range of 0-131068

      //left prox sensor controls how much randomness is added to position
      I2CReadWord = ReadWordI2C(0x00);       //read capacitance from sensor
      if(I2CReadWord > CapOffset2) CapValue2 = I2CReadWord - CapOffset2;  //subtract baseline
      else CapValue2 = 0;


//comment out the following line to remove the prox randomizer to Position         
//      Position += ((CapValue2 >> 4) * (randomNumber & 0x000000FF));

//We need to bound a grain's starting position so that it doesn't over- or underrun the write buffer
//This is a function of length and pitch (modified by freezemode)
      GrainSize = lut_grain_size[uint16_t(Pot1Avg) >> 7];  //table is 256 elements long
      EnvelopeSpeed = WINDOWSIZE / GrainSize; 
      if(Position < (GrainPitch*GrainSize)) Position = GrainPitch*GrainSize;
      if(Position > DELAY1SIZE - (GrainPitch*GrainSize)) Position = DELAY1SIZE - (GrainPitch*GrainSize);
      break;

    case 3:
    //ADCA(3) = POT4
    //ADCB(2) = POT2
    //SWA = TACT8 = SW4
    //SWB = TACT4 = SW2
      Pot2Val = analogRead(A2) << 5;  //normalize 10-bit ADC to 0-32767 range
      Pot2Avg = (Pot2Avg * (1.00f - POTFILTERCOEFF)) + ((float(Pot2Val) * POTFILTERCOEFF)); //low-pass filter of POT3

      Pot4Val = (analogRead(A3) << 5);
      Pot4Avg = (Pot4Avg * (1.00f - POTFILTERCOEFF)) + ((float(Pot4Val) * POTFILTERCOEFF)); //low-pass filter of POT3

      if(FindPeak(&Piezo1))   //process tap1
      {
//uncomment the next 2 lines to map piezo1.Volume to grain size
//        GrainSize = lut_grain_size[Piezo1.Volume];  //table is 256 elements long
//        EnvelopeSpeed = WINDOWSIZE / GrainSize; 
        Try2SowGrain();
      }

      if(FindPeak(&Piezo2))
      {
//uncomment the next 2 lines to map piezo2.Volume to grain size        
//        GrainSize = lut_grain_size[Piezo2.Volume];  //table is 256 elements long
//        EnvelopeSpeed = WINDOWSIZE / GrainSize; 
        Try2SowGrain();
      }
 
      break;
  }

  //next Mux
  MuxSel++;
  if (MuxSel > 3) MuxSel = 0;
  if (MuxSel & 0x0001) digitalWrite(SEL0, 1);
  else digitalWrite(SEL0, 0);
  if (MuxSel & 0x0002) digitalWrite(SEL1, 1);
  else digitalWrite(SEL1, 0);
}


void Try2SowGrain()
{
  randomNumber = rp2040.hwrand32();  //use RP2350's random number hardware to get a fresh random number

    //try to find an open grain
  SowGrain = 1;

  for (int i = 0; i < NUMBEROFGRAINS; i++) {
    if (SowGrain) {
      if (!grains[i].EnvOn)  //did we find an open grain?
      {
        grains[i].GrnPitch = GrainPitch;
        grains[i].GrnOffset = DELAY1SIZEF - Position;
        grains[i].EnvSpeed = EnvelopeSpeed;
        grains[i].GrnPtrF = grains[i].GrnOffset;
        grains[i].EnvPtrF = 0.0f;
        grains[i].EnvOn = 1;
        SowGrain = 0;
      }
    }
  }
}



uint16_t FindPeak(struct Piezo *PiezoN)
{
  uint16_t retval = 0;
  switch(PiezoN->State)
  {
    case 0:  //starting state.  Piezo signal is below threshold
      PiezoN->OldPeak = 0;
      PiezoN->Peak = analogRead(PiezoN->ADCChan);
      if(PiezoN->Peak > PiezoN->Thresh) //if signal goes above threshold
      {
        PiezoN->OldPeak = PiezoN->Peak;  //store the highest voltage it got to
        PiezoN->State = 1;               //and increment the state
        PiezoN->Timeout = currentMillis+10; //10msec peak acquisition period
      }
      break;

    case 1:
       if (currentMillis > PiezoN->Timeout)  //if we're past the peak acquisition period
        { //create the volume as a function of the old peak - the DC offset
          PiezoN->Volume = (((PiezoN->OldPeak - (PiezoN->Accum >> 8))>>2) * PiezoN->VolCorrection) >> 8;
          if(PiezoN->Volume > 255) PiezoN->Volume = 255;   //cap the volume at 255
           PiezoN->Timeout = currentMillis + 100;  //no more retriggering for 200msec
           PiezoN->State = 2;
           retval = 1;
        }
       else
       {
        PiezoN->Peak = analogRead(PiezoN->ADCChan);  //still acquiring the peak
        if (PiezoN->Peak > PiezoN->OldPeak) PiezoN->OldPeak = PiezoN->Peak;  //if it's greater than oldpeak, update oldpeak
       }
      break;

      case 2:
      case 3:
        if (currentMillis > PiezoN->Timeout)  //if we're past the retriggering debounce period,
        {
          PiezoN->State = 0;   //reset the state
        } 
        break;
  }
  return(retval);
}



void loop1()  //Proc1 handles the audio
{
  i2sIn.read32(&InLeft, &InRight);
//  LSample = InLeft;  //transfer InLeft and InRight to the working registers
//  RSample = InRight;

  Delay1[Time & DELAY1MASK] = InLeft >> 16;  //write left input channel to the delay buffer

  fOutLeft = fGrainSum;  //send the sum of all grains out the left channel...
  fOutLeft *= Compressor;  //...multiplied by the compressor to help keep it from clipping

  if (fOutLeft > 32000.0f)    //if output is positive clipping
  {
    fOutLeft = 32000.0f;      //bound the output
    Compressor *= 0.99f;      //and reduce the compression gain
  } 
  else if (fOutLeft < -32000.0f)   //same if output is negative clipping
  {
    fOutLeft = -32000.0f;
    Compressor *= 0.99f;
  }
  else if(Compressor < 1.0f)Compressor += 0.0002f;  //No clipping.  Roughly 100msec compressor recovery time


  fOutRight = float(InLeft >> 16);  // for the moment, send dry to the right channel
  if (fOutRight > 32000.0f) fOutRight = 32000.0f;
  if (fOutRight < -32000.0f) fOutRight = -32000.0f;

  LSample = int32_t(fOutLeft) << 16;
  RSample = int32_t(fOutRight) << 16;
  i2s.write32(LSample, RSample);  //pass along data

  if (!FreezeMode) Time++;  //time increments every sample, unless we've frozen

//For each grain, we need to generate an interpolated envelope, then an interpolated sample from the delay line
//then multiply them together and add them to GrainSum
  fGrainSum = 0.0f;
  for (int i = 0; i < NUMBEROFGRAINS; i++) 
  {
    if (grains[i].EnvOn) //only do this if the grain is active
    {
      grains[i].EnvPtrF += grains[i].EnvSpeed;  //increment envelope pointer by envelope speed
      if (grains[i].EnvPtrF >= WINDOWSIZE)    //if we've reached the end of the envelope
      {
        grains[i].Envelope = 0.0f;   //the grain is done, so turn it off
        grains[i].EnvOn = 0;
      } 
      else //still processing the envelope
      {//The envelope table is only the first half, so when the pointer reaches the halfway point, we need to flip it
        TempF = grains[i].EnvPtrF >= WINDOWHALFSIZE ? WINDOWSIZE - grains[i].EnvPtrF : grains[i].EnvPtrF;
        grains[i].EnvPtr = int32_t(TempF);  //cast the pointer into an int
        grains[i].EnvPtrResidue = TempF - grains[i].EnvPtr;  //this is faster than modf
        grains[i].EnvPtrInterpTo = (grains[i].EnvPtr + 1);   //Pointer plus 1 is the location you are interpolating to.
                                                            //this works because the half table has one extra point.
        grains[i].Envelope = (lut_window[grains[i].EnvPtr] * (1.00f - grains[i].EnvPtrResidue)) + (lut_window[grains[i].EnvPtrInterpTo] * grains[i].EnvPtrResidue);  //linear interpolation

        grains[i].GrnPtrF += grains[i].GrnPitch;  //a similar process for the grain audio pointer
        if (grains[i].GrnPtrF < 0.0) grains[i].GrnPtrF += DELAY1SIZEF;  //keep it in range of 0 to Delay1 size
        if (grains[i].GrnPtrF >DELAY1SIZEF) grains[i].GrnPtrF -= DELAY1SIZEF;
        grains[i].GrnPtr = int32_t(grains[i].GrnPtrF);  //this messes up for negative pitches
        grains[i].GrnPtrResidue = grains[i].GrnPtrF - grains[i].GrnPtr;
        grains[i].GrnPtrInterpTo = grains[i].GrnPtr + 1;  //and the InterpTo pointer
        fGrainSum += grains[i].Envelope * ((float(Delay1[(Time + grains[i].GrnPtr) & DELAY1MASK]) * (1.00f - grains[i].GrnPtrResidue)) + (float(Delay1[(Time + grains[i].GrnPtrInterpTo) & DELAY1MASK]) * grains[i].GrnPtrResidue));
      }
    }
  }
}

//I2C routines for controlling the capacitance (prox) sensors
void WaveDeckI2C_begin(void) {
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();  //join I2C bus (address optional for master)

  WriteWordI2C(0x0C, 0x8000);  //FDC Config.  Software reset
  delay(500);                  //not sure how long a software reset takes
  WriteWordI2C(0x08, 0x1C00);  //config 1 is ch1 alone
  WriteWordI2C(0x09, 0x5C00);  //config 2 is ch3 alone
  WriteWordI2C(0x0C, 0x0DC0);  //FDC Config.  measurement 1 and 2 400 sps repeat

  I2CReadWord = ReadWordI2C(0x08);
}


uint16_t ReadWordI2C(uint8_t Address)
//need to time this out
{
  uint16_t w = 0;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(Address);  // register to read
  Wire.endTransmission();

  //assumes stop condition, so address is automatically reset to 0

  Wire.requestFrom(I2C_ADDRESS, 2);  // two bytes
  while (Wire.available() < 2) {
    // waiting
  }
  w = (Wire.read() << 8) | Wire.read();
  return w;
}


void WriteWordI2C(uint8_t Address, uint16_t DataWord)
//need to time this out
{
  uint8_t DataByte = 0;

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(Address);  // register to read

  DataByte = DataWord >> 8;
  Wire.write(DataByte);
  DataByte = DataWord & 0x00FF;
  Wire.write(DataByte);

  Wire.endTransmission();
  return;
}
