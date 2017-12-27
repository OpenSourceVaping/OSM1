#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 Lexigraph, Inc.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software, 
// electronic circuit designs and schematics, hardware designs, images and associated documentation 
// files (the "Software"), to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of 
// the Software, and to permit persons to whom the Software is furnished to do so, subject to the 
// following conditions: The above copyright notice and this permission notice shall be included 
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// common1.cpp
//
// Created: 08/12/2017
// Original code adapted from: https://github.com/jgeisler0303/QTouchADCArduino
// Author : trk - Converted to Atmel Studio and added support for 328P and 32u4
// BLE: http://www.raytac.com/products.php?subid=57
//
// trk -  09/03/2017 -  Cleaned output output to supply full power, added this comment.
//
// trk -  09/09/2017 -  Added support for Arduino IDE. Reparamaterized for multi-platform build. 
//                      Added support for multiple 32u4 parts.
//                      Reorganized adc support for initialization.
//                      Added temporary max limit on power output.
//

// defines the 32u4 in AVR: __AVR_ATmega32U4__

// Select the part/configuration you need below
//
#if 0
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ARDUINO MICRO PRO

// Micro Pro is defined by SparkFun
// https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Pro-Micro-v10.pdf
//
#define ARDUINO_MICRO_PRO (1)
#define PLATFORM (ARDUINO_MICRO_PRO)

#endif // 0

#if 1
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ARDUINO MICRO

// Arduino Micro is a discontinued standard Arduino part
// https://www.arduino.cc/en/uploads/Main/arduino-micro-schematic.pdf
//
#define ARDUINO_MICRO (1)
#define PLATFORM (ARDUINO_MICRO)
#define ATMEL_STUDIO (1)

#endif // 1

////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define F_CPU 16000000L // Specify oscillator frequency

#if defined(ATMEL_STUDIO)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>

#else

// Atmel conveniences
//
#define _BV(x) (0x1 << (x))

#endif // defined(ATMEL_STUDIO)

#if defined(PLATFORM) && defined(ARDUINO_MICRO_PRO)

// Arduino Micro Pro Pin #9 on circuit board which is Atmel PB#5
#define PWM_PIN (5)   
#define PWM_DDR  DDRB
#define PWM_PORT PORTB
l
#define SLIDER_PIN (4)
#define SLIDER_DDR DDRF
#define SLIDER_PORT PORTF

#define TOUCH_DDR DDRF
#define TOUCH_PORT PORTF

#define TOUCH_PIN1    (6)             // Pin # connected to touch in TOUCH_DDR/PORT
#define TOUCH_PIN2    (7)             // this is currently only used as a supply of Vcc to charge the s&h cap

#elif defined(PLATFORM) && defined(ARDUINO_MICRO)

// Arduino Micro Pin #9 on circuit board which is Atmel PB#5
#define PWM_PIN (5)   
#define PWM_DDR  DDRB
#define PWM_PORT PORTB

#define SLIDER_PIN (7)
#define SLIDER_DDR DDRF
#define SLIDER_PORT PORTF

#define TOUCH_DDR DDRF
#define TOUCH_PORT PORTF

#define TOUCH_PIN1    (0)             // Pin # connected to touch in TOUCH_DDR/PORT
#define TOUCH_PIN2    (1)             // this is currently only used as a supply of Vcc to charge the s&h cap

#else

#error "No PWM_PIN defined because no PLATFORM is defiend...."

#endif // defined(PLATFORM)

////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PWM_OFF_TRISTATE (1)
#define PWM_ENABLED      (0)

////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ADC constants
#define ADMUX_MASK  0xF                // mask the mux bits in the ADMUX register
#define MUX_GND     0xF                // mux value for connecting the ADC unit internally to GND
  
#define MUX_REF_VCC 0x40              // value to set the ADC reference to Vcc
#define CHARGE_DELAY  (5)             // time it takes for the capacitor to get charged/discharged in microseconds
#define TRANSFER_DELAY  (5)           // time it takes for the capacitors to exchange charge
#define TOUCH_VALUE_BASELINE (-165)   // this is the value my setup measures when the probe is not touched. 
                                      // For your setup this might be different. 
#define TOUCH_VALUE_SCALE (5)         // Limit touch range to 0..31

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pwmSetState(uint8_t tristate)
{
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  
  if (tristate)
  {
    PWM_DDR &= ~_BV(PWM_PIN);  // set mode input
    PWM_PORT &= ~_BV(PWM_PIN); // disable pullups - effectively now in tri-state
    
    TCCR1B = _BV(WGM12);      // 0 from CS10..CS12 which disables timer.
    OCR1A = 0;

    // Tri-state mode relies on the 15K ohm resistor on MOSFET gate
    // to completely shut off the MOSFET.  Better than having the 
    // processor be the only means to shut it off.
  }
  else
  {
    PWM_DDR |= _BV(PWM_PIN);
    PWM_PORT |= _BV(PWM_PIN);  // enable pullups
    
    // There are quite a number of PWM modes available but for the
    // sake of simplicity we'll just use the 8-bit Fast PWM mode.
    // This is done by setting the WGM10 and WGM12 bits.  We
    // Setting COM1A1 tells the microcontroller to set the
    // output of the OCR1A pin low when the timer's counter reaches
    // a compare value (which will be explained below).  
    //
    // TCCR1B using _BV(CS10+CS12) defines the frequency of the PWM
    // CS10, CS11, CS12 define three bits used to set a value from 0 => OFF,
    // to 0x5 maximum clock division (slowest PWM output frequence).  0x1 =>
    // least amount of division hence the fastest PWM
    //
    // NOTE: Using |= and &= here work on Atmel Studio but not Arduino IDE.
    //       Probably the AVR runtime is somewhat different in terms of reseting.
    //
    TCCR1B = _BV(WGM12) | _BV(CS10+CS12) ;
  }
}

void pwmOut(int8_t pwmv)
{
  // Fire OCRA with a value 0..255
  //
  OCR1A = pwmv;
}

// Generic adc reader.  Not used for touch.
//
uint16_t adcRead(uint8_t adcx) 
{
  // adcx is the analog pin we want to use.  ADMUX's first few bits are
  // the binary representations of the numbers of the pins so we can
  // just 'OR' the pin's number with ADMUX to select that pin.
  // We first zero the four bits by setting ADMUX equal to its higher
  // four bits. 
  //
  ADMUX = (ADMUX & 0xf8) | (adcx & 0x7);
  
  // This starts the conversion. 
  //
  ADCSRA |= _BV(ADSC);
  
  // This is an idle loop that just wait around until the conversion
  // is finished.  It constantly checks ADCSRA's ADSC bit, which we just
  // set above, to see if it is still set.  This bit is automatically
  // reset (zeroed) when the conversion is ready so if we do this in
  // a loop the loop will just go until the conversion is ready. 
  //
  while ( (ADCSRA & _BV(ADSC)) );
  
  // Finally, we return the converted value to the calling function. 
  //
  return ADC;
}

// Common setup for both touch and regular adc.
//
// Note - set all pins on SLIDER_DDR so that this must be called first.
// Otherwise subsequent calls touching SLIDER_DDR/PORT will get lost.
//
void adcSetup(uint8_t adcx)
{
  // from: http://www.elecrom.com/avr-tutorial-2-avr-input-output/
  //
  SLIDER_DDR = ~_BV(adcx);   // Setup to input
  SLIDER_PORT = ~_BV(adcx);  // Go into tri-state mode

  // from: http://maxembedded.com/2011/06/the-adc-of-the-avr/
  // AREF = AVcc
  ADMUX = _BV(REFS0);
 
  // ADC Enable and prescaler of 128
  // 16000000/16 = 250K
  //
  // NOTE: Need to decide if the scaler here is the best choice.
  //
  ADCSRA = _BV(ADEN) | _BV(ADPS2)  | _BV(ADPS1) ;
  ADCSRB = 0x0 ;
}

// Return a touch value from the adc.  Used to discriminate between 
// capacitor differences when a touch is present.
//
uint16_t touchProbe(uint8_t pin, uint8_t partner, bool dir) 
{
  uint8_t mask;

  mask = _BV(pin) | _BV(partner);

  // config pins as push-pull output
  //
  TOUCH_DDR |= mask; 
  
  if (dir)
  {
    // set partner high to charge the  s&h cap and pin low to discharge touch probe cap
    //
    TOUCH_PORT = (TOUCH_PORT & ~_BV(pin)) | _BV(partner); 
  }
  else
  {
    // set pin high to charge the touch probe and pin low to discharge s&h cap cap
    //
    TOUCH_PORT = (PORTF & ~_BV(partner)) | _BV(pin); 
  }
  
  // charge/discharge s&h cap by connecting it to partner
  //
  ADMUX = MUX_REF_VCC | partner; // select partner as input to the ADC unit
  
  _delay_ms(CHARGE_DELAY); // wait for the touch probe and the s&h cap to be fully charged/dsicharged
  
  TOUCH_DDR &= ~mask;     // config pins as input
  TOUCH_PORT &= ~mask;    // disable the internal pullup to make the ports high Z
  
  // connect touch probe cap to s&h cap to transfer the charge
  //
  ADMUX = MUX_REF_VCC | pin; // select pin as ADC input
  
  _delay_ms(TRANSFER_DELAY); // wait for charge to be transfered
  
  ADCSRA |= _BV(ADSC);       // start measurement
  
  while ( (ADCSRA & _BV(ADSC)) );   // wait for measurement to complete
  
  return ADC; // return conversion result
}

uint16_t adc1, adc2;    // store the average of the charge resp. discharge measurement
int touchProbeVal;      // store the resulting touch measurement

// Sample current touch. Return 0..31.  The higher the value the more "certain" there is a touch.
//
uint8_t probeTouch(void)
{
  int i;
  int16_t idx;
  
  // 4 measurements are taken and averaged to improve noise immunity
  //
  for (i = 0; i < 4; i++) 
  {
    // first measurement: charge touch probe, discharge ADC s&h cap, connect the two, measure the voltage
    //
    adc1 += touchProbe(TOUCH_PIN1, TOUCH_PIN2, false);  // accumulate the results for the averaging
    
    // second measurement: discharge touch probe, charge ADC s&h cap, connect the two, measure the voltage
    //
    adc2 += touchProbe(TOUCH_PIN1, TOUCH_PIN2, true);   // accumulate the results for the averaging
  }
  
  adc1 >>= 2; // divide the accumulated measurements by 16
  adc2 >>= 2;
  
  // the value of adc1 (probe charged) gets higher when the probe its touched, the value of adc2 
  // (s&h charged) gets lower when the probe is touched, so, it has to be be subtracted to amplify 
  // the detection accuracy
  //
  touchProbeVal = adc1 - adc2; 
  
  // normalize to a range of 0..31
  //
  idx = (touchProbeVal - TOUCH_VALUE_BASELINE);     // offset probe_val by value of untouched probe
  
  if (idx < 0) 
  {
    idx= 0; // limit the index!!!
  }
  
  idx /= TOUCH_VALUE_SCALE;                         // scale the index
  
  if (idx > 31) 
  {
    idx= 31;                                        // limit the index!!!
  }
  
  adc1 = 0; // clear the averaging variables for the next run
  adc2 = 0;
  
  return idx;  // 0..31
}

uint8_t outputEnabled;      // Holds state of MOSFET control. 

// You may or may not need to include this depending on your platform
// Include this for the 32u4, exclude it for the M0 for example
//#include <SoftwareSerial.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"

#include "BluefruitConfig.h"

//#define SERIALDBG (1)

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

// This app was tested on iOS with the following apps:
//
// https://itunes.apple.com/us/app/midimittr/id925495245?mt=8
// https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8
//
// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Adafruit Bluefruit LE"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEMIDI midi(ble);

bool isConnected = false;

// A small helper
void error(const __FlashStringHelper*err) {

#if defined(SERIALDBG)
  Serial.println(err);
#endif

  while (1)
  {
     digitalWrite(LED_BUILTIN, HIGH); 
     delay(100);                      
     digitalWrite(LED_BUILTIN, LOW);  
     delay(100);                      
  }
}

// callback
void connected(void)
{
  isConnected = true;

#if defined(SERIALDBG)
  Serial.println(F(" CONNECTED!"));
#endif

  delay(100);

}

void disconnected(void)
{
#if defined(SERIALDBG)
  Serial.println("disconnected");
#endif

  isConnected = false;
}

#define _EEPROMBase (0)
#define _cmdMode (0)
#define _usbMode (1)

bool cmdMode = false;
bool usbMode = false;
int cmdState = 0;

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
#if defined(SERIALDBG)
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");
  Serial.print(cmdState , HEX); Serial.print(" ");
  Serial.print(cmdMode? 1: 0 , HEX); Serial.print(" ");

  Serial.println();
#endif 

  if (status != 0x90) // chan 1
    return;

  switch (cmdState)
  {
    case 0:
      if (byte1 == 0x4F)
      {
        if (byte2 == 0) cmdState++;
      }
      else if (cmdMode && byte1 == 0x50)
      {
         if (byte2 == 0) cmdState = 100;
      }
      else
      {
        cmdState = 0;
      }
      break;

    case 100:
      if (byte1 == 0x4E)
      {
        if (cmdMode && byte2 == 0) cmdState++;
      }
      else
      {
        cmdState = 0;
      }
      break;

   case 101:
      if (byte1 == 0x4C)
      {
        if (cmdMode && byte2 == 0) 
        {
          usbMode = !usbMode;
          EEPROM.write(_EEPROMBase + _usbMode, usbMode? 1 : 0);
          digitalWrite(6, usbMode? HIGH : LOW);
          digitalWrite(LED_BUILTIN, LOW); 
          delay(100);                      
          digitalWrite(LED_BUILTIN, HIGH);  
          cmdState = 0;
        }
      }
      else
      {
        cmdState = 0;
      }
      break;

    case 1:
      if (byte1 == 0x51)
      {
        if (byte2 == 0) cmdState++;
      }
      else
      {
        cmdState = 0;
      }
      break;

    case 2:
      if (byte1 == 0x4D)
      {
        if (byte2 == 0) cmdState++;
      }
      else
      {
        cmdState = 0;
      }
      break;

    case 3:
      if (byte1 == 0x41)
      {
        if (byte2 == 0) cmdState++;
      }
      else
      {
        cmdState = 0;
      }
      break;

    case 4:
      if (byte1 == 0x48)
      {
        if (byte2 == 0) 
        {
          cmdMode = !cmdMode;
          EEPROM.write(_EEPROMBase + _cmdMode, cmdMode? 1 : 0);
          cmdState = 0;
        }
      }
      else
      {
        cmdState = 0;
      }
      break;

    default:
      cmdState = 0;
      break;
  }

#if defined(SERIALDBG)
  Serial.print(cmdState , HEX); Serial.print(" ");
  Serial.print(cmdMode? 1: 0 , HEX); Serial.print(" ");

  Serial.println();
#endif  

  digitalWrite(LED_BUILTIN, cmdMode? (byte2 != 0? LOW : HIGH) : (byte2 != 0? HIGH : LOW));
}

void PWMsetup()
{
  outputEnabled = 0;
  adc1 = 0;
  adc2 = 0;

  // Call first...  Setup is for both just adc reads in general and for touch.
  //
  adcSetup(SLIDER_PIN);
  adcRead(SLIDER_PIN);
  
  pwmSetState(PWM_OFF_TRISTATE);
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);

  cmdMode = (EEPROM.read(_EEPROMBase + _cmdMode) != 0);
  digitalWrite(LED_BUILTIN, cmdMode? HIGH : LOW);

  usbMode = (EEPROM.read(_EEPROMBase + _usbMode) != 0);
  digitalWrite(6, usbMode? HIGH : LOW);

  PWMsetup();
  
#if defined(SERIALDBG)
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit MIDI Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
#endif 

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

#if defined(SERIALDBG)
  Serial.println( F("OK!") );
#endif

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */

#if defined(SERIALDBG)
    Serial.println(F("Performing a factory reset: "));
#endif

    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

#if defined(SERIALDBG)
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
#endif

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

#if defined(SERIALDBG)
  Serial.println(F("Enable MIDI: "));
#endif

  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  
#if defined(SERIALDBG)
  Serial.print(F("Waiting for a connection..."));
#endif

}

#define MAX_ADC_LIMIT  (190)

int firstNote = 0x4F - 24;
int playingNote = 0;

void PWMloop()
{
  uint16_t adcv;

  // Wait until touch says we have something more than noise.
  //
  if (probeTouch() > 25)
  {
    adcv = 0;
    
    // Two samples and average...
    //
    adcv += adcRead(SLIDER_PIN) & 0x3FF;
    adcv += adcRead(SLIDER_PIN) & 0x3FF;
    adcv = adcv / 2;
    
    // Squash down to 0..255 and clamp
    //
    adcv = 0xFF & (adcv >> 2);
    
    // Check that we have at least some significant bits.
    // Fire if we do...
    //
    if ((adcv & 0xF8) > 0)
    {
      if (!outputEnabled)
      {
        pwmSetState(PWM_ENABLED);
        outputEnabled = 1;

        if (isConnected)
        {
          playingNote = firstNote + ((adcv & 0xFF) / 16);
          midi.send(0x90, playingNote , 0x64);
        }
      }

      adcv = 0xFF & adcv;

      // Limit to 300W at max power at .12 ohms for now...
      //
      if (adcv > MAX_ADC_LIMIT)
      {
        adcv = MAX_ADC_LIMIT;  
      }
      
      pwmOut(adcv);

      if (isConnected)
      {
        int nextNote;

        nextNote = firstNote + ((adcv & 0xFF) / 16);
        if (nextNote != playingNote)
        {
           midi.send(0x80, playingNote, 0x64);
           playingNote = nextNote;
           midi.send(0x90, playingNote , 0x64);
        }
      }
    }
    else
    {
      // Drop into tri-state and disable PWM.
      // In tri-state the 15K resistor will shut down the MOSFET.
      //
      if (outputEnabled)
      {
        pwmSetState(PWM_OFF_TRISTATE);
        outputEnabled = 0;
        
        if (isConnected)
        {
          midi.send(0x80, playingNote, 0x64);
        }

        playingNote = 0;
      }
    }
  }
  else
  {
    // No touch, shutdown as above.
    //
    if (outputEnabled)
    {
      pwmSetState(PWM_OFF_TRISTATE);
      outputEnabled = 0;
    }
  }
  
  _delay_ms(50); // take 20 measurements per second
}

int trigger = 0;

void loop(void)
{
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(100);

  if (!cmdMode)
  {
    PWMloop();
  }

  trigger += 1;

  if (trigger > 5)
  {
  }
}
