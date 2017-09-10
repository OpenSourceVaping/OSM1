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

#endif // 1

#if 1
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ARDUINO MICRO

// Arduino Micro is a discontinued standard Arduino part
// https://www.arduino.cc/en/uploads/Main/arduino-micro-schematic.pdf
//
#define ARDUINO_MICRO (1)
#define PLATFORM (ARDUINO_MICRO)
#define ATMEL_STUDIO (1)

#endif // 0

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// STANDARD ARDUINO IDE ROUTINES
///

void setup()
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

void loop()
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
      }

      adcv = 0xFF & adcv;

      // Limit to 300W at max power at .12 ohms for now...
      //
      if (adcv > 190)
      {
        adcv = 190;  
      }
      
      pwmOut(adcv);
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

#if defined(ATMEL_STUDIO)

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// AVR VERSION: Simply call Arduino IDE versions of setup() and loop()
///

int main(void)
{
  setup();

  while (1)
  {
    loop();
  }
}

#endif // #if defined(ATMEL_STUDIO)

