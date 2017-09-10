//
// common.cpp
//
// Created: 08/12/2017
// Original code adapted from: https://github.com/jgeisler0303/QTouchADCArduino
// Author : trk - Converted to Atmel Studio and added support for 328P and 32u4
// BLE: http://www.raytac.com/products.php?subid=57
//
//

#if defined(__ATMEL_32U4__)
#define ANALOG_PORT DDRF
#define ANALOG_CONFIG PORTF
#define PWM_OUTPUT DDRB
#define PWM_PIN PB5
#else if defined (__ATMEL_328P__)
#define ANALOG_PORT DDRC
#define ANALOG_CONFIG DDRF
#else
#endif // __ATMEL_32U4__

#define F_CPU 16000000L // Specify oscillator frequency

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
 
// ADC constants
#define ADMUX_MASK	0b00001111		// mask the mux bits in the ADMUX register
#define MUX_GND		0b00001111		// mux value for connecting the ADC unit internally to GND
#define MUX_REF_VCC	0b01000000		// value to set the ADC reference to Vcc

// this is an exponential series to model the perception of the LED brightness by the human eye
//
const uint16_t ledFadeTable[32] = {0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 17, 21, 25, 30, 36, 43, 51, 61, 73, 87, 104, 125, 149, 178, 213, 255};

// Selects ADC0/ADC1 (TPIN1,TPIN2) with the respective 0,1 values.
// On 32u4 this is PF0/PF1 (pins 40/41) [ Arduino Micro: https://www.arduino.cc/en/uploads/Main/arduino-micro-schematic.pdf ]
// On 328P - ?
//
// Pins select ADC0 & ADC1 which on the Arduinos are various different user pins.
//

#define TPIN1 (0)
#define TPIN2 (1)			// this is currently only used as a supply of Vcc to charge the s&h cap

#define CHARGE_DELAY	(5)		// time it takes for the capacitor to get charged/discharged in microseconds

#define TRANSFER_DELAY  (5)		// time it takes for the capacitors to exchange charge

#define TOUCH_VALUE_BASELINE (-165)	// this is the value my setup measures when the probe is not touched.
					//  For your setup this might be different. In order for the LED to fade correctly, you will have to adjust this value

#define TOUCH_VALUE_SCALE	(5)	// this is also used for the LED fading. The value should be chosen such that the value
					// measured when the probe is fully touched minus TOUCH_VALUE_BASELINE is scaled to 31, e.g.
					// untouched_val= 333, touched_val= 488, difference= 155, divide by 5 to get 31.

void touch_setup()
{
	// prepare the ADC unit for one-shot measurements see the atmega328 data sheet for explanations of the registers and values
	//
	ADMUX = 0b01000000;			// Vcc as voltage reference (bits76), right adjustment (bit5), use ADC0 as input (bits3210)
	
	ADCSRA = 0b11000100;			// enable ADC (bit7), initialize ADC (bit6), no auto-trigger (bit5), don't clear int-flag
						// (bit4), no interrupt (bit3), clock div by 16@16Mhz=1MHz (bit210) ADC should run at 50kHz to 200kHz, 1MHz gives decreased resolution
	
	ADCSRB = 0b00000000;			// autotrigger source free running (bits210) doesn't apply
	
	while ( ADCSRA & _BV(ADSC) );		// wait for first conversion to complete
 }

uint16_t touchProbe(uint8_t pin, uint8_t partner, bool dir)
{
	uint8_t mask;

	mask = _BV(pin) | _BV(partner);
	
	// config pins as push-pull output
	//
	ANALOG_PORT |= mask;
	
	if (dir)
	{
		// set partner high to charge the S&H cap and pin low to discharge touch probe cap
		//
		ANALOG_PORT = (ANALOG_PORT & ~_BV(pin)) | _BV(partner);
	}
	else
	{
		// set pin high to charge the touch probe and pin low to discharge s&h cap cap
		//
		ANALOG_PORT = (ANALOG_PORT & ~_BV(partner)) | _BV(pin);
	}
	
	// charge/discharge s&h cap by connecting it to partner
	//
	ADMUX = MUX_REF_VCC | partner; // select partner as input to the ADC unit
	 
	_delay_ms(CHARGE_DELAY); // wait for the touch probe and the s&h cap to be fully charged/dsicharged
	
	// config pins as input
	//
	ANALOG_CONFIG &= ~mask;
	
	// disable the internal pullup to make the ports high Z
	//
	ANALOG_PORT &= ~mask;
	 
	// connect touch probe cap to s&h cap to transfer the charge
	//
	ADMUX = MUX_REF_VCC | pin; // select pin as ADC input
	 
	_delay_ms(TRANSFER_DELAY); // wait for charge to be transfered
	 
	// start measurement
	//
	ADCSRA |= _BV(ADSC);
	
	// wait for conversion to complete
	//
	while ( ADCSRA & _BV(ADSC) );
	
	return ADC;
}

uint16_t adcRead(uint8_t adcx)
{
	// adcx is the analog pin we want to use.  ADMUX's first few bits are
	// the binary representations of the numbers of the pins so we can
	// just 'OR' the pin's number with ADMUX to select that pin.
	// We first zero the four bits by setting ADMUX equal to its higher four bits.
	//
	ADMUX &= 0xf0;
	ADMUX |= adcx;

	// This starts the conversion.
	//
	ADCSRA |= _BV(ADSC);

	// This is an idle loop that just wait around until the conversion
	// is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	// set above, to see if it is still set.  This bit is automatically
	// reset (zeroed) when the conversion is ready so if we do this in
	//a loop the loop will just go until the conversion is ready.
	//
	while ( ADCSRA & _BV(ADSC) );

	// Finally, we return the converted value to the calling function.
	//
	return ADC;
}
 
 uint8_t loop(void)
 {
	 uint8_t i;
	 int16_t idx;
	 uint16_t adc1, adc2;		// store the average of the charge resp. discharge measurement
	 uint16_t probeVal;		// store the resulting touch measurement
	 
	 adc1 = 0;
	 adc2 = 0;
	 
	 // 4 measurements are taken and averaged to improve noise immunity
	 //
	 for (i = 0; i < 4; i++)
	 {
		 // first measurement: charge touch probe, discharge ADC s&h cap, connect the two, measure the voltage
		 //
		 adc1 += touch_probe(TPIN1, TPIN2, false); // accumulate the results for the averaging

		 // second measurement: discharge touch probe, charge ADC s&h cap, connect the two, measure the voltage
		 //
		 adc2 += touch_probe(TPIN1, TPIN2, true); // accumulate the results for the averaging
	 }
	 
	 adc1 >>= 2;	// divide the accumulated measurements by 16
	 adc2 >>= 2;

	 // resulting raw touch value
	 //
	 probeVal = adc1 - adc2;	// the value of adc1 (probe charged) gets higher when the probe its touched,
					// the value of adc2 (s&h charged) gets lower when the probe is touched, so,
					// it has to be be subtracted to amplify the detection accuracy
	 
	 // calculate the index to the LED fading table
	 //
	 idx = (probeVal - TOUCH_VALUE_BASELINE); // offset probeVal by value of untouched probe
	 
	 // Range of idx limited to size of ledFadeTable
	 //
	 if (idx < 0)
	 {
		idx= 0; // limit the index!!!
	 }
	 
	 idx /= TOUCH_VALUE_SCALE;	// scale the index
	 
	 if (idx > 31)
	 {
		 idx= 31;		// limit the index!!!
	 }
	 
	 return idx;
 }

void pwmOut(int8_t pwmv)
{
	// Set the counter value to anything between 0..255 inclusive
	//
	OCR1A = pwmv;
}

void pwmSetup(void)
{
	// We will be using OCR1A as our PWM output.  The pin is dependent on the hardware...
	//
	PWM_OUTPUT |= _BV(PWM_PIN);

	// There are quite a number of PWM modes available but for the
	// sake of simplicity we'll just use the 8-bit Fast PWM mode.
	// This is done by setting the WGM10 and WGM12 bits.  We
	// Setting COM1A1 tells the microcontroller to set the
	// output of the OCR1A pin low when the timer's counter reaches
	// a compare value (which will be explained below).  CS10 being
	// set simply turns the timer on without a prescaler (so at full
	// speed).  The timer is used to determine when the PWM pin should be
	// on and when it should be off.
	//
	TCCR1A |= _BV(COM1A1) | _BV(WGM10);
	TCCR1B |= _BV(CS10) | _BV(WGM12);
}

int main(void)
{
/*
    DDRB = 0b100000; // configure pin 7 of PORTB as output (digital pin 13 on the Arduino Mega2560)
     
    while(1)
    {
        PORTB = 0b100000; // set 7th bit to HIGH
        _delay_ms(250);
        PORTB = 0b00000000; // set 7th bit to LOW
        _delay_ms(500);
    }
	*/
	touch_setup();
	pwmSetup();

	/*
	while (1) {
		pwmOut(120);
		_delay_ms(100);
		pwmOut(0);
		_delay_ms(200);
	}

	while (1)
	{
		pwmOut(ledFadeTable[loop()]);
		_delay_ms(100);
	}

	while (1)
	{
		uint16_t adcv;

		adcv = adcRead(7) & 0x3FF;
		adcv = adcv / 2;
		adcv = 0x1F & (adcv >> 5);
		
		pwmOut(ledFadeTable[adcv]);
	}
	*/

	while(1)
	{
		uint16_t adcv;

		if (loop() > 25)
		{
			adcv = 0;
			adcv += adcRead(7) & 0x3FF;
			adcv += adcRead(7) & 0x3FF;
			adcv = adcv / 2;
			adcv = 0x1F & (adcv >> 5);

			pwmOut(ledFadeTable[adcv]);
		}
		else
		{
			pwmOut(0);
		}

	  	 _delay_ms(100); // take 100 measurements per second

	}

	return 1;
}


