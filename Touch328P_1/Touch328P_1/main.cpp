/*
 * main.cpp
 *
 * Created: 08/12/2017
 * Author : trk
 */ 
 
#define F_CPU 16000000L // Specify oscillator frequency
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
 
unsigned int adc1, adc2;        // store the average of the charge resp. discharge measurement
int probe_val;                  // store the resulting touch measurement

 //#define _BV(x) (0x1 << (x))

 // ADC constants
 #define ADMUX_MASK  0b00001111 // mask the mux bits in the ADMUX register
 #define MUX_GND 0b00001111 // mux value for connecting the ADC unit internally to GND
 #define MUX_REF_VCC 0b01000000 // value to set the ADC reference to Vcc

 const uint16_t ledFadeTable[32] = {0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 17, 21, 25, 30, 36, 43, 51, 61, 73, 87, 104, 125, 149, 178, 213, 255}; // this is an exponential series to model the perception of the LED brightness by the human eye

// Selects ADC0/ADC1 (TPIN1,TPIN2) with the respective 0,1 values.
// On 32u4 this is PF0/PF1 (pins 40/41) [ Arduino Micro: https://www.arduino.cc/en/uploads/Main/arduino-micro-schematic.pdf ]
// On 328P  
 #define TPIN1 0
 #define TPIN2 1 // this is currently only used as a supply of Vcc to charge the s&h cap

 #define CHARGE_DELAY  5 // time it takes for the capacitor to get charged/discharged in microseconds
 #define TRANSFER_DELAY  5 // time it takes for the capacitors to exchange charge
 #define TOUCH_VALUE_BASELINE -165 // this is the value my setup measures when the probe is not touched. For your setup this might be different. In order for the LED to fade correctly, you will have to adjust this value
 #define TOUCH_VALUE_SCALE 5 // this is also used for the LED fading. The value should be chosen such that the value measured when the probe is fully touched minus TOUCH_VALUE_BASELINE is scaled to 31, e.g. untouched_val= 333, touched_val= 488, difference= 155, divide by 5 to get 31.

 void touch_setup() {
	 // prepare the ADC unit for one-shot measurements
	 // see the atmega328 data sheet for explanations of the registers and values
	 ADMUX = 0b01000000; // Vcc as voltage reference (bits76), right adjustment (bit5), use ADC0 as input (bits3210)
	 ADCSRA = 0b11000100; // enable ADC (bit7), initialize ADC (bit6), no auto-trigger (bit5), don't clear int-flag  (bit4), no interrupt (bit3), clock div by 16@16Mhz=1MHz (bit210) ADC should run at 50kHz to 200kHz, 1MHz gives decreased resolution
	 ADCSRB = 0b00000000; // autotrigger source free running (bits210) doesn't apply
	 while(ADCSRA & (1<<ADSC)){  } // wait for first conversion to complete
 }

 uint16_t touch_probe(uint8_t pin, uint8_t partner, bool dir) {
	 uint8_t mask= _BV(pin) | _BV(partner);
	 
	 DDRC|= mask; // config pins as push-pull output
	 if(dir)
	 PORTC= (PORTC & ~_BV(pin)) | _BV(partner); // set partner high to charge the  s&h cap and pin low to discharge touch probe cap
	 else
	 PORTC= (PORTC & ~_BV(partner)) | _BV(pin); // set pin high to charge the touch probe and pin low to discharge s&h cap cap
	 
	 // charge/discharge s&h cap by connecting it to partner
	 ADMUX = MUX_REF_VCC | partner; // select partner as input to the ADC unit
	 
	 _delay_ms(CHARGE_DELAY); // wait for the touch probe and the s&h cap to be fully charged/dsicharged
	 
	 DDRC&= ~mask; // config pins as input
	 PORTC&= ~mask; // disable the internal pullup to make the ports high Z
	 
	 // connect touch probe cap to s&h cap to transfer the charge
	 ADMUX= MUX_REF_VCC | pin; // select pin as ADC input
	 
	 _delay_ms(TRANSFER_DELAY); // wait for charge to be transfered
	 
	 // measure
	 ADCSRA|= (1<<ADSC); // start measurement
	 while(ADCSRA & (1<<ADSC)){  } // wait for conversion to complete
	 return ADC; // return conversion result
 }
 
 void pwmOut(int8_t pwmv)
 {
 		OCR1A = pwmv;
 }

 void loop() 
 {	 
	 // 4 measurements are taken and averaged to improve noise immunity
	 for (int i=0; i<4; i++) {
		 // first measurement: charge touch probe, discharge ADC s&h cap, connect the two, measure the voltage
		 adc1+= touch_probe(TPIN1, TPIN2, false); // accumulate the results for the averaging

		 // second measurement:discharge touch probe, charge ADC s&h cap, connect the two, measure the voltage
		 adc2+= touch_probe(TPIN1, TPIN2, true); // accumulate the results for the averaging
	 }
	 adc1>>=2; // divide the accumulated measurements by 16
	 adc2>>=2;

	 // resulting raw touch value
	 probe_val= adc1-adc2; // the value of adc1 (probe charged) gets higher when the probe its touched, the value of adc2 (s&h charged) gets lower when the probe is touched, so, it has to be be subtracted to amplify the detection accuracy
	 
	 // calculate the index to the LED fading table
	 int16_t idx= (probe_val-TOUCH_VALUE_BASELINE); // offset probe_val by value of untouched probe
	 if(idx<0) idx= 0; // limit the index!!!
	 idx/= TOUCH_VALUE_SCALE; // scale the index
	 if(idx>31) idx= 31; // limit the index!!!
	 
	 // fade the LED
	 pwmOut(ledFadeTable[idx]);
	 
	 adc1= 0; // clear the averaging variables for the next run
	 adc2= 0;
	 _delay_ms(10); // take 100 measurements per second
 }

 void pwmSetup(void) 
 {

	/**
	 * We will be using OCR1A as our PWM output which is the
	 * same pin as PB1.
	 */
	DDRB |= _BV(PB1);

	/**
	 * There are quite a number of PWM modes available but for the
	 * sake of simplicity we'll just use the 8-bit Fast PWM mode.
	 * This is done by setting the WGM10 and WGM12 bits.  We 
	 * Setting COM1A1 tells the microcontroller to set the 
	 * output of the OCR1A pin low when the timer's counter reaches
	 * a compare value (which will be explained below).  CS10 being
	 * set simply turns the timer on without a prescaler (so at full
	 * speed).  The timer is used to determine when the PWM pin should be
	 * on and when it should be off.
	 */
	TCCR1A |= _BV(COM1A1) | _BV(WGM10);
	TCCR1B |= _BV(CS10) | _BV(WGM12);

	/**
	 *  This loop is used to change the value in the OCR1A register.
	 *  What that means is we're telling the timer waveform generator
	 *  the point when it should change the state of the PWM pin.
	 *  The way we configured it (with _BV(COM1A1) above) tells the
	 *  generator to have the pin be on when the timer is at zero and then
	 *  to turn it off once it reaches the value in the OCR1A register.
	 *
	 *  Given that we are using an 8-bit mode the timer will reset to zero
	 *  after it reaches 0xff, so we have 255 ticks of the timer until it
	 *  resets.  The value stored in OCR1A is the point within those 255
	 *  ticks of the timer when the output pin should be turned off
	 *  (remember, it starts on).
	 *
	 *  Effectively this means that the ratio of pwm / 255 is the percentage
	 *  of time that the pin will be high.  Given this it isn't too hard
	 *  to see what when the pwm value is at 0x00 the LED will be off
	 *  and when it is 0xff the LED will be at its brightest.
	 *
	uint8_t pwm = 0x00;
	bool up = true;
	for(;;) {

		OCR1A = pwm;

		pwm += up ? 1 : -1;
		if (pwm == 0xff)
			up = false;
		else if (pwm == 0x00)
			up = true;

		_delay_ms(10);
	}
	*/
}

int main(void)
{
/*
    DDRB = 0b100000; // configure pin 7 of PORTB as output (digital pin 13 on the Arduino Mega2560)
     
    while(1)
    {
        PORTB = 0b10; // set 7th bit to HIGH
        _delay_ms(250);
        PORTB = 0b00000000; // set 7th bit to LOW
        _delay_ms(500);
    }
*/
	touch_setup();
	pwmSetup();

	pwmOut(60);

	while(1)
	{
	  loop();
	}

	return 1;
}


