/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <avr/io.h>
#include <util/atomic.h>
#include "Configuration.hh"
#include <avr/interrupt.h>
#include "ExtruderMotor.hh"
#include "EepromMap.hh"

using namespace eeprom;

// Enable pin D5 is also OC0B.

int16_t last_extruder_speed;
bool stepper_motor_mode;
bool external_stepper_motor_mode;
int16_t stepper_accumulator;
uint8_t stepper_phase;

bool swap_motor = false;
Pin motor_enable_pin = HB1_ENABLE_PIN;
Pin motor_dir_pin = HB1_DIR_PIN;

Pin external_enable_pin = ES_ENABLE_PIN;
Pin external_dir_pin = ES_DIR_PIN;
Pin external_step_pin = ES_STEP_PIN;

// FIXME: Hardcoded steps per revolution. Eventually, this needs to be configurable
// Set to 200 for standard Makerbot Stepper Motor Driver V2.3
// Set to 5 * 200 for MakerGear 1:5 geared stepper
uint16_t extruder_steps_per_rev = 400;

#ifdef PWM_STEPPER
	bool dir = 1; // direction 1 => forward, 0 => backward
	volatile float ext_stepper_ticks_per_step = 0;
	volatile float ext_stepper_counter = 0;
#else
	volatile uint32_t ext_stepper_ticks_per_step = 0;
	volatile int32_t ext_stepper_counter = 0;
#endif

// TIMER0 is used to PWM motor driver A enable on OC0B.
void initExtruderMotor() {
	last_extruder_speed = 0;
	HB1_ENABLE_PIN.setDirection(true);
	HB1_ENABLE_PIN.setValue(false);
	HB1_DIR_PIN.setDirection(true);
	HB2_ENABLE_PIN.setDirection(true);
	HB2_ENABLE_PIN.setValue(false);
	HB2_DIR_PIN.setDirection(true);
	stepper_motor_mode = false;
	stepper_accumulator = 0;
	stepper_phase = 1;
	// Check eeprom map to see if motor has been swapped to other driver chip
	uint16_t ef = getEeprom16(EXTRA_FEATURES,EF_DEFAULT);
	if ((ef & EF_SWAP_MOTOR_CONTROLLERS) != 0) {
		swap_motor = true;
		motor_enable_pin = HB2_ENABLE_PIN;
		motor_dir_pin = HB2_DIR_PIN;
	}
}

void setStepperMode(bool mode, bool external/* = false*/) {
	stepper_motor_mode = mode && !external;
	external_stepper_motor_mode = mode && external;

	if (stepper_motor_mode) {

#ifdef PWM_STEPPER
		TCCR0A = _BV(WGM01) | _BV(WGM00);  // Leave pin off by default
		TCCR0B = _BV(CS01) | _BV (CS00); // 64x prescaler
		TIMSK0 = _BV(TOIE0); // Interrupt on overflow
		OCR0A = 0;
		OCR0B = 0;
		setExtruderMotor(last_extruder_speed);
#else
		TCCR0A = 0;
		TCCR0B = _BV(CS01) | _BV(CS00);
		TIMSK0 = _BV(TOIE0);
#endif
	} else if (external_stepper_motor_mode) {
		// Setup pins
		external_enable_pin.setDirection(true);
		external_enable_pin.setValue(true); // true = disabled

		external_dir_pin.setDirection(true);
		external_dir_pin.setValue(true); // true = forward

		external_step_pin.setDirection(true);
		external_step_pin.setValue(false);

		// CTC Mode
		TCCR0A = _BV(WGM01);
		// Timer/Counter 0 Output Compare A Match Interrupt On
		TIMSK0  = _BV(OCIE1A);
		// 1/(16,000,000 / 8*(1+OCR0A)) = ES_TICK_LENGTH/2 micros/tick
		OCR0A = ES_TICK_LENGTH-1;
		// 8x prescaler, with CTC mode: 16MHz/8 = 2 MHz timer ticks
		TCCR0B = _BV(CS01);

	} else {
		TCCR0A = _BV(WGM01) | _BV(WGM00);  // Leave pin off by default
		TCCR0B = _BV(CS01) | _BV(CS00);
		OCR0B = 0;
		TIMSK0 = 0; // no interrupts needed.
		setExtruderMotor(last_extruder_speed);
	}
}

void setExtruderMotor(int16_t speed) {
	if (speed == last_extruder_speed) return;
	last_extruder_speed = speed;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef PWM_STEPPER
		if (stepper_motor_mode) {
			dir = speed >= 0;
			if (!dir) { speed = -speed; }
			if (speed > 255) { speed = 255; }
			OCR0A = speed;
			OCR0B = speed;
		}
		else
#endif
		if (!stepper_motor_mode && !external_stepper_motor_mode) {
			TIMSK0 = 0;
			if (speed == 0) {
				TCCR0A = _BV(WGM01) | _BV(WGM00);
				motor_enable_pin.setValue(false);
			} else if (speed == 255) {
				TCCR0A = _BV(WGM01) | _BV(WGM00);
				motor_enable_pin.setValue(true);
			} else {
				motor_enable_pin.setValue(true);
				if (swap_motor) {
					TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
				} else {
					TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
				}
			}
			bool backwards = speed < 0;
			if (backwards) { speed = -speed; }
			if (speed > 255) { speed = 255; }
			motor_dir_pin.setValue(!backwards);
			if (swap_motor) {
				OCR0A = speed;
			} else {
				OCR0B = speed;
			}
		}
	}
}

// set the motor's  RPM -- in microseconds for one full revolution
void setExtruderMotorRPM(uint32_t micros, bool direction) {

#ifdef PWM_STEPPER
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (micros == 0) {
			ext_stepper_ticks_per_step = 0; // implies a pause
			TCCR0A = _BV(WGM01) | _BV(WGM00);
		} else {
			// 64*(1+OCRA) / 16,000,000 = ES_TICK_LENGTH (in microseconds)
			// 64 times prescaler, running from a 16 MHz clock.
			// if we choose OCR0A to be 255, the comparison and overflow interrupt would be the same
			// therefore the compare could be used for PWM and overflow for calculating new ticks.
			// We cast to uint32_t to avoid overflow but improve accuracy before float division
			ext_stepper_ticks_per_step = (float) micros / ((uint32_t) ES_TICK_LENGTH * (uint32_t) extruder_steps_per_rev); // calculate
		}

		dir = direction;

	}
#else
	if (!external_stepper_motor_mode) return;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (micros > 0) {
			// 60,000,000 is one RPM
			// 1,000,000 is one RPS
			ext_stepper_ticks_per_step = (micros / ES_TICK_LENGTH) / extruder_steps_per_rev;
			//ext_stepper_counter = 0;

			// Timer/Counter 0 Output Compare A Match Interrupt On
      // This is now done in setExtruderMotorOn()
      // TIMSK0  = _BV(OCIE1A);

			external_dir_pin.setValue(direction); // true = forward
			external_enable_pin.setValue(false); // true = disabled
			external_step_pin.setValue(false);
			// DEBUG_LED.setValue(true);
		} else {
			// Timer/Counter 0 Output Compare A Match Interrupt Off
			TIMSK0  = 0;
			external_enable_pin.setValue(true); // true = disabled
			ext_stepper_ticks_per_step = 0;
			// DEBUG_LED.setValue(false);
		}
	}
#endif
}

#ifdef DEFAULT_EXTERNAL_STEPPER
void setExtruderMotorOn(bool on)
{
	if (!external_stepper_motor_mode) return;
  // Disable stepping but hold torque by disabling interrupt
	if (on) {
		TIMSK0  = _BV(OCIE1A);
	} else {
		TIMSK0  = 0;
	}
}
#endif

// ## H-Bridge Stepper Driving using Timer0 Overflow ##

#ifdef PWM_STEPPER
	// These alternative half-step patterns allows the stepper wires to be connected in the same colour sequence as on Gen3 stepper boards
	const uint8_t hb1_en_pattern = 0xbb;
	const uint8_t hb1_dir_pattern = 0xc3;
	const uint8_t hb2_en_pattern = 0xee;
	const uint8_t hb2_dir_pattern = 0xf0;
#else
	const uint8_t hb1_en_pattern = 0xdd;
	const uint8_t hb1_dir_pattern = 0xc3;
	const uint8_t hb2_en_pattern = 0x77;
	const uint8_t hb2_dir_pattern = 0xf0;
#endif

// at speed 255, ~80Hz full-stepping
const int16_t acc_rollover = (6375/2);

//volatile uint8_t stepper_pwm = 0;
#ifdef PWM_STEPPER
	inline void setStep() {
		const uint8_t mask = 1 << stepper_phase;
		HB1_DIR_PIN.setValue(hb1_dir_pattern & mask);
		HB2_DIR_PIN.setValue(hb2_dir_pattern & mask);
		// NB: Timer0B corresponds to hb1, Timer0A corresponds to hb2
		TCCR0A =  (hb1_en_pattern & mask ? _BV(COM0B1) : 0) \
				| (hb2_en_pattern & mask ? _BV(COM0A1) : 0) \
				 | _BV(WGM01) | _BV(WGM00);
	}

	ISR(TIMER0_OVF_vect) {
		if (ext_stepper_ticks_per_step != 0) {
			// Yes, this is floating point arithmetic in an interrupt!
			// Ugly, but works, and greatly improves accuracy.
			++ext_stepper_counter;
			if (ext_stepper_counter >= ext_stepper_ticks_per_step) {
				ext_stepper_counter -= ext_stepper_ticks_per_step;
				if (dir) {
					stepper_phase = (stepper_phase + 1) & 0x07;
				} else {
					stepper_phase = (stepper_phase - 1) & 0x07;
				}
				setStep();
			}
		}
	}

#else
	inline void setStep() {
		const bool enable = (last_extruder_speed != 0) && (((stepper_pwm++) & 0x01) == 0);
		const uint8_t mask = 1 << stepper_phase;
		HB1_DIR_PIN.setValue((hb1_dir_pattern & mask) != 0);
		HB1_ENABLE_PIN.setValue( enable && ((hb1_en_pattern & mask) != 0) );
		HB2_DIR_PIN.setValue((hb2_dir_pattern & mask) != 0);
		HB2_ENABLE_PIN.setValue( enable && ((hb2_en_pattern & mask) != 0) );

	ISR(TIMER0_OVF_vect) {
		stepper_accumulator += last_extruder_speed;
		if (stepper_accumulator >= acc_rollover) {
			stepper_accumulator -= acc_rollover;
			stepper_phase = (stepper_phase + 2) & 0x07;
		} else if (stepper_accumulator < 0) {
			stepper_accumulator += acc_rollover;
			stepper_phase = (stepper_phase - 2) & 0x07;
		}
		setStep();
	}

// ## External Stepper Driving using Timer 0 Compare A ##

	ISR(TIMER0_COMPA_vect) {
		if (ext_stepper_ticks_per_step > 0) {
			++ext_stepper_counter;
			if (ext_stepper_counter >= ext_stepper_ticks_per_step) {
				external_step_pin.setValue(true);
				ext_stepper_counter -= ext_stepper_ticks_per_step;
				external_step_pin.setValue(false);
			}
		}
	}

#endif
