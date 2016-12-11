#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

#define BIT_NUM        8
#define ENCODER_NUM    2
#define ENCODER_PORT   PORTC
#define ENCODER_DDR    DDRC
#define ENCODER_PIN    PINC
#define MOTOR_NUM      2
#define MOTOR_PORT     PORTB
#define MOTOR_DDR      DDRB
#define MOTOR_PIN      PINB

volatile uint8_t motor_pin_pwm[MOTOR_NUM] = { 1, 3 };
volatile uint8_t motor_pin_in1[MOTOR_NUM] = { 0, 4 };
volatile uint8_t motor_pin_in2[MOTOR_NUM] = { 2, 5 };

uint8_t encoder_buffer[BIT_NUM];

volatile uint8_t porthistory = 0xFF;
ISR(PCINT1_vect) {
  uint8_t changedbits;
  changedbits = ENCODER_PIN ^ porthistory;
  porthistory = ENCODER_PIN;

  uint8_t x;
  for (x=0; x<ENCODER_NUM; x++) {
    if (changedbits & _BV(x)) {
      uint16_t count = (x == 0)?TCNT0:TCNT2;
      uint8_t count_lo = count & 0xFF;
      uint8_t count_hi = count >> 8;

      // high
      if (ENCODER_PIN & _BV(x)) {
        // Time since last low
        encoder_buffer[(x*4)]   = count_lo; // 0, 4
        encoder_buffer[(x*4)+1] = count_hi; // 1, 5
      // low
      } else {
        // Time since last high
        encoder_buffer[(x*4)+2] = count_lo; // 2, 6
        encoder_buffer[(x*4)+3] = count_hi; // 3, 7
      }
      // Reset timer if an encoder pin changes.
      if (x == 0) {
        TCNT0 = 0;
      } else {
        TCNT2 = 0;
      }

    }
  }
}

// settings for I2C
uint8_t I2C_buffer[BIT_NUM];
#define I2C_SLAVE_ADDRESS 0x11
void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status);

void sensor_setup(void) {
  // Sensor pin is input
  ENCODER_DDR &= ~_BV(0) & ~_BV(1);

  // Enable pull-up resistor
  ENCODER_PORT |= _BV(0) | _BV(1);

  // Interrupt on any logical change
  // PORTC, PIN[0,1]
  PCICR |= _BV(PCIE1);
  PCMSK1 |= _BV(0) | _BV(1);

  // Motor pin is output
  MOTOR_DDR |= _BV(motor_pin_pwm[0]) | _BV(motor_pin_pwm[1]); // FIXME: Timer pins default?
  MOTOR_DDR |= _BV(motor_pin_in1[0]) | _BV(motor_pin_in1[1]);
  MOTOR_DDR |= _BV(motor_pin_in2[0]) | _BV(motor_pin_in2[1]);
  // Start at 0% duty cycle.
  // FIXME: Inverted? 100% duty = slow or fast?
  OCR1A = 255;
  OCR1B = 255;

  // Timer configuration

  // Timer 1, PWM
  TCCR1A = _BV(WGM10) | _BV(COM1A1); // PWM phase correct mode, inverted


  // Timer 0, encoder
  TCCR0A = _BV(WGM21); // CTC Mode
  TCCR0B = _BV(CS21); // Clock = ClkI/O / 8

  // Timer 2, encoder
  TCCR2A = _BV(WGM21); // CTC Mode
  TCCR2B = _BV(CS21); // Clock = ClkI/O / 8

}

int main(void) {
  // Initialize I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            &handle_I2C_interrupt       // pointer to callback function
            );

  sensor_setup();

  sei();

  // give our slave address and enable I2C
  TWI_enable_slave_mode(  I2C_SLAVE_ADDRESS,      // device address of slave
                          0,                      // slave address mask
                          0                       // enable general call
                          );

  while(1) {}

  return(0);
}

// Write then read.
void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status){
    if (TWI_match_addr == I2C_SLAVE_ADDRESS && status == TWI_success) {

      // Send command to motors
      uint8_t x;
      for (x=0; x<MOTOR_NUM; x++) {
        uint8_t direction = I2C_buffer[(x*4)];
        uint8_t velocity  = I2C_buffer[(x*4)+1];
        //I2C_buffer[(x*4)+2]
        //I2C_buffer[(x*4)+3]

        // Direction
        switch (direction) {
          // foward
          case 255:
            MOTOR_PORT |= _BV(motor_pin_in1[x]);
            MOTOR_PORT &= ~_BV(motor_pin_in2[x]);
            break;
          // reverse
          case 126:
            MOTOR_PORT |= _BV(motor_pin_in2[x]);
            MOTOR_PORT &= ~_BV(motor_pin_in1[x]);
            break;
          // brake
          default:
            MOTOR_PORT |= _BV(motor_pin_in1[x]);
            MOTOR_PORT |= _BV(motor_pin_in2[x]);
            break;
        }

        // Velocity
        // Adjust PWM duty-cycle.
        // FIXME: Inverted? 100% duty = slow or fast?
        if (x == 0) {
          OCR1A = velocity;
        } else {
          OCR1B = velocity;
        }
      }

      // Set buffer to be returned on next read cycle
      // Copy in sensor state
      for (x=0; x<BIT_NUM; x++) {
        I2C_buffer[x] = encoder_buffer[x];
      }
    }
}
