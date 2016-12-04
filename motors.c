#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

#define BIT_NUM      2

// Optical encoder Counts
static volatile uint8_t encoder_count[BIT_NUM];

// Measurement Timer Interrupt
ISR(TIMER2_COMPA_vect) {
  uint8_t x;
  for (x=0; x<BIT_NUM; x++) {
    encoder_count[x] = 0;
  }
}

volatile uint8_t echoporthistory = 0xFF;
ISR(PCINT0_vect) {
  uint8_t changedbits;
  changedbits = PINB ^ echoporthistory;
  echoporthistory = PINB;

  uint8_t x;
  for (x=0; x<BIT_NUM; x++) {
    if(changedbits & _BV(x)) {
      encoder_count[x]++;
    }
  }

}

// settings for I2C
uint8_t I2C_buffer[BIT_NUM];
#define I2C_SLAVE_ADDRESS 0x11
void handle_I2C_interrupt(volatile uint8_t TWI_match_addr, uint8_t status);

void sensor_setup(void) {
  // Sensor pin is input
  DDRC &= ~_BV(0) & ~_BV(1);

  // Enable pull-up resistor
  PORTC |= _BV(0) | _BV(1);

  // Interrupt on any logical change
  PCICR |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK1 |= _BV(0) | _BV(1);

  // Timer configuration
  TCCR2A = _BV(WGM21); // CTC Mode
  TCCR2B = _BV(CS21); // Clock = ClkI/O / 8
  // FIXME: Configure a time which makes sense.
  OCR2A = 1;
  TIMSK2 |= _BV(OCIE2A); // Enable Interrupt TimerCounter2 Compare Match A
}

int main(void) {
  // Initialize I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            0
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
      // TODO: Send command to motors
      // Set buffer to be returned on next read cycle
      // Copy in sensor state
      uint8_t x;
      for (x=0; x<BIT_NUM; x++) {
        I2C_buffer[x] = encoder_count[x];
      }
    }
}
