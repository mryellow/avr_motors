#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
//#include <stdio.h>
#include "twi.h"

#define BIT_NUM      8
#define ENCODER_NUM  2
#define ENCODER_PIN  PINC
#define ENCODER_PORT PORTC
#define ENCODER_DDR  DDRC

uint8_t encoder_buffer[BIT_NUM];

volatile uint8_t echoporthistory = 0xFF;
ISR(PCINT1_vect) {
  uint8_t changedbits;
  changedbits = ENCODER_PIN ^ echoporthistory;
  echoporthistory = ENCODER_PIN;

  uint8_t x;
  for (x=0; x<ENCODER_NUM; x++) {
    if (changedbits & _BV(x)) {
      uint8_t lo = (x === 0)?TCNT0L:TCNT2L;
      uint8_t hi = (x === 0)?TCNT0H:TCNT2H;
      // high
      if (ENCODER_PIN & _BV(x)) {
        // Time since last low
        encoder_buffer[(x*4)]   = lo; // 0, 4
        encoder_buffer[(x*4)+1] = hi; // 1, 5
      // low
      } else {
        // Time since last high
        encoder_buffer[(x*4)+2] = lo; // 2, 6
        encoder_buffer[(x*4)+3] = hi; // 3, 7
      }
      // Reset timer if an encoder pin changes.
      if (x === 0) {
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

  // Timer configuration

  // Timer 0, encoder
  TCCR0A = _BV(WGM21); // CTC Mode
  TCCR0B = _BV(CS21); // Clock = ClkI/O / 8
  // FIXME: Configure a time which makes sense.
  OCR0A = 3906; // 1 Second
  TIMSK0 |= _BV(OCIE0A); // Enable Interrupt TimerCounter2 Compare Match A

  // Timer 2, encoder
  TCCR2A = _BV(WGM21); // CTC Mode
  TCCR2B = _BV(CS21); // Clock = ClkI/O / 8
  // FIXME: Configure a time which makes sense.
  OCR2A = 3906; // 1 Second
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

      // Direction
      // Velocity

      // Set buffer to be returned on next read cycle
      // Copy in sensor state
      uint8_t x;
      for (x=0; x<BIT_NUM; x++) {
        I2C_buffer[x] = encoder_buffer[x];
      }
    }
}
