#pragma once
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"

typedef struct{
 volatile unsigned int OUT; //offset 0x0504
 volatile unsigned int OUTSET; //offset 0x0508
 volatile unsigned int OUTCLR; //offset 0x050C
 volatile unsigned int IN; //offset 0x0510
 volatile unsigned int DIR; //offset 0x0514
 volatile unsigned int DIRSET; //offset 0x0518
 volatile unsigned int DIRCLR; //offset 0x051C
} GPIO_Type;

// Define pins for buttons and LEDs
// Set direction of GPIO 17 & 18 to input
void init_buttons(void) {
  NRF_GPIO->DIRCLR &= ~(1 << 17 | 1 << 18); //clear bit 4 of the NRF_GPIO OUT
}

// Set direction of GPIO 21 & 22 to output
void init_leds(void) {
  NRF_GPIO->DIRSET &= ~(1 << 21 | 1 << 22);
}

// Set the output pin 21 to low (LEDs are active low)
void led1_on(void) {
  NRF_GPIO->OUTCLR |= (1 << 21);
}

// Set the output pin 22 to low
void led2_on(void) {
  NRF_GPIO->OUTCLR |= (1 << 22);
}

// Set the output pin 23 to low
void led3_on(void) {
  NRF_GPIO->OUTCLR |= (1 << 23);
}

// Set the output pin 24 to low
void led4_on(void) {
  NRF_GPIO->OUTCLR |= (1 << 24);
}

// Set output pins 21 & 22 to high (1)
void turn_all_leds_off(void) {
  NRF_GPIO->OUTSET |= (1 << 21 | 1 << 22 | 1 << 23 | 1 << 24);
}
