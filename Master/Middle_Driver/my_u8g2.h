#ifndef __MY_U8G2_H
#define __MY_U8G2_H

#include "u8g2.h"

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8g2Init(u8g2_t *u8g2);
void delay_us(uint32_t us);

#endif
