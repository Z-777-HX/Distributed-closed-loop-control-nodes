#ifndef __DRAW_PID_H
#define __DRAW_PID_H
#define POINT_NUM 128
#define IN_MIN -60.0f
#define IN_MAX 58.0f
#define OUT_MIN 0.0f
#define OUT_MAX 63.0f
#define Y 31.0f
#define TIME_DRAW 100

#include <stdint.h>

void add_to_buf(int8_t *buf, float cur_speed);
uint8_t map_val_to_y(float cur_speed);
void draw_speed_curve(int8_t *buf1, int8_t *buf2);

#endif
