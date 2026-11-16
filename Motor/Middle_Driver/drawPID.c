#include "drawPID.h"
#include "u8g2.h"

extern u8g2_t u8g2;

void add_to_buf(int8_t* buf, float cur_speed)
{
	uint8_t i = 0;
	for (; i < POINT_NUM - 1; i++)
	{
		buf[i] = buf[i + 1];
	}
	buf[i] = (int8_t)cur_speed;
}

uint8_t map_val_to_y(float cur_speed)
{
	float ret_y = 0.0f;
	if (cur_speed >= 0.0f)
	{
		ret_y = Y - ((Y - OUT_MIN) / (IN_MAX - 0.0f)) * cur_speed;
		if (ret_y < OUT_MIN) ret_y = OUT_MIN;
		ret_y += 0.5f;
	}
	else
	{
		ret_y = Y - ((OUT_MAX - Y) / (0.0f - IN_MIN)) * cur_speed;
		if (ret_y > OUT_MAX) ret_y = OUT_MAX;
		ret_y -= 0.5f;
	}
	return (uint8_t)ret_y;
}

void draw_speed_curve(int8_t* buf1, int8_t* buf2)
{
	u8g2_ClearBuffer(&u8g2);
	
	uint8_t y1 = 0, y2 = 0, y3 = 0, y4 = 0;
	for (uint8_t i = 0; i < POINT_NUM - 1; i++)
	{
		y1 = map_val_to_y(buf1[i]);
		y2 = map_val_to_y(buf1[i + 1]);
		y3 = map_val_to_y(buf2[i]);
		y4 = map_val_to_y(buf2[i + 1]);
		u8g2_DrawLine(&u8g2, i, y1, i + 1, y2);
		u8g2_DrawLine(&u8g2, i, y3, i + 1, y4);
		u8g2_DrawStr(&u8g2, 0, y3, "TARGET");
	}

	u8g2_DrawLine(&u8g2, 0, 31, 127, 31);
	u8g2_DrawLine(&u8g2, 125, 33, 127, 31);
	u8g2_DrawLine(&u8g2, 125, 29, 127, 31);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
	u8g2_DrawGlyph(&u8g2, 120, 42, 't');
	u8g2_DrawStr(&u8g2, 1, 42, "y=0");
	u8g2_SendBuffer(&u8g2);
}
