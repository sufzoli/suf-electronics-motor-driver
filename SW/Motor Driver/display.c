#include "display.h"
#include "control.h"

#ifdef __USE_ILI9341__
#include "ili9341.h"
#include "ili9341_fonts.h"
#endif

#ifdef __USE_HD44780__
 #include "hd44780.h"
#endif

unsigned char display_busy = 1;
unsigned char display_mode;
unsigned char display_rpm_set_state=0;

unsigned long display_graph_prev=0;

void DISPLAY_Init()
{
#ifdef __USE_ILI9341__
	ILI9341_Init();
#endif
#ifdef __USE_HD44780__
	HD44780_Init();
#endif
	display_busy = 0;

	// This is used in control mode auto & rpm - may have to put somewhere else
	display_rpm_set_state = 1;
}

void DISPLAY_Mode(unsigned char mode)
{
	display_busy = 1;
	display_mode = mode;
#ifdef __USE_ILI9341__
	ILI9341_FillScreen();
#endif
	switch(mode)
	{
		case DISPLAY_MODE_NORMAL:
#ifdef __USE_ILI9341__
			ILI9341_PrintStr(&Font11x18, "Duty:", 5, 60, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
			ILI9341_PrintStr(&Font11x18, "%", 125, 60, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
#endif
#ifdef __USE_HD44780__
			HD44780_LocationSet(DISPLAY_POS_RPM_X + 6, DISPLAY_POS_RPM_Y);
			HD44780_DisplayString("rpm");
			HD44780_LocationSet(DISPLAY_POS_DUTY_X, DISPLAY_POS_DUTY_Y);
			HD44780_DisplayString("duty:");
			HD44780_LocationSet(DISPLAY_POS_DUTY_X+ 10, DISPLAY_POS_DUTY_Y);
			HD44780_DisplayString("%");
#endif
			break;
		case DISPLAY_MODE_CAL:
#ifdef __USE_ILI9341__
			ILI9341_PrintStr(&Font11x18, "Calibrating...", 5, 5, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
//			ILI9341_FilledReactangle(5,5,10,10,ILI9341_COLOR_WHITE);
			ILI9341_FilledReactangle(5,30,5,224,ILI9341_COLOR_WHITE);
			ILI9341_FilledReactangle(5,224,254,224,ILI9341_COLOR_WHITE);
#endif
#ifdef __USE_HD44780__
			HD44780_LocationSet(DISPLAY_POS_CAL_TEXT_X, DISPLAY_POS_CAL_TEXT_Y);
			HD44780_DisplayString("Calibrating...");
			HD44780_LocationSet(DISPLAY_POS_CAL_DUTY_X, DISPLAY_POS_CAL_DUTY_Y);
			HD44780_DisplayString("duty:");
			HD44780_LocationSet(DISPLAY_POS_CAL_RPM_X, DISPLAY_POS_CAL_RPM_Y);
			HD44780_DisplayString("rpm:");
#endif
			break;
		default:
			break;
	}
	display_busy = 0;
}

void DISPLAY_RPM(unsigned long rpm)
{
	if(!display_busy)
	{
		display_busy = 1;
		if(display_mode == DISPLAY_MODE_NORMAL)
		{
#ifdef __USE_ILI9341__
			ILI9341_DisplayN_POS(&Font16x26, rpm, 117, 5, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 5, 0);
#endif
#ifdef __USE_HD44780__
			HD44780_DisplayN_POS(rpm, DISPLAY_POS_RPM_X + 4, DISPLAY_POS_RPM_Y,8,0);
#endif
		}
		else
		{
#ifdef __USE_ILI9341__
			ILI9341_DisplayN_POS(&Font7x10, rpm, 274, 5, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 5, 0);
#endif
		}
		display_busy = 0;
	}
}

void DISPLAY_DUTY(unsigned int duty)
{
	unsigned long duty_percent;
	if(!display_busy)
	{
		display_busy = 1;
		duty_percent = duty << 2;	// the PWM has 250 steps. If I shift by 2 means x4. The result is percentage x10
		if(display_mode == DISPLAY_MODE_NORMAL)
		{
#ifdef __USE_ILI9341__
			ILI9341_DisplayN_POS(&Font11x18, duty_percent, 65, 60, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 5, 1);
#endif
#ifdef __USE_HD44780__
			HD44780_DisplayN_POS(duty_percent, DISPLAY_POS_DUTY_X + 9, DISPLAY_POS_DUTY_Y,5,1);
#endif
		}
		else
		{
#ifdef __USE_ILI9341__
			ILI9341_DisplayN_POS(&Font7x10, duty_percent, 274, 15, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 5, 1);
#endif
		}
		display_busy = 0;
	}
}
void DISPLAY_RPM_SET(unsigned long rpm)
{
	if(!display_busy && display_mode == DISPLAY_MODE_NORMAL)
	{
		display_busy = 1;
		if(display_rpm_set_state)
		{
#ifdef __USE_ILI9341__
			ILI9341_DisplayN_POS(&Font11x18, rpm, 130, 40, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 5, 0);
#endif
		}
		else
		{
#ifdef __USE_ILI9341__
			ILI9341_PrintStr(&Font11x18, "     ", 130, 40, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
#endif
		}
		display_busy = 0;
	}
}

void DISPLAY_CTRL_MODE(unsigned char mode)
{
	char* mode_text;
#ifdef __USE_ILI9341__
	unsigned long mode_color;
#endif
	if(!display_busy && display_mode == DISPLAY_MODE_NORMAL)
	{
		display_busy = 1;
		switch(mode)
		{
			case CONTROL_MODE_AUTO:
				mode_text = "AUTO";
#ifdef __USE_ILI9341__
				mode_color = ILI9341_COLOR_GREEN;
#endif
				break;
			case CONTROL_MODE_DUTY:
				mode_text = "DUTY";
#ifdef __USE_ILI9341__
				mode_color = ILI9341_COLOR_RED;
#endif
				break;
			case CONTROL_MODE_RPM:
				mode_text = "RPM ";
#ifdef __USE_ILI9341__
				mode_color = ILI9341_COLOR_YELLOW;
#endif
				break;
			default:
				break;
		}
#ifdef __USE_ILI9341__
		ILI9341_PrintStr(&Font11x18, mode_text, 266, 216, mode_color, ILI9341_COLOR_BLACK);
#endif
		display_busy = 0;
	}
}

void DISPLAY_CAL_DATA(unsigned long period, unsigned long duty)
{
//	unsigned long graph_y = (period <= DISPLAY_PERIOD_MAX ? period : DISPLAY_PERIOD_MAX) >> 7;
	unsigned long graph_y = (period <= DISPLAY_PERIOD_MAX ? period : DISPLAY_PERIOD_MAX) / (DISPLAY_PERIOD_MAX / 215);
	while(display_busy);
	display_busy = 1;
#ifdef __USE_ILI9341__
	if((display_graph_prev + 1 < graph_y || display_graph_prev - 1 > graph_y) && duty > 0)
	{
		ILI9341_FilledReactangle(
				4 + duty,
				224 - (graph_y > display_graph_prev ? graph_y : display_graph_prev),
				4 + duty,
				224 - (graph_y > display_graph_prev ? display_graph_prev : graph_y),
				ILI9341_COLOR_YELLOW);
	}
	display_graph_prev = graph_y;
	ILI9341_DrawPixel(5 + duty, 224 - graph_y, ILI9341_COLOR_YELLOW);
#endif
	display_busy = 0;
}

