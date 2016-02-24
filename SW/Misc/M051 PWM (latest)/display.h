
#define __USE_ILI9341__
// #define __USE_HD44780__

#define DISPLAY_POS_RPM_X	5
#define DISPLAY_POS_RPM_Y	0

#define DISPLAY_POS_DUTY_X  0
#define DISPLAY_POS_DUTY_Y  1

#define DISPLAY_POS_CAL_TEXT_X 0
#define DISPLAY_POS_CAL_TEXT_Y 0
#define DISPLAY_POS_CAL_DUTY_X  0
#define DISPLAY_POS_CAL_DUTY_Y  1
#define DISPLAY_POS_CAL_RPM_X  20
#define DISPLAY_POS_CAL_RPM_Y  0

#define DISPLAY_MODE_NORMAL 0
#define DISPLAY_MODE_CAL	1

#define DISPLAY_PERIOD_MAX 12500

extern unsigned char display_rpm_set_state;

extern void DISPLAY_Init();
extern void DISPLAY_Mode(unsigned char mode);
extern void DISPLAY_RPM(unsigned long rpm);
extern void DISPLAY_DUTY(unsigned int duty);
extern void DISPLAY_RPM_SET(unsigned long rpm);
extern void DISPLAY_CTRL_MODE(unsigned char mode);
extern void DISPLAY_CAL_DATA(unsigned long period, unsigned long duty);

