
#define DISPLAY_POS_RPM_X	5
#define DISPLAY_POS_RPM_Y	0

#define DISPLAY_POS_DUTY_X  0
#define DISPLAY_POS_DUTY_Y  1

void DISPLAY_Init();
void DISPLAY_RPM(unsigned long rpm);
void DISPLAY_DUTY(unsigned int duty);
