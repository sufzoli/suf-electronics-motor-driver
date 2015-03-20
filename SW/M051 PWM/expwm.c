#include "M051Series.h"
#include "sys.h"
// #include "pwm.h"
#include "expwm.h"
// Object like pwm handling


unsigned char _expwm_pwma_isinit = 0;
unsigned char _expwm_pwmb_isinit = 0;
unsigned char _expwm_pwm01_isinit = 0;
unsigned char _expwm_pwm23_isinit = 0;
unsigned char _expwm_pwm45_isinit = 0;
unsigned char _expwm_pwm67_isinit = 0;

EXPWM_InterruptCallback_Type _expwm_callback_arr[8];


void EXPWM_Init(expwmtype data)
{
	uint32_t PWM_MODULE = data->Chanell < 4 ? PWMA : PWMB;
	unsigned char Chanell = data->Chanell;
	if(Chanell > 4)
	{
		Chanell -= 4;
	}
	if((data->Chanell == 0 || data->Chanell == 1) && !_expwm_pwm01_isinit)
	{
		SYSCLK->APBCLK = SYSCLK->APBCLK | SYSCLK_APBCLK_PWM01_EN_Msk;
		SYSCLK->CLKSEL1 = SYSCLK->CLKSEL1 | SYSCLK_CLKSEL1_PWM01_HCLK;
		_expwm_pwm01_isinit = 1;
	}
	if((data->Chanell == 2 || data->Chanell == 3) && !_expwm_pwm23_isinit)
	{
		SYSCLK->APBCLK = SYSCLK->APBCLK | SYSCLK_APBCLK_PWM23_EN_Msk;
		SYSCLK->CLKSEL1 = SYSCLK->CLKSEL1 | SYSCLK_CLKSEL1_PWM23_HCLK;
		_expwm_pwm23_isinit = 1;
	}
	if((data->Chanell == 4 || data->Chanell == 5) && !_expwm_pwm45_isinit)
	{
		SYSCLK->APBCLK = SYSCLK->APBCLK | SYSCLK_APBCLK_PWM45_EN_Msk;
		SYSCLK->CLKSEL2 = SYSCLK->CLKSEL2 | SYSCLK_CLKSEL2_PWM45_HCLK;
		_expwm_pwm45_isinit = 1;
	}
	if((data->Chanell == 6 || data->Chanell == 7) && !_expwm_pwm67_isinit)
	{
		SYSCLK->APBCLK = SYSCLK->APBCLK | SYSCLK_APBCLK_PWM67_EN_Msk;
		SYSCLK->CLKSEL2 = SYSCLK->CLKSEL2 | SYSCLK_CLKSEL2_PWM67_HCLK;
		_expwm_pwm67_isinit = 1;
	}

	if(data->Chanell < 4)
	{
		// Initialize module a interrupt
		if(!_expwm_pwma_isinit)
		{
			NVIC_EnableIRQ(PWMA_IRQn);
			_expwm_pwma_isinit = 1;
		}
	}
	else
	{
		// Initialize module b interrupt
		if(!_expwm_pwmb_isinit)
		{
			NVIC_EnableIRQ(PWMB_IRQn);
			_expwm_pwmb_isinit = 1;
		}
	}
	_PWM_SET_TIMER_AUTO_RELOAD_MODE(PWM_MODULE,Chanell);
	_PWM_SET_TIMER_PRESCALE(PWM_MODULE,Chanell, data->Prescaler);
	_PWM_SET_TIMER_CLOCK_DIV(PWM_MODULE,Chanell,data->Divider);
	_PWM_SET_PWM_COMP_VALUE(PWM_MODULE,Chanell,data->Duty);
	_PWM_SET_TIMER_LOADED_VALUE(PWM_MODULE,Chanell,data->Period);
	switch(data->Port)
	{
		case EXPWM_PORT2:
			_PWM_ENABLE_PWM_OUT(PWM_MODULE,0x1UL << (data->Port + 8));
			break;
		case EXPWM_PORT4:
			_PWM_ENABLE_PWM_OUT(PWMA,0x1UL << data->Port);
			break;
		default:
			break;
	}
	if(data->Callback != 0)
	{
		_PWM_ENABLE_TIMER_PERIOD_INT(PWM_MODULE, Chanell);
		_expwm_callback_arr[data->Chanell] = data->Callback;
	}
	_PWM_ENABLE_TIMER(PWM_MODULE, Chanell);
}



/*
// Reset PWMA channel0~channel3
SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
SYS->IPRSTC2 = 0;

*/


void PWMA_IRQHandler(void)
{
	int i;
	for(i=0;i<4;i++)
	{
		_PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMA,i);
		if(_expwm_callback_arr[i] != 0)
		{
			_expwm_callback_arr[i]();
		}
	}
}

void PWMB_IRQHandler(void)
{
	int i;
	for(i=4;i<8;i++)
	{
		_PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMB,i-4);
		if(_expwm_callback_arr[i] != 0)
		{
			_expwm_callback_arr[i]();
		}
	}
}
