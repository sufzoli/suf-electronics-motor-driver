#include "hd44780.h"
#include "xhw_types.h"
int main(void)
{
	HD44780Init();
	// HD44780CursorOn(xtrue);
	HD44780DisplayString("Hello World!");
    while(1)
    {
    }
}
