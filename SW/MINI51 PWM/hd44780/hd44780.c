#include "hd44780.h"


void hd44780_wr_hi_nibble( uint8_t data )
{
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT4) = data & 0x10 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT5) = data & 0x20 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT6) = data & 0x40 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT7) = data & 0x80 ? 1 : 0;
/*
	if ( data & 0x10 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	}
	if ( data & 0x20 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	}
	if ( data & 0x40 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	}
	if ( data & 0x80 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	}
*/
    /* set the EN signal */
    hd44780_EN_On();

    /* wait */
    hd44780_EN_high_delay();

    /* reset the EN signal */
    hd44780_EN_Off();

}


#if HD44780_CONF_BUS == HD44780_FUNC_BUS_4BIT

void hd44780_wr_lo_nibble( uint8_t data )
{
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT4) = data & 0x01 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT5) = data & 0x02 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT6) = data & 0x04 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT7) = data & 0x08 ? 1 : 0;
/*
	if ( data & 0x01 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	}
	if ( data & 0x02 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	}
	if ( data & 0x04 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	}
	if ( data & 0x08 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	}
*/
    /* set the EN signal */
    hd44780_EN_On();

    /* wait */
    hd44780_EN_high_delay();

    /* reset the EN signal */
    hd44780_EN_Off();

}


/* 4bit bus version */
void hd44780_write( uint8_t data )
{
	/* send the data bits - high nibble first */
	hd44780_wr_hi_nibble( data );
	hd44780_wr_lo_nibble( data );

}
#endif /* HD44780_CONF_BUS == HD44780_FUNC_BUS_4BIT */


#if HD44780_CONF_BUS == HD44780_FUNC_BUS_8BIT

/* 8bit bus version */
void hd44780_write( uint8_t data )
{
	/* set the data bits */
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT0) = data & 0x01 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT1) = data & 0x02 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT2) = data & 0x04 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT3) = data & 0x08 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT4) = data & 0x10 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT5) = data & 0x20 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT6) = data & 0x40 ? 1 : 0;
	GPIO_PIN_DATA(HD44780_DATAPORT, HD44780_DATABIT7) = data & 0x80 ? 1 : 0;


/*
	if ( data & 0x01 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT0 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT0 );
	}
	if ( data & 0x02 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT1 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT1 );
	}
	if ( data & 0x04 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT2 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT2 );
	}
	if ( data & 0x08 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT3 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT3 );
	}
	if ( data & 0x10 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT4 );
	}
	if ( data & 0x20 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT5 );
	}
	if ( data & 0x40 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT6 );
	}
	if ( data & 0x80 ) {
		GPIO_SetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	} else {
		GPIO_ResetBits( HD44780_DATAPORT, HD44780_DATABIT7 );
	}
*/
    /* tell the lcd that we have a command to read in */
    hd44780_EN_On();

    /* wait long enough so that the lcd can see the command */
    hd44780_EN_high_delay();

    /* reset the ce line */
    hd44780_EN_Off();
    hd44780_init_end_delay();

}
#endif /* HD44780_CONF_BUS == HD44780_FUNC_BUS_8BIT */


void hd44780_wr_cmd( uint8_t cmd )
{
	hd44780_RS_Off();
	hd44780_write( cmd );
}


void hd44780_wr_data( uint8_t data )
{
	hd44780_RS_On();
	hd44780_write( data );
}


void hd44780_init( void )
{

#if HD44780_CONF_BUS == HD44780_FUNC_BUS_8BIT
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT0, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT1, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT2, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT3, GPIO_PMD_OUTPUT);
#endif

	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT4, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT5, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT6, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_DATAPORT, HD44780_DATABIT7, GPIO_PMD_OUTPUT);

	_GPIO_SET_PIN_MODE(HD44780_RS_PORT, HD44780_RS_BIT, GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(HD44780_EN_PORT, HD44780_EN_BIT, GPIO_PMD_OUTPUT);

    // Output default value : E disable
	hd44780_EN_Off();
    // Set Entry Mode: Interface Data Length, Character Font, Display Line
	SYS_SysTickDelay(1000);



	/* From CooCox
    //
    // Enable GPIO Port that used
    //
#if (HD44780_INTERFACE_DATA_LEN == HD44780_INTERFACE_DATA_LEN_8)
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D7));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D6));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D5));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D4));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D3));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D2));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D1));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D0));
#else
#if (HD44780_INTERFACE_DATA_LEN == HD44780_INTERFACE_DATA_LEN_4)
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D7));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D6));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D5));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_D4));
#endif
#endif

    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_E));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_RS));
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(HD44780_PIN_RW));
#endif

    //
    // Set Pins Type to GPIO Output
    //
#if (HD44780_INTERFACE_DATA_LEN == HD44780_INTERFACE_DATA_LEN_8)
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D7);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D6);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D5);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D4);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D3);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D2);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D1);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D0);
#else
#if (HD44780_INTERFACE_DATA_LEN == HD44780_INTERFACE_DATA_LEN_4)
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D7);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D6);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D5);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_D4);
#endif
#endif

    xGPIOSPinTypeGPIOOutput(HD44780_PIN_E);
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_RS);
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    xGPIOSPinTypeGPIOOutput(HD44780_PIN_RW);
#endif
    //
    // Output default value : E disable
    //
    xGPIOSPinWrite(HD44780_PIN_E, HD44780_E_DISABLE);

    //
    // Set Entry Mode: Interface Data Length, Character Font, Display Line
    //
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    while(HD44780Busy());
#else
    xSysCtlDelay(HD44780_WRITEDELAY);
#endif

#if (defined(HD44780_DISPLAY_LINE) && (HD44780_DISPLAY_LINE == 2))
    HD44780WriteCmd(HD44780_CMD_FUNCTION_SET(HD44780_INTERFACE_DATA_LEN |
                                             HD44780_FUNCTION_SET_N_2 |
                                             HD44780_CHARACTER_FONT));
#endif

#if (defined(HD44780_DISPLAY_LINE) && (HD44780_DISPLAY_LINE == 1))
    HD44780WriteCmd(HD44780_CMD_FUNCTION_SET(HD44780_INTERFACE_DATA_LEN |
                                             HD44780_FUNCTION_SET_N_1 |
                                             HD44780_CHARACTER_FONT));
#endif

    //
    // Display on & Cursor Blink
    //
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    while(HD44780Busy());
#else
    xSysCtlDelay(HD44780_WRITEDELAY);
#endif
    HD44780WriteCmd(HD44780_CMD_DISPLAY_CTRL(HD44780_DISPLAY_CTRL_D |
                                             0 |
                                             HD44780_DISPLAY_CTRL_B));

    //
    // Clear LCD
    //
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    while(HD44780Busy());
#else
    xSysCtlDelay(HD44780_WRITEDELAY);
#endif
    HD44780WriteCmd(HD44780_CMD_CLS);

    //
    // Cursor Move Mode: Increment
    //
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    while(HD44780Busy());
#else
    xSysCtlDelay(HD44780_WRITEDELAY);
#endif
    HD44780WriteCmd(HD44780_CMD_ENTRY_MODE_SET(HD44780_ENTRY_MODE_SET_ID_INC |
                                               0));
#if (HD44780_RWMODE == HD44780_RWMODE_READWRITE)
    while(HD44780Busy());
#else
    xSysCtlDelay(HD44780_POSTINITDELAY);
#endif

*/


/* Orig
	// clear control bits
	hd44780_EN_Off();
	hd44780_RS_Off();
	// hd44780_RW_Off();

    // wait initial delay for LCD to settle
    // reset procedure - 3 function calls resets the device
    hd44780_init_delay();
    hd44780_wr_hi_nibble( HD44780_CMD_RESET );
    hd44780_init_delay2();
    hd44780_wr_hi_nibble( HD44780_CMD_RESET );
    hd44780_init_delay3();
    hd44780_wr_hi_nibble( HD44780_CMD_RESET );

#if HD44780_CONF_BUS == HD44780_FUNC_BUS_4BIT
    // 4bit interface
    hd44780_wr_hi_nibble( HD44780_CMD_FUNCTION );
#endif // HD44780_CONF_BUS == HD44780_FUNC_BUS_4BIT

    // sets the configured values - can be set again only after reset
    hd44780_function( HD44780_CONF_BUS, HD44780_CONF_LINES, HD44780_CONF_FONT );

    // turn the display off with no cursor or blinking
    hd44780_display( HD44780_DISP_OFF, HD44780_DISP_CURS_OFF, HD44780_DISP_BLINK_OFF );

    // clear the display
    hd44780_clear();

    // addr increment, shift cursor
    hd44780_entry( HD44780_ENTRY_ADDR_INC, HD44780_ENTRY_SHIFT_DISP );

    hd44780_init_delay();
*/
}


void hd44780_write_string( char *s )
{
    uint32_t i;
    for( i=0; s[i]!='\0'; ++i ) {
        hd44780_write_char( s[i] );
    }
}


void hd44780_write_line( uint8_t line, char *msg )
{
	if ( line >= HD44780_DISP_ROWS ) return;
	/* TODO: dokonËiù... */
}
