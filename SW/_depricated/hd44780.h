
// #define HD44780_PIN_D0
// #define HD44780_PIN_D1
// #define HD44780_PIN_D2
// #define HD44780_PIN_D3
#define HD44780_PIN_D4	P30
#define HD44780_PIN_D5	P31
#define HD44780_PIN_D6	P32
#define HD44780_PIN_D7	P33

#define HD44780_PIN_RS	P35
#define HD44780_PIN_E	P34
// #define HD44780_PIN_RW	P36

#define HD44780_FUNC_BUS	HD44780_FUNC_BUS_4BIT
#define HD44780_FUNC_LINES	HD44780_FUNC_LINES_2
#define HD44780_FUNC_FONT	HD44780_FUNC_FONT_5x8

#define HD44780_EN_DELAY	2000
#define HD44780_INIT_DELAY	20000

#define HD44780_RS_COMMAND      0
#define HD44780_RS_DATA         1
#define HD44780_E_DISABLE       0
#define HD44780_E_ENABLE        1

/* COMMANDS */
#define HD44780_CMD_RESET            	0x30     /*!< Resets display - used in init 3x */
#define HD44780_CMD_CLEAR            	0x01     /*!< Clears display */
#define HD44780_CMD_RETURN_HOME      	0x02     /*!< Sets DDRAM pointer to 0 */
#define HD44780_CMD_ENTRY_MODE       	0x04     /*!< Sets how the pointer is updated after a character write */
#define HD44780_CMD_DISP	          	0x08     /*!< Display settings */
#define HD44780_CMD_SHIFT            	0x10     /*!< Cursor and display movement */
#define HD44780_CMD_FUNC	         	0x20     /*!< Screen type setup */
#define HD44780_CMD_CGRAM_ADDR       	0x40     /*!< Sets CGRAM address */
#define HD44780_CMD_DDRAM_ADDR       	0x80     /*!< Sets DDRAM address */

/* ENTRY_MODE Command parameters */
#define HD44780_ENTRY_SHIFT_DISP 		0x01	 /*!< Shift display */
#define HD44780_ENTRY_SHIFT_CURS 		0x00	 /*!< Shift cursor */
#define HD44780_ENTRY_ADDR_INC   		0x02     /*!< Increments pointer */
#define HD44780_ENTRY_ADDR_DEC   		0x00	 /*!< Decrements pointer */

/* FUNCTION Command parameters */
#define HD44780_FUNC_BUS_8BIT  			0x10      /*!< 8 bit bus */
#define HD44780_FUNC_BUS_4BIT  			0x00      /*!< 4 bit bus */
#define HD44780_FUNC_LINES_2   			0x08      /*!< 2 lines */
#define HD44780_FUNC_LINES_1   			0x00      /*!< 1 line */
#define HD44780_FUNC_FONT_5x10 			0x04      /*!< 5x10 font */
#define HD44780_FUNC_FONT_5x8  			0x00      /*!< 5x8 font */

/* DISPLAY Command parameters */
#define HD44780_DISP_ON       			0x04      /*!< Enables the display */
#define HD44780_DISP_OFF      			0x00      /*!< Disables the display */
#define HD44780_DISP_CURS_ON  			0x02      /*!< Enables cursor */
#define HD44780_DISP_CURS_OFF 			0x00      /*!< Disables cursor */
#define HD44780_DISP_BLINK_ON			0x01      /*!< Enables cursor blinking */
#define HD44780_DISP_BLINK_OFF  		0x00      /*!< Disables cursor blinking */

/* SHIFT Command parameters */
#define HD44780_SHIFT_DISPLAY    		0x08      /*!< Shifts the display or shifts the cursor if not set */
#define HD44780_SHIFT_CURSOR    		0x00      /*!< Shifts the display or shifts the cursor if not set */
#define HD44780_SHIFT_RIGHT      		0x04      /*!< Shift to the right */
#define HD44780_SHIFT_LEFT      		0x00      /*!< Shift to the left  */

extern void HD44780_Init();
extern void HD44780_DisplayString(const char* pcString);
extern void HD44780_LocationSet(unsigned char x, unsigned char y);
extern void HD44780_DisplayN_POS(unsigned long n, unsigned char x, unsigned char y, unsigned char len, unsigned char dp);
