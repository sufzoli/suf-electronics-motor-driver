#include "ili9341_fonts.h"

#define ILI9341_RESET_PORT	P11
#define ILI9341_DC_PORT	P10

#define ILI9341_WIDTH 320
#define ILI9341_HEIGHT 240

#define ILI9341_DC_COMMAND (ILI9341_DC_PORT = 0)
#define ILI9341_DC_DATA (ILI9341_DC_PORT = 1)

// Commands
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MADCTL				0x36
#define ILI9341_PIXEL_FORMAT		0x3A
#define ILI9341_WDB					0x51
#define ILI9341_WCD					0x53
#define ILI9341_RGB_INTERFACE		0xB0
#define ILI9341_FRC					0xB1
#define ILI9341_BPC					0xB5
#define ILI9341_DFC					0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC					0xF7

// Command parameters
// MADCTL
#define ILI9341_MADCTL_MY			0x80
#define ILI9341_MADCTL_MX			0x40
#define ILI9341_MADCTL_MV			0x20
#define ILI9341_MADCTL_ML			0x10
#define ILI9341_MADCTL_BGR			0x08
#define ILI9341_MADCTL_MH			0x04

// POWERA
#define ILI9341_POWERA_REG_VD_155	0x30
#define ILI9341_POWERA_REG_VD_14	0x31
#define ILI9341_POWERA_REG_VD_15	0x32
#define ILI9341_POWERA_REG_VD_165	0x33
#define ILI9341_POWERA_REG_VD_16	0x34
#define ILI9341_POWERA_REG_VD_17	0x35

// Colors
#define ILI9341_COLOR_WHITE			0xFFFF
#define ILI9341_COLOR_BLACK			0x0000
#define ILI9341_COLOR_RED			0xF800
#define ILI9341_COLOR_GREEN			0x07E0
#define ILI9341_COLOR_GREEN2		0xB723
#define ILI9341_COLOR_BLUE			0x001F
#define ILI9341_COLOR_BLUE2			0x051D
#define ILI9341_COLOR_YELLOW		0xFFE0
#define ILI9341_COLOR_ORANGE		0xFBE4
#define ILI9341_COLOR_CYAN			0x07FF
#define ILI9341_COLOR_MAGENTA		0xA254
#define ILI9341_COLOR_GRAY			0x7BEF //1111 0111 1101 1110
#define ILI9341_COLOR_BROWN			0xBBCA

extern void ILI9341_SPISend(unsigned long data, unsigned char len, unsigned char dc);
//extern void ILI9341_SPISendRaw(unsigned long data);
#define ILI9341_SendCommand(x)	ILI9341_SPISend(x, 8, 0)
#define ILI9341_SendData(x)	ILI9341_SPISend(x, 8, 1)
#define ILI9341_SendDataWord(x)	ILI9341_SPISend(x, 16, 1)

extern void ILI9341_Init();
extern void ILI9341_FillScreen();
extern void ILI9341_DrawPixel(unsigned int x, unsigned int  y, uint32_t color);
extern void ILI9341_SetCursorPosition(unsigned int  x1, unsigned int  y1, unsigned int  x2, unsigned int y2);
void ILI9341_PrintChar(fonttype *font, unsigned char code, unsigned long x, unsigned long y, unsigned long frontcolor, unsigned long backcolor);
void ILI9341_PrintStr(fonttype *font, char* str, unsigned long x, unsigned long y, unsigned long frontcolor, unsigned long backcolor);


