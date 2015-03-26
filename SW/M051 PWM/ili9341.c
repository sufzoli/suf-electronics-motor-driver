#include "sys.h"
#include "spi.h"
#include "gpio.h"

#include "ili9341.h"
#include "ili9341_fonts.h"

void ILI9341_SPISend(unsigned long data, unsigned char len, unsigned char dc)
{
	// if we need to change the transfer width
	if((SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) != SPI_CNTRL_TX_BIT_LEN(len))
	{
		// wait until the FIFO and the port gets free (last transmit finished)
		while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
		// check if the Data/Command bit need to be changed and change it
		if(ILI9341_DC_PORT != dc)
		{
			ILI9341_DC_PORT = dc;
		}
		// Change data length
		SPI0->CNTRL = (SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(len);
	}
	else
	{
		// check if the Data/Command bit need to be changed
		if(ILI9341_DC_PORT != dc)
		{
			// wait until the FIFO and the port gets free (last transmit finished)
			while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
			// change the Data/Command bit
			ILI9341_DC_PORT = dc;
		}
	}
	while(SPI0->CNTRL & SPI_CNTRL_TX_FULL_Msk);
	// transmit the data
	_SPI_WRITE_TX_BUFFER0(SPI0, data);
}

void ILI9341_SPISendRaw(unsigned long data)
{
	while(SPI0->CNTRL & SPI_CNTRL_TX_FULL_Msk);
	_SPI_WRITE_TX_BUFFER0(SPI0, data);
}

void ILI9341_SPIDataLen(unsigned char len)
{
	while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
	SPI0->CNTRL = (SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(len);
}

void ILI9341_HardReset()
{
    ILI9341_RESET_PORT = 0; // Reset
    SYS_SysTickDelay(1000);
    ILI9341_RESET_PORT = 1;
    SYS_SysTickDelay(1000);
}

void ILI9341_Init()
{
    // Configure SPI0 as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction
//    SPI0->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
//                     SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(8);
    SPI0->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
                     SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(8) | SPI_CNTRL_FIFO_MODE_EN;
    // Enable the automatic hardware slave select function. Select the SS pin and configure as low-active.
    SPI0->SSR = SPI_SSR_HW_AUTO_ACTIVE_LOW;
    // Set BCn bit to 1 for more flexible clock configuration.
	_SPI_SET_CLOCK_SETTING_NOT_BACKWARD_COMPATIBLE(SPI0);
	// Set IP clock divider. SPI clock rate = HCLK / (9+1)
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | SPI_DIVIDER_DIV(1);

    ILI9341_DC_DATA; // D/C
    ILI9341_HardReset(); // Reset

	ILI9341_SendCommand(ILI9341_RESET);
	SYS_SysTickDelay(5000);	// 5ms wait (described in the datasheet)

	ILI9341_SendCommand(ILI9341_POWERA);
	ILI9341_SendData(0x39);
	ILI9341_SendData(0x2C);
	ILI9341_SendData(0x00);
	ILI9341_SendData(ILI9341_POWERA_REG_VD_16);
	ILI9341_SendData(0x02);
	ILI9341_SendCommand(ILI9341_POWERB);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0xC1);
	ILI9341_SendData(0x30);
	ILI9341_SendCommand(ILI9341_DTCA);
	ILI9341_SendData(0x85);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x78);
	ILI9341_SendCommand(ILI9341_DTCB);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_POWER_SEQ);
	ILI9341_SendData(0x64);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x12);
	ILI9341_SendData(0x81);
	ILI9341_SendCommand(ILI9341_PRC);
	ILI9341_SendData(0x20);
	ILI9341_SendCommand(ILI9341_POWER1);
	ILI9341_SendData(0x23);
	ILI9341_SendCommand(ILI9341_POWER2);
	ILI9341_SendData(0x10);
	ILI9341_SendCommand(ILI9341_VCOM1);
	ILI9341_SendData(0x3E);
	ILI9341_SendData(0x28);
	ILI9341_SendCommand(ILI9341_VCOM2);
	ILI9341_SendData(0x86);
	ILI9341_SendCommand(ILI9341_MADCTL);	// Orientation control
	ILI9341_SendData(ILI9341_MADCTL_MY | ILI9341_MADCTL_MX | ILI9341_MADCTL_MV  | ILI9341_MADCTL_BGR);
	ILI9341_SendCommand(ILI9341_PIXEL_FORMAT);
	ILI9341_SendData(0x55);
	ILI9341_SendCommand(ILI9341_FRC);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x18);
	ILI9341_SendCommand(ILI9341_DFC);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x82);
	ILI9341_SendData(0x27);
	ILI9341_SendCommand(ILI9341_3GAMMA_EN);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0xEF);
	ILI9341_SendCommand(ILI9341_PAGE_ADDR);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x01);
	ILI9341_SendData(0x3F);
	ILI9341_SendCommand(ILI9341_GAMMA);
	ILI9341_SendData(0x01);
	ILI9341_SendCommand(ILI9341_PGAMMA);
	ILI9341_SendData(0x0F);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0x2B);
	ILI9341_SendData(0x0C);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x4E);
	ILI9341_SendData(0xF1);
	ILI9341_SendData(0x37);
	ILI9341_SendData(0x07);
	ILI9341_SendData(0x10);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x09);
	ILI9341_SendData(0x00);
	ILI9341_SendCommand(ILI9341_NGAMMA);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x0E);
	ILI9341_SendData(0x14);
	ILI9341_SendData(0x03);
	ILI9341_SendData(0x11);
	ILI9341_SendData(0x07);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0xC1);
	ILI9341_SendData(0x48);
	ILI9341_SendData(0x08);
	ILI9341_SendData(0x0F);
	ILI9341_SendData(0x0C);
	ILI9341_SendData(0x31);
	ILI9341_SendData(0x36);
	ILI9341_SendData(0x0F);
	ILI9341_SendCommand(ILI9341_SLEEP_OUT);

	SYS_SysTickDelay(5000);

	ILI9341_SendCommand(ILI9341_DISPLAY_ON);
//	ILI9341_SendCommand(ILI9341_GRAM);

//	SYS_SysTickDelay(3000);

	ILI9341_FillScreen();


    /*
    ILI9341_SendCommand(0x01);
    SYS_SysTickDelay(2000);

    ILI9341_SendCommand(0xCF);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x8B);
    ILI9341_SendData(0X30);

    ILI9341_SendCommand(0xED);
    ILI9341_SendData(0x67);
    ILI9341_SendData(0x03);
    ILI9341_SendData(0X12);
    ILI9341_SendData(0X81);

    ILI9341_SendCommand(0xE8);
    ILI9341_SendData(0x85);
    ILI9341_SendData(0x10);
    ILI9341_SendData(0x7A);

    ILI9341_SendCommand(0xCB);
    ILI9341_SendData(0x39);
    ILI9341_SendData(0x2C);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x34);
    ILI9341_SendData(0x02);

    ILI9341_SendCommand(0xF7);
    ILI9341_SendData(0x20);

    ILI9341_SendCommand(0xEA);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x00);

    ILI9341_SendCommand(0xC0);                                                      // Power control
    ILI9341_SendData(0x1B);                                                   // VRH[5:0]

    ILI9341_SendCommand(0xC1);                                                      // Power control
    ILI9341_SendData(0x10);                                                   // SAP[2:0];BT[3:0]

    ILI9341_SendCommand(0xC5);                                                      // VCM control
    ILI9341_SendData(0x3F);
    ILI9341_SendData(0x3C);

    ILI9341_SendCommand(0xC7);                                                      // VCM control2
    ILI9341_SendData(0XB7);

    ILI9341_SendCommand(0x36);                                                      // Memory Access Control
    ILI9341_SendData(0x08);

    ILI9341_SendCommand(0x3A);
    ILI9341_SendData(0x55);

    ILI9341_SendCommand(0xB1);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x1B);

    ILI9341_SendCommand(0xB6);                                                      // Display Function Control
    ILI9341_SendData(0x0A);
    ILI9341_SendData(0xA2);


    ILI9341_SendCommand(0xF2);                                                      // 3Gamma Function Disable
    ILI9341_SendData(0x00);

    ILI9341_SendCommand(0x26);                                                      // Gamma curve selected
    ILI9341_SendData(0x01);

    ILI9341_SendCommand(0xE0);                                                      // Set Gamma
    ILI9341_SendData(0x0F);
    ILI9341_SendData(0x2A);
    ILI9341_SendData(0x28);
    ILI9341_SendData(0x08);
    ILI9341_SendData(0x0E);
    ILI9341_SendData(0x08);
    ILI9341_SendData(0x54);
    ILI9341_SendData(0XA9);
    ILI9341_SendData(0x43);
    ILI9341_SendData(0x0A);
    ILI9341_SendData(0x0F);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x00);

    ILI9341_SendCommand(0XE1);                                                      // Set Gamma
    ILI9341_SendData(0x00);
    ILI9341_SendData(0x15);
    ILI9341_SendData(0x17);
    ILI9341_SendData(0x07);
    ILI9341_SendData(0x11);
    ILI9341_SendData(0x06);
    ILI9341_SendData(0x2B);
    ILI9341_SendData(0x56);
    ILI9341_SendData(0x3C);
    ILI9341_SendData(0x05);
    ILI9341_SendData(0x10);
    ILI9341_SendData(0x0F);
    ILI9341_SendData(0x3F);
    ILI9341_SendData(0x3F);
    ILI9341_SendData(0x0F);

    ILI9341_SendCommand(0x11);                                                      // Exit Sleep
    SYS_SysTickDelay(120);
    ILI9341_SendCommand(0x29);                                                      // Display on
    ILI9341_FillScreen();
*/

}


void ILI9341_DrawPixel(unsigned int x, unsigned int  y, uint32_t color)
{
	ILI9341_SetCursorPosition(x, y, x, y);
	ILI9341_SendCommand(ILI9341_GRAM);
	ILI9341_SendData(color >> 8);
	ILI9341_SendData(color & 0xFF);
}


void ILI9341_SetCursorPosition(unsigned int  x1, unsigned int  y1, unsigned int  x2, unsigned int y2)
{
	ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
	ILI9341_SendDataWord(x1);
	ILI9341_SendDataWord(x2);
	ILI9341_SendCommand(ILI9341_PAGE_ADDR);
	ILI9341_SendDataWord(y1);
	ILI9341_SendDataWord(y2);
}

void ILI9341_FillScreen(void)
{
	unsigned long i;
	ILI9341_SetCursorPosition(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
	ILI9341_SendCommand(ILI9341_GRAM);

	ILI9341_SPIDataLen(32);
	ILI9341_DC_DATA;
	for(i=0; i < ((ILI9341_WIDTH * ILI9341_HEIGHT) >> 1);i++)
	{
		ILI9341_SPISendRaw(0); // Black
	}
}

void ILI9341_PrintChar(fonttype *font, unsigned char code, unsigned long x, unsigned long y, unsigned long frontcolor, unsigned long backcolor)
{
	void *lineptr;
	unsigned long line;
	unsigned long i,j;

	ILI9341_SetCursorPosition(x,y,x+font->xsize,y+font->ysize);
	ILI9341_SendCommand(ILI9341_GRAM);

	ILI9341_SPIDataLen(16);
	ILI9341_DC_DATA;

	for(i = 0; i < font->ysize; i++)
	{
		lineptr = font->fontptr + ((i  + (((unsigned long)(code - 32)) * (font->ysize))) << (font->bytelen - 1));
		switch(font->bytelen)
		{
			case 1:
				line = *((unsigned char*)lineptr);
				break;
			case 2:
				line = *((unsigned short*)lineptr);
				break;
			case 4:
				line = *((unsigned long*)lineptr);
			default:
				break;
		}

		for(j=0; j < font->xsize +1; j++)
		{
			ILI9341_SPISendRaw((((line) & (0x01uL << ((font->bytelen << 3) - j))) > 0) ? frontcolor : backcolor);
		}
	}
}



void ILI9341_PrintStr(fonttype *font, char* str, unsigned long x, unsigned long y, unsigned long frontcolor, unsigned long backcolor)
{
	int i = 0;
	while(str[i] != 0)
	{
		ILI9341_PrintChar(font, str[i], x + (i * (font->xsize+1)), y , frontcolor, backcolor);
		i++;
	}
}
