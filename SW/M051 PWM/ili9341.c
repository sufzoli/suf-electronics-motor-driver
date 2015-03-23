#include "sys.h"
#include "spi.h"
#include "gpio.h"

#include "ili9341.h"
#include "ili9341_fonts.h"

void ILI9341_SendData(unsigned char data)
{
	ILI9341_DC_DATA;
	// write the first data of source buffer to Tx register of SPI0.
	_SPI_WRITE_TX_BUFFER0(SPI0, data);
	// set the GO_BUSY bit of SPI0
	_SPI_SET_GO(SPI0);
	// Check the GO_BUSY bit
//	while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
	SYS_SysTickDelay(30);
}

void ILI9341_SendDataWord(unsigned int data)
{
	SPI0->CNTRL &= ~SPI_CNTRL_TX_BIT_LEN_Msk;
	SPI0->CNTRL |= SPI_CNTRL_TX_BIT_LEN(16);
	ILI9341_DC_DATA;
	// write the first data of source buffer to Tx register of SPI0.
	_SPI_WRITE_TX_BUFFER0(SPI0, data);
	// set the GO_BUSY bit of SPI0
	_SPI_SET_GO(SPI0);
	SYS_SysTickDelay(20);
	// Check the GO_BUSY bit
//	while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
	SPI0->CNTRL &= ~SPI_CNTRL_TX_BIT_LEN_Msk;
	SPI0->CNTRL |= SPI_CNTRL_TX_BIT_LEN(8);
}

void ILI9341_SendCommand(unsigned char data)
{
	ILI9341_DC_COMMAND;
	// write the first data of source buffer to Tx register of SPI0.
	_SPI_WRITE_TX_BUFFER0(SPI0, data);
	// set the GO_BUSY bit of SPI0
	_SPI_SET_GO(SPI0);
	// Check the GO_BUSY bit
//	while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY);
	SYS_SysTickDelay(30);
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
	unsigned short x;
	x = Font7x10[20];
    /* Configure SPI0 as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    SPI0->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
                     SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(8);
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSR = SPI_SSR_HW_AUTO_ACTIVE_LOW;
    /* Set BCn bit to 1 for more flexible clock configuration. */
	_SPI_SET_CLOCK_SETTING_NOT_BACKWARD_COMPATIBLE(SPI0);
	/* Set IP clock divider. SPI clock rate = HCLK / (9+1) */
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | SPI_DIVIDER_DIV(99);

    ILI9341_DC_DATA; // D/C
    ILI9341_HardReset(); // Reset

	ILI9341_SendCommand(ILI9341_RESET);
	SYS_SysTickDelay(2000);

	ILI9341_SendCommand(ILI9341_POWERA);
	ILI9341_SendData(0x39);
	ILI9341_SendData(0x2C);
	ILI9341_SendData(0x00);
	ILI9341_SendData(0x34);
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
	ILI9341_SendCommand(ILI9341_MAC);
	ILI9341_SendData(0x48);
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

	SYS_SysTickDelay(1000);

	ILI9341_SendCommand(ILI9341_DISPLAY_ON);
	ILI9341_SendCommand(ILI9341_GRAM);

	SYS_SysTickDelay(1000);

	// ILI9341_FillScreen();


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
	ILI9341_SendData(x1 >> 8);
	ILI9341_SendData(x1 & 0xFF);
	ILI9341_SendData(x2 >> 8);
	ILI9341_SendData(x2 & 0xFF);

	ILI9341_SendCommand(ILI9341_PAGE_ADDR);
	ILI9341_SendData(y1 >> 8);
	ILI9341_SendData(y1 & 0xFF);
	ILI9341_SendData(y2 >> 8);
	ILI9341_SendData(y2 & 0xFF);
}

void ILI9341_PrintChar()
{

}

/*
void ILI9341_SetCol(unsigned int StartCol,unsigned int EndCol)
{
	ILI9341_SendCommand(0x2A);                                                      // Column Command address
	ILI9341_SendDataWord(StartCol);
	ILI9341_SendDataWord(EndCol);
}

void ILI9341_SetPage(unsigned int StartPage,unsigned int EndPage)
{
	ILI9341_SendCommand(0x2B);                                                      // Column Command address
	ILI9341_SendDataWord(StartPage);
	ILI9341_SendDataWord(EndPage);
}

void ILI9341_FillScreen(void)
{
	unsigned int i;
	ILI9341_SetCol(0, 239);
	ILI9341_SetPage(0, 319);
	ILI9341_SendCommand(0x2c);                                                  // start to write to display ram

//    TFT_DC_HIGH;
//    TFT_CS_LOW;

    for(i=0; i<38400; i++)
    {
    	ILI9341_SendData(0xFF);
    	ILI9341_SendData(0xFF);
    	ILI9341_SendData(0xFF);
    	ILI9341_SendData(0xFF);
    }
//    TFT_CS_HIGH;
}
*/

/*
void TFT::setPixel(INT16U poX, INT16U poY,INT16U color)
{
    sendCMD(0x2A);                                                      // Column Command address
    sendData(poX);
    sendData(poX);

    sendCMD(0x2B);                                                      // Column Command address
    sendData(poY);
    sendData(poY);

    sendCMD(0x2c);
    sendData(color);
}
*/

/*
inline void ILI9341_SendCommand(INT8U index)
{
    TFT_DC_LOW;
    TFT_CS_LOW;
    SPI.transfer(index);
    TFT_CS_HIGH;
}

inline void ILI9341_SendData(INT8U data)
{
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data);
    TFT_CS_HIGH;
}

inline void sendData(INT16U data)
{
    INT8U data1 = data>>8;
    INT8U data2 = data&0xff;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data1);
    SPI.transfer(data2);
    TFT_CS_HIGH;
}
*/
