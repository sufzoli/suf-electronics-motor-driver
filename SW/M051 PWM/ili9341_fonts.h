
#ifndef __ILI9341_FONTS_H__
#define __ILI9341_FONTS_H__

typedef struct
{
	unsigned long xsize;
	unsigned long ysize;
	unsigned char bytelen;
	void * fontptr;
} fonttype;

extern const fonttype Font7x10;
// extern const unsigned char Font7x10[];

#endif
