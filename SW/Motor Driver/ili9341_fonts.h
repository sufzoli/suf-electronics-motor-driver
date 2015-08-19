
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
extern const fonttype Font11x18;
extern const fonttype Font16x26;

#endif
