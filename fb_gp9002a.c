/*
 * FB driver for the Futaba GP9002A VFD sold by Adafruit
 *
 * The display is set to 2 bits per pixel, but is converted here from RGB565
 *
 * Copyright (C) 2014 Alex O'Neill
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME	       "fb_gp9002a"
#define WIDTH          128
#define HEIGHT         64
#define TXBUFLEN       64*32
#define DEFAULT_GAMMA  "40" /* gamma is used to control contrast in this driver */
#define CNVT_RGB565_2BIT(x) (((x & 0x1F) >> 3) +(((x >> 5) & 0x3F) >> 2) + (((x >> 11) & 0x1F) >> 3)) >> 6

/*
* These register defines taken from Adafruit's GP9002A library 
* 
*/
#define GP9002_DISPLAYSOFF 0x00
#define GP9002_DISPLAY1ON 0x01
#define GP9002_DISPLAY2ON 0x02
#define GP9002_ADDRINCR 0x04
#define GP9002_ADDRHELD 0x05
#define GP9002_CLEARSCREEN 0x06
#define GP9002_CONTROLPOWER 0x07
#define GP9002_DATAWRITE 0x08
#define GP9002_DATAREAD 0x09
#define GP9002_LOWERADDR1 0x0A
#define GP9002_HIGHERADDR1 0x0B
#define GP9002_LOWERADDR2 0x0C
#define GP9002_HIGHERADDR2 0x0D
#define GP9002_ADDRL 0x0E
#define GP9002_ADDRH 0x0F
#define GP9002_OR 0x10
#define GP9002_XOR 0x11
#define GP9002_AND 0x12
#define GP9002_BRIGHT 0x13
#define GP9002_DISPLAY 0x14
#define GP9002_DISPLAY_MONOCHROME 0x10
#define GP9002_DISPLAY_GRAYSCALE 0x14
#define GP9002_INTMODE 0x15
#define GP9002_DRAWCHAR 0x20
#define GP9002_CHARRAM 0x21
#define GP9002_CHARSIZE 0x22
#define GP9002_CHARBRIGHT 0x24


static void write_reg8_bus8(struct fbtft_par *par, int len, ...)
{
	va_list args;
	int i, ret;
	u8 *buf = (u8 *)par->buf;

	if (unlikely(par->debug & DEBUG_WRITE_REGISTER)) {
		va_start(args, len);
		for (i = 0; i < len; i++) {
			buf[i] = (u8)va_arg(args, unsigned int);
		}
		va_end(args);
		fbtft_par_dbg_hex(DEBUG_WRITE_REGISTER, par, par->info->device, u8, buf, len, "%s: ", __func__);
	}

	va_start(args, len);

	*buf = (u8)va_arg(args, unsigned int);
	if (par->gpio.dc != -1)
		gpio_set_value(par->gpio.dc, 1);
	ret = par->fbtftops.write(par, par->buf, sizeof(u8));
	if (ret < 0) {
		va_end(args);
		dev_err(par->info->device, "%s: write() failed and returned %d\n", __func__, ret);
		return;
	}
	len--;

	if (len) {
		i = len;
		while (i--) {
			*buf++ = (u8)va_arg(args, unsigned int);
		}
		if (par->gpio.dc != -1)
			gpio_set_value(par->gpio.dc, 0);
		ret = par->fbtftops.write(par, par->buf, len * (sizeof(u8)));
		if (ret < 0) {
			va_end(args);
			dev_err(par->info->device, "%s: write() failed and returned %d\n", __func__, ret);
			return;
		}
	}
	if (par->gpio.dc < 1)
		gpio_set_value(par->gpio.dc, 1);
	va_end(args);
}


static int init_display(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	par->fbtftops.reset(par);

	/* Function set */
	write_reg(par, GP9002_DISPLAY, GP9002_DISPLAY_GRAYSCALE);
	write_reg(par, GP9002_CLEARSCREEN);
	write_reg(par, GP9002_DISPLAY1ON);



	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par, "%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	write_reg(par, GP9002_ADDRL, 0x0); /* Set the lower address. Must set lower address first */
	write_reg(par, GP9002_ADDRH, 0x0); /* Set upper address */
}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16 = (u16 *)par->info->screen_base;
	u8 *buf = par->txbuf.buf;
	int x, y, i;
	int ret = 0;



	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s()\n", __func__);

	for (x=0;x<128;x++) {
		for (y=0;y<16;y++) {
			*buf = 0x00;
			for (i=0;i<4;i++) {
				*buf |=  CNVT_RGB565_2BIT(vmem16[(y*4+i)*128+x]) << i*2;
			}
			buf++;
		}
	}

	/* Write data */
	gpio_set_value(par->gpio.dc, 0);
	ret = par->fbtftops.write(par, par->txbuf.buf, TXBUFLEN);
	if (ret < 0)
		dev_err(par->info->device, "%s: write failed and returned: %d\n", __func__, ret);

	return ret;
}


static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{
	/*Brightness values
	 * Register 0x13
	 * 0x00 / 0   - full brightness
	 * 0x07 / 7   - 90 percent
	 * 0x0E / 14  - 80 percent
	 * 0x15 / 21  - 70 percent
	 * 0x1C / 28  - 60 percent
	 * 0x24 / 36  - 50 percent
	 * 0x2B / 43  - 40 percent
	 * 0x32 / 50  - 30 percent
	 * 0xFF / 255 - Blank
	 * Forumla is 
	 * y = 255 for x = 255
	 * y = (x/32 * 7) for y <= 28
	 * y = (x/32 * 7)+1 for y > 28
	 */
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);
	uint8_t lum_val, switch_val;
	/* map values */
	switch_val = (uint8_t) curves[0];
	switch_val = ~switch_val; /* flip the values */
	if(switch_val == 0){ /*if we have full brightness, go ahead and skip our multiplication step*/
		lum_val = 0;
		goto writeval;
	}
	switch_val = switch_val >> 5; /* shorthand divide by 32 */
	lum_val = switch_val * 0x7; /* multiply by 7 to get proper value */
	if(lum_val > 28) /* The brightness register follows multiples of 7 up until 35 (becomes 36 at this point) */
		lum_val++; /*increment to match formula */

	writeval:
	write_reg(par, GP9002_BRIGHT, lum_val); /* write the new value */

	return 0;
}


static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.txbuflen = TXBUFLEN,
	.gamma_num = 1,
	.gamma_len = 1,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.write_register = write_reg8_bus8,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.write_vmem = write_vmem,
		.set_gamma = set_gamma,
	},
	.backlight = 1,
};
FBTFT_REGISTER_DRIVER(DRVNAME, "futaba,gp9002a", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("spi:gp9002a");

MODULE_DESCRIPTION("FB driver for the Futaba GP9002A VFD display");
MODULE_AUTHOR("Alex O'Neill");
MODULE_LICENSE("GPL");
