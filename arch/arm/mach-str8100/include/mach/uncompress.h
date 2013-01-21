/*******************************************************************************
 *
 *  Copyright (c) 2008 Cavium Networks 
 * 
 *  This file is free software; you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License, Version 2, as 
 *  published by the Free Software Foundation. 
 *
 *  This file is distributed in the hope that it will be useful, 
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or 
 *  NONINFRINGEMENT.  See the GNU General Public License for more details. 
 *
 *  You should have received a copy of the GNU General Public License 
 *  along with this file; if not, write to the Free Software 
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or 
 *  visit http://www.gnu.org/licenses/. 
 *
 *  This file may also be available under a different license from Cavium. 
 *  Contact Cavium Networks for more information
 *
 ******************************************************************************/

#ifndef __ASM_ARCH_UNCOMPRESS_H__
#define __ASM_ARCH_UNCOMPRESS_H__

#include <mach/star_uart.h>

#define flush(x)	


unsigned long __stack_chk_guard;

void __stack_chk_fail(void)
{
   
}

#if !defined(CONFIG_MACH_TS75XX)
static void star_putstr(const char *s)
{
	while (*s) {
		volatile unsigned int status = 0;

		do {
			status = __UART_LSR(0);
		} while (!((status & THR_EMPTY) == THR_EMPTY));

		__UART_THR(0) = *s;

		if (*s == '\n') {
			do {
				status = __UART_LSR(0);
			} while (!((status & THR_EMPTY) == THR_EMPTY));
			__UART_THR(0) = '\r';
		}
		s++;
	}
}
#endif

#if !defined(CONFIG_MACH_TS75XX)
static const char * const digits="0123456789ABCDEF";
static void ser_puts_hex8(unsigned char hex)
{ 
   char buf[3];
   buf[0] = digits[(hex >> 4) & 0xF];
   buf[1] = digits[hex & 0xF];
      
   buf[2] = '\0';
   putstr(buf);
}

void ser_puts_hex32(unsigned long hex)
{
   char buf[9];
   int i;
   
   for(i=7; i >= 0; i--)
   {
      buf[7-i] = digits[(hex >> (i * 4)) & 0xF];
   }
   
   buf[8] = '\0';
  putstr(buf);
}


static void putc(int c)
{
 star_putstr((const char *)&c);
}
#endif

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif
