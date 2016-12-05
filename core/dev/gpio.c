/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * @(#)$Id: gpio.c,v 1.7 2009/02/24 21:30:20 adamdunkels Exp $
 */


#include "dev/gpio.h"
//#include "sys/clock.h"
#include "sys/energest.h"
#include "contiki-conf.h"
#include <legacymsp430.h>

/*---------------------------------------------------------------------------*/
static unsigned char gpio, invert;

/*---------------------------------------------------------------------------*/
void
gpio_arch_init(void)
{
  GPIO_PxDIR |= (GPIO_CONF_RED | GPIO_CONF_GREEN | GPIO_CONF_YELLOW);
  GPIO_PxOUT |= (GPIO_CONF_RED | GPIO_CONF_GREEN | GPIO_CONF_YELLOW);
}
/*---------------------------------------------------------------------------*/
unsigned char
gpio_arch_get(void)
{
  return ((GPIO_PxOUT & GPIO_CONF_RED) ? 0 : GPIO_RED)
    | ((GPIO_PxOUT & GPIO_CONF_GREEN) ? 0 : GPIO_GREEN)
    | ((GPIO_PxOUT & GPIO_CONF_YELLOW) ? 0 : GPIO_YELLOW);
}
/*---------------------------------------------------------------------------*/
void
gpio_arch_set(unsigned char gpio)
{
  GPIO_PxOUT = (GPIO_PxOUT & ~(GPIO_CONF_RED|GPIO_CONF_GREEN|GPIO_CONF_YELLOW))
    | ((gpio & GPIO_RED) ? 0 : GPIO_CONF_RED)
    | ((gpio & GPIO_GREEN) ? 0 : GPIO_CONF_GREEN)
    | ((gpio & GPIO_YELLOW) ? 0 : GPIO_CONF_YELLOW);
}

/*---------------------------------------------------------------------------*/
static void
show_gpio(unsigned char changed)
{
  if(changed & GPIO_GREEN) {
    /* Green did change */
    if((invert ^ gpio) & GPIO_GREEN) {
      ENERGEST_ON(ENERGEST_TYPE_LED_GREEN);
    } else {
      ENERGEST_OFF(ENERGEST_TYPE_LED_GREEN);
    }
  }
  if(changed & GPIO_YELLOW) {
    if((invert ^ gpio) & GPIO_YELLOW) {
      ENERGEST_ON(ENERGEST_TYPE_LED_YELLOW);
    } else {
      ENERGEST_OFF(ENERGEST_TYPE_LED_YELLOW);
    }
  }
  if(changed & GPIO_RED) {
    if((invert ^ gpio) & GPIO_RED) {
      ENERGEST_ON(ENERGEST_TYPE_LED_RED);
    } else {
      ENERGEST_OFF(ENERGEST_TYPE_LED_RED);
    }
  }
  gpio_arch_set(gpio ^ invert);
}
/*---------------------------------------------------------------------------*/
void
gpio_init(void)
{
  gpio_arch_init();
  gpio = invert = 0;
}
/*---------------------------------------------------------------------------*/
void
gpio_blink(void)
{
  /* Blink all gpio. */
  unsigned char inv;
  inv = ~(gpio ^ invert);
  gpio_invert(inv);

  //clock_delay(400);

  gpio_invert(inv);
}
/*---------------------------------------------------------------------------*/
unsigned char
gpio_get(void) {
  return gpio_arch_get();
}
/*---------------------------------------------------------------------------*/
void
gpio_on(unsigned char ledv)
{
  unsigned char changed;
  changed = (~gpio) & ledv;
  gpio |= ledv;
  show_gpio(changed);
}
/*---------------------------------------------------------------------------*/
void
gpio_off(unsigned char ledv)
{
  unsigned char changed;
  changed = gpio & ledv;
  gpio &= ~ledv;
  show_gpio(changed);
}
/*---------------------------------------------------------------------------*/
void
gpio_toggle(unsigned char ledv)
{
  gpio_invert(ledv);
}
/*---------------------------------------------------------------------------*/
/*   invert the invert register using the gpio parameter */
void
gpio_invert(unsigned char ledv) {
  invert = invert ^ ledv;
  show_gpio(ledv);
}
/*---------------------------------------------------------------------------*/
void gpio_green(int o) { o?gpio_on(GPIO_GREEN):gpio_off(GPIO_GREEN); }
void gpio_yellow(int o) { o?gpio_on(GPIO_YELLOW):gpio_off(GPIO_YELLOW); }
void gpio_red(int o) { o?gpio_on(GPIO_RED):gpio_off(GPIO_RED); }
/*---------------------------------------------------------------------------*/
