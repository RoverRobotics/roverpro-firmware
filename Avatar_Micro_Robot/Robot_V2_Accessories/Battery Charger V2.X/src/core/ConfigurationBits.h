/*=============================================================================
File: ConfigurationBits.h

Description:  This file encapsulates the configuration bits set for this 
  project, ensuring they are consistent across modules.

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#ifndef CONFIG_BITS_H
#define CONFIG_BITS_H

/*---------------------------Dependencies------------------------------------*/
#include <p24FJ256GB106.h>

/*
 	disable the JTAG port
	disable code protection
	enable writes to program memory
	diable Clip On Emulation Mode
  disable the Watchdog Timer
	//configure the Watchdog postscaler to divide-by-256, 1:256
	  T_WDT (ms) = prescaler_factor * postscaler_factor
	             FWPSA = 1 by default = Prescaler ratio of 1:128
	             LPRC = 31kHz
	             = T_LPRC * 128 * 256 ~= 1.06s TODO: confirm this
  
  share emulator functions with PGEC1/PGED1
*/
_CONFIG1(JTAGEN_OFF & GCP_ON & GWRP_OFF & COE_OFF & FWDTEN_ON & ICS_PGx1
         & WDTPS_PS2048)

  //WDTPS_2048 - about 10s (not verified)

/*
	disable two-speed start-up for Internal External Switch Over Mode
	disable both Clock Switching and Fail-safe Clock Monitor
	configure primary OSCillator Output functions as port I/O (RC15)
	select the HS oscillator mode
	enable the Phased-Lock Loop module for the primary oscillator
	Oscillator input divided by 5 (20MHz input)
  Unlimited Writes To RP Registers
*/
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS & FNOSC_PRIPLL & 
         PLLDIV_DIV5 & IOL1WAY_OFF)

#endif
