//For compiling custom optiboot
//make atmega328 BIGBOOT SUPPORT_EEPROM BAUD_RATE=9600L LED_START_FLASHES=4 LED_DATA_FLASH OPTIBOOT_CUSTOMVER=1 AVR_FREQ=8000000L LED=C5 SOFT_UART
//make atmega328 BIGBOOT SUPPORT_EEPROM BAUD_RATE=19200L LED_START_FLASHES=2 LED_DATA_FLASH OPTIBOOT_CUSTOMVER=1 AVR_FREQ=8000000L LED=C5 SOFT_UART
//make atmega328 BIGBOOT SUPPORT_EEPROM BAUD_RATE=19200L LED_START_FLASHES=2 LED_DATA_FLASH OPTIBOOT_CUSTOMVER=1 AVR_FREQ=8000000L LED=C5

#define FUNC_READ 1
#define FUNC_WRITE 1
/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* http://optiboot.googlecode.com                         */
/*                                                        */
/* Arduino-maintained version : See README.TXT            */
/* http://code.google.com/p/arduino/                      */
/*  It is the intent that changes not relevant to the     */
/*  Arduino production envionment get moved from the      */
/*  optiboot project to the arduino project in "lumps."   */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Fully supported:                                       */
/*   ATmega168 based devices  (Diecimila etc)             */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Beta test (believed working.)                          */
/*   ATmega8 based devices (Arduino legacy)               */
/*   ATmega328 non-picopower devices                      */
/*   ATmega644P based devices (Sanguino)                  */
/*   ATmega1284P based devices                            */
/*   ATmega1280 based devices (Arduino Mega)              */
/*                                                        */
/* Alpha test                                             */
/*   ATmega32                                             */
/*                                                        */
/* Work in progress:                                      */
/*   ATtiny84 based devices (Luminet)                     */
/*                                                        */
/* Does not support:                                      */
/*   USB based devices (eg. Teensy, Leonardo)             */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */

/* Copyright 2013-2015 by Bill Westfield.                 */
/* Copyright 2010 by Peter Knight.                        */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */

/**********************************************************/
/*                                                        */
/* BIGBOOT:                                              */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate.                              */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/* UART:                                                  */
/* UART number (0..n) for devices with more than          */
/* one hardware uart (644P, 1284P, etc)                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/*  the optiboot source tree changes since v3.            */
/* Version 5 was created at the time of the new Makefile  */
/*  structure (Mar, 2013), even though no binaries changed*/
/* It would be good if versions implemented outside the   */
/*  official repository used an out-of-seqeunce version   */
/*  number (like 104.6 if based on based on 4.5) to       */
/*  prevent collisions.                                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:					  */
/*							  */
/* Aug 2014						  */
/* 6.2 WestfW: make size of length variables dependent    */
/*              on the SPM_PAGESIZE.  This saves space    */
/*              on the chips where it's most important.   */
/* 6.1 WestfW: Fix OPTIBOOT_CUSTOMVER (send it!)	  */
/*             Make no-wait mod less picky about	  */
/*               skipping the bootloader.		  */
/*             Remove some dead code			  */
/* Jun 2014						  */
/* 6.0 WestfW: Modularize memory read/write functions	  */
/*             Remove serial/flash overlap		  */
/*              (and all references to NRWWSTART/etc)	  */
/*             Correctly handle pagesize > 255bytes       */
/*             Add EEPROM support in BIGBOOT (1284)       */
/*             EEPROM write on small chips now causes err */
/*             Split Makefile into smaller pieces         */
/*             Add Wicked devices Wildfire		  */
/*	       Move UART=n conditionals into pin_defs.h   */
/*	       Remove LUDICOUS_SPEED option		  */
/*	       Replace inline assembler for .version      */
/*              and add OPTIBOOT_CUSTOMVER for user code  */
/*             Fix LED value for Bobuino (Makefile)       */
/*             Make all functions explicitly inline or    */
/*              noinline, so we fit when using gcc4.8     */
/*             Change optimization options for gcc4.8	  */
/*             Make ENV=arduino work in 1.5.x trees.	  */
/* May 2014                                               */
/* 5.0 WestfW: Add support for 1Mbps UART                 */
/* Mar 2013                                               */
/* 5.0 WestfW: Major Makefile restructuring.              */
/*             See Makefile and pin_defs.h                */
/*             (no binary changes)                        */
/*                                                        */
/* 4.6 WestfW/Pito: Add ATmega32 support                  */
/* 4.6 WestfW/radoni: Don't set LED_PIN as an output if   */
/*                    not used. (LED_START_FLASHES = 0)   */
/* Jan 2013						  */
/* 4.6 WestfW/dkinzer: use autoincrement lpm for read     */
/* 4.6 WestfW/dkinzer: pass reset cause to app in R2      */
/* Mar 2012                                               */
/* 4.5 WestfW: add infrastructure for non-zero UARTS.     */
/* 4.5 WestfW: fix SIGNATURE_2 for m644 (bad in avr-libc) */
/* Jan 2012:                                              */
/* 4.5 WestfW: fix NRWW value for m1284.                  */
/* 4.4 WestfW: use attribute OS_main instead of naked for */
/*             main().  This allows optimizations that we */
/*             count on, which are prohibited in naked    */
/*             functions due to PR42240.  (keeps us less  */
/*             than 512 bytes when compiler is gcc4.5     */
/*             (code from 4.3.2 remains the same.)        */
/* 4.4 WestfW and Maniacbug:  Add m1284 support.  This    */
/*             does not change the 328 binary, so the     */
/*             version number didn't change either. (?)   */
/* June 2011:                                             */
/* 4.4 WestfW: remove automatic soft_uart detect (didn't  */
/*             know what it was doing or why.)  Added a   */
/*             check of the calculated BRG value instead. */
/*             Version stays 4.4; existing binaries are   */
/*             not changed.                               */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.		  */
/**********************************************************/

#define OPTIBOOT_MAJVER 6
#define OPTIBOOT_MINVER 2

// #define BIGBOOT
// #define SOFT_UART
// #define SUPPORT_EEPROM
// #define VIRTUAL_BOOT_PARTITION
// #define BAUD_RATE 115200L
#define LED_DATA_FLASH
// #define LED_START_FLASHES 1
// #define OPTIBOOT_CUSTOMVER 1
/*
 * OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
 * of optiboot.  That way you don't wind up with very different code that
 * matches the version number of a "released" optiboot.
 */

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__((section(".version"))) 
optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
// #include "SoftwareSerial.h"
/*
 * Note that we use our own version of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory.  Sorry for the name overlap.
 */
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#if BAUD_ACTUAL <= BAUD_RATE
  #define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)
  #if BAUD_ERROR >= 5
    #error BAUD_RATE error greater than -5%
  #elif BAUD_ERROR >= 2
    #warning BAUD_RATE error greater than -2%
  #endif
#else
  #define BAUD_ERROR (( 100*(BAUD_ACTUAL - BAUD_RATE) ) / BAUD_RATE)
  #if BAUD_ERROR >= 5
    #error BAUD_RATE error greater than 5%
  #elif BAUD_ERROR >= 2
    #warning BAUD_RATE error greater than 2%
  #endif
#endif

#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE 
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE 
#endif
#endif // baud rate fastn check

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif


/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */

// #define O_VERIFY 0x00
// #define O_WRITE 0x01

#define STR_PAGESIZE 225

#define RSP_OK 0x00
#define RSP_CHECKSUM_FAIL 0x01
#define RSP_INSUFFICIENT 0x02
#define RSP_FINISHED 0x03

uint8_t dataCnt=0;
uint8_t tempPtr;
uint16_t length;
uint16_t address;
uint16_t calcAddress;
uint32_t bRead;


uint8_t buff[STR_PAGESIZE];
uint8_t databuf[SPM_PAGESIZE];

static uint8_t HEX2DEC(uint8_t);
static uint8_t getHexByte();
static inline uint8_t getDataLen(uint8_t ptr);
static uint8_t checkRecord(uint8_t write);
void discardUpdateRequest(uint8_t rvalue);
static uint8_t DownloadHexFile(uint32_t, uint8_t);
static inline void printDebug();
static inline void printCheckSumError(void);

#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
// #define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#endif


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

void __attribute__((noinline)) putch(char);
uint8_t __attribute__((noinline)) getch(void);
void __attribute__((noinline)) watchdogConfig(uint8_t x);

static inline void getNch(uint8_t);
#if LED_START_FLASHES > 0
static inline void flash_led(uint8_t);
#endif


static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
			       uint16_t address, pagelen_t len);
// static inline void read_mem(uint8_t memtype,
			    // uint16_t address, pagelen_t len);


// static inline uint8_t getLength();
static inline void convertIntToChar(uint32_t d);
static void requestPage(uint8_t,uint32_t);
static void cleareeprom(uint16_t);
// static inline void printEEPROMContents();

#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart(uint8_t rstFlags) __attribute__ ((naked));
/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
// #undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
// #define buff    ((uint8_t*)(RAMSTART))

#define appstart_vec (0)

#define GETLENGTH(x) (((prgSize-bRead)/STR_PAGESIZE>0)?(STR_PAGESIZE):(prgSize-bRead))

/* main program starts here */
int main(void) {

  // register uint16_t address = 0;

  uint8_t ch;
  uint16_t eadd;
  uint32_t prgSize;

  watchdogConfig(WATCHDOG_8S);

  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   */

  putch('\r'); 
  putch('\n');

  // After the zero init loop, this is the first code to run.
  //
  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega16__)
  SP=RAMEND;  // This is done by hardware reset
#endif

  /*
   * modified Adaboot no-wait mod.
   * Pass the reset reason to app.  Also, it appears that an Uno poweron
   * can leave multiple reset flags set; we only want the bootloader to
   * run on an 'external reset only' status
   */
  ch = MCUSR;
  MCUSR = 0;
  eadd=900;
  //prg update request not set... do not enter into bootloader
  if((eeprom_read_byte((uint8_t *)(eadd)))!=0x01)
  {
    appStart(ch);
    // if (ch & (_BV(WDRF) | _BV(BORF) | _BV(PORF) | _BV(EXTRF)))
      // appStart(0);
  }
  putch('\r'); 
  putch('\n');
  putch('\r');
  putch('\n');

#if LED_START_FLASHES > 0
  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
#endif

#ifndef SOFT_UART
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega16__)
  UCSRA = _BV(U2X); //Double speed mode USART
  UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
  UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#else
  UART_SRA = _BV(U2X0); //Double speed mode USART0
  UART_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
  UART_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif
#endif


  // Set up watchdog to trigger after 1s
  watchdogConfig(WATCHDOG_8S);

#if (LED_START_FLASHES > 0) || defined(LED_DATA_FLASH)
  /* Set LED pin as output */
  LED_DDR |= _BV(LED);
#endif
  putch('\r'); 
  putch('\n');

#ifdef SOFT_UART
  /* Set TX pin as output */
  UART_DDR |= _BV(UART_TX_BIT);
#endif

#if LED_START_FLASHES > 0
  /* Flash onboard LED to signal entering of bootloader */
  // flash_led(LED_START_FLASHES * 2);
#endif
  eadd=904;
  prgSize=eeprom_read_dword((uint32_t *)(eadd));
  // prgSize =22876L;
  // bRead=0;

  // uint8_t desttype;

  /* Forever loop: exits by causing WDT reset */
    /* Write memory, length is big endian and is in bytes */
  eadd=912;
  if(eeprom_read_byte((uint8_t*)eadd)!=0x01)
  {
    if(DownloadHexFile(prgSize,0x00))
    {
      putch('V');
      putch('D');
      putch('\r');
      putch('\n');
      eeprom_write_byte((uint8_t*)eadd,0x01);
    }
    else        //remove update request and set the reason for same..
      discardUpdateRequest(0x0A);       //cannot verify hex file

    watchdogConfig(WATCHDOG_250MS);
    for(;;)
    {}
  }
  else        //hex file is verified.....
  {
    if(DownloadHexFile(prgSize,0x01))
    {
      watchdogReset();
      watchdogConfig(WATCHDOG_OFF);    // shorten WD timeout
      cleareeprom(1023);   //clear contents of EEPROM
      discardUpdateRequest(0x01);       //set successfully updated
      flash_led(10);
      appStart(ch);
    }
  }
    return 0;
}

void discardUpdateRequest(uint8_t rvalue)
{
  uint16_t add;
  add=908;
  eeprom_write_byte((uint8_t*)add,rvalue);
  add=900;
  eeprom_write_byte((uint8_t*)add,0x00);
}

static uint8_t DownloadHexFile(uint32_t prgSize, uint8_t op)
{
      // uint8_t dataCnt;

      flash_led(6);
      address=calcAddress=0;
      pagelen_t savelength;
      bRead=0;
    while(bRead<prgSize) {

      watchdogReset();
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      savelength=GETLENGTH(savelength);
      requestPage(savelength,bRead);
      
      getNch(2);
      length=0;
      while(length<savelength)
        buff[length++]=getch();
      getNch(6);
      
      length=tempPtr=0;
      uint8_t resp=RSP_INSUFFICIENT;
      while((resp=checkRecord(op))==RSP_OK)
      {
            tempPtr+=2;
            length+=2;
      }
      if(resp==RSP_CHECKSUM_FAIL)
      {
          uint16_t eadd=908;
          eeprom_write_byte((uint8_t *)(eadd), 0x02);
          printCheckSumError();
          // printDebug();
          return 0x00;
      }
      else if(resp==RSP_FINISHED)
            length+=2;

        // printDebug();
        bRead+=length;
        // printDebug();

    }
    return 0x01;
}

void printDebug()
{
      convertIntToChar(address);
      putch('\n');
      convertIntToChar(calcAddress);
      putch('\n');
}

void printCheckSumError()
{
  putch('C');
  putch('K');
  putch('E');
}
    // watchdogConfig(WATCHDOG_250MS);
        // while (1)           // and busy-loop so that WD causes
      // ;             //  a reset and app start.

    // appStart(ch);

 	// watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
    // while (1)			      // and busy-loop so that WD causes
      // ;		
    
    /* Read memory block mode, length is big endian.  */
    // else if(ch == STK_READ_PAGE) {
    //   uint8_t desttype;
    //   GETLENGTH(length);

    //   desttype = getch();

    //   verifySpace();

    //   read_mem(desttype, address, length);
    // }
  // }

// #define getHexByte(x) length+=2; x=((HEX2DEC(buff[tempPtr++])<<4) + HEX2DEC(buff[tempPtr++]))
// #define HEX2DEC(x)  (((x < 'A') ? ((x) - 48) : ((x) - 55)))

static uint8_t HEX2DEC(uint8_t x)
{
  return (((x < 'A') ? ((x) - 48) : ((x) - 55)));
}

static uint8_t getHexByte()
{
  length+=2;
  return ((HEX2DEC(buff[tempPtr++])<<4) + HEX2DEC(buff[tempPtr++]));
}

static inline uint8_t getDataLen(uint8_t ptr)   
{
  //do not pass referenece or pointer to this funtion...  
  return ((HEX2DEC(buff[ptr++])<<4) + HEX2DEC(buff[ptr++]));
}

// inline void printNewLine()
// { putch('\n');}

static void initWrite()
{
        writebuffer('F', databuf, address, dataCnt);
        dataCnt=0;  
}

static uint8_t checkRecord(uint8_t writeRecord)
{
  if((STR_PAGESIZE - length)<((getDataLen(tempPtr+1)*2)+11))  //invalid record not having necessary data
    return RSP_INSUFFICIENT;

  // putch(databuf[tempPtr]);
  tempPtr++;    //discard ":" from record
  length++;

  uint8_t data_len=getHexByte();
  uint8_t len=data_len;
  // convertIntToChar(data_len);

  uint8_t addrh = getHexByte();
  uint8_t addrl = getHexByte();

  uint16_t curAddress=addrh;
  curAddress = curAddress << 8;
  curAddress += addrl;

  uint8_t recType=getHexByte();
  if(calcAddress!=curAddress && recType==0)
  {
    if(curAddress-address>=SPM_PAGESIZE)
    {
      if(dataCnt>0)
        initWrite();
      address=calcAddress=curAddress;
    }
    else
    {
      while(calcAddress!=curAddress)
      {
        databuf[dataCnt++]=0xFF;
        calcAddress++;
      }
    }
  }
  uint8_t checkSum=data_len + addrh + addrl + recType;
  
  while(len--)
  {
    databuf[dataCnt]=getHexByte();
    checkSum+=databuf[dataCnt++];
    calcAddress++;
  }

  checkSum=(~checkSum) + 1;
  uint8_t ck=getHexByte();

  if(ck!=checkSum)
    return RSP_CHECKSUM_FAIL;     //incorrect record

  if(writeRecord)
  {
      // printDebug();
      if(dataCnt==SPM_PAGESIZE || (recType==1 && dataCnt>0))
      {
        initWrite();
        address=calcAddress;
      }
  }
  if(recType==1)
    return RSP_FINISHED;
  return RSP_OK;      //correct record and written
}

void putch(char ch) {
#ifndef SOFT_UART
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega16__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

#ifdef SOFT_UART
    watchdogReset();
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
#else
  while(!(UART_SRA & _BV(RXC0)))
    ;
  if (!(UART_SRA & _BV(FE0))) {
    watchdogReset();
  }

  ch = UART_UDR;
#endif

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega16__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  // verifySpace();
}

#if LED_START_FLASHES > 0

void flash_led(uint8_t count) {
  do {
    TCNT1 = -(F_CPU/(1024*16));
    TIFR1 = _BV(TOV1);
    while(!(TIFR1 & _BV(TOV1)));
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega16__)
    LED_PORT ^= _BV(LED);
#else
    LED_PIN |= _BV(LED);
#endif
    watchdogReset();
  } while (--count);
}

#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}

void appStart(uint8_t rstFlags) {
  // save the reset flags in the designated register
  //  This can be saved in a main program by putting code in .init0 (which
  //  executes before normal c init code) to save R2 to a global variable.
  __asm__ __volatile__ ("mov r2, %0\n" :: "r" (rstFlags));

  // watchdogConfig(WATCHDOG_OFF);
  watchdogReset();
  // Note that appstart_vec is defined so that this works with either
  // real or virtual boot partitions.
  __asm__ __volatile__ (
    // Jump to 'save' or RST vector
    "ldi r30,%[rstvec]\n"
    "clr r31\n"
    "ijmp\n"::[rstvec] "M"(appstart_vec)
  );
}

static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
			       uint16_t address, pagelen_t len)
{
    switch (memtype) {
    case 'E': // EEPROM
#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
        while(len--) {
	    eeprom_write_byte((uint8_t *)(address++), *mybuff++);
        }
#else
	/*
	 * On systems where EEPROM write is not supported, just busy-loop
	 * until the WDT expires, which will eventually cause an error on
	 * host system (which is what it should do.)
	 */
	while (1)
	    ;
#endif
	break;
    default:  // FLASH
	/*
	 * Default to writing to Flash program memory.  By making this
	 * the default rather than checking for the correct code, we save
	 * space on chips that don't support any other memory types.
	 */
	{
	    // Copy buffer into programming buffer
	    uint8_t *bufPtr = mybuff;
	    uint16_t addrPtr = (uint16_t)(void*)address;

	    /*
	     * Start the page erase and wait for it to finish.  There
	     * used to be code to do this while receiving the data over
	     * the serial link, but the performance improvement was slight,
	     * and we needed the space back.
	     */
	    __boot_page_erase_short((uint16_t)(void*)address);
	    boot_spm_busy_wait();

	    /*
	     * Copy data from the buffer into the flash write buffer.
	     */
	    do {
		uint16_t a;
		a = *bufPtr++;
		a |= (*bufPtr++) << 8;
		__boot_page_fill_short((uint16_t)(void*)addrPtr,a);
		addrPtr += 2;
	    } while (len -= 2);

	    /*
	     * Actually Write the buffer to flash (and wait for it to finish.)
	     */
	    __boot_page_write_short((uint16_t)(void*)address);
	    boot_spm_busy_wait();
#if defined(RWWSRE)
	    // Reenable read access to flash
	    boot_rww_enable();
#endif
	} // default block
	break;
    } // switch
}

// static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t length)
// {
//     uint8_t ch;

//     switch (memtype) {

// #if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
//     case 'E': // EEPROM
// 	do {
// 	    putch(eeprom_read_byte((uint8_t *)(address++)));
// 	} while (--length);
// 	break;
// #endif
//     default:
// 	do {
// // #ifdef VIRTUAL_BOOT_PARTITION
// //         // Undo vector patch in bottom page so verify passes
// // 	    if (address == rstVect0) ch = rstVect0_sav;
// // 	    else if (address == rstVect1) ch = rstVect1_sav;
// // 	    else if (address == saveVect0) ch = saveVect0_sav;
// // 	    else if (address == saveVect1) ch = saveVect1_sav;
// // 	    else ch = pgm_read_byte_near(address);
// // 	    address++;
// // #elif defined(RAMPZ)
// // 	    // Since RAMPZ should already be set, we need to use EPLM directly.
// // 	    // Also, we can use the autoincrement version of lpm to update "address"
// // 	    //      do putch(pgm_read_byte_near(address++));
// // 	    //      while (--length);
// // 	    // read a Flash and increment the address (may increment RAMPZ)
// // 	    __asm__ ("elpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
// // #else
// // 	    // read a Flash byte and increment the address
// // 	    __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
// // #endif
// 	    putch(ch);
// 	} while (--length);
// 	break;
//     } // switch
// }


void convertIntToChar(uint32_t d)
{

  uint8_t t[6];
  uint8_t temp=0;
    if(d==0)
    {
      putch('0');
      return;
    }

    // uint8_t *bufPtr = buff;
    while(d>0)
    {
      // *bufPtr++=(d%10)+0x30;
        t[temp++]=(d%10)+0x30;
        d=d/10;
    }
     while(temp--)
        putch(t[temp]);
}


void requestPage(uint8_t plen,uint32_t pos)
{
    putch('A');
    putch('T');
    putch('+');
    putch('F');
    putch('S');
    putch('R');
    putch('E');
    putch('A');
    putch('D');
    putch('=');
    putch('C');
    putch(':');
    putch('\\');
    putch('U');
    putch('s');
    putch('e');
    putch('r');
    putch('\\');
    putch('F');
    putch('T');
    putch('P');
    putch('\\');
    putch('m');
    putch('.');
    putch('b');
    putch('i');
    putch('n');
    putch(',');
    putch('1');
    putch(',');
    
    // issueRead("AT+FSREAD=C:\\User\\FTP\\m.bin,1,",30);
    convertIntToChar(plen);
    putch(',');
    convertIntToChar(pos);
    putch('\r');
    putch('\n');
    // issueRead("\r\n",2);
}

void cleareeprom(uint16_t len)
{
      while(len--) {
      eeprom_write_byte((uint8_t *)(len), 0xFF);
      }
}
// static inline uint8_t getLength(prgSize)
// {
//   // if(((prgSize-b)/SPM_PAGESIZE>0))
//     // return SPM_PAGESIZE;
//   if(((prgSize-bRead)/STR_PAGESIZE>0))
//     return STR_PAGESIZE;
//   else
//     return (prgSize-bRead);
// }