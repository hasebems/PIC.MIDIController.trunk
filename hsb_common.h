/*
 * File:   hsb_common.h
 * Author: jca03205
 *
 * Created on 2014/12/23, 9:42
 */

#ifndef HSB_COMMON_H
#define	HSB_COMMON_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

//  define function prototype
typedef void (*InitFunc)(void);
typedef void (*AppliFunc)(void);
typedef void (*InterruptFunc)(void);
const InitFunc initFunc[];
const AppliFunc appFunc[];
const InterruptFunc intFunc[];

//  Send MIDI
extern void setMidiBuffer( uint8_t status, uint8_t dt1, uint8_t dt2 );
//	Rcv MIDI by USART
extern uint8_t getRecievedMIDI( void );

//  Counter
extern long			counter10msec;	//	one loop 243 days
extern bool			event5msec;
extern bool			event10msec;
extern bool			event100msec;

extern uint8_t		pwm16msInterval;
extern int			globalCount;
extern int			i2cComErr;

//  GPIO
#define	IN4			PORTCbits.RC7
#define	IN3			PORTCbits.RC6
#define	IN2			PORTCbits.RC5
#define	IN1			PORTCbits.RC4
#define OUT		    PORTCbits.RC3
#define LEDR        PORTCbits.RC2
#define LEDG        PORTCbits.RC1
#define LEDB        PORTCbits.RC0

#define OUT4	    PORTCbits.RC3
#define OUT3        PORTCbits.RC2
#define OUT2        PORTCbits.RC1
#define OUT1        PORTCbits.RC0

#endif	/* HSB_COMMON_H */
