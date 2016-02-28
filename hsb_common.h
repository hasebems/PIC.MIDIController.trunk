/*
 * File:   hsb_common.h
 * Author: jca03205
 *
 * Created on 2014/12/23, 9:42
 */

#ifndef HSB_COMMON_H
#define	HSB_COMMON_H

#ifdef	__cplusplus
extern "C" {
#endif

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

//  Counter
extern long			counter10msec;	//	one loop 243 days
extern bool			event5msec;
extern bool			event10msec;
extern bool			event100msec;

extern uint8_t		midiExp;    //  for Full Color LED
extern int			doremi;     //  for Full Color LED

//  GPIO
#define	IN4			PORTCbits.RC7
#define	IN3			PORTCbits.RC6
#define	IN2			PORTCbits.RC5
#define	IN1			PORTCbits.RC4
#define OUT		    PORTCbits.RC3
#define LEDR        PORTCbits.RC2
#define LEDG        PORTCbits.RC1
#define LEDB        PORTCbits.RC0


#ifdef	__cplusplus
}
#endif

#endif	/* HSB_COMMON_H */
