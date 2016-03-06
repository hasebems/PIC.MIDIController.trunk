/*
 * File:   touchmidi.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 2:46 PM
 */


#include <xc.h>
#include "hsb_common.h"
#include "config.h"
#include "i2cdevice.h"

#ifdef TOUCH_MIDI
/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static bool			isFinishedWriting;
static unsigned char swState[2];

/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void TouchMIDI_init(void)
{
	isFinishedWriting = false;
	swState[0] = swState[1] = 0;
	OUT = 1;
}

/*----------------------------------------------------------------------------*/
//
//      Write CY8CMBR3110 Config Data
//
/*----------------------------------------------------------------------------*/
const unsigned char tCY8CMBR3110_ConfigData[128] =
{
	0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,	//	0x00
	0xff,0xff,0x0f,0x00,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,	//	0x10
	0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,	//	0x20
	0x05,0x00,0x00,0x02,0x00,0x02,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x1E,0x1E,0x00,	//	0x30
	0x00,0x1E,0x1E,0x00,0x00,0x00,0x01,0x01,
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,	//	0x40
	0x00,0x00,0x00,0x00,0x11,0x02,0x01,0x58,
	0x00,0x37,0x01,0x00,0x00,0x0A,0x00,0x00,	//	0x50
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	//	0x60
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	//	0x70
	0x00,0x00,0x00,0x00,0x00,0x00,0xde,0x66
};
/*----------------------------------------------------------------------------*/
void writeConfig( void )
{
#if USE_I2C_CY8CMBR3110
	int err;

	if (( event100msec == true ) && ( isFinishedWriting == true )){

		err = MBR3110_checkDevice();
		if ( err ){
			setMidiBuffer( 0xb0, 0x10, err & 0x7f );
		}
	}

	/* If a button is pressed... */
    if (( IN4 == 0 ) && ( isFinishedWriting == false )){
		err = MBR3110_writeConfig((unsigned char*)tCY8CMBR3110_ConfigData);
		if ( err ){
			setMidiBuffer( 0xb0, 0x10, err & 0x7f );
		}
		else {
			isFinishedWriting = true;
		}
    }
#endif
}

/*----------------------------------------------------------------------------*/
//
//     Check Touch Sensor & Generate MIDI Event
//
/*----------------------------------------------------------------------------*/
void checkTouch( void )
{
#if USE_I2C_CY8CMBR3110
	unsigned char sw[2];
	int			err;

	err = MBR3110_readTouchSw(sw);
	if ( err ){
		setMidiBuffer( 0xb0, 0x11, err & 0x7f );
	}
	else {
		if ( (swState[0]&0x01)^(sw[0]&0x01) ){
			if ( sw[0] & 0x01 ){ setMidiBuffer(0x90,0x3c,0x7f);}
			else {	setMidiBuffer( 0x90,0x3c,0x00 );}
		}
		if ( (swState[0]&0x02)^(sw[0]&0x02) ){
			if ( sw[0] & 0x02 ){ setMidiBuffer(0x90,0x3e,0x7f);}
			else {	setMidiBuffer( 0x90,0x3e,0x00 );}
		}
		if ( (swState[0]&0x04)^(sw[0]&0x04) ){
			if ( sw[0] & 0x04 ){ setMidiBuffer(0x90,0x40,0x7f);}
			else {	setMidiBuffer( 0x90,0x40,0x00 );}
		}
		if ( (swState[0]&0x08)^(sw[0]&0x08) ){
			if ( sw[0] & 0x08 ){ setMidiBuffer(0x90,0x41,0x7f);}
			else {	setMidiBuffer( 0x90,0x41,0x00 );}
		}
		if ( (swState[0]&0x10)^(sw[0]&0x10) ){
			if ( sw[0] & 0x10 ){ setMidiBuffer(0x90,0x43,0x7f);}
			else {	setMidiBuffer( 0x90,0x43,0x00 );}
		}
		if ( (swState[0]&0x20)^(sw[0]&0x20) ){
			if ( sw[0] & 0x20 ){ setMidiBuffer(0x90,0x45,0x7f);}
			else {	setMidiBuffer( 0x90,0x45,0x00 );}
		}

		if ( swState[1]^sw[1] ){ setMidiBuffer( 0xb0, 0x12, sw[1]&0x7f );}
		swState[0] = sw[0];
		swState[1] = sw[1];
	}
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void TouchMIDI_appli(void)
{
	writeConfig();
	checkTouch();

	//	Heartbeat
	if ( isFinishedWriting == true ){
		OUT = ((counter10msec & 0x001e) == 0x0000)? 0:1;		//	350msec
	}
}
#endif
