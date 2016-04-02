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
static unsigned char swState[2];

/*----------------------------------------------------------------------------*/
//
//      Write CY8CMBR3110 Config Data
//
/*----------------------------------------------------------------------------*/
const unsigned char tCY8CMBR3110_ConfigData[128] =
{
	//	0.4pF, Auto Reset disable
	0xFF,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
	0xFF,0xFF,0x0F,0x00,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,
	0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
	0x05,0x00,0x00,0x02,0x00,0x02,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x1E,0x1E,0x00,
	0x00,0x1E,0x1E,0x00,0x00,0x00,0x01,0x01,

	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
	0x00,0x00,0x00,0x00,0x11,0x02,0x01,0x08,
	0x00,0x37,0x01,0x00,0x00,0x0A,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xC2,0x07
};
/*----------------------------------------------------------------------------*/
void writeConfig( void )
{
#if USE_I2C_CY8CMBR3110
	unsigned char checksum1, checksum2;
	checksum1 = tCY8CMBR3110_ConfigData[126];
	checksum2 = tCY8CMBR3110_ConfigData[127];
	if ( MBR3110_checkWriteConfig(checksum1,checksum2) == 0 ){
		int err = MBR3110_writeConfig((unsigned char*)tCY8CMBR3110_ConfigData);
		if ( err ){
			setMidiBuffer( 0xb0, 0x10, err & 0x7f );
		}
	}
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void TouchMIDI_init(void)
{
	swState[0] = swState[1] = 0;
	writeConfig();
}

/*----------------------------------------------------------------------------*/
//
//     Check Touch Sensor & Generate MIDI Event
//
/*----------------------------------------------------------------------------*/
void checkTouch( void )
{
#if USE_I2C_CY8CMBR3110
	const uint8_t tNote[10] = { 0x3c, 0x3e, 0x40, 0x41, 0x43, 0x45, 0x47, 0x48, 0x4a, 0x4c };
	unsigned char sw[2];
	int			err;

	err = MBR3110_readTouchSw(sw);
	if ( err ){
		setMidiBuffer( 0xb0, 0x11, err & 0x7f );
	}
	else {
		for ( int i=0; i<8; i++ ){
			uint8_t	bitPtn = 0x01 << i;
			if ( (swState[0]&bitPtn)^(sw[0]&bitPtn) ){
				if ( sw[0] & bitPtn ){ setMidiBuffer(0x90,tNote[i],0x7f);}
				else {	setMidiBuffer( 0x90,tNote[i],0x00 );}
			}
		}

		for ( int j=8; j<10; j++ ){
			uint8_t	bitPtn = 0x01 << (j-8);
			if ( (swState[1]&bitPtn)^(sw[1]&bitPtn) ){
				if ( sw[1] & bitPtn ){ setMidiBuffer(0x90,tNote[j],0x7f);}
				else {	setMidiBuffer( 0x90,tNote[j],0x00 );}
			}
		}

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
	checkTouch();

	//	Heartbeat
	if ( IN1 ){ OUT1 = (counter10msec & 0x0020)? 0:1; }
	if ( IN2 ){ OUT2 = (counter10msec & 0x0020)? 0:1; }
	if ( IN3 ){ OUT3 = (counter10msec & 0x0020)? 0:1; }
	if ( IN4 ){ OUT4 = (counter10msec & 0x0020)? 0:1; }
}
#endif
