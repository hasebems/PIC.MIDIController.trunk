/*
 * File:   ledexp.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 1:49 PM
 */


#include <xc.h>
#include "hsb_common.h"
#include "config.h"
#include "i2cdevice.h"

#ifdef LED_EXP
/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t		adCnt;
static uint16_t 	colorArray[4];
static uint16_t 	dbgColorArray[4];

/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void LedExp_init(void)
{
	adCnt = 0;
	for (int i=0;i<4;i++){
		colorArray[i] = 0;
		dbgColorArray[i] = 0;
	}
}
/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void LedExp_appli(void)
{
	signed short volume=0;
#if USE_I2C_ADS1015
	int err = ADS1015_getVolume(&volume);
	if ( err != 0 ){ return; }
#endif

	//	Adjust Volume Value
	volume -= 50;
	if ( volume < 0 ){ volume = 0; }

	//	Output Debug MIDI
	signed short diff = volume - dbgColorArray[adCnt];
	if (( event100msec == true ) && ((diff > 16 ) || ( diff < -16 ))){
		dbgColorArray[adCnt] = volume;
		setMidiBuffer( 0xb0 | adCnt, 0x10, (uint8_t)((volume & 0xf000)>>12) );
		setMidiBuffer( 0xb0 | adCnt, 0x11, (uint8_t)((volume & 0x0f00)>>8) );
		setMidiBuffer( 0xb0 | adCnt, 0x12, (uint8_t)((volume & 0x00f0)>>4) );
		setMidiBuffer( 0xb0 | adCnt, 0x13, (uint8_t)(volume & 0x000f) );
	}

	//	Update colorArray[]
	colorArray[adCnt] = volume;
	adCnt++;
	if ( adCnt >= 3 ){ adCnt = 0;}
#if USE_I2C_ADS1015
	ADS1015_setNext(adCnt);
#endif

#if USE_I2C_PCA9685
	err = PCA9685_setFullColorLED( 0, (unsigned short*)colorArray );
#endif

#if USE_I2C_ACM1602N1
	unsigned char lcdStr[] = "Hello, World!";
	err = ACM1602N1_setString( 0, lcdStr, 13 );
#endif
}
#endif
