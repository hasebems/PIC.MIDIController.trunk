/*
 * File:   honeycomb.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 1:49 PM
 */


#include <xc.h>
#include "hsb_common.h"
#include "config.h"
#include "i2cdevice.h"

#ifdef HONEYCOMB_BELL
/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
#define		VOLUME_ARRAY_MAX	4

static uint8_t		adCnt;
static uint16_t 	colorArray[VOLUME_ARRAY_MAX];
static uint16_t 	dbgColorArray[VOLUME_ARRAY_MAX];
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
void Honeycomb_init(void)
{
	adCnt = 0;
	for (int i=0;i<VOLUME_ARRAY_MAX;i++){
		colorArray[i] = 0;
		dbgColorArray[i] = 0;
	}

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
//      Check Volume & Set colorArray[]
//
/*----------------------------------------------------------------------------*/
void checkVolume( void )
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
	if ( adCnt >= VOLUME_ARRAY_MAX-1 ){ adCnt = 0;}
#if USE_I2C_ADS1015
	ADS1015_setNext(adCnt);
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void Honeycomb_appli(void)
{
	int err;

	//	Touch MIDI OUT
	checkTouch();

	//	Check Volume & Set colorArray[]
	checkVolume();

#if USE_I2C_PCA9685
	err = PCA9685_setFullColorLED( 0, (unsigned short*)colorArray );
	err = PCA9685_setFullColorLED( 1, (unsigned short*)colorArray );
#endif

#if USE_I2C_ACM1602N1
	unsigned char lcdStr[] = "Hello, World!";
	err = ACM1602N1_setString( 0, lcdStr, 13 );
#endif
}
#endif
