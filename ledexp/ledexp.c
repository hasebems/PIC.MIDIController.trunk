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
static unsigned short colorArray[4];

/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void LedExp_init(void)
{
	adCnt = 0;
	for (int i=0;i<4;i++){ colorArray[i] = 0; }
}
/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void LedExp_appli(void)
{
#if USE_I2C_ADS1015
	signed short volume=0;
	int err = ADS1015_getVolume(&volume);
	if ( err != 0 ){ return; }

	colorArray[adCnt] = volume;
	adCnt++;
	if ( adCnt >= 3 ){ adCnt = 0;}
	ADS1015_setNext(adCnt);
#endif

#if USE_I2C_PCA9685
	err = PCA9685_setFullColorLED( 0, colorArray );
#endif

#if USE_I2C_ACM1602N1
	unsigned char lcdStr[] = "Hello, World!";
	err = ACM1602N1_setString( 0, lcdStr, 13 );
#endif


}
#endif
