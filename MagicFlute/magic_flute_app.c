/*
 * File:   magic_flute_app.c
 * Author: hasebems
 *
 * Created on 2016/3/1, 0:00
 */

#include "hsb_common.h"
#include "config.h"
#include "analyse_pressure.h"
#include "analyse_touch.h"
#include "i2cdevice.h"

#ifdef MAGIC_FLUTE
/*----------------------------------------------------------------------------*/
//
//      Macros
//
/*----------------------------------------------------------------------------*/
//	for doremi
#define		NO_NOTE				12
#define		WARNING				(NO_NOTE+1)

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t		midiExp;
static uint8_t		doremi;
static uint8_t		crntNote;

static uint8_t		lastMod;
static uint8_t		lastPrt;

static bool         nowPlaying;
static bool			isFinishedWriting;

/*----------------------------------------------------------------------------*/
//
//      Functions
//
/*----------------------------------------------------------------------------*/



 /*----------------------------------------------------------------------------*/
 //
 //      Full Color LED by Interrupt
 //
 /*----------------------------------------------------------------------------*/
 const uint8_t tColorTable[13][3] = {

 	//	this Value * midiExp(0-127) / 16 = 0x00-0xfe : PWM count value
 	// R	 G		B
 	{ 0x20,  0x00,  0x00  },   //  red		C
 	{ 0x1a,  0x06,  0x00  },   //  red		C#
 	{ 0x16,  0x0a,  0x00  },   //  orange	D
 	{ 0x14,  0x0c,  0x00  },   //  orange	D#
 	{ 0x0c,  0x14,  0x00  },   //  yellow	E
 	{ 0x00,  0x20,  0x00  },   //  green	F
 	{ 0x00,  0x10,  0x10  },   //  green	F#
 	{ 0x00,  0x00,  0x20  },   //  blue		G
 	{ 0x04,  0x00,  0x1c  },   //  blue		G#
 	{ 0x08,  0x00,  0x18  },   //  violet	A
 	{ 0x0c,  0x00,  0x14  },   //  violet	A#
 	{ 0x18,  0x00,  0x08  },   //  violet	B
 	{ 0x00,  0x00,  0x00  },   //  none
 };
 //-------------------------------------------------------------------------
void magicFlute_int(void)
{
	if ( doremi == WARNING ){
		LEDG = 0;
		LEDB = 0;
		LEDR = (globalCount & 0x0400 )? 1:0;	//	250msec
		return;
	}

    //	PWM Full Color LED
    uint16_t ledCnt;
    ledCnt = ((uint16_t)tColorTable[doremi][0]*midiExp)>>4;
    LEDR = ((uint16_t)pwm16msInterval >= ledCnt)? 1:0;

    ledCnt = ((uint16_t)tColorTable[doremi][1]*midiExp)>>4;
    LEDG = ((uint16_t)pwm16msInterval >= ledCnt)? 1:0;

    ledCnt = ((uint16_t)tColorTable[doremi][2]*midiExp)>>4;
    LEDB = ((uint16_t)pwm16msInterval >= ledCnt)? 1:0;
}

/*----------------------------------------------------------------------------*/
//
//      CY8CMBR3110 Write Configuration
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
		LEDR = 0;
		LEDB = 0;
		LEDG = 1;
		int err = MBR3110_writeConfig((unsigned char*)tCY8CMBR3110_ConfigData);
		if ( err ){
			setMidiBuffer( 0xb0, 0x10, err & 0x7f );
			doremi = WARNING;
		}
		LEDG = 0;
	}
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Initialize
//
/*----------------------------------------------------------------------------*/
void magicFlute_init(void)
{
	midiExp = 0;
    doremi = NO_NOTE;
	crntNote = 96;
	lastMod = 0;
	lastPrt = 0;
	nowPlaying = false;
	isFinishedWriting = false;

    //  H/W init
    LEDR = 0;
	LEDB = 0;
	LEDG = 0;

    AnalysePressure_Init();
    AnalyseTouch_init();
	writeConfig();
}
/*----------------------------------------------------------------------------*/
//
//      Touch Sensor
//
/*----------------------------------------------------------------------------*/
#if USE_I2C_CY8CMBR3110
static void touchSensor( void )
{
	unsigned char	tch[2];
	int	err;

	err = MBR3110_readTouchSw(tch);
	if ( err != 0 ){
        i2cComErr = err + 80;
    }

	AnalyseTouch_setNewTouch(tch[0]);
	if ( event10msec ){
		uint8_t mdNote;
		if ( AnalyseTouch_catchEventOfPeriodic(&mdNote,counter10msec) == true ){
			if ( nowPlaying == true ){
				setMidiBuffer(0x90,mdNote,0x7f);
				setMidiBuffer(0x90,crntNote,0x00);
				if ( doremi < WARNING ){
					doremi = mdNote%12;
				}
			}
			else {
				setMidiBuffer(0xa0,mdNote,0x01);
				setMidiBuffer(0xa0,crntNote,0x00);
			}
	        crntNote = mdNote;
		}
	}

	//	Program CY8CMBR3110 config Data
	if (( IN4 == 0 ) && ( isFinishedWriting == false )){
		isFinishedWriting = true;
		writeConfig();
	}
}
#endif
/*----------------------------------------------------------------------------*/
//
//      Pressure Sensor
//
/*----------------------------------------------------------------------------*/
#if USE_I2C_LPS25H
static void pressureSensor( void )
{
	int	prs;
	int err;

	err = LPS25H_getPressure(&prs);
	if ( err != 0 ){
        i2cComErr = err;
    }

	AnalysePressure_setNewRawPressure(prs);
	if ( event5msec == true ){
		if ( AnalysePressure_catchEventOfPeriodic(&midiExp) == true ){
			setMidiBuffer(0xb0,0x0b,midiExp);
			if (( midiExp > 0 ) && ( nowPlaying == false )){
	            setMidiBuffer(0x90,crntNote,0x7f);
		        nowPlaying = true;
				if ( doremi < WARNING ){
					doremi = crntNote%12;
				}
			}
		    if (( midiExp == 0 ) && ( nowPlaying == true )){
			    setMidiBuffer(0x90,crntNote,0);
				nowPlaying = false;
				if ( doremi < WARNING ){
					doremi = NO_NOTE;
				}
			}
		}
    }
}
#endif
/*----------------------------------------------------------------------------*/
//
//      Accelerator Sensor
//
/*----------------------------------------------------------------------------*/
#if USE_I2C_ADXL345
//-------------------------------------------------------------------------
#define		MAX_ANGLE		32
const uint8_t tCnvModDpt[MAX_ANGLE] = {
	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	1,	1,	2,	2,
	3,	4,	5,	6,	7,	8,	9,	10,
	12,	14,	16,	19,	22,	25,	28,	31,
};
//-------------------------------------------------------------------------
const uint8_t tCnvPrtDpt[MAX_ANGLE] = {
	0,	0,	0,	0,	0,	0,	0,	0,
	10,	20,	30,	40,	50,	60,	70,	70,
	80,	80,	80,	80,	90,	90,	90,	90,
	100,100,100,100,110,110,110,110,
};
//-------------------------------------------------------------------------
static void acceleratorSensor( void )
{
	signed short acl[3] = { 0,0,0 };
	int incli, err;

	err = ADXL345_getAccel(0,acl);
	if ( err != 0 ){
        i2cComErr = err + 40;
    }
//    if ( DIPSW1 == 1 ){ //  Ocarina
//    	incli = -acl[1]/512;
//    }
//    else {              //  Recorder
    	incli = acl[0]/512;
//    }

	uint8_t modVal, prtVal;
	if ( incli < 0 ){
		modVal = 0;
		prtVal = incli * (-1);
		if ( prtVal >= MAX_ANGLE ){
	        prtVal = MAX_ANGLE-1;
		}
	}
	else {
		prtVal = 0;
		modVal = incli;
		if ( modVal >= MAX_ANGLE ){
	        modVal = MAX_ANGLE-1;
	    }
	}

	//	lessen variation of modulation
	modVal = tCnvModDpt[modVal];
	if ( modVal > lastMod ){
		lastMod++;
		setMidiBuffer(0xb0,0x01,lastMod);
	}
	else if ( modVal < lastMod ){
		lastMod--;
		setMidiBuffer(0xb0,0x01,lastMod);
	}

	prtVal = tCnvPrtDpt[prtVal];
	if ( prtVal != lastPrt ){
		lastPrt = prtVal;
		setMidiBuffer(0xb0,0x05,lastPrt);
	}
}
#endif
/*----------------------------------------------------------------------------*/
//
//      Application
//
/*----------------------------------------------------------------------------*/
void magicFlute_appli(void)
{
    //  Touch Sensor
#if USE_I2C_CY8CMBR3110
    touchSensor();
#endif

    //	Air Pressure Sensor
#if USE_I2C_LPS25H
    pressureSensor();
#endif

    //  accelerator sensor
#if USE_I2C_ADXL345
    acceleratorSensor();
#endif
}
#endif
