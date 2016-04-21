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
#define		HONEYCOMB_CELL_MAX	4
#define		VOLUME_ARRAY_MAX	4

static uint8_t		adCnt;
static uint16_t 	colorArray[HONEYCOMB_CELL_MAX][VOLUME_ARRAY_MAX];
static uint16_t 	dbgColorArray[VOLUME_ARRAY_MAX];
static unsigned char swState;
static int			autoCount;

static uint8_t	rxBuffer[3];
static int rxBufferCount;

/*----------------------------------------------------------------------------*/
//
//      Decay Light Effect Class (DLE)
//
/*----------------------------------------------------------------------------*/
const unsigned short tLEDPattern[12][VOLUME_ARRAY_MAX] =
{
		//	R	B	G
		{2000,0,0,2000},		//	C
		{1800,0,1200,2000},
		{1500,0,1500,2000},		//	D
		{1200,0,1800,2000},
		{800,0,1900,2000},		//	E
		{0,0,2000,2000},		//	F
		{0,1000,2000,2000},
		{0,2000,0,2000},		//	G
		{500,2000,0,2000},
		{1000,2000,0,2000},		//	A
		{2000,1500,0,2000},
		{2000,2000,2000,2000},	//
};
/*----------------------------------------------------------------------------*/
#define		DECAY_COUNT_MAX			50		//	* 10msec
#define		START_DECAY_COUNT		15		//	* 10msec
#define		DECAY_COEF				10		//	* (COEF-1)/COEF
/*----------------------------------------------------------------------------*/
typedef struct {
	int			count;						//	decay count
	uint16_t	orgColor[VOLUME_ARRAY_MAX];
	uint16_t*	colorArrayPtr;				//	Pointer to colorArray[]
	uint8_t		doremi;						//
} DLE;
/*----------------------------------------------------------------------------*/
void DLE_on( DLE* this, uint16_t* ptr, uint8_t doremi )
{
	this->count = -1;
	this->colorArrayPtr = ptr;
	this->doremi = doremi;
	for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
		*(this->colorArrayPtr+i) = this->orgColor[i] = tLEDPattern[doremi][i];
	}
}
/*----------------------------------------------------------------------------*/
void DLE_off( DLE* this )
{
	this->count = 0;
}
/*----------------------------------------------------------------------------*/
int DLE_count( DLE* this )
{
	if ( this->count >= 0 ){
		this->count++;
		if ( this->count > START_DECAY_COUNT ){
			int dcy = this->count - START_DECAY_COUNT;
			for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
				this->orgColor[i] = (this->orgColor[i]*(DECAY_COEF-1))/DECAY_COEF;
				*(this->colorArrayPtr+i) = this->orgColor[i];
			}
		}

		if ( this->count >= DECAY_COUNT_MAX ){
			//	End of Decay
			for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
				*(this->colorArrayPtr+i) = 0;
			}
			this->count = -2;
			return 1;
		}
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static DLE dle[2];

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
	autoCount = 0;
	for (int i=0;i<VOLUME_ARRAY_MAX;i++){
		for ( int j=0; j<HONEYCOMB_CELL_MAX; j++ ){
			colorArray[j][i] = 0;
		}
		dbgColorArray[i] = 0;
	}

	for (int j=0;j<3;j++ ){ rxBuffer[j] = 0;}
	rxBufferCount = 0;

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
	const uint8_t tNote[4] = { 0x3c, 0x3f, 0x42, 0x45 };
	unsigned char sw[2];
	int			err;

	err = MBR3110_readTouchSw(sw);
	if ( err ){
		setMidiBuffer( 0xb0, 0x11, err & 0x7f );
	}
	else {
		for ( int i=0; i<4; i++ ){
			uint8_t	bitPtn = 0x01 << i;
			if ( (swState&bitPtn)^(sw[0]&bitPtn) ){
				if ( sw[0] & bitPtn ){ setMidiBuffer(0x90,tNote[i],0x7f);}
				else {	setMidiBuffer( 0x90,tNote[i],0x00 );}
			}
		}
		swState = sw[0];
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
#if USE_I2C_ADS1015
	signed short volume=0;

	int err = ADS1015_getVolume(&volume);
	if ( err != 0 ){ return; }

	//	Adjust Volume Value
	volume -= 50;
	if ( volume < 0 ){ volume = 0; }
	volume = volume*5/2;

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
	colorArray[0][adCnt] = colorArray[1][adCnt] = volume;
	adCnt++;
	if ( adCnt >= VOLUME_ARRAY_MAX-1 ){ adCnt = 0;}

	ADS1015_setNext(adCnt);
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Automatic Lighting
//
/*----------------------------------------------------------------------------*/
#define		INTERVAL	100		//	*10msec
/*----------------------------------------------------------------------------*/
void automaticLighting( void )
{
	if ( event10msec ){
		int color = (autoCount / INTERVAL)%24;
		int cell = color & 0x0001;
		color /= 2;
		if ( autoCount%INTERVAL == 0 ){
			DLE_on( &dle[cell], &colorArray[cell][0], color );
		}
		else if ( autoCount%INTERVAL == 1 ){
			DLE_off( &dle[cell] );
		}
		else {
			DLE_count( &dle[cell] );
		}
		autoCount++;
		if ( autoCount < 0 ){ autoCount = 0;}
	}
}
/*----------------------------------------------------------------------------*/
//
//      Analyze USART Rx Data as MIDI
//
/*----------------------------------------------------------------------------*/
void analyzeRecievedMIDI( void )
{
	uint8_t	rcvDt;

	rcvDt = getRecievedMIDI();
	if ( rcvDt != 0xff ){
		if ((rcvDt & 0x80) && ((rcvDt&0xf0) != 0xf0 )){
			//	Status Byte
			rxBufferCount = 0;
			rxBuffer[rxBufferCount++] = rcvDt;
		}
		else if (!(rcvDt & 0x80) && (rxBufferCount > 0)){
			//	Data Byte
			rxBuffer[rxBufferCount++] = rcvDt;
		}
	}

	if (rxBufferCount > 1){
		uint8_t	statusByte = rxBuffer[0];
		if (rxBufferCount == 2){
			switch (statusByte&0xf0){
				case 0xc0:	case 0xd0:{
					setMidiBuffer( statusByte, rxBuffer[1], 0);
					rxBufferCount = 0;
					break;
				}
				default: break;
			}
		}
		else if (rxBufferCount == 3){
			switch (statusByte&0xf0){
				case 0x90:	case 0x80:	case 0xa0:	case 0xb0:	case 0xe0:{
					setMidiBuffer( statusByte, rxBuffer[1], rxBuffer[2]);
					rxBufferCount = 0;
					break;
				}
				default: break;
			}
		}
		if ( rxBufferCount >= 3 ){ rxBufferCount = 0;}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void Honeycomb_appli(void)
{
	int err;

	//	Analyze Rx MIDI
	analyzeRecievedMIDI();

	//	Touch MIDI OUT
	checkTouch();

	//	Check Volume & Set colorArray[]
	//checkVolume();
	automaticLighting();

#if USE_I2C_PCA9685
	err = PCA9685_setFullColorLED( 0, (unsigned short*)colorArray[0] );
	err = PCA9685_setFullColorLED( 1, (unsigned short*)colorArray[1] );
#endif

#if USE_I2C_ACM1602N1
	unsigned char lcdStr[] = "Hello, World!";
	err = ACM1602N1_setString( 0, lcdStr, 13 );
#endif
}
#endif
