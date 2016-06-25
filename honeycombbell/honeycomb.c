/*
 * File:   honeycomb.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 1:49 PM
 */


#include	<xc.h>
#include	<stdbool.h>
#include	"hsb_common.h"
#include	"config.h"
#include	"i2cdevice.h"

#ifdef HONEYCOMB_BELL
/*----------------------------------------------------------------------------*/
//
//      Basic Macros
//
/*----------------------------------------------------------------------------*/
#define		HONEYCOMB_CELL_MAX	8
#define		CELL_MAX_PER_BLOCK	4
#define		VOLUME_ARRAY_MAX	4	//	number of Max Color
#define		DOREMI_MAX			12

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t		adCnt;
static uint16_t 	colorArray[HONEYCOMB_CELL_MAX][VOLUME_ARRAY_MAX];
static bool			colorArrayEvent[HONEYCOMB_CELL_MAX];
static uint16_t 	dbgColorArray[VOLUME_ARRAY_MAX];
static uint16_t 	swState;
static int			autoCountPerMeasure;
static bool			beatOn;
static bool			beatOff;
static uint8_t		beatCell;
static uint8_t		elementNum;
static int			cellMax;

static uint8_t		rxBuffer[3];
static uint8_t		rxRunningStatus;
static int 			rxBufferCount;

/*----------------------------------------------------------------------------*/
//
//      Decay Light Effect Class (DLE)
//
/*----------------------------------------------------------------------------*/
const unsigned short tLEDPattern[DOREMI_MAX][VOLUME_ARRAY_MAX] =
{
	//	R		B		G
		{2000,	0,		0,		2000},		//	C
		{1900,	0,		300,	2000},
		{1800,	0,		600,	2000},		//	D
		{1500,	0,		800,	2000},
		{1000,	0,		1000,	2000},		//	E
		{0,		0,		2000,	2000},		//	F
		{0,		1500,	1500,	2000},
		{0,		2000,	0,		2000},		//	G
		{200,	1800,	0,		2000},
		{500,	1500,	0,		2000},		//	A
		{800,	1000,	0,		2000},
		{1200,	300,	0,		2000}		//	B
};
/*----------------------------------------------------------------------------*/
#define		DECAY_COUNT_MAX			50		//	* 10msec
#define		START_DECAY_COUNT		15		//	* 10msec
#define		DECAY_COEF				10		//	* (COEF-1)/COEF
/*	_count	*/
#define		SEG_NOT_USE				(-2)
#define		SEG_IN_TOUCH			(-1)
#define		SEG_DECAY				0
/*----------------------------------------------------------------------------*/
typedef struct {
	int			_count;						//	decay count
	uint16_t	_orgColor[VOLUME_ARRAY_MAX];
	uint16_t*	_colorArrayPtr;				//	Pointer to colorArray[]
	uint8_t		_doremi;						//
} DLE;
/*----------------------------------------------------------------------------*/
int DLE_getSegment( DLE* this ){ return this->_count; }
/*----------------------------------------------------------------------------*/
void DLE_init( DLE* this )
{
	this->_count = SEG_NOT_USE;		//	decay count
	this->_colorArrayPtr = 0;		//	Pointer to colorArray[]
	this->_doremi = 0;
	for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
		this->_orgColor[i] = 0;
	}
}
/*----------------------------------------------------------------------------*/
void DLE_on( DLE* this, uint16_t* ptr, uint8_t doremi )
{
	this->_count = SEG_IN_TOUCH;
	this->_colorArrayPtr = ptr;
	this->_doremi = doremi;
	while ( doremi >= DOREMI_MAX ){ doremi -= DOREMI_MAX; } 
	for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
		*(this->_colorArrayPtr+i) = this->_orgColor[i] = tLEDPattern[doremi][i];
	}
}
/*----------------------------------------------------------------------------*/
void DLE_off( DLE* this )
{
	this->_count = SEG_DECAY;
}
/*----------------------------------------------------------------------------*/
bool DLE_count( DLE* this )
{
	if ( this->_count >= SEG_DECAY ){
		this->_count++;
		if (( this->_count > START_DECAY_COUNT ) && ( this->_count < DECAY_COUNT_MAX )){
			int dcy = this->_count - START_DECAY_COUNT;
			for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
				this->_orgColor[i] = (this->_orgColor[i]*(DECAY_COEF-1))/DECAY_COEF;
				*(this->_colorArrayPtr+i) = this->_orgColor[i];
			}
			return true;
		}
		else if ( this->_count >= DECAY_COUNT_MAX ){
			//	End of Decay
			for ( int i=0; i<VOLUME_ARRAY_MAX; i++ ){
				*(this->_colorArrayPtr+i) = 0;
			}
			this->_count = SEG_NOT_USE;
			return true;
		}
	}
	return false;
}
/*----------------------------------------------------------------------------*/
static DLE dle[HONEYCOMB_CELL_MAX];

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
			setMidiBuffer( 0xb0, 0x10, (uint8_t)err & 0x7f );
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
	//	Initialize Global Variables
	adCnt = 0;
	autoCountPerMeasure = 0;
	beatOn = false;
	beatOff = false;
	beatCell = 0;
	elementNum = 0;
	cellMax	= 0;

	for (int i=0;i<VOLUME_ARRAY_MAX;i++){
		for ( int j=0; j<HONEYCOMB_CELL_MAX; j++ ){
			colorArray[j][i] = 0;
		}
		dbgColorArray[i] = 0;
	}

	for (int j=0;j<3;j++ ){ rxBuffer[j] = 0;}
	rxRunningStatus = 0;
	rxBufferCount = 0;

	for (int k=0;k<HONEYCOMB_CELL_MAX;k++ ){
		DLE_init( &dle[k] );
		colorArrayEvent[k] = false;
	}

	writeConfig();

	//	Check Port
	if ( IN1 == 1 ){ cellMax = CELL_MAX_PER_BLOCK; }
	else { cellMax = CELL_MAX_PER_BLOCK*2; }

	//	Light On
	OUT1 = 1;
}

/*----------------------------------------------------------------------------*/
//
//      Detect Element by MIDI special command
//		Set "beatOn,beatOff,beatCell,elementNum"
//
/*----------------------------------------------------------------------------*/
uint8_t detectElementByMidi( uint8_t dt )
{
	uint8_t data = dt & 0x4f;
	uint8_t elm = ( dt & 0x30 )>>4;
	if ( elm < 3 ){ elm += 1; }
	if ( dt & 0x40 ){ beatOn = true; }
	else { beatOff = true; }
	beatCell = dt & 0x0f;
	elementNum = elm;
	data |= elm << 4;
	return data;
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
		if (rcvDt & 0x80){
			//	Status Byte
			rxBuffer[0] = rcvDt;
			rxBufferCount = 1;
		}
		else {
			//	Check Running Status
			if ( rxBufferCount == 0 ){
				rxBuffer[0] = rxRunningStatus;
				rxBufferCount++;
			}

			//	Save Data Byte
			if ( rxBufferCount < 3 ){
				rxBuffer[rxBufferCount] = rcvDt;
				rxBufferCount++;
			}

			//	judge termination
			uint8_t	statusByte = rxBuffer[0];
			if (rxBufferCount == 2){
				switch (statusByte&0xf0){
					case 0xc0:	case 0xd0:{
						setMidiBuffer( statusByte, rxBuffer[1], 0);
						rxRunningStatus = statusByte;
						rxBufferCount = 0;
						break;
					}
					default: break;
				}
			}
			else if (rxBufferCount == 3){
				switch (statusByte&0xf0){
					case 0xb0:{
						//	HCB Element detection
						if ( rxBuffer[1] == 0x53 ){
							rxBuffer[2] = detectElementByMidi( rxBuffer[2] );
						}
						setMidiBuffer( statusByte, rxBuffer[1], rxBuffer[2]);
						rxRunningStatus = statusByte;
						rxBufferCount = 0;
						break;
					}
					case 0x90:	case 0x80:	case 0xa0:	case 0xe0:{
						setMidiBuffer( statusByte, rxBuffer[1], rxBuffer[2]);
						rxRunningStatus = statusByte;
						rxBufferCount = 0;
						break;
					}
					default: break;
				}
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//     Check Touch Sensor & Generate MIDI Event
//
/*----------------------------------------------------------------------------*/
void checkTouch( void )
{
#if USE_I2C_CY8CMBR3110
	const uint8_t tNote[10] = { 0x3c, 0x3f, 0x42, 0x45, 0x48, 0x4b, 0x4e, 0x51, 0x60, 0x60 };
	unsigned char sw[2];
	uint16_t	sw16;
	int			err;

	err = MBR3110_readTouchSw(sw);
	sw16 = sw[0] | ((uint16_t)sw[1]<<8);
	if ( err ){
		setMidiBuffer( 0xb0, 0x11, (uint8_t)err & 0x7f );
	}
	else {
		for ( int i=0; i<cellMax; i++ ){
			uint16_t	bitPtn;
			int 		shiftBit = i;
			if ( i >= CELL_MAX_PER_BLOCK ){ shiftBit++;}
			bitPtn = 0x0001 << shiftBit;

			if ( (swState&bitPtn)^(sw16&bitPtn) ){
				uint8_t note = tNote[i] +2 -elementNum;
				if ( sw16 & bitPtn ){
					setMidiBuffer(0x90,note,0x7f);
					DLE_on( &dle[i], &colorArray[i][0], note-0x3c );
					colorArrayEvent[i] = true;
				}
				else {
					setMidiBuffer( 0x90,note,0x00 );
					DLE_off( &dle[i] );
				}
			}
		}
		swState = sw16;
	}
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Check Volume & Set colorArray[]
//
/*----------------------------------------------------------------------------*/
#if USE_I2C_ADS1015
void checkVolume( void )
{
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
}
#endif
/*----------------------------------------------------------------------------*/
//
//      Automatic Lighting
//
/*----------------------------------------------------------------------------*/
#define		INTERVAL_PER_BEAT		25		//	*10msec
#define		BEAT_BLINK_TIME			10		//	*10msec
/*----------------------------------------------------------------------------*/
void periodicJobs( void )
{
	if ( event10msec ){
		//	In the first 500msec, Beat doesn't work
		if ( counter10msec < 50 ){ return; }

		//	Beat Master
		if ( elementNum == 0 ){
			int cnt = autoCountPerMeasure%INTERVAL_PER_BEAT;
			beatCell = (uint8_t)(autoCountPerMeasure/INTERVAL_PER_BEAT);
			if ( cnt == 0 ){
				beatOn = true;
				setMidiBuffer( 0xb0, 0x53, beatCell | 0x40 );
			}
			else if ( cnt == BEAT_BLINK_TIME ){
				beatOff = true;
				setMidiBuffer( 0xb0, 0x53, beatCell );
			}

			autoCountPerMeasure++;
			if ( autoCountPerMeasure < 0 ){ autoCountPerMeasure = 0;}
			else if ( autoCountPerMeasure >= (INTERVAL_PER_BEAT*cellMax) ){ autoCountPerMeasure = 0;}
		}

		//	Light Decay
		for ( int i=0; i<cellMax; i++ ){
			if ( DLE_count( &dle[i] ) == true ){
				colorArrayEvent[i] = true;
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Beat lighting
//
/*----------------------------------------------------------------------------*/
void beatLighting( void )
{
	int seg = DLE_getSegment(&dle[beatCell]);
	if ( seg == SEG_NOT_USE ){
		if ( beatOn == true ){
			//	Light Beat
			for ( int j=0; j<VOLUME_ARRAY_MAX; j++ ){
				colorArray[beatCell][j] = 2000 / (((beatCell&0x01)*4)+1);
			}
			colorArrayEvent[beatCell] = true;
		}
		if ( beatOff == true ){
			//	turn off
			for ( int k=0; k<VOLUME_ARRAY_MAX; k++ ){
				colorArray[beatCell][k] = 0;
			}
			colorArrayEvent[beatCell] = true;
		}
	}
	beatOn = beatOff = false;
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

	//	Check Volume
	//checkVolume();

	//	Generate beats, light decay
	periodicJobs();

	//	Light beat LED
	beatLighting();

#if USE_I2C_PCA9685
	for (int i=0; i<cellMax; i++){
		if ( colorArrayEvent[i] == true ){
			err = PCA9685_setFullColorLED( i/CELL_MAX_PER_BLOCK, i%CELL_MAX_PER_BLOCK, (unsigned short*)colorArray[i] );
			colorArrayEvent[i] = false;
		}
	}
#endif

#if USE_I2C_ACM1602N1
	unsigned char lcdStr[] = "Hello, World!";
	err = ACM1602N1_setString( 0, lcdStr, 13 );
#endif
}
#endif
