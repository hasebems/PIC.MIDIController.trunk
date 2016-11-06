/*
 * File:   honeycomb.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 1:49 PM
 */


#include	<xc.h>
#include	<stdbool.h>
#include	"system.h"
#include	"hsb_common.h"
#include	"config.h"
#include	"i2cdevice.h"

#ifdef HONEYCOMB_BELL
/*----------------------------------------------------------------------------*/
//
//      Basic Macros
//
/*----------------------------------------------------------------------------*/
#define		HONEYCOMB_CELL_LED_MAX		12
#define		HONEYCOMB_CELL_LED_MAX_L	8
#define		HONEYCOMB_CELL_TOUCH_MAX	10

#define		CELL_MAX_PER_BLOCK	4
#define		VOLUME_ARRAY_MAX	4	//	number of Max Color
#define		DOREMI_MAX			12
#define		MAX_BEATS			8

#define		DETECT_ID_1			IN1
#define		DETECT_ID_2			IN2
#define		DETECT_ID_3			IN3
#define		OUTPUT_ENABLE		OUT1
#define		DEBUG_LED			OUT4

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t		adCnt;
static uint16_t 	colorArray[HONEYCOMB_CELL_LED_MAX][VOLUME_ARRAY_MAX];
static bool			colorArrayEvent[HONEYCOMB_CELL_LED_MAX];
static uint16_t 	dbgColorArray[VOLUME_ARRAY_MAX];
static uint16_t 	swState;
static int			autoCountPerMeasure;
static bool			beatOn;
static bool			beatOff;
static uint8_t		beatCell;
static uint8_t		elementNum;

static int			boardID;	//	0,1,2,3 ( more than 4 yet )
static bool			highestID;

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
		{1850,	0,		250,	2000},
		{1700,	0,		500,	2000},		//	D
		{1300,	0,		750,	2000},
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
	int8_t		_count;							//	decay count
	uint8_t		_doremi;						//
	uint16_t	_orgColor[VOLUME_ARRAY_MAX];
	uint16_t*	_colorArrayPtr;					//	Pointer to colorArray[]
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
static DLE dle[HONEYCOMB_CELL_LED_MAX] @ 0x240;


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

	uint8_t	checkID = 0;
	if ( DETECT_ID_1 ){ checkID |= 0x01; }
	if ( DETECT_ID_2 ){ checkID |= 0x02; }
	if ( DETECT_ID_3 ){ checkID |= 0x04; }
	boardID = (int)checkID;
	highestID = true;

	for (int i=0;i<VOLUME_ARRAY_MAX;i++){
		for ( int j=0; j<HONEYCOMB_CELL_LED_MAX; j++ ){
			colorArray[j][i] = 0;
		}
		dbgColorArray[i] = 0;
	}

	for (int j=0;j<3;j++ ){ rxBuffer[j] = 0;}
	rxRunningStatus = 0;
	rxBufferCount = 0;

	for (int k=0;k<HONEYCOMB_CELL_LED_MAX;k++ ){
		DLE_init( &dle[k] );
		colorArrayEvent[k] = false;
	}

	writeConfig();

	PCA9685_init(0);
	PCA9685_init(1);
	if (!( boardID & 0x01 )){ PCA9685_init(2);}

	//	Light On
	OUTPUT_ENABLE = 1;
	__delay_ms(10);
	DEBUG_LED = 1;
}

/*----------------------------------------------------------------------------*/
//
//      Generate LED on/off
//
/*----------------------------------------------------------------------------*/
void generateLedEvent( int ledIndex, int doremi, bool onoff )
{
	if ( onoff == true ){
		DLE_on( &dle[ledIndex], &colorArray[ledIndex][0], doremi%12 );
		colorArrayEvent[ledIndex] = true;
	}
	else {
		DLE_off( &dle[ledIndex] );
	}
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
	uint8_t dtID = ( dt & 0x30 )>>4;
	if ( dt & 0x40 ){ beatOn = true; }
	else { beatOff = true; }
	beatCell = dt & 0x0f;
	if ( boardID < dtID ){ highestID = false; }
	return dt;
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
					case 0x90:{
						uint8_t note = rxBuffer[1];
						uint8_t vel = rxBuffer[2];
						if ((( boardID == 2 ) && ( note == 0x4e )) ||
							(( boardID == 0 ) && ( note == 0x42 ))){
							if (vel){ generateLedEvent(10,6,true); }
							else 	{ generateLedEvent(10,6,false); }
						}
						if ((( boardID == 2 ) && ( note == 0x51 )) ||
							(( boardID == 0 ) && ( note == 0x45 ))){
							if (vel){ generateLedEvent(11,9,true); }
							else 	{ generateLedEvent(11,9,false); }
						}
						//	fall through
					}
					case 0x80:	case 0xa0:	case 0xe0:{
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
static const uint8_t tNote[2][HONEYCOMB_CELL_TOUCH_MAX] = {
	{ 0x3e, 0x41, 0x44, 0x47, 0x3d, 0x40, 0x43, 0x46, 0x3c, 0x3f },
	{ 0x46, 0x43, 0x40, 0x3d, 0x47, 0x44, 0x41, 0x3e, 0x45, 0x42 }
};
static const int tCnv2LedNum[2][HONEYCOMB_CELL_TOUCH_MAX] = {
	{	0,1,2,3,4,5,6,7,8,9 },
	{	7,6,5,4,3,2,1,0,-1,-1 }
};
/*----------------------------------------------------------------------------*/
void generateNoteMessage( bool onoff, int tchCount )
{
	uint8_t note = tNote[boardID&0x01][tchCount];
	int		ledIndex = tCnv2LedNum[boardID&0x01][tchCount];
	bool	avoidNote = false;	//	it means this note message never affects LEDs this board operates.

	if ( boardID & 0x02 ){
		note += 12;
		if (( note == 0x4e ) || ( note == 0x51 )){ avoidNote = true;}
	}
	else {
		if (( note == 0x42 ) || ( note == 0x45 )){ avoidNote = true; }
	}

	if ( onoff == true ){
		setMidiBuffer(0x90, note, 0x7f);
		if ( avoidNote == false ){
			DLE_on( &dle[ledIndex], &colorArray[ledIndex][0], note%12 );
			colorArrayEvent[ledIndex] = true;
		}
	}
	else {
		setMidiBuffer( 0x90, note, 0x00 );
		if ( avoidNote == false ){ DLE_off( &dle[ledIndex] ); }
	}
}
/*----------------------------------------------------------------------------*/
void checkTouch( void )
{
#if USE_I2C_CY8CMBR3110
	unsigned char sw[2];
	uint16_t	sw16;
	int			err;

	err = MBR3110_readTouchSw(sw);
	sw16 = sw[0] | ((uint16_t)sw[1]<<8);
	if ( err ){
		setMidiBuffer( 0xb0, 0x11, (uint8_t)err & 0x7f );
	}
	else {
		for ( int i=0; i<HONEYCOMB_CELL_TOUCH_MAX; i++ ){
			uint16_t	bitPtn = 0x0001 << i;

			//	check touch event
			if ( (swState&bitPtn)^(sw16&bitPtn) ){
				bool onoff = false;
				if ( sw16 & bitPtn ){ onoff = true; }
				generateNoteMessage(onoff,i);
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
#define		BEAT_BLINK_TIME			15		//	*10msec
/*----------------------------------------------------------------------------*/
void periodicJobs( void )
{
	if ( event10msec ){
		//	In the first 500msec, Beat doesn't work
		if ( counter10msec < 50 ){ return; }

		//	Beat Master
		if ( highestID == true ){
			int cnt = autoCountPerMeasure%INTERVAL_PER_BEAT;
			beatCell = (uint8_t)(autoCountPerMeasure/INTERVAL_PER_BEAT);
			if ( cnt == 0 ){
				beatOn = true;
				setMidiBuffer( 0xb0, 0x53, beatCell | 0x40 | (boardID<<4) );
			}
			else if ( cnt == BEAT_BLINK_TIME ){
				beatOff = true;
				setMidiBuffer( 0xb0, 0x53, beatCell | (boardID<<4) );
			}

			autoCountPerMeasure++;
			if ( autoCountPerMeasure < 0 ){ autoCountPerMeasure = 0;}
			else if ( autoCountPerMeasure >= (INTERVAL_PER_BEAT*MAX_BEATS) ){ autoCountPerMeasure = 0;}
		}

		//	Light Decay
		for ( int i=0; i<HONEYCOMB_CELL_LED_MAX; i++ ){
			if ( DLE_count( &dle[i] ) == true ){
				colorArrayEvent[i] = true;
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Beat lighting
//		detect led blinking pattern at every beatCell
/*----------------------------------------------------------------------------*/
static const uint16_t tBeatToLedBitMap[2][MAX_BEATS] =
{
	{	0x0421, 0x0842, 0x0184, 0x0218, 0x0421, 0x0842, 0x0184, 0x0218 },	//	ID 0,2
	{	0x0022, 0x0044, 0x0088, 0x0011, 0x0022, 0x0044, 0x0088, 0x0011 }	//	ID 1,3
};
/*----------------------------------------------------------------------------*/
void beatLighting( void )
{
	uint16_t bitPtn = tBeatToLedBitMap[boardID & 0x01][beatCell];
	uint16_t bitMsk = 0x0001;

	for ( int i=0; i<HONEYCOMB_CELL_LED_MAX; i++ ){
		if ( bitMsk & bitPtn ){
			int seg = DLE_getSegment(&dle[i]);
			if ( seg == SEG_NOT_USE ){
				if ( beatOn == true ){
					//	Light Beat
					for ( int j=0; j<VOLUME_ARRAY_MAX; j++ ){
						colorArray[i][j] = 2000; // / (((i&0x01)*4)+1);
					}
					colorArrayEvent[i] = true;
				}
				if ( beatOff == true ){
					//	turn off
					for ( int k=0; k<VOLUME_ARRAY_MAX; k++ ){
						colorArray[i][k] = 0;
					}
					colorArrayEvent[i] = true;
				}
			}
		}
		bitMsk<<=1;
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
	int	ledMax = HONEYCOMB_CELL_LED_MAX;
	if ( boardID & 0x01 ){ ledMax = HONEYCOMB_CELL_LED_MAX_L; }
	for (int i=0; i<ledMax; i++){
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
