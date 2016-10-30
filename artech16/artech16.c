/*
 * File:   touchmidi.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 2:46 PM
 */


#include <xc.h>
#include <stdbool.h>
#include "hsb_common.h"
#include "config.h"
#include "i2cdevice.h"
#include "system.h"

#ifdef ARTECH16
/*----------------------------------------------------------------------------*/
//
//      Macros
//
/*----------------------------------------------------------------------------*/
#define	PORTC_DIR_REFOUT		0b00000000
#define	PORTC_DIR_REFIN			0b11111111
#define	BRIGHTNESS_JUDGE_CNT	4		//	*250usec = 1msec
#define	MAX_PERIOD				150		//	*1msec
#define	MIN_PERIOD				10		//	*1msec

#define	LEDX					PORTBbits.RB6

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
//	for  Reflectance Sensor
static int			refSnsStatus;			//	Reflectance sensor interrupt process manage (0-3))
//static int			sameConditionCounter;	//	indecates how long same condition keeps
//static int			whitePeriod;			//	roller Speed during white(faster means lower but not 0)
//static int			blackPeriod;			//	roller Speed during black
static uint8_t		photoState;
static uint8_t		lastPhotoState;
volatile int		doNothing;

static uint8_t		rxBuffer[3];
static uint8_t		rxRunningStatus;
static int 			rxBufferCount;

static uint8_t		beatCell;
static uint8_t		elementNum;

static bool			startScanSw;
static bool			duringScan;

static long			lifeCount;

/*----------------------------------------------------------------------------*/
//
//      Interrupt function
//
/*----------------------------------------------------------------------------*/
void Artech16_intrpt( void )
{
	//	Reflectance Sensor
	++refSnsStatus;
	if ( refSnsStatus == BRIGHTNESS_JUDGE_CNT ){
		photoState = PORTC;		//	Get Reflectance

		TRISC   = PORTC_DIR_REFOUT;	//	Change to Outoput Port
		PORTC	= 0xff;				//	Start Pulse

		//	Same Condition retain count
//		if ( isWhiteLast != white ){
//			if (( sameConditionCounter > MIN_PERIOD ) && ( sameConditionCounter < MAX_PERIOD )){
//				//	Change Event
//				if ( white ){ blackPeriod = sameConditionCounter;}
//				else { whitePeriod = sameConditionCounter;}
//				//whitePeriod = sameConditionCounter;
//			}
//			sameConditionCounter = 0;
//			isWhiteLast = white;
//		}
//		else if ( MAX_PERIOD > sameConditionCounter ){
//			++sameConditionCounter;
//		}
//		else {
//			whitePeriod = blackPeriod = MAX_PERIOD;
//		}

		__delay_us(10);
		TRISC   = PORTC_DIR_REFIN;	//	Change to Input Port
		refSnsStatus = 0;
	}
}
/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void Artech16_init(void)
{
	int err;
	unsigned char selfCheckResult;

	refSnsStatus = 0;
	photoState = 0;
	lastPhotoState = 0;
	startScanSw = 1;
	duringScan = false;
	lifeCount = 0;
	elementNum = 0;

	quitI2c();
	TRISC   = PORTC_DIR_REFIN;
	TRISB   =	0b10110000;			//I2C master mode, UART Tx/Rx(set INPUT)
	LEDX = 0;
}

/*----------------------------------------------------------------------------*/
//
//      Send MIDI
//
/*----------------------------------------------------------------------------*/
void sendMIDI( uint8_t byte1, uint8_t byte2, uint8_t byte3 )
{
	setSerialMidiBuffer( byte1, byte2, byte3 );

	if ( duringScan == true ){
		setUsbMidiBuffer( byte1, byte2, byte3 );
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
						sendMIDI( statusByte, rxBuffer[1], 0);
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
							uint8_t	dtbyte = rxBuffer[2];
							dtbyte++;
							if ( dtbyte > 2 ){ dtbyte = 2; }
							if ( elementNum != dtbyte ){
								elementNum = dtbyte;
							}
						}
						else {
							sendMIDI( statusByte, rxBuffer[1], rxBuffer[2]);
							rxRunningStatus = statusByte;
							rxBufferCount = 0;
						}
						break;
					}
					case 0x90:	case 0x80:	case 0xa0:	case 0xe0:{
						sendMIDI( statusByte, rxBuffer[1], rxBuffer[2]);
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
//      Check Start Switch
//
/*----------------------------------------------------------------------------*/
void checkStartSwitch( void )
{
	//	Switch Check
	bool startSw = PORTBbits.RB4;
	if ( startSw ^ startScanSw ){
		if ( startSw == false ){
			if ( duringScan == false ){
				duringScan = true;
				LEDX = 1;
			}
			else {
				duringScan = false;
				LEDX = 0;
			}
			setUsbMidiBuffer(0xb0,120,0);	// all sound off
		}
		startScanSw = startSw;
	}
}

/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void periodicJob( void )
{
	sendMIDI( 0xb0, 0x53, elementNum );

	if ( duringScan == true ){
		if ( (lifeCount % 5) < 2 ){ LEDX = 1;}
		else { LEDX = 0; }
	}
}

/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
const uint8_t bitTbl[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
void Artech16_appli(void)
{
	int	i;

	//	Periodic Job
	if ( event100msec ){
		lifeCount++;
		periodicJob();
	}

	//	Analyze Rx MIDI
	analyzeRecievedMIDI();

	checkStartSwitch();

	if ( event10msec && ( lifeCount > 10 )){
		uint8_t	pstt = photoState;
		if ( pstt != lastPhotoState ){
//			sendMIDI(0xb0,0x01,pstt&0x7f);
			for ( i=0; i<8; i++ ){
				if ( (pstt&~lastPhotoState) & bitTbl[i] ){
					sendMIDI(0x90,0x3c+i+elementNum*8,0x7f);
				}
			}
			for ( i=0; i<8; i++ ){
				if ( (~pstt&lastPhotoState) & bitTbl[i] ){
					sendMIDI(0x90,0x3c+i+elementNum*8,0x00);
				}
			}
			lastPhotoState = pstt;
		}
	}
}
#endif
