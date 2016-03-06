/*
 * File:   config.c
 * Author: jca03205
 *
 * Created on January 1, 2016
 */

#include	"hsb_common.h"
#include	"config.h"

/*----------------------------------------------------------------------------*/
//
//      External Function
//
/*----------------------------------------------------------------------------*/
void TouchMIDI_init(void);
void TouchMIDI_appli(void);

/*----------------------------------------------------------------------------*/
void dummy(void){}

/*----------------------------------------------------------------------------*/
//
//      Set Function
//
/*----------------------------------------------------------------------------*/
const InitFunc initFunc[MAX_INIT_FUNC_NUM] = {
	TouchMIDI_init,
};
/*----------------------------------------------------------------------------*/
const AppliFunc appFunc[MAX_APPLI_FUNC_NUM] = {
	TouchMIDI_appli,
};
/*----------------------------------------------------------------------------*/
const InterruptFunc intFunc[MAX_INT_FUNC_NUM] = {
	dummy
};
