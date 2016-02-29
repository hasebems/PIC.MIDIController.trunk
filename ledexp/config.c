/*
 * File:   config.c
 * Author: jca03205
 *
 * Created on February 28, 2016, 1:51 PM
 */

#include	"hsb_common.h"
#include	"config.h"

/*----------------------------------------------------------------------------*/
//
//      External Function
//
/*----------------------------------------------------------------------------*/
void LedExp_init(void);
void LedExp_appli(void);

/*----------------------------------------------------------------------------*/
void dummy(void){}

/*----------------------------------------------------------------------------*/
//
//      Set Function
//
/*----------------------------------------------------------------------------*/
const InitFunc initFunc[MAX_INIT_FUNC_NUM] = {
	LedExp_init,
//	TouchMIDI_init
};
/*----------------------------------------------------------------------------*/
const AppliFunc appFunc[MAX_APPLI_FUNC_NUM] = {
	LedExp_appli,
//	TouchMIDI_appli
};
/*----------------------------------------------------------------------------*/
const InterruptFunc intFunc[MAX_INT_FUNC_NUM] = {
	dummy
};
