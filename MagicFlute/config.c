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
void magicFlute_init(void);
void magicFlute_appli(void);
void magicFlute_int(void);

/*----------------------------------------------------------------------------*/
//
//      Set Function
//
/*----------------------------------------------------------------------------*/
const InitFunc initFunc[MAX_INIT_FUNC_NUM] = {
	magicFlute_init
};
/*----------------------------------------------------------------------------*/
const AppliFunc appFunc[MAX_APPLI_FUNC_NUM] = {
	magicFlute_appli
};
/*----------------------------------------------------------------------------*/
const InterruptFunc intFunc[MAX_INT_FUNC_NUM] = {
	magicFlute_int
};
