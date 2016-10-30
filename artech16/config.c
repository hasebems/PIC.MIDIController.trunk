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
void Artech16_init(void);
void Artech16_appli(void);
void Artech16_intrpt(void);

/*----------------------------------------------------------------------------*/
void dummy(void){}

/*----------------------------------------------------------------------------*/
//
//      Set Function
//
/*----------------------------------------------------------------------------*/
const InitFunc initFunc[MAX_INIT_FUNC_NUM] = {
	Artech16_init,
};
/*----------------------------------------------------------------------------*/
const AppliFunc appFunc[MAX_APPLI_FUNC_NUM] = {
	Artech16_appli,
};
/*----------------------------------------------------------------------------*/
const InterruptFunc intFunc[MAX_INT_FUNC_NUM] = {
	Artech16_intrpt,
};
