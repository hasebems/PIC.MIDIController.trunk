/*
 * File:   config.h
 * Author: jca03205
 *
 * Created on 2015/01/31, 9:13
 */

#ifndef MFCONFIG_H
#define	MFCONFIG_H

//  Set Application
#define     HONEYCOMB_BELL
//#define     TOUCH_MIDI

//  Max Application Num
#define     MAX_INIT_FUNC_NUM               1
#define     MAX_APPLI_FUNC_NUM              1
#define     MAX_INT_FUNC_NUM                1


#define     MF_FIRM_VERSION                 0

//  Set H/W
#define     USE_I2C_LPS331AP                0   //  Pressure Sensor
#define     USE_I2C_MPR121                  0   //  Touch Sensor
#define		USE_I2C_CY8CMBR3110             1
#define     USE_I2C_ADXL345                 0   //  Accelerator Sensor
#define     USE_I2C_PCA9685                 1   //  LED Driver
#define     USE_I2C_ADS1015                 1   //  ADC
#define     USE_I2C_ACM1602N1               1   //  LCD Module

#endif	/* MFCONFIG_H */
