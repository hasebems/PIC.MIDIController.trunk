/*
 * File:   config.h
 * Author: jca03205
 *
 * Created on January 1, 2016
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

//  Set Application
#define     MAGIC_FLUTE


//  Max Application Num
#define     MAX_INIT_FUNC_NUM               1
#define     MAX_APPLI_FUNC_NUM              1
#define     MAX_INT_FUNC_NUM                1

#define     MF_FIRM_VERSION                 0

//  Set H/W
#define     USE_I2C_LPS25H					1   //  Pressure Sensor
#define     USE_I2C_MPR121                  0   //  Touch Sensor
#define		USE_I2C_CY8CMBR3110             1   //  Touch Sensor(Cypress)
#define     USE_I2C_ADXL345                 1   //  Accelerator Sensor
#define     USE_I2C_PCA9685                 0   //  LED Driver
#define     USE_I2C_ADS1015                 0   //  ADC
#define     USE_I2C_ACM1602N1               0   //  LCD Module

#ifdef	__cplusplus
}
#endif

#endif	/* MFCONFIG_H */
