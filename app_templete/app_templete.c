/*
 * File:   app_templete.c
 * Author: Masahiko HASEBE
 *
 * Created on February xx, 2016, xx:xx PM
 */


#include <xc.h>
#include "hsb_common.h"
#include "config.h"

//  ●新しい PIC MIDI Project の作り方

//  1.まずこのファイルをベースにアプリを作成
//      変数、初期化関数、割り込み関数の作成

//  2.新しいアプリの登録
//      config.h に、アプリのマクロ作成
//      そのマクロで、関連処理、関数を囲む
//      必要なハードウェアの ON/OFF

//  3.関数の登録
//      config.c に、1.の関数を登録

//  4.アプリで使用できるサービスは hsb_common.h に
//      hsb_common.h にあるサービスを使って、アプリを作成


//  ●新しい I2C ハードウェアドライバの作り方

//  1.i2cdevice.c に初期化とサービスをコーディング

//  2.i2cdevice.h に上記サービスの extern 追加

//  3.マクロを決めて、config.h に記載
//      コンパイル時に無駄にコンパイルされないようマクロで囲む


//  ●Trunk の Framework 修正

//  1.サービスを追加
//      必要に応じて main.c や hsb_common.h に処理を付け加え、サービス追加

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
//	static xxx

/*----------------------------------------------------------------------------*/
//
//      Init function
//
/*----------------------------------------------------------------------------*/
void Templete_init(void)
{
}
/*----------------------------------------------------------------------------*/
//
//      Application function
//
/*----------------------------------------------------------------------------*/
void Templete_appli(void)
{

}
