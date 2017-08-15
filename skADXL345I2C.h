// skADXL345I2C.h
// ＡＤＸＬ３４５関数ライブラリ用インクルードファイル(I2Cバージョン)
#ifndef _SKADXL345I2C_H_
#define _SKADXL345I2C_H_

#ifndef _XTAL_FREQ
 // Unless already defined assume 8MHz system frequency
 // This definition is required to calibrate __delay_us() and __delay_ms()
 #define _XTAL_FREQ 32000000       // 使用するPIC等により動作周波数値を設定する
#endif

// Ｉ２Ｃのアドレス
#define ADXL345_ADRES 0x1D        // ADXL345のアドレス(ALT ADDRESSピンがHIGHの時)
// レジスタのアドレス
#define DEVID         0x00        // 固定のデバイスIDコードを格納
#define BW_RATE       0x2C        // データレートおよび電力モード制御
#define DATA_FORMAT   0x31        // データフォーマット制御
#define FIFO_CTL      0x38        // FIFO制御
#define POWER_CTL     0x2D        // 省電力機能を制御
#define DATA_X0       0x32        // 読み出すデータ(X軸-LOW)の位置

#define DEVID_VALUE   0xE5        // 固定のデバイスIDコードの内容


int acceler_Init() ;
int acceler_Read(int *X, int *Y, int *Z) ;

#endif
