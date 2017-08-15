// skI2Cmaster.h
// ？h？Q？b？？？M？？？}？X？^？[？？？？？p？？？C？u？？？？？p？C？？？N？？？[？h？t？@？C？？
#ifndef _SKI2CMASTER_H_
#define _SKI2CMASTER_H_

int AckCheck;

void InitI2C_Master2() ;
int  I2C_Send2(unsigned char adrs,int len,char *buf) ;
int  I2C_Receive2(unsigned char adrs,int len,char *buf) ;


#endif
