/*******************************************************************************
*  skI2Cmaster - ？h？Q？b？？？M？？？s？？？}？X？^？[？p？？？？？？？？？C？u？？？？                    *
*                                                                              *
*    InterI2C       - ？h？Q？b？？？A？？？？？？？？？？？？？？                                 *
*    InitI2C_Master - ？h？Q？b？？？M？？？}？X？^？[？？？[？h？？？？？？？？？？？s？？？？？？             *
*    I2C_Send      - ？X？？？[？u？？？f？[？^？？？w？？？？？？？？？？？？？？？？？M？？？？？？？？            *
*    I2C_Receive   - ？X？？？[？u？？？？？f？[？^？？？w？？？？？？？？？？？？？？？？？M？？？？？？？？          *
*                                                                              *
*    ？？？？？FSDA/SCL？s？？？？？K？？？u？f？W？^？？？？？？？s？？？v？？？？？？？？？s？？？？？？？？？？？B         *
*          ？？？？I2C？？？？？？？C？u？？？？？？？V？r？b？g？A？h？？？X？？？[？h？？？？？？？？？？？？？B         *
* ============================================================================ *
*  VERSION DATE        BY                    CHANGE/COMMENT                    *
* ---------------------------------------------------------------------------- *
*  1.00    2011-12-23  ？？？？？？？H？[(？？？？？？？？)  Create                            *
*  2.00    2012-01-21  ？？？？？？？H？[(？？？？？？？？)  ？X？？？[？u？？？？？？ACK？？？？？？？？？？？？？X   *
* ============================================================================ *
*  PIC 12F1822 16F1827                                                         *
*  MPLAB IDE(V8.63)                                                            *
*  HI-TECH C Compiler for PIC10/12/16 MCUs Version 9.80 in Lite mode           *
*******************************************************************************/
#include <pic.h>
#include "skI2Clib.h"

int AckCheck=0;

// ？A？C？h？？？？？？？？？`？F？b？N
// ACKEN RCEN PEN RSEN SEN R/W BF ？？？S？？？O？？？？？n？j
void I2C_IdleCheck2(char mask)
{
     while (( I2C_SSPCON2 & 0x1F ) | (I2C_SSPSTAT & mask)) ;
}
/*******************************************************************************
*  interrupt InterI2C( void )                                                  *
*    ？h？Q？b？？？A？？？？？？？？？？？？？？                                                  *
*******************************************************************************/
/*******************************************************************************
*  InitI2C_Master()                                                            *
*    ？h？Q？b？？？M？？？}？X？^？[？？？[？h？？？？？？？？？？？s？？？？？？                              *
*                                                                              *
*   ？？)CPU？？？？(FOSC)8MHz？？？？？？？？？？？？？A？？？？？？？？？？SSP1ADD？？？？？X？？？？？K？v？？？L？？？？？？*
*******************************************************************************/
void InitI2C_Master2()
{
     SSPSTAT= 0b10000000 ;   // ？W？？？？？x？？？[？h？？？？？？？？？？(100kHz)
     SSPCON1= 0b00101000 ;   // SDA/SCL？s？？？？I2C？？？g？p？？？A？}？X？^？[？？？[？h？？？？？？
     SSPADD = 0x52       ;   // ？N？？？b？N=FOSC/((SSPADD + 1)*4) 8MHz/((0x13+1)*4)=0.1(100KHz)
     SSPIE = 1 ;             // SSP(I2C)？？？？？？？？？？？？？？？？？？
     BCLIE = 1 ;             // MSSP(I2C)？o？X？？？？？？？？？？？？？？？？？？？？？？
     PEIE   = 1 ;             // ？？？？？？？u？？？？？？？？？？？？？？？？？？
     GIE    = 1 ;             // ？S？？？？？？？？？？？？？？？？？？？？？？ 
     SSPIF = 0 ;             // SSP(I2C)？？？？？？？？？t？？？O？？？N？？？A？？？？
     BCLIF = 0 ;             // MSSP(I2C)？o？X？？？？？？？？？？？？？t？？？O？？？N？？？A？？？？
}
/*******************************************************************************
*  ans = I2C_Send(adrs,len,buf)                                                *
*    ？X？？？[？u？？？f？[？^？？？w？？？？？？？？？？(len)？？？？？？？M？？？？？？？？                       *
*                                                                              *
*    adrs : ？X？？？[？u？？？A？h？？？X？？？w？？？？？？(8？`119？？？？？？)                         *
*    len  : ？？？M？？？？？f？[？^(？o？C？g)？？？？？？？？？w？？？？？？                             *
*    buf  : ？？？M？？？？？f？[？^？？？i？[？？？？？z？？？？？w？？？？？？                             *
*    ans  : 0=？？？？？@1=？X？？？[？u？？？？？？？？？？？？？@2=？？？？？？？f？[？^？？？？？M？？？？？？？？？？     *
*******************************************************************************/
int I2C_Send2(unsigned char adrs,int len,char *buf)
{
     int i , ans ;

     // ？X？^？[？g(START CONDITION)
     I2C_IdleCheck2(0x5) ;
     SSPCON2bits.SEN = 1 ;
     // [？X？？？[？u？？？A？h？？？X+？X？？？[？u？？？？？M？？？v？？]？？？？？M？？？？
     I2C_IdleCheck2(0x5) ;
     AckCheck = 1 ;
     SSPBUF = (char)(adrs<<1) ;             // ？A？h？？？X？？？？？M R/W=0
     while (AckCheck) ;                      // ？？？？？？？？？？ACK？？？？？？？？？？
     ans = SSPCON2bits.ACKSTAT ;
     if (ans == 0) {
          // [？f？[？^]？？？？？M？？？？
          for (i=0 ; i<len ; i++) {
               I2C_IdleCheck2(0x5) ;
               AckCheck = 1 ;
               SSPBUF = (char)*buf ;        // ？f？[？^？？？？？M
               buf++ ;
               while (AckCheck) ;            // ？？？？？？？？？？ACK？？？？？？？？？？
               ans = SSPCON2bits.ACKSTAT ;
               if (ans != 0) {
                    ans = 2 ;                // ？？？？？？NOACK？？？？？？？？
                    break ;
               }
          }
     }
     // ？X？g？b？v(STOP CONDITION)
     I2C_IdleCheck2(0x5) ;
     SSPCON2bits.PEN = 1 ;
     return ans ;
}
/*******************************************************************************
*  ans = I2C_Receive(adrs,len,buf)                                             *
*    ？X？？？[？u？？？？？f？[？^？？？w？？？？？？？？？？(len)？？？？？？？M？？？？？？？？                     *
*                                                                              *
*    adrs : ？X？？？[？u？？？A？h？？？X？？？w？？？？？？(8？`119？？？？？？)                         *
*    len  : ？？？M？？？？？f？[？^(？o？C？g)？？？？？？？？？w？？？？？？                             *
*    buf  : ？？？M？？？？？f？[？^？？？i？[？？？？？z？？？？？w？？？？？？                             *
*    ans  : 0=？？？？？@1=？X？？？[？u？？？？？？？？？？？？                                     *
*******************************************************************************/
int I2C_Receive2(unsigned char adrs,int len,char *buf)
{
	unsigned dt ;
     int i , ans ;
     
     // ？X？^？[？g(START CONDITION)
     I2C_IdleCheck2(0x5) ;
     SSPCON2bits.SEN = 1 ;
     // [？X？？？[？u？？？A？h？？？X+？X？？？[？u？？？f？[？^？v？？]？？？？？M？？？？
     I2C_IdleCheck2(0x5) ;
     AckCheck = 1 ;
     SSPBUF = (char)((adrs<<1)+1) ;    // ？A？h？？？X？？？？？M R/W=1
     while (AckCheck) ;                 // ？？？？？？？？？？ACK？？？？？？？？？？
     ans = SSPCON2bits.ACKSTAT ;
     if (ans == 0) {
          for (i=1 ; i<=len ; i++) {
               // [？f？[？^]？？？？？M？？？？
               I2C_IdleCheck2(0x5) ;
               SSPCON2bits.RCEN = 1 ;  // ？？？M？？？？？？？？？？
               I2C_IdleCheck2(0x4) ;
               *buf = SSPBUF ;         // ？？？M
               buf++ ;
               I2C_IdleCheck2(0x5) ;
               if (i=len) SSPCON2bits.ACKDT = 1 ; // ACK？f？[？^？？NOACK
               else       SSPCON2bits.ACKDT = 0 ; // ACK？f？[？^？？ACK
               SSPCON2bits.ACKEN = 1 ;            // ACK？f？[？^？？？？？？
          }
     }
     // ？X？g？b？v(STOP CONDITION)
     I2C_IdleCheck2(0x5) ;
     SSPCON2bits.PEN = 1 ;
     return ans ;
}
