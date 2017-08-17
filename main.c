/* 
 * File:   main.c
 * Author: takumi
 *
 * Created on 2016/12/28, 7:08
 */

#include <stdio.h>
#include <stdlib.h>
#include "skADXL345I2C.h"
#include "skI2Cmaster.h"
#include <math.h>
#define _XTAL_FREQ 32000000

// PIC16F1939 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


int Xangle, Yangle, j = 0, l = 0, i = 3, k = 7, m, wall, timer;
char buf[6];
int ans, point, pointa, maxa, maxb, maxc, count, flag, q = 1, another = 1;

char pi, pj;

void trans();
void setup();
void show();
void clear();
void start();
void end();
void fill(char);
void copy();
void swap(char[]);

void RandInit(int seed) {
    T4CON = 0b01111011;
    PR4 = 254;
    TMR4 = seed;
    T4CONbits.TMR4ON = 1;
}

char Rand(char div) {
    return TMR4 % div;
}


char data[8][8] = {
    {0, 0, 0, 0, 0, 0, 2, 0},
    {0, 0, 0, 0, 0, 2, 2, 1},
    {0, 0, 0, 0, 2, 2, 2, 1},
    {0, 0, 0, 2, 2, 2, 2, 1},
    {0, 0, 2, 2, 2, 2, 2, 1},
    {0, 2, 2, 2, 2, 2, 2, 1},
    {2, 2, 2, 2, 2, 2, 2, 1},
    {0, 1, 1, 1, 1, 1, 1, 1},
};

char copya[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 2, 0, 0, 0, 0, 0, 0},
    {0, 2, 0, 0, 0, 0, 0, 0},
    {0, 2, 2, 2, 2, 0, 0, 0},
    {0, 2, 0, 0, 0, 0, 0, 0},
    {0, 2, 0, 0, 0, 0, 0, 0},
    {0, 0, 2, 2, 2, 2, 2, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};

char sela[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 2, 0, 2, 0, 0},
    {0, 0, 2, 2, 1, 2, 2, 0},
    {0, 2, 2, 2, 1, 2, 0, 0},
    {2, 2, 2, 2, 1, 2, 2, 2},
    {0, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};

char selb[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 2, 0, 0, 0, 0},
    {0, 2, 0, 2, 0, 0, 0, 0},
    {2, 2, 1, 2, 2, 2, 0, 0},
    {0, 1, 1, 2, 0, 0, 0, 0},
    {0, 0, 0, 2, 2, 2, 2, 2},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};

char selc[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 2, 2, 2, 0, 0, 0, 0},
    {0, 2, 0, 0, 1, 1, 0, 0},
    {0, 2, 2, 0, 1, 1, 0, 0},
    {0, 2, 0, 0, 1, 1, 1, 0},
    {0, 2, 0, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};

char seld[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0, 0, 0, 0},
    {0, 1, 0, 0, 2, 2, 0, 0},
    {0, 1, 1, 0, 2, 2, 0, 0},
    {0, 1, 0, 0, 2, 2, 2, 0},
    {0, 1, 0, 0, 2, 0, 2, 0},
    {0, 0, 0, 0, 2, 2, 2, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};

void accele() {
    int x1, y1, z1, x2, y2, z2, x, y, z;

    x1 = 0;
    y1 = 0;
    z1 = 0;
    for (int lcoun = 0; lcoun < 10; lcoun++) {
        acceler_Read(&x, &y, &z);
        x1 += x;
        y1 += y;
        z1 += z;
    }
    x2 = x1 / 10;
    y2 = y1 / 10;
    z2 = z1 / 10;


    Xangle = (int) (atan2(x2 + 10, z2 - 4) / 3.14159 * 180.0); // X方向
    Yangle = (int) (atan2(y2 - 5, z2 - 4) / 3.14159 * 180.0); // Y方向
}

int ifcheck() {
    if (PORTBbits.RB2 == 0) {
        __delay_us(500);
        if (PORTBbits.RB2 == 0)return 1;
    }
    return 0;
}

void interrupt Timer(void) {
    if (SSPIF == 1) {
        if (AckCheck == 1) AckCheck = 0;
        SSPIF = 0; // ？t？？？O？N？？？A
    } else if (BCLIF == 1) {
        BCLIF = 0;
    } else if (T0IF == 1) {
        timer--;
        if (timer < 0)buf[5] = 0;
        if (another == 0) {
            j = (j + 1) % 10;
            l = (l + 1) % 400;
            m = (m + 1) % 20;
            if (j == 0) {
                if (Xangle + 180 > 181 && i != 7) {
                    i++;
                    point++;
                } else if (Xangle + 180 < 179 && i != 0) {
                    i--;
                    point++;
                }
                /*
                if (Yangle + 180 < 177 && k != 7) {
                    k++;
                } else if (Yangle + 180 > 183 && k != 0) {
                    k--;
                }
                 */
            }
            if (l == 0) {
                count++;
                buf[4] = 1;
            }
            if (m == 0) {
                q = 1 - q;
                point++;
                char a = TMR4 % 8;
                TMR4 -= point;
                char new[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                if (q == 1)new[a] = 1;

                for (int p = 6; p >= 0; p--) {
                    for (int o = 0; o < 8; o++) {
                        if (p == 6 && o == i) {
                            if (data[6][i] == 1) {
                                buf[5] = 3;
                                timer = 15;
                                point -= 10;
                                if (point < 0)point = 0;
                            }
                        } else data[p + 1][o] = data[p][o];
                        data[p][o] = 0;
                    }
                }
                for (int o = 0; o < 8; o++) {
                    data[0][o] = new[o];
                }
            }
            data[7][i] = 2;
            T0IF = 0;
        }
        if (another == 1) {
            T0IF = 0;
            j = (j + 1) % 10;
            l = (l + 1) % 400;
            m = (m + 1) % 20;
            if (j == 0) {
                if (Xangle + 180 > 181 && i != 7) {
                    i++;
                    pointa++;
                } else if (Xangle + 180 < 179 && i != 0) {
                    i--;
                    pointa++;
                }
                if (Yangle + 180 < 177 && k != 7) {
                    k++;
                    pointa++;
                } else if (Yangle + 180 > 183 && k != 0) {
                    k--;
                    pointa++;
                }
            }
            if (l == 0) {
                count++;
                buf[4] = 1;
            }
            if (k == pi && i == pj) {
                pointa += 60;
                timer = 15;
                buf[5] = 1;
                pi = TMR4 % 8;
                pj = TMR4 % 8;
                TMR4 -= point;
            }
            point = pointa / 3;
            clear();
            data[pi][pj] = 1;
            data[k][i] = 2;
        }

        if (another == 2) {
            wall = (wall + 1) % 400;
            j = (j + 1) % 10;
            l = (l + 1) % 400;
            m = (m + 1) % 50;
            if (j == 0) {
                data[k][1] = 0;
                if (ifcheck()) {
                    if (k != 0)k--;
                } else {
                    if (m == 0 && k != 7)k++;
                }
            }
            if (wall == 100 || wall == 200 || wall == 300 || wall == 0) {
                for (int di = 0; di < 8; di++) {
                    for (int dj = 1; dj < 7; dj++) {
                        if (data[dj][di] == 1) {
                            data[dj][di] = 0;
                            data[dj][di - 1] = 1;
                        }
                    }
                }
                for (int di = 1; di < 7; di++) {
                    if (data[di][1] == 1) {
                        point++;
                        timer = 15;
                        buf[5] = 1;
                    }
                }
                if (wall == 0) {
                    data[(TMR4 % 6) + 1][7] = 1;
                    for (int di = 1; di < 7; di++)data[di][6] = 0;
                    TMR4 -= point;
                }
            }
            //point = 100;
            for (int di = 1; di < 7; di++) {
                //if (data[di][7] == 1)point = 800;
            }
            if (data[k][1] == 1)count = 10;
            data[k][1] = 2;
            T0IF = 0;
        }
    }
}

int main(int argc, char** argv) {
    setup();

    OPTION_REG = 0b00000111;
    TMR0 = 0;
    T0IF = 0;
    T0IE = 0;


    InitI2C_Master2();
    acceler_Init();

    //eeprom_write(0,0);
    //eeprom_write(1,0);
    //eeprom_write(2,0);
    //eeprom_write(3,0);
    //eeprom_write(5,0);
    //eeprom_write(6,0);

    int seed = eeprom_read(4);
    RandInit(seed);

    point = 46;
    maxa = eeprom_read(0) * 256 + eeprom_read(1);
    maxb = eeprom_read(2) * 256 + eeprom_read(3);
    maxc = eeprom_read(5) * 256 + eeprom_read(6);


    another = 0;

    int precount = 0;

    while (1) {
        if (LATBbits.LATB3 == 0)LATBbits.LATB3 = 1;
        point = 0;
        precount++;
        precount %= 30;
        if (precount == 0)accele();
        if (Yangle<-80) {
            another = 2;
            for (int i = 0; i < 8; i++)for (int j = 0; j < 8; j++)data[i][j] = selc[i][j];
            show();
            trans();
            if (ifcheck()) {
                start();
                while (flag == 1) {
                    accele();
                    show();
                    trans();
                    if (count == 10)end();
                }
            }
        } else if (Xangle > 0) {
            for (int i = 0; i < 8; i++)for (int j = 0; j < 8; j++)data[i][j] = selb[i][j];
            another = 1;
            show();
            trans();
            if (ifcheck()) {
                start();
                while (flag == 1) {
                    accele();
                    show();
                    trans();
                    if (count == 10)end();
                }
            }
        } else {
            another = 0;
            for (int i = 0; i < 8; i++)for (int j = 0; j < 8; j++)data[i][j] = sela[i][j];
            show();
            trans();
            if (ifcheck()) {
                start();
                while (flag == 1) {
                    accele();
                    show();
                    trans();
                    if (count == 10)end();
                }
            }
        }
        /*if (another == 0) {
            start();
            while (flag == 1) {
                accele();
                show();
                trans();
            }
            another = 1 - another;
        } else {
            start();
            while (flag == 1) {
                accele();
                show();
                trans();
            }
            another = 1 - another;
        }*/
    }

    return (EXIT_SUCCESS);
}

void start() {
    LATBbits.LATB3 = 0;
    if (another == 0) {
        i = 3;
        point = 0;
        buf[4] = 46;
        count = 0;
        trans();
        flag = 1;
        copy();
        int wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 100;
        buf[5] = 1;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        buf[5] = 0;
        trans();

        clear();
        T0IE = 1;
    } else if (another == 1) {
        point = 0;
        pointa = 0;
        buf[4] = 46;
        count = 0;
        trans();
        flag = 1;
        for (int i = 0; i < 8; i++)for (int j = 0; j < 8; j++)data[i][j] = copya[i][j];
        int wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 100;
        buf[5] = 1;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        buf[5] = 0;
        trans();

        clear();

        pi = Rand(8);
        pj = Rand(8);
        data[pi][pj] = 1;
        data[4][4] = 2;
        if (pi == 4 && pj == 4) data[5][5] = 2;

        T0IE = 1;
    } else {
        LATBbits.LATB3 = 1;
        point = 0;
        pointa = 0;
        buf[4] = 46;
        count = 0;
        trans();
        flag = 1;
        for (int i = 0; i < 8; i++)for (int j = 0; j < 8; j++)data[i][j] = seld[i][j];
        int wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 2;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 60;
        buf[5] = 0;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        wait = 100;
        buf[5] = 1;
        trans();
        while (wait > 0) {
            show();
            wait--;
        }
        buf[5] = 0;
        trans();

        clear();
        k = 4;

        for (int di = 0; di < 8; di++) {
            data[0][di] = 1;
            data[7][di] = 1;
        }

        T0IE = 1;
    }
}

void end() {
    flag = 0;
    T0IE = 0;
    if (another == 0) {
        if (point > maxa) {
            maxa = point;

            buf[0] = point / 256;
            buf[1] = point % 256;
            buf[2] = maxa / 256;
            buf[3] = maxa % 256;
            ans = I2C_Send2(46, 5, buf);
            buf[4] = 0;

            eeprom_write(0, maxa / 256);
            eeprom_write(1, maxa % 256);
            fill(2);
            int wait = 460;
            buf[5] = 1;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        } else {
            fill(1);
            int wait = 460;
            buf[5] = 3;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        }
    } else if (another == 1) {
        if (point > maxb) {
            maxb = point;

            buf[0] = point / 256;
            buf[1] = point % 256;
            buf[2] = maxb / 256;
            buf[3] = maxb % 256;
            ans = I2C_Send2(46, 5, buf);
            buf[4] = 0;

            eeprom_write(2, maxb / 256);
            eeprom_write(3, maxb % 256);
            fill(2);
            int wait = 460;
            buf[5] = 1;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        } else {
            fill(1);
            int wait = 460;
            buf[5] = 3;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        }
    } else {
        if (point > maxc) {
            maxc = point;

            buf[0] = point / 256;
            buf[1] = point % 256;
            buf[2] = maxc / 256;
            buf[3] = maxc % 256;
            ans = I2C_Send2(46, 5, buf);
            buf[4] = 0;

            eeprom_write(5, maxc / 256);
            eeprom_write(6, maxc % 256);
            fill(2);
            int wait = 460;
            int wait = 460;
            buf[5] = 1;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        } else {
            fill(1);
            int wait = 460;
            buf[5] = 3;
            trans();
            while (wait > 0) {
                show();
                wait--;
            }
            buf[5] = 0;
            trans();
        }
    }

    eeprom_write(4, TMR4 % 256);

    count = 0;

    return;
}

void setup() {
    OSCCON = 0b01110000;
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELE = 0x00;
    TRISA = 0x00;
    TRISB = 0b00000100;
    TRISC = 0b00011000;
    TRISD = 0x00;
    TRISE = 0x00;
    WPUBbits.WPUB2 = 1;
    OPTION_REGbits.nWPUEN = 1;

    LATBbits.LATB4 = 0;
    LATBbits.LATB3 = 1;

    T0IF = 0;
    return;
}

void show() {


    LATC1 = 0;
    LATC0 = 1;
    LATC1 = 1;
    LATC1 = 0;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (data[i][j] == 1) { //red
                LATD -= (1 << j);
            } else if (data[i][j] == 2) { //green
                LATA -= (1 << j);
            } else { //none
            }
        }
        __delay_us(700);

        LATA = 0xFF;
        LATD = 0xFF;


        LATC0 = 0;
        LATC1 = 1;
        LATC1 = 0;
    }

    return;
}

void clear() {
    for (int color = 0; color < 8; color++)for (int j = 0; j < 8; j++)data[color][j] = 0;
}

void fill(char color) {
    for (int colori = 0; colori < 8; colori++)for (int j = 0; j < 8; j++)data[colori][j] = color;
}

void trans() {
    if (another == 0) {
        buf[0] = point / 256;
        buf[1] = point % 256;
        buf[2] = maxa / 256;
        buf[3] = maxa % 256;
    } else if (another == 1) {
        buf[0] = point / 256;
        buf[1] = point % 256;
        buf[2] = maxb / 256;
        buf[3] = maxb % 256;
    } else {
        buf[0] = point / 256;
        buf[1] = point % 256;
        buf[2] = maxc / 256;
        buf[3] = maxc % 256;
    }
    ans = I2C_Send2(46, 6, buf);
    buf[4] = 0;

    return;
}

void copy() {
    data[0][0] = 0;
    data[0][1] = 0;
    data[0][2] = 0;
    data[0][3] = 0;
    data[0][4] = 0;
    data[0][5] = 0;
    data[0][6] = 2;
    data[0][7] = 0;
    data[1][0] = 0;
    data[1][1] = 0;
    data[1][2] = 0;
    data[1][3] = 0;
    data[1][4] = 0;
    data[1][5] = 2;
    data[1][6] = 2;
    data[1][7] = 1;
    data[2][0] = 0;
    data[2][1] = 0;
    data[2][2] = 0;
    data[2][3] = 0;
    data[2][4] = 2;
    data[2][5] = 2;
    data[2][6] = 2;
    data[2][7] = 1;
    data[3][0] = 0;
    data[3][1] = 0;
    data[3][2] = 0;
    data[3][3] = 2;
    data[3][4] = 2;
    data[3][5] = 2;
    data[3][6] = 2;
    data[3][7] = 1;
    data[4][0] = 0;
    data[4][1] = 0;
    data[4][2] = 2;
    data[4][3] = 2;
    data[4][4] = 2;
    data[4][5] = 2;
    data[4][6] = 2;
    data[4][7] = 1;
    data[5][0] = 0;
    data[5][1] = 2;
    data[5][2] = 2;
    data[5][3] = 2;
    data[5][4] = 2;
    data[5][5] = 2;
    data[5][6] = 2;
    data[5][7] = 1;
    data[6][0] = 2;
    data[6][1] = 2;
    data[6][2] = 2;
    data[6][3] = 2;
    data[6][4] = 2;
    data[6][5] = 2;
    data[6][6] = 2;
    data[6][7] = 1;
    data[7][0] = 0;
    data[7][1] = 1;
    data[7][2] = 1;
    data[7][3] = 1;
    data[7][4] = 1;
    data[7][5] = 1;
    data[7][6] = 1;
    data[7][7] = 1;
}
