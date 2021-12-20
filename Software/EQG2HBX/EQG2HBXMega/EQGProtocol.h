/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/

#include <Arduino.h>

#ifndef EQGProtocol
#define EQGProtocol

#define CR      0x0d
#define LF      0x0a

float SIDEREALSECS        = 86164.091;         // Some astronomical constants
float SOLARSECS           = 86400;
float LUNARSECS           = 89309;


#define EQG_CMNDSTART     0x01
#define EQG_WAITFORCR     0x77
#define EQG_INTERPRET     0x78

#define EQGVERSION        0x000402    // Simulate EQ6

// :I := ( :b * 1296000 / :a ) / Speed    ( where Speed is in arcsec/sec )
// If :I is greater than about 10, then the slew will need to use :G = LoSpeed mode
// If :I is less than 10, then the slew will need :G = HiRate, and :I := I * :g
//    a-AxxValue (Ticks/rev)  := AxxVanes * 4 * AxxGbxRatio * ( Axx Transfer ) * AxxWormTeeth 
//    b-AxxValue              := 6460.09 * AxxRatio * a-AxxValue * 15.041069 / 1,296,000

// Speed = g*(b*129600/a)/I
// ==============================
// IVALUE = (axis[EQGMOTOR].bVALUE * 1296000) / axis[EQGMOTOR].STEPSPER360)

#define EQG_AzCENTRE      ETX_AzCENTRE 
#define EQG_AltCENTRE     ETX_AltCENTRE
#define EQG_gVALUE        0x000010

#define EQGMAXIMUMSPEED   12          // 0x0C

// EQG 'G' Command      - SET move parameters
#define DIRECTION       0x00000001      // Increasing(0)  Decreasing(1)
#define HEMISPHERE      0x00000002      // North(0)       South(1)     

// EQG 'f' Command      - GET Motor status bit definitions
//         Get axis tracking/slewing "status"    // =ABC[0D]
                                                 // A  xxx0      0 means GOTO,    1 means SLEW           ***  these are diff to :G usage
                                                 //              0 means "actually doing" the goto. On stopping, it reverts to Slew Mode
                                                 //    xx0x      0 means  +ve,    1 means  -ve
                                                 //    x0xx      0 means LoRate,  1 means HiSpeed        ***
                                                 // B  xxx0      0 means stopped, 1 means moving,
                                                 //    xx0x      0 means OK,      1 means blocked   ( For DC motors only )
                                                 // C  xxx0      1 means axis is Initialised/Energised
                                                 //    xx0x      1 means level switch ON            ( AltAz mounts and DEC only )
                                                 
// MotorState bit definitions
// A: nibble 1
#define MOVESLEW        0x0001      // Step(0)        Slew(1)
#define MOVEDECR        0x0002      // Increasing(0)  Decreasing(1)
#define MOVEHIGH        0x0004      // Low(0)         High(1)
// B: nibble2
#define MOVEAXIS        0x0010      // Stopped(0)     Moving(1)
#define MOVEFACE        0x0020      // Front(0)       Rear(1)
// C: nibble3
#define MOVEACTIVE      0x0100      // Inactive(0)    Active(1)

void EQGState(void);
void EQGError(unsigned char);
void EQGAction(void);

void TimerDelaymS(unsigned long);
  
bool EQGRx(void);
void EQGTx(unsigned char);
void EQGTxHex(unsigned char);
void EQGTxHex2(unsigned char);
void EQGTxHex3(unsigned int);
void EQGTxHex6(unsigned long);

// debug
void putbyte(unsigned char);
void putbyte(unsigned char);
void puthexn(unsigned char);
void puthexb(unsigned char);
void puthexw(unsigned int);
void puthexl(unsigned long);
void putdecn(unsigned char);
void putdecb(unsigned char);
void putdecw(unsigned int);
void putdecl(unsigned long);
#endif

