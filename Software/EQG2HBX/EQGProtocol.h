/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/
#ifndef EQGProtocol
#define EQGProtocol

#define CR      0x0d
#define LF      0x0a

#define SIDEREALSECS      86164         // Some astronomical constants
#define SOLARSECS         86400
#define LUNARSECS         89309
#define ARCSECS           360*60*60

#define EQG_CMNDSTART     0x01
#define EQG_WAITFORCR     0x77
#define EQG_INTERPRET     0x78

#define EQGVERSION        0x000501    // Simulate EQ6

#define EQG_CENTRE        ETX_CENTRE
#define EQG_RAbVALUE      AzSIDEREALRATE*(AzSTEPSPER360/SIDEREALSECS)
#define EQG_DECbVALUE     AltSIDEREALRATE*(AzSTEPSPER360/SIDEREALSECS)
#define EQG_gVALUE        0x00000010

#define EQGMAXIMUMSPEED   12          // 0x0C

// EQG 'G' Command      - SET move parameters
#define DIRECTION       0x0001      // N/E(0)       S/W(1)
//#define HEMISPHERE      0x0002      // North(0)     South(1)
//#define SLEWSTEP        0x0010      // Slew(0)      Step(1)
//#define HIGHLOW         0x0020      // High(0)      Low(1)       

// 3 HIGH SPEED SLEW
// 2 LOW  SPEED GOTO
// 1 LOW  SPEED SLEW
// 0 HIGH SPEED GOTO

// EQG 'f' Command      - GET Motor status bit definitions
// MotorState bit definitions
// nibble 1
#define MOVESTEP        0x0001      // Slew(0)        Step(1)
#define MOVEDIRN        0x0002      // Decreasing(0)  Increasing(1)
#define MOVELOW         0x0004      // High(0)        Low(1)
// nibble2
#define MOVEAXIS        0x0010      // Stopped(0)     Moving(1)
#define MOVEFACE        0x0020      // Front(0)       Rear(1)
// nibble3
#define COILACTIVE      0x0100      // Inactive(0)    Active(1)

#define FORWARD         MOVEDIRN
#define REVERSE         0

void EQGState(void);
void EQGError(unsigned char);
void EQGAction(void);

void TimerDelaymS(unsigned long);
  
void EQGRx(void);
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
