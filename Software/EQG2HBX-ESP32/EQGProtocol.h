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

float SIDEREALSECS        = 86164.098903691;         // Some astronomical constants
float SOLARSECS           = 86400;
float LUNARSECS           = 89309;


#define EQG_CMNDSTART     0x01
#define EQG_WAITFORCR     0x77
#define EQG_INTERPRET     0x78

/*
//	Get Motor Controller Version 
//	:e1[0D]
//	=llhhMM[0D]
//	===========
e  0    6  Get Motor Controller Version          // =llhhMM[0D]  MM = mount type,
																								 //             x00 = "EQ6Pro"
																								 //             x01 = "HEQ5"
																								 //             x02 = "EQ5"
																								 //             x03 = "EQ3"
																								 //             x04 = "EQ8"
																								 //             x05 = "AZEQ6"
																								 //             x06 = "AZEQ5"
																								 //             hh.ll = board version    hh=x00..x07 = equatorial
																								 //                                        =x08..xFF = altaz
*/
// :qm

// A    8  
//      4  
//      2  in PPEC
//      1  in PPECTraining

// B    8 is AZEQ
//      4 has HomeIndexer
//      2 has PPEC
//      1 has Encoder

// C    8	has Wifi
//      4	hasHalfCurrentTracking							// ref :Wx06....
//      2	has CommonSlewStart									// Supports ":J3"
//      1 has PolarLed
/*
// A
AxisFeatures[Axis1].inPPECTraining = rafeatures & 0x00000010;
AxisFeatures[Axis1].inPPEC = rafeatures & 0x00000020;
// B
AxisFeatures[Axis1].hasEncoder = rafeatures & 0x00000001;
AxisFeatures[Axis1].hasPPEC = rafeatures & 0x00000002;
AxisFeatures[Axis1].hasHomeIndexer = rafeatures & 0x00000004;
AxisFeatures[Axis1].isAZEQ = rafeatures & 0x00000008;
// C
AxisFeatures[Axis1].hasPolarLed = rafeatures & 0x00001000;
AxisFeatures[Axis1].hasCommonSlewStart = rafeatures & 0x00002000; // supports :J3
AxisFeatures[Axis1].hasHalfCurrentTracking = rafeatures & 0x00004000;
AxisFeatures[Axis1].hasWifi = rafeatures & 0x00008000;
*/
typedef struct SkyWatcherFeatures
{
	bool inPPECTraining = false;
	bool inPPEC = false;
	bool hasEncoder = false;
	bool hasPPEC = false;

	bool hasHomeIndexer = false;
	bool isAZEQ = false;
	bool hasPolarLed = false;
	bool hasCommonSlewStart = false; // supports :J3

	bool hasHalfCurrentTracking = false;
	bool hasWifi = false;
} SkyWatcherFeatures;

enum SkywatcherGetFeatureCmd
{
	GET_INDEXER_CMD = 0x00,
	GET_FEATURES_CMD = 0x01
};

enum SkywatcherSetFeatureCmd
{
	START_PPEC_TRAINING_CMD = 0x00,
	STOP_PPEC_TRAINING_CMD = 0x01,
	TURN_PPEC_ON_CMD = 0x02,
	TURN_PPEC_OFF_CMD = 0X03,
	ENCODER_ON_CMD = 0x04,
	ENCODER_OFF_CMD = 0x05,
	DISABLE_FULL_CURRENT_LOW_SPEED_CMD = 0x0006,
	ENABLE_FULL_CURRENT_LOW_SPEED_CMD = 0x0106,
	RESET_HOME_INDEXER_CMD = 0x08
};

// Get Motor Controller Assets
//	:qm010000[0D] 
//	=ABCDEF[0D]
//	============
// A    8  not defined
//      4  not defined
//      2  PPEC ON
//      1  PPEC training in progress,
// B    8  supports AZ/EQ
//      4  has Home Sensors
//      2  supports PPEC
//      1  supports dual encoders
// C    8  has WIFI
//      4  supports half current tracking          // ref :Wx06....
//      2  axes slews must start independently     // ie cant use :J3
//      1  has polar LED
// D
// E
// F
// EQ6    returns !0
//								 ABCDEF
// AZEQ5          =0B6000  at boot
// AZEQ6          =0B3000
// EQ8            =076000
//								EFCDAB
#define AEQ6				0x003000				// !:J3,  Polar LED
#define AEQ5				0x003008				// !:J3,  Polar LED, az/eq
#define AEQ3				0x003000				// !:J3,  Polar LED
#define AAZEQ5			0x00B008				// !:J3,  Polar LED, WiFi, AZ/EQ

// Motor firmware versions
#define VEQ6        0x000204    // Pretend EQ6/5
#define VHEQ5       0x010204    // Pretend HEQ5
#define VEQ5        0x020204    // Pretend EQ5
#define VEQ3        0x030301    // Pretend EQ3
#define VEQ8        0x040301    // Pretend EQ3
#define VAZEQ6      0x050211		// Pretend AZEQ6
#define VAZEQ5      0x060301		// Pretend AZEQ5

#define EQGVERSION	VEQ6
#define EQGASSETS		AEQ6


// :I := ( :b * 1296000 / :a ) / Speed    ( where Speed is in arcsec/sec )
// If :I is greater than about 10, then the slew will need to use :G = LoSpeed mode
// If :I is less than 10, then the slew will need :G = HiRate, and :I := I * :g
//    a-AxxValue (Ticks/rev)  := AxxVanes * 4 * AxxGbxRatio * ( Axx Transfer ) * AxxWormTeeth 
//    b-AxxValue              := 6460.09 * AxxRatio * a-AxxValue * 15.041069 / 1,296,000

// Speed = g*(b*129600/a)/I
// ==============================
// IVALUE = (axis[EQGMOTOR].bVALUE * 1296000) / axis[EQGMOTOR].STEPSPER360)

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

