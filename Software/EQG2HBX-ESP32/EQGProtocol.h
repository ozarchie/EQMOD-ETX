/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/

#pragma once

#define CR      0x0d
#define LF      0x0a

float SIDEREALSECS        = 86164.098903691;         // Some astronomical constants
float SOLARSECS           = 86400;
float LUNARSECS           = 89309;

#define SKYWATCHER_SIDEREAL_DAY   86164.09053083288
#define SKYWATCHER_SIDEREAL_SPEED 15.04106864
#define SKYWATCHER_STELLAR_DAY    86164.098903691
#define SKYWATCHER_STELLAR_SPEED  15.041067179

#define EQG_CMNDSTART     0x01
#define EQG_WAITFORCR     0x77
#define EQG_INTERPRET     0x78

/*
	tmpMCVersion = Revu24str2long(response + 1);
	MCVersion = ((tmpMCVersion & 0xFF) << 16) | ((tmpMCVersion & 0xFF00)) | ((tmpMCVersion & 0xFF0000) >> 16);
	MountCode = MCVersion & 0xFF;
*/

/*
//	Get Motor Controller Version 
//	:e1[0D]
//	="llhhMM"[0D]							(6 bytes - hex		encoded)
//	Revu24str2long = MMhhll		(3 bytes - binary encoded)
//  MCVersion = llhhMM				(3 bytes - binary encoded)
//  MountCode = MM						(1 byte  - binary encoded)
//	=================================================================================
e  0    6  Get Motor Controller Version          // ="llhhMM"[0D]  MM = mount code,
																								 //             x00 = "EQ6Pro"
																								 //             x01 = "HEQ5"
																								 //             x02 = "EQ5"
																								 //             x03 = "EQ3"
																								 //             x04 = "EQ8"
																								 //             x05 = "AZEQ6"
																								 //             x06 = "AZEQ5"
																								 //             x80 = "GT"
																								 //             x81 = "MF"
																								 //             x82 = "114GT"
																								 //             x90 = "DOB"
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
// Types
enum SkywatcherCommand
{
	Initialize = 'F',
	InquireMotorBoardVersion = 'e',
	InquireGridPerRevolution = 'a',
	InquireTimerInterruptFreq = 'b',
	InquireHighSpeedRatio = 'g',
	InquirePECPeriod = 's',
	InstantAxisStop = 'L',
	NotInstantAxisStop = 'K',
	SetAxisPositionCmd = 'E',
	GetAxisPosition = 'j',
	GetAxisStatus = 'f',
	SetSwitch = 'O',
	SetMotionMode = 'G',
	SetGotoTargetIncrement = 'H',
	SetBreakPointIncrement = 'M',
	SetGotoTarget = 'S',
	SetBreakStep = 'U',
	SetStepPeriod = 'I',
	StartMotion = 'J',
	GetStepPeriod = 'D', // See Merlin protocol http://www.papywizard.org/wiki/DevelopGuide
	ActivateMotor = 'B', // See eq6direct implementation http://pierre.nerzic.free.fr/INDI/
	SetST4GuideRateCmd = 'P',
	GetHomePosition = 'd', // Get Home position encoder count (default at startup)
	SetFeatureCmd = 'W', // EQ8/AZEQ6/AZEQ5 only
	GetFeatureCmd = 'q', // EQ8/AZEQ6/AZEQ5 only
	InquireAuxEncoder = 'd', // EQ8/AZEQ6/AZEQ5 only
	NUMBER_OF_SkywatcherCommand
};

enum SkywatcherAxis
{
	Axis1 = 0, // RA/AZ
	Axis2 = 1, // DE/ALT
	NUMBER_OF_SKYWATCHERAXIS
};
char AxisCmd[2]{ '1', '2' };

enum SkywatcherDirection
{
	BACKWARD = 0,
	FORWARD = 1
};
enum SkywatcherSlewMode
{
	SLEW = 0,
	GOTO = 1
};
enum SkywatcherSpeedMode
{
	LOWSPEED = 0,
	HIGHSPEED = 1
};

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
// D		0000
// E		0000
// F		0000
// EQ6    returns !0
//								 ABCDEF
// AZEQ5          =0B6000  at boot
// AZEQ6          =0B3000
// EQ8            =076000
//								 EFCDAB

// Mount Assets
//										EFCDAB
#define AEQ6				0x003000				//			 !:J3, PolarLED   
#define AEQ5				0x003008				//			 !:J3, PolarLED		; AZ/EQ
#define AEQ3				0x003000				//			 !:J3, PolarLED		
#define AAZEQ5			0x00B008				// WiFi, !:J3, PolarLED		; AZ/EQ
#define AAZEQ6			0x00B008				// WiFi, !:J3, PolarLED		; AZ/EQ

// Motor firmware versions
#define VEQ6        0x000402    // Pretend EQ6			V 2.04	yyyy.mm.dd
#define VHEQ5       0x010204    // Pretend HEQ5			V 2.04	yyyy.mm.dd
#define VEQ5        0x020207    // Pretend EQ5			V 2.07	yyyy.mm.dd
#define VEQ3        0x030207    // Pretend EQ3			V 2.07	yyyy.mm.dd
#define VEQ8        0x040211    // Pretend EQ8			V 2.11	2014.11.10
#define VAZEQ6      0x050211		// Pretend AZEQ6		V 2.11	2014.11.10
#define VAZEQ5      0x060301		// Pretend AZEQ5		V 3.01	2015.08.10

#define EQGVERSION	VEQ6
#define EQGASSETS		AEQ6


// :I := ( :b * (360*60*60) / :a ) / Speed    ( where Speed is in arcsec/sec )
// If :I is greater than about 10, then the slew will need to use :G = LoSpeed mode
// If :I is less than 10, then the slew will need :G = HiRate, and :I := I * :g
//    a-AxxValue (Ticks/rev)  := AxxVanes * 4 * AxxGbxRatio * ( Axx Transfer ) * AxxWormTeeth 
//    b-AxxValue              := 6460.09 * AxxRatio * a-AxxValue * 15.041069 / (360*60*60)

// Speed = g*(b*(360*60*60)/a)/I
// ==============================
// IVALUE = (axis[EQGMOTOR].bVALUE * (360*60*60)) / axis[EQGMOTOR].STEPSPER360)

#define EQG_gVALUE        0x000010

#define EQGMAXIMUMSPEED   12

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
void puthexb(unsigned char);
void puthexw(unsigned int);
void puthexl(unsigned long);
void putdecb(unsigned char);
void putdecw(unsigned int);
void putdecl(unsigned long);
