/*
 * Copyright 2014-2020 NXP Semiconductors
 * Copyright 2020 GOODIX
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef TFA98XXPARAMETERS_H_
#define TFA98XXPARAMETERS_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

typedef struct tfa_msg {
	uint8_t msg_size;
	unsigned char cmdId[3];
	int data[9];
} tfa_msg_t;

//#if (defined(WIN32) || defined(_X64))
///* These warnings are disabled because it is only given by Windows and there is no easy fix */
//#pragma warning(disable:4200)
//#pragma warning(disable:4214)
//#endif

/*
 * profiles & volumesteps
 *
 */
#define TFA_MAX_PROFILES			(64)
#define TFA_MAX_MSGS				(10)

// the pack pragma is required to make that the size in memory
// matches the actual variable lenghts
// This is to assure that the binary files can be transported between
// different platforms.
#pragma pack (push, 1)

/*
 * typedef for 24 bit value using 3 bytes
 */
typedef struct uint24 {
  uint8_t b[3];
} uint24_t;
/*
 * the generic header
 *   all char types are in ASCII
 */
typedef struct tfaHeader {
	uint16_t id;
    char version[2];     // "V_" : V=version, vv=subversion
    char subversion[2];  // "vv" : vv=subversion
    uint16_t size;       // data size in bytes following CRC
    uint32_t CRC;        // 32-bits CRC for following data
    char customer[8];    // “name of customer”
    char application[8]; // “application name”
    char type[8];		 // “application type name”
} tfaHeader_t;

typedef enum tfaSamplerate {
	fs_8k,       // 8kHz
	fs_11k025,   // 11.025kHz
	fs_12k,      // 12kHz
	fs_16k,      // 16kHz
	fs_22k05,    // 22.05kHz
	fs_24k,      // 24kHz
	fs_32k,      // 32kHz
	fs_44k1,     // 44.1kHz
	fs_48k,      // 48kHz
	fs_96k,      // 96kHz
	fs_192k,     // 192kHz
	fs_count     // Should always be last item.
} tfaSamplerate_t;

// Keep in sync with tfaSamplerate_t !
static const int tfaSamplerateHz[fs_count] = { 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 96000, 192000 };


/*
 * coolflux direct memory access
 */
typedef struct tfaDspMem {
	uint8_t  type;		/* 0--3: p, x, y, iomem */
	uint16_t address;	/* target address */
	uint8_t size;		/* data size in words */
	int words[];		/* payload  in signed 32bit integer (two's complement) */
} tfaDspMem_t;

/*
 * the biquad coefficients for the API together with index in filter
 *  the biquad_index is the actual index in the equalizer +1
 */
#define BIQUAD_COEFF_SIZE       6

/*
* Output fixed point coeffs structure
*/
typedef struct {
	int a2;
	int a1;	
	int b2;	
	int b1;	
	int b0;	
}tfaBiquad_t;

typedef struct tfaBiquadOld {
  uint8_t bytes[BIQUAD_COEFF_SIZE*sizeof(uint24_t)];
}tfaBiquadOld_t;

typedef struct tfaBiquadFloat {
  float headroom;
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
} tfaBiquadFloat_t;

/*
* EQ filter definitions
* Note: This is not in line with smartstudio (JV: 12/12/2016)
*/
typedef enum tfaFilterType {
	fCustom,		//User defined biquad coefficients
	fFlat,			//Vary only gain
	fLowpass,		//2nd order Butterworth low pass
	fHighpass,		//2nd order Butterworth high pass
	fLowshelf,
	fHighshelf,
	fNotch,
	fPeak,
	fBandpass,
	f1stLP,
	f1stHP,
	fElliptic
} tfaFilterType_t;

/*
 * filter parameters for biquad (re-)calculation
 */
typedef struct tfaFilter {
  tfaBiquadOld_t biquad;
  uint8_t enabled;
  uint8_t type; // (== enum FilterTypes, assure 8bits length)
  float frequency;
  float Q;
  float gain;
} tfaFilter_t ;  //8 * float + int32 + byte == 37

/* 
 * biquad params for calculation
*/

#define TFA_BQ_EQ_INDEX 0
#define TFA_BQ_ANTI_ALIAS_INDEX 10
#define TFA_BQ_INTEGRATOR_INDEX 13

/*
* Loudspeaker Compensation filter definitions
*/
typedef struct tfaLsCompensationFilter {
  tfaBiquad_t biquad;
  uint8_t lsCompOn;  // Loudspeaker compensation on/off; when 'off', the DSP code doesn't apply the bwExt => bwExtOn GUI flag should be gray to avoid confusion
  uint8_t bwExtOn;   // Bandwidth extension on/off
  float fRes;        // [Hz] speaker resonance frequency
  float Qt;          // Speaker resonance Q-factor
  float fBwExt;      // [Hz] Band width extension frequency
  float samplingFreq;// [Hz] Sampling frequency
} tfaLsCompensationFilter_t;

/*
* Anti Aliasing Elliptic filter definitions
*/
typedef struct tfaAntiAliasFilter {
  tfaBiquad_t biquad;	/**< Output results fixed point coeffs */
  uint8_t enabled;
  float cutOffFreq;   // cut off frequency
  float samplingFreq; // sampling frequency
  float rippleDb;     // range: [0.1 3.0]
  float rolloff;      // range: [-1.0 1.0]
} tfaAntiAliasFilter_t;

/**
* Integrator filter input definitions
*/
typedef struct tfaIntegratorFilter {
  tfaBiquad_t biquad;	/**< Output results fixed point coeffs */
  uint8_t type;             /**< Butterworth filter type: high or low pass */
  float  cutOffFreq;        /**< cut off frequency in Hertz; range: [100.0 4000.0] */
  float  samplingFreq;      /**< sampling frequency in Hertz */
  float  leakage;           /**< leakage factor; range [0.0 1.0] */
} tfaIntegratorFilter_t;


typedef struct tfaEqFilter {
  tfaBiquad_t biquad;
  uint8_t enabled;
  uint8_t type;       // (== enum FilterTypes, assure 8bits length)
  float cutOffFreq;   // cut off frequency, // range: [100.0 4000.0]
  float samplingFreq; // sampling frequency
  float Q;            // range: [0.5 5.0]
  float gainDb;       // range: [-10.0 10.0]
} tfaEqFilter_t ;  //8 * float + int32 + byte == 37

typedef struct tfaContAntiAlias {
	int8_t index; 	/**< index determines destination type; anti-alias, integrator,eq */
	uint8_t type;
	float cutOffFreq;   // cut off frequency
	float samplingFreq;
	float rippleDb;     // integrator leakage
	float rolloff;
	uint8_t bytes[5*3];	// payload 5*24buts coeffs
}tfaContAntiAlias_t;

typedef struct tfaContIntegrator {
	int8_t index; 	/**< index determines destination type; anti-alias, integrator,eq */
	uint8_t type;
	float cutOffFreq;   // cut off frequency
	float samplingFreq;
	float leakage;     // integrator leakage
	float reserved;
	uint8_t bytes[5*3];	// payload 5*24buts coeffs
}tfaContIntegrator_t;

typedef struct tfaContEq {
  int8_t index;
  uint8_t type;			// (== enum FilterTypes, assure 8bits length)
  float cutOffFreq;		// cut off frequency, // range: [100.0 4000.0]
  float samplingFreq;	// sampling frequency
  float Q;				// range: [0.5 5.0]
  float gainDb;			// range: [-10.0 10.0]
  uint8_t bytes[5*3];	// payload 5*24buts coeffs
} tfaContEq_t ;		//8 * float + int32 + byte == 37

typedef union tfaContBiquad {
	tfaContEq_t eq;
	tfaContAntiAlias_t aa;
	tfaContIntegrator_t in;
}tfaContBiquad_t;

#define TFA_BQ_EQ_INDEX			0
#define TFA_BQ_ANTI_ALIAS_INDEX	10
#define TFA_BQ_INTEGRATOR_INDEX 13
#define TFA98XX_MAX_EQ			10

typedef struct tfaEqualizer {
  tfaFilter_t filter[TFA98XX_MAX_EQ];
} tfaEqualizer_t;

/*
 * files
 */
#define HDR(c1,c2) (c2<<8|c1) // little endian
typedef enum tfaHeaderType {
    paramsHdr		= HDR('P','M'), /* container file */
    volstepHdr	 	= HDR('V','P'),
    patchHdr	 	= HDR('P','A'),
    speakerHdr	 	= HDR('S','P'),
    presetHdr	 	= HDR('P','R'),
    configHdr	 	= HDR('C','O'),
    equalizerHdr	= HDR('E','Q'),
    drcHdr			= HDR('D','R'),
    msgHdr			= HDR('M','G'),	/* generic message */
    infoHdr			= HDR('I','N')
} tfaHeaderType_t;

/*
 * equalizer file
 */
#define tfa_EQ_VERSION    '1'
#define tfa_EQ_SUBVERSION "00"
typedef struct tfaEqualizerFile {
	tfaHeader_t hdr;
	uint8_t samplerate; 				 // ==enum samplerates, assure 8 bits
    tfaFilter_t filter[TFA98XX_MAX_EQ];// note: API index counts from 1..10
} tfaEqualizerFile_t;

/*
 * patch file
 */
#define tfa_PA_VERSION    '1'
#define tfa_PA_SUBVERSION "00"
typedef struct tfaPatchFile {
	tfaHeader_t hdr;
	uint8_t data[];
} tfaPatch_t;

/*
 * generic message file
 *   -  the payload of this file includes the opcode and is send straight to the DSP
 */
#define tfa_MG_VERSION    '3'
#define tfa_MG_SUBVERSION "00"
typedef struct tfa_msg_file {
	tfaHeader_t hdr;
	uint8_t data[];
} tfa_msg_file_t;

/*
 * NOTE the tfa98xx API defines the enum tfa9xxx_config_type that defines
 *          the subtypes as decribes below.
 *          tfa98xx_dsp_config_parameter_type() can be used to get the
 *           supported type for the active device..
 */
/*
 * config file V1 sub 1
 */
#define tfa_CO_VERSION		'1'
#define tfa_CO3_VERSION		'3'
#define tfa_CO_SUBVERSION1	"01"
typedef struct tfaConfigS1File {
	tfaHeader_t hdr;
	uint8_t data[55*3];
} tfaConfigS1_t;

/*
 * config file V1 sub 2
 */
#define tfa_CO_SUBVERSION2 "02"
typedef struct tfaConfigS2File {
	tfaHeader_t hdr;
	uint8_t data[67*3];
} tfaConfigS2_t;

/*
 * config file V1 sub 3
 */
#define tfa_CO_SUBVERSION3 "03"
typedef struct tfaConfigS3File {
	tfaHeader_t hdr;
	uint8_t data[67*3];
} tfaConfigS3_t;

/*
 * config file V1.0
 */
#define tfa_CO_SUBVERSION "00"
typedef struct tfaConfigFile {
	tfaHeader_t hdr;
	uint8_t data[];
} tfaConfig_t;

/*
 * preset file
 */
#define tfa_PR_VERSION    '1'
#define tfa_PR_SUBVERSION "00"
typedef struct tfaPresetFile {
	tfaHeader_t hdr;
	uint8_t data[];
} tfaPreset_t;

/*
 * drc file
 */
#define tfa_DR_VERSION    '1'
#define tfa_DR_SUBVERSION "00"
typedef struct tfaDrcFile {
	tfaHeader_t hdr;
	uint8_t data[];
} tfaDrc_t;

/*
 * drc file
 * for tfa 2 there is also a xml-version
 */
#define tfa_DR3_VERSION    '3'
#define tfa_DR3_SUBVERSION "00"
typedef struct tfaDrcFile2 {
	tfaHeader_t hdr;
	uint8_t version[3];
	uint8_t data[];
} tfaDrc2_t;

/*
 * speaker file header
 */
struct tfaSpkHeader {
	struct tfaHeader hdr;
	char name[8];	// speaker nick name (e.g. “dumbo”)
	char vendor[16];
	char type[8];
	//	dimensions (mm)
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint16_t ohm;
};

/*
 * speaker file
 */
#define tfa_SP_VERSION    '1'
#define tfa_SP_SUBVERSION "00"
typedef struct tfaSpeakerFile {
	tfaHeader_t hdr;
	char name[8];	// speaker nick name (e.g. “dumbo”)
	char vendor[16];
	char type[8];
	//	dimensions (mm)
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint8_t ohm_primary;
	uint8_t ohm_secondary;
	uint8_t data[]; //payload TFA98XX_SPEAKERPARAMETER_LENGTH
} tfaSpeakerFile_t;



struct tfaFWVer {
	uint8_t Major;
	uint8_t minor;
	uint8_t minor_update:6;
	uint8_t Update:2;
};

struct tfaFWMsg {
	struct tfaFWVer fwVersion;
	struct tfa_msg payload;
};

typedef struct tfaLiveData {
	char name[25];
	char addrs[25];
	int tracker;
	int scalefactor;
} tfaLiveData_t;

#define tfa_SP3_VERSION  '3'
#define tfa_SP3_SUBVERSION "00"
struct tfaSpeakerFileMax2 {
	tfaHeader_t hdr;
	char name[8];	// speaker nick name (e.g. “dumbo”)
	char vendor[16];
	char type[8];
	//	dimensions (mm)
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint8_t ohm_primary;
	uint8_t ohm_secondary;
	struct tfaFWMsg FWmsg; //payload including FW ver and Cmd ID
};

/*
 * parameter container file
 */
/*
 * descriptors
 * Note 1: append new DescriptorType at the end
 * Note 2: add new descriptors to dsc_name[] in tfaContUtil.c
 */
typedef enum tfaDescriptorType {
	dscDevice,		// device list
	dscProfile,		// profile list
	dscRegister,	// register patch
	dscString,		// ascii, zero terminated string
	dscFile,		// filename + file contents
	dscPatch,		// patch file
	dscMarker,		// marker to indicate end of a list
	dscMode,
	dscSetInputSelect,
	dscSetOutputSelect,
	dscSetProgramConfig,
	dscSetLagW,
	dscSetGains,
	dscSetvBatFactors,
	dscSetSensesCal,
	dscSetSensesDelay,
	dscBitfield,
	dscDefault,		// used to reset bitfields to there default values
	dscLiveData,
	dscLiveDataString,
	dscGroup,
	dscCmd,
	dscSetMBDrc,
	dscFilter,
	dscNoInit,
	dscFeatures,
	dscCfMem,		// coolflux memory x,y,io
	dscSetFwkUseCase,
	dscSetVddpConfig,
	dscTfaHal, 
	dscInfoText,	// info keyword for storing text into container
	dscInfoFile,	// info keyword for storing a file into container
	dsc_last,		// trailer
	dsc_listend=-1
} tfaDescriptorType_t;

#define TFA_BITFIELDDSCMSK 0x7fffffff
typedef struct tfaDescPtr {
	uint32_t offset:24;
	uint32_t  type:8; // (== enum tfaDescriptorType, assure 8bits length)
}tfaDescPtr_t;

/*
 * generic file descriptor
 */
typedef struct tfaFileDsc {
	tfaDescPtr_t name;
	uint32_t size;	// file data length in bytes
	uint8_t data[]; //payload
} tfaFileDsc_t;


/*
 * device descriptor list
 */
typedef struct tfaDeviceList {
	uint8_t length;			// nr of items in the list
	uint8_t bus;			// bus
	uint8_t dev;			// device
	uint8_t func;			// subfunction or subdevice
	uint32_t devid;			// device  hw fw id
	tfaDescPtr_t name;	// device name
	tfaDescPtr_t list[];	// items list
} tfaDeviceList_t;

/*
 * profile descriptor list
 */
typedef struct tfaProfileList {
	uint32_t length:8;		// nr of items in the list + name
	uint32_t group:8;		// profile group number
	uint32_t ID:16;			// profile ID
	tfaDescPtr_t name;	// profile name
	tfaDescPtr_t list[];	// items list (lenght-1 items)
} tfaProfileList_t;
#define TFA_PROFID 0x1234

/*
 * livedata descriptor list
 */
typedef struct tfaLiveDataList {
	uint32_t length:8;		// nr of items in the list
	uint32_t ID:24;			// profile ID
	tfaDescPtr_t name;	        // livedata name
	tfaDescPtr_t list[];	        // items list
} tfaLiveDataList_t;
#define TFA_LIVEDATAID 0x5678

/*
 * Bitfield descriptor
 */
typedef struct tfaBitfield {
	uint16_t  value;
	uint16_t  field; // ==datasheet defined, 16 bits
} tfaBitfield_t;

/*
 * Bitfield enumuration bits descriptor
 */
typedef struct tfaBfEnum {
	unsigned int  len:4;		// this is the actual length-1
	unsigned int  pos:4;
	unsigned int  address:8;
} tfaBfEnum_t;

/*
 * Register patch descriptor
 */
typedef struct tfaRegpatch {
	uint8_t   address;	// register address
	uint16_t  value;	// value to write
	uint16_t  mask;		// mask of bits to write
} tfaRegpatch_t;

/*
 * Mode descriptor
 */
typedef struct tfaUseCase {
	int value;	// mode value, maps to enum tfa9xxx_Mode
} tfaMode_t;

/*
 * NoInit descriptor
 */
typedef struct tfaNoInit {
	uint8_t value;	// noInit value
} tfaNoInit_t;

/*
 * Features descriptor
 */
typedef struct tfaFeatures {
	uint16_t value[3];	// features value
} tfaFeatures_t;


/*
 * the container file
 *   - the size field is 32bits long (generic=16)
 *   - all char types are in ASCII
 */
#define tfa_PM_VERSION  '1'
#define tfa_PM3_VERSION '3'
#define tfa_PM_SUBVERSION '1'
typedef struct tfaContainer {
    char id[2];					// "XX" : XX=type
    char version[2];			// "V_" : V=version, vv=subversion
    char subversion[2];			// "vv" : vv=subversion
    uint32_t size;				// data size in bytes following CRC
    uint32_t CRC;				// 32-bits CRC for following data
    uint16_t rev;				// "extra chars for rev nr"
    char customer[8];			// “name of customer”
    char application[8];		// “application name”
    char type[8];				// “application type name”
    uint16_t ndev;	 			// "nr of device lists"
    uint16_t nprof;	 			// "nr of profile lists"
    uint16_t nliveData;			// "nr of livedata lists"
    tfaDescPtr_t index[];	// start of item index table
} tfaContainer_t;

#pragma pack (pop)

#endif /* TFA98XXPARAMETERS_H_ */
