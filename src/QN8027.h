/*

	QN8027 Arduino library
	QN8027 is a single chip Stereo FM RDS Exciter with I2C Interface

	Martin Ondrak, 2017-2018
	Univerzita Pardubice
	martas.ondrak@gmail.com

*/

#ifndef _QN8027_h
#define _QN8027_h

#include <Arduino.h>

#define		TX_POWER_MAX		75
#define		RDSBUF_SIZE			128		//8 (bytes) is the smallest value

#define		FREQ_DEFAULT		88.9f

#define		QN8027_ADDRESS		(0x2C)

#define		QN8027_REG_SYSTEM	(0x00)	//SYSTEM Sets device modes, resets.
#define		QN8027_REG_CH1		(0x01)	//Lower 8 bits of 10-bit channel index.
#define		QN8027_REG_GPLT		(0x02)	//Audio controls, gain of TX pilot frequency deviation.
#define		QN8027_REG_XTL		(0x03)	//XCLK pin control.
#define		QN8027_REG_VGA		(0x04)	//TX mode input impedance, crystal frequency setting.
#define		QN8027_REG_CID1		(0x05)	//Device ID numbers.
#define		QN8027_REG_CID2		(0x06)	//Device ID numbers.
#define		QN8027_REG_STATUS	(0x07)	//Device status indicators.
#define		QN8027_REG_RDSD0	(0x08)	//RDS data byte 0.
#define		QN8027_REG_RDSD1	(0x09)	//RDS data byte 1.
#define		QN8027_REG_RDSD2	(0x0A)	//RDS data byte 2.
#define		QN8027_REG_RDSD3	(0x0B)	//RDS data byte 3.
#define		QN8027_REG_RDSD4	(0x0C)	//RDS data byte 4.
#define		QN8027_REG_RDSD5	(0x0D)	//RDS data byte 5.
#define		QN8027_REG_RDSD6	(0x0E)	//RDS data byte 6.
#define		QN8027_REG_RDSD7	(0x0F)	//RDS data byte 7.
#define		QN8027_REG_PAC		(0x10)	//PA output power target control.
#define		QN8027_REG_FDEV		(0x11)	//Specify total TX frequency deviation.
#define		QN8027_REG_RDS		(0x12)	//Specify RDS frequency deviation, RDS mode selection.

#define		QN8027_WRITEREG		(0x58)
#define		QN8027_READREG		(0x59)

//SYSTEM - 0x00
#define		QN8027_BIT_RESET		(7)
#define		QN8027_BIT_RECAL		(6)
#define		QN8027_BIT_IDLE			(5)
#define		QN8027_BIT_MONO			(4)
#define		QN8027_BIT_MUTE			(3)
#define		QN8027_BIT_RDSRDY		(2)
#define		QN8027_BIT_CH_HI		(0)

#define		QN8027_MASK_RESET		(1 << QN8027_BIT_RESET) //1 = RESET
#define		QN8027_MASK_RECAL		(1 << QN8027_BIT_RECAL)
#define		QN8027_MASK_IDLE		(1 << QN8027_BIT_IDLE) //0 = IDLE
#define		QN8027_MASK_MONO		(1 << QN8027_BIT_MONO) //0 = Stereo
#define		QN8027_MASK_MUTE		(1 << QN8027_BIT_MUTE) //0 = Not Muted
#define		QN8027_MASK_RDSRDY		(1 << QN8027_BIT_RDSRDY) //1 = RDS Ready
#define		QN8027_MASK_CH_HI		(3 << QN8027_BIT_CH_HI)

//GPLT - 0x02
#define		QN8027_BIT_PREEMP		(7)
#define		QN8027_BIT_PRIV			(6)
#define		QN8027_BIT_T1M			(4)

#define		QN8027_MASK_PREEMP		(1 << QN8027_BIT_PREEMP)
#define		QN8027_MASK_PRIV		(1 << QN8027_BIT_PRIV)
#define		QN8027_MASK_T1M			(3 << QN8027_BIT_T1M)
#define		QN8027_MASK_TXPLT		(15)

//XTL - 0x03
#define		QN8027_BIT_XINJ			(6)

#define		QN8027_MASK_XINJ		(3 << QN8027_BIT_XINJ)
#define		QN8027_MASK_XISEL		(31)

//VGA - 0x04
#define		QN8027_BIT_XSEL			(7)
#define		QN8027_BIT_GVGA			(4)
#define		QN8027_BIT_GDB			(2)
#define		QN8027_BIT_RIN			(0)

#define		QN8027_MASK_XSEL		(1 << QN8027_BIT_XSEL) //0 = 12 MHz; 1 = 24 MHz
#define		QN8027_MASK_GVGA		(3 << QN8027_BIT_GVGA)
#define		QN8027_MASK_GDB			(3 << QN8027_BIT_GDB)
#define		QN8027_MASK_RIN			(3)

//CID1 - 0x05
#define		QN8027_BIT_CID1			(2)

#define		QN8027_MASK_CID1		(3 << QN8027_BIT_CID1)
#define		QN8027_MASK_CID2		(3)

//CID2 - 0x06
#define		QN8027_BIT_CID3			(4)

#define		QN8027_MASK_CID3		(15 << QN8027_BIT_CID3)
#define		QN8027_MASK_CID4		(15)

//STATUS - 0x07
#define		QN8027_BIT_AUD_UPK		(4)
#define		QN8027_BIT_RDS_UPD		(3)

#define		QN8027_MASK_AUD_UPK		(15 << QN8027_BIT_AUD_UPK)
#define		QN8027_MASK_RDS_UPD		(1 << QN8027_BIT_RDS_UPD)
#define		QN8027_MASK_FSM			(7)

//PAC - 0x10
#define		QN8027_BIT_TXPD_CLR		(7)

#define		QN8027_MASK_TXPD_CLR	(1 << QN8027_BIT_TXPD_CLR)
#define		QN8027_MASK_PA_TRGT		(127)

//RDS - 0x12
#define		QN8027_BIT_RDSEN		(7)

#define		QN8027_MASK_RDSEN		(1 << QN8027_BIT_RDSEN) //0 - Off; 1 - On
#define		QN8027_MASK_RDSFDEV		(127)


#define		FREQ_FLOAT2CHANNEL(X)	(round((X - 76) / 0.05f))


typedef enum
{
	STEREO = 0,
	MONO = 1
}audioMode_e;

typedef enum
{
	PREEMP_50US = 0,
	PREEMP_75US = 1
}preEmphasis_e;

typedef enum
{
	POWERSAVE_58S = 0,
	POWERSAVE_59S = 1,
	POWERSAVE_60S = 2, 
	POWERSAVE_NONE = 3
}powerSave_e;

typedef enum
{
	GAIN_TXPILOT_0 = 0,
	GAIN_TXPILOT_1 = 1,
	GAIN_TXPILOT_2 = 2,
	GAIN_TXPILOT_3 = 3
}gainTxPilot_e;

typedef enum
{
	CLOCK_XTAL = 0,
	CLOCK_SQIN = 1,
	CLOCK_OSC = 2,
}clockSource_e;

typedef enum
{
	CLOCKSPEED_12 = 0,
	CLOCKSPEED_24 = 1
}clockSpeed_e;

typedef enum
{
	RIN_0 = 0,
	RIN_1 = 1,
	RIN_2 = 2,
	RIN_3 = 3
}inputResistance_e;

typedef enum
{
	GAIN_0 = 0,
	GAIN_1 = 1,
	GAIN_2 = 2,
	GAIN_3 = 3,
	GAIN_4 = 4,
	GAIN_5 = 5
}inputGain_e;

typedef enum
{
	TXGAIN_0 = 0,
	TXGAIN_1 = 1,
	TXGAIN_2 = 2
}txGain_e;





class QN8027
{
public:
	void begin(float fmFreq);
	void begin(uint8_t sda, uint8_t scl);
	void begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq);

	void reset(void);
	
	void setFrequency(float freq);
	void setTxPower(uint8_t pwr);
	void enable(boolean enable);

	void rdsBufSet(const char buf[RDSBUF_SIZE], uint16_t length);
	void rdsTick(void);
	void rdsRefresh(void);
	void rdsEnable(boolean enable);

	void getCurrentRds(char rdsBuf[9]);


protected:
	
private:
	void _writeRegister(uint8_t addr, uint8_t data);
	void _updateRegister(uint8_t addr, uint8_t data, uint8_t mask);
	uint8_t _readRegister(uint8_t addr);

	void _setFrequency(uint16_t freqChannel);

	void _rotateRdsBuf(uint8_t num);

	void _writeRds();

	uint8_t pages;
	uint8_t pageNumber;
	uint8_t rdsBufPtr;
	uint8_t rdsBufLength;
	uint8_t rdsDisplayPtr;
	char rdsBuf[RDSBUF_SIZE];

};

//typedef void (*callback_t)(void);



#endif

