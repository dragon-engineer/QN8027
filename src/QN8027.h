/*
	QN8027 Arduino library
	Stereo FM transmitter with RDS, I2C address 0x2C.

	Register values follow the Quintic QN8027 datasheet.
	Martin Ondrak, 2017-2026. Refactored 2026.
*/

#ifndef QN8027_H
#define QN8027_H

#include <Arduino.h>

#define TX_POWER_MIN		20
#define TX_POWER_MAX		75
#define RDSBUF_SIZE			128
#define FREQ_DEFAULT		88.9f

class QN8027
{
public:
	// GVGA, register 0x04 bits 5:4. Steps are 3 dB.
	enum class InputGain : uint8_t
	{
		Db0 = 0,
		Db3 = 1,
		Db6 = 2,
		Db9 = 3
	};

	// GDB, register 0x04 bits 3:2. Same 3 dB steps as the input PGA.
	enum class DigitalGain : uint8_t
	{
		Db0 = 0,
		Db3 = 1,
		Db6 = 2,
		Db9 = 3
	};

	// RIN, register 0x04 bits 1:0.
	enum class InputResistance : uint8_t
	{
		Ohm80k = 0,
		Ohm40k = 1,
		Ohm20k = 2,
		Ohm10k = 3
	};

	// XSEL, register 0x04 bit 7.
	enum class Crystal : uint8_t
	{
		Mhz12 = 0,
		Mhz24 = 1
	};

	// XINJ, register 0x03 bits 7:6.
	enum class ClockSource : uint8_t
	{
		Crystal = 0,
		External = 1
	};

	// PREEMP, register 0x02 bit 7.
	enum class PreEmphasis : uint8_t
	{
		Us50 = 0,
		Us75 = 1
	};

	// T1M, register 0x02 bits 5:4. Disabled is what a normal transmission uses.
	enum class PowerSave : uint8_t
	{
		After58s = 0,
		After59s = 1,
		After60s = 2,
		Disabled = 3
	};

	// RDS program type, block B bits 9:5. 0 is "none".
	enum class ProgramType : uint8_t
	{
		None = 0,
		News = 1,
		CurrentAffairs = 2,
		Information = 3,
		Sport = 4,
		Education = 5,
		Drama = 6,
		Culture = 7,
		Science = 8,
		Varied = 9,
		Pop = 10,
		Rock = 11,
		EasyListening = 12,
		LightClassical = 13,
		SeriousClassical = 14,
		OtherMusic = 15,
		Weather = 16,
		Finance = 17,
		Children = 18,
		Social = 19,
		Religion = 20,
		PhoneIn = 21,
		Travel = 22,
		Leisure = 23,
		Jazz = 24,
		Country = 25,
		NationalMusic = 26,
		Oldies = 27,
		Folk = 28,
		Documentary = 29,
		AlarmTest = 30,
		Alarm = 31
	};

	// Pilot deviation code, register 0x02 bits 3:0. 9 is the stereo pilot this board uses.
	static constexpr uint8_t PilotCode = 9;

	void begin(float fmFreq);
	void begin(uint8_t sda, uint8_t scl);
	void begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq);

	void setFrequency(float mhz);
	void setTxPower(uint8_t power);
	void enable(bool on);

	void setCrystal(Crystal crystal);
	void setClockSource(ClockSource source);
	void setCrystalDrive(uint8_t drive);
	void setInputGain(InputGain gain);
	void setDigitalGain(DigitalGain gain);
	void setInputResistance(InputResistance resistance);
	void setPreEmphasis(PreEmphasis emphasis);
	void setPowerSave(PowerSave mode);
	void setPilotDeviation(uint8_t code);
	void setDeviationKhz(float khz);
	void setStereo(bool stereo);
	void mute(bool mute);

	void setRdsEnabled(bool on);
	void setRdsDeviationKhz(float khz);
	void setRdsPi(uint16_t pi);
	void setProgramType(ProgramType type);
	void setProgramService(const char *name);
	void setRadiotext(const char *text);
	void rdsTick(void);
	void rdsRefresh(void);
	void getCurrentRds(char text[9]);

private:
	static constexpr uint8_t Address = 0x2C;

	static constexpr uint8_t RegSystem = 0x00;
	static constexpr uint8_t RegChannel = 0x01;
	static constexpr uint8_t RegGplt = 0x02;
	static constexpr uint8_t RegClock = 0x03;
	static constexpr uint8_t RegVga = 0x04;
	static constexpr uint8_t RegRds0 = 0x08;
	static constexpr uint8_t RegPower = 0x10;
	static constexpr uint8_t RegDeviation = 0x11;
	static constexpr uint8_t RegRds = 0x12;

	static constexpr uint8_t BitReset = 7;
	static constexpr uint8_t BitRecal = 6;
	static constexpr uint8_t BitIdle = 5;
	static constexpr uint8_t BitMono = 4;
	static constexpr uint8_t BitMute = 3;
	static constexpr uint8_t BitRdsReady = 2;

	void reset(void);
	void writeClock(void);
	void writeVga(void);
	void writeAudio(void);
	void writeRdsControl(void);
	void writeDeviation(void);
	void writePower(void);
	void writeSystemBits(uint8_t data, uint8_t mask);
	void writeProgramService(void);
	void writeRadiotext(void);
	void latchRdsGroup(void);

	void writeRegister(uint8_t addr, uint8_t data);
	void updateRegister(uint8_t addr, uint8_t data, uint8_t mask);
	uint8_t readRegister(uint8_t addr);
	static uint16_t mhzToChannel(float mhz);
	static uint8_t khzToSteps(float khz, float stepKhz, uint8_t maxStep);

	Crystal crystal = Crystal::Mhz12;
	ClockSource clockSource = ClockSource::Crystal;
	uint8_t crystalDrive = 0x0F;
	InputGain inputGain = InputGain::Db0;
	DigitalGain digitalGain = DigitalGain::Db0;
	InputResistance inputResistance = InputResistance::Ohm40k;
	PreEmphasis preEmphasis = PreEmphasis::Us50;
	PowerSave powerSave = PowerSave::Disabled;
	uint8_t pilot = PilotCode;
	uint8_t deviation = 109;
	uint8_t txPower = TX_POWER_MAX;
	bool stereo = true;
	bool muted = false;
	bool rdsOn = false;
	uint8_t rdsDeviation = 10;
	uint16_t rdsPi = 0x4000;
	ProgramType programType = ProgramType::None;
	uint8_t rdsAbFlag = 0;

	bool live = false;
	uint8_t rdsOffset = 0;
	uint8_t rdsLength = 0;
	uint8_t psSegment = 0;
	uint8_t rtSegment = 0;
	uint8_t rtHold = 0;
	uint8_t rdsPhase = 0;
	char psName[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
	char rdsBuf[RDSBUF_SIZE] = {};
};

#endif
