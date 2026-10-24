/*
	QN8027 Arduino library
	Stereo FM transmitter with RDS, I2C address 0x2C.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "QN8027.h"

static constexpr float DeviationStepKhz = 0.69f;
static constexpr float RdsDeviationStepKhz = 0.35f;

void QN8027::begin(float fmFreq)
{
	Wire.begin();
	reset();
	setFrequency(fmFreq);
	enable(true);
}

#if (defined(ESP32) || defined(ESP8266))
void QN8027::begin(uint8_t sda, uint8_t scl)
{
	begin(sda, scl, 100000, FREQ_DEFAULT);
}

void QN8027::begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq)
{
	Wire.begin(sda, scl, i2cFreq);
	reset();
	setFrequency(fmFreq);
	enable(true);
}
#endif

void QN8027::reset(void)
{
	writeRegister(RegSystem, 1 << BitReset);
	delay(80);

	writeClock();
	delay(80);

	writeVga();
	delay(80);

	writeRegister(RegSystem, 1 << BitRecal);
	delay(80);

	writeRdsControl();
	delay(80);

	uint8_t system = 0;
	if (!stereo) system |= (1 << BitMono);
	if (muted) system |= (1 << BitMute);
	writeRegister(RegSystem, system);
	delay(80);

	writeAudio();
	delay(80);

	writeDeviation();
	delay(80);

	writePower();
	live = true;
}

void QN8027::setCrystal(Crystal value)
{
	crystal = value;
	if (live) writeVga();
}

void QN8027::setClockSource(ClockSource source)
{
	clockSource = source;
	if (live) writeClock();
}

void QN8027::setCrystalDrive(uint8_t drive)
{
	crystalDrive = drive > 31 ? 31 : drive;
	if (live) writeClock();
}

void QN8027::setInputGain(InputGain gain)
{
	inputGain = gain;
	if (live) writeVga();
}

void QN8027::setDigitalGain(DigitalGain gain)
{
	digitalGain = gain;
	if (live) writeVga();
}

void QN8027::setInputResistance(InputResistance resistance)
{
	inputResistance = resistance;
	if (live) writeVga();
}

void QN8027::setPreEmphasis(PreEmphasis emphasis)
{
	preEmphasis = emphasis;
	if (live) writeAudio();
}

void QN8027::setPowerSave(PowerSave mode)
{
	powerSave = mode;
	if (live) writeAudio();
}

void QN8027::setPilotDeviation(uint8_t code)
{
	pilot = code > 15 ? 15 : code;
	if (live) writeAudio();
}

void QN8027::setDeviationKhz(float khz)
{
	deviation = khzToSteps(khz, DeviationStepKhz, 255);
	if (live) writeDeviation();
}

void QN8027::setStereo(bool on)
{
	stereo = on;
	if (live) writeSystemBits(on ? 0 : (1 << BitMono), 1 << BitMono);
}

void QN8027::mute(bool on)
{
	muted = on;
	if (live) writeSystemBits(on ? (1 << BitMute) : 0, 1 << BitMute);
}

void QN8027::setTxPower(uint8_t power)
{
	if (power < TX_POWER_MIN) power = TX_POWER_MIN;
	if (power > TX_POWER_MAX) power = TX_POWER_MAX;
	txPower = power;
	if (live) writePower();
}

void QN8027::enable(bool on)
{
	writeSystemBits(on ? (1 << BitIdle) : 0, 1 << BitIdle);
}

void QN8027::setFrequency(float mhz)
{
	uint16_t channel = mhzToChannel(mhz);
	writeRegister(RegChannel, (uint8_t)channel);
	writeSystemBits((uint8_t)(channel >> 8), 0x03);
}

void QN8027::setRdsEnabled(bool on)
{
	rdsOn = on;
	if (live) writeRdsControl();
}

void QN8027::setRdsDeviationKhz(float khz)
{
	rdsDeviation = khzToSteps(khz, RdsDeviationStepKhz, 127);
	if (live) writeRdsControl();
}

void QN8027::setRdsPi(uint16_t pi)
{
	rdsPi = pi;
}

void QN8027::setProgramType(ProgramType type)
{
	programType = type;
}

void QN8027::setProgramService(const char *name)
{
	memset(psName, ' ', 8);
	if (name != nullptr)
	{
		size_t length = strlen(name);
		if (length > 8) length = 8;
		memcpy(psName, name, length);
	}
	psSegment = 0;
}

void QN8027::setRadiotext(const char *text)
{
	size_t length = 0;
	if (text != nullptr)
	{
		length = strlen(text);
		if (length > 64) length = 64;
	}
	if (length == rdsLength && (length == 0 || memcmp(rdsBuf, text, length) == 0)) return;

	memset(rdsBuf, ' ', RDSBUF_SIZE);
	if (length > 0) memcpy(rdsBuf, text, length);
	rdsLength = (uint8_t)length;
	rdsOffset = 0;
	rtSegment = 0;
	rtHold = 0;
	rdsAbFlag ^= 1;
}

void QN8027::rdsTick(void)
{
	if (rdsLength <= 4) return;

	uint8_t next = rdsOffset + 4;
	rdsOffset = next >= rdsLength ? 0 : next;
}

void QN8027::rdsRefresh(void)
{
	if (!rdsOn) return;

	// One group 0A, then one group 2A. The name and the radiotext both stay on the air.
	if ((rdsPhase & 1) == 0) writeProgramService();
	else writeRadiotext();
	rdsPhase++;
}

void QN8027::getCurrentRds(char text[9])
{
	uint8_t start = (rdsOffset / 8) * 8;
	memcpy(text, &rdsBuf[start], 8);
	text[8] = 0;
}

void QN8027::writeClock(void)
{
	uint8_t value = ((uint8_t)clockSource << 6) | (crystalDrive & 0x1F);
	writeRegister(RegClock, value);
}

void QN8027::writeVga(void)
{
	uint8_t value = ((uint8_t)crystal << 7) |
		((uint8_t)inputGain << 4) |
		((uint8_t)digitalGain << 2) |
		((uint8_t)inputResistance & 0x03);
	writeRegister(RegVga, value);
}

void QN8027::writeAudio(void)
{
	// 50/75 µs, power-save timer, and the 4-bit stereo pilot code.
	uint8_t value = ((uint8_t)preEmphasis << 7) |
		((uint8_t)powerSave << 4) |
		(pilot & 0x0F);
	writeRegister(RegGplt, value);
}

void QN8027::writeRdsControl(void)
{
	uint8_t value = rdsDeviation & 0x7F;
	if (rdsOn) value |= 0x80;
	writeRegister(RegRds, value);
}

void QN8027::writeDeviation(void)
{
	writeRegister(RegDeviation, deviation);
}

void QN8027::writePower(void)
{
	writeRegister(RegPower, txPower);
}

void QN8027::writeSystemBits(uint8_t data, uint8_t mask)
{
	updateRegister(RegSystem, data, mask);
}

void QN8027::writeProgramService(void)
{
	// Group 0A. Block D carries two of the eight station-name characters.
	uint8_t segment = psSegment & 0x03;
	uint8_t stereoBit = (segment == 0 && stereo) ? 1 : 0;
	uint16_t blockB = (((uint16_t)programType & 0x1F) << 5) | (1 << 3) | ((uint16_t)stereoBit << 2) | segment;

	writeRegister(RegRds0 + 0, (uint8_t)(rdsPi >> 8));
	writeRegister(RegRds0 + 1, (uint8_t)rdsPi);
	writeRegister(RegRds0 + 2, (uint8_t)(blockB >> 8));
	writeRegister(RegRds0 + 3, (uint8_t)blockB);
	writeRegister(RegRds0 + 4, 0xE0);
	writeRegister(RegRds0 + 5, 0xCD);
	writeRegister(RegRds0 + 6, (uint8_t)psName[segment * 2]);
	writeRegister(RegRds0 + 7, (uint8_t)psName[segment * 2 + 1]);
	latchRdsGroup();

	psSegment = (segment + 1) & 0x03;
}

void QN8027::writeRadiotext(void)
{
	// Group 2A. Each segment is four characters. Addresses 0..15 fill the TEXT field.
	uint8_t segments = rdsLength == 0 ? 1 : (uint8_t)((rdsLength + 3) / 4);
	if (segments > 16) segments = 16;
	if (rtSegment >= segments) rtSegment = 0;

	uint8_t segment = rtSegment;
	uint16_t start = (uint16_t)segment * 4;
	uint16_t blockB = (0x2 << 12) |
		(((uint16_t)programType & 0x1F) << 5) |
		((uint16_t)(rdsAbFlag & 0x01) << 4) |
		segment;

	writeRegister(RegRds0 + 0, (uint8_t)(rdsPi >> 8));
	writeRegister(RegRds0 + 1, (uint8_t)rdsPi);
	writeRegister(RegRds0 + 2, (uint8_t)(blockB >> 8));
	writeRegister(RegRds0 + 3, (uint8_t)blockB);
	writeRegister(RegRds0 + 4, (uint8_t)rdsBuf[start]);
	writeRegister(RegRds0 + 5, (uint8_t)rdsBuf[start + 1]);
	writeRegister(RegRds0 + 6, (uint8_t)rdsBuf[start + 2]);
	writeRegister(RegRds0 + 7, (uint8_t)rdsBuf[start + 3]);
	latchRdsGroup();

	// Send each four-character piece twice so a receiver can catch it before the next one.
	rtHold++;
	if (rtHold < 2) return;
	rtHold = 0;
	rtSegment = (uint8_t)(segment + 1);
	if (rtSegment >= segments) rtSegment = 0;
}

void QN8027::latchRdsGroup(void)
{
	uint8_t system = readRegister(RegSystem);
	system ^= (1 << BitRdsReady);
	writeRegister(RegSystem, system);
}

void QN8027::writeRegister(uint8_t addr, uint8_t data)
{
	Wire.beginTransmission(Address);
	Wire.write(addr);
	Wire.write(data);
	Wire.endTransmission(true);
}

uint8_t QN8027::readRegister(uint8_t addr)
{
	Wire.beginTransmission(Address);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(Address, (uint8_t)1);
	return Wire.read();
}

void QN8027::updateRegister(uint8_t addr, uint8_t data, uint8_t mask)
{
	uint8_t updated = (readRegister(addr) & ~mask) | (data & mask);
	writeRegister(addr, updated);
}

uint16_t QN8027::mhzToChannel(float mhz)
{
	int channel = (int)round((mhz - 76.0f) / 0.05f);
	if (channel < 0) channel = 0;
	if (channel > 640) channel = 640;
	return (uint16_t)channel;
}

uint8_t QN8027::khzToSteps(float khz, float stepKhz, uint8_t maxStep)
{
	int steps = (int)(khz / stepKhz + 0.5f);
	if (steps < 0) steps = 0;
	if (steps > maxStep) steps = maxStep;
	return (uint8_t)steps;
}
