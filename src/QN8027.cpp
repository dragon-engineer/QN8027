/*

	QN8027 Arduino Library
	QN8027 is a single chip Stereo FM RDS Exciter with I2C Interface

	Martin Ondrak, 2017 - 2018
	martas.ondrak@gmail.com

*/
#include <Arduino.h>

#include <Wire.h>
#include "QN8027.h"

// Usually once a few seconds
void QN8027::rdsTick(void)
{
	_rotateRdsBuf(4);
}

void QN8027::rdsRefresh(void)
{
	_writeRds();
}

void QN8027::_rotateRdsBuf(uint8_t num)
{
	if (rdsBufPtr + num < rdsBufLength)
	{	
		rdsBufPtr += num;
		pageNumber++;
		
		if (!(pageNumber % 2))
		{
			rdsDisplayPtr++;
		}
		
	}
	else
	{	
		rdsBufPtr = 0;
		pageNumber = 0;
		rdsDisplayPtr = 0;
	}

}

void QN8027::_writeRegister(uint8_t addr, uint8_t data)
{	
	Wire.beginTransmission(QN8027_ADDRESS);
	Wire.write(addr);
	Wire.write(data);
	Wire.endTransmission(true);
}

uint8_t QN8027::_readRegister(uint8_t addr)
{
	Wire.beginTransmission(QN8027_ADDRESS);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(QN8027_ADDRESS, 1);
	return Wire.read();
}

void QN8027::_updateRegister(uint8_t addr, uint8_t data, uint8_t mask)
{
	uint8_t dataOld = _readRegister(addr);
	uint8_t dataNew = (dataOld & ~mask) | (data & mask);
	_writeRegister(addr, dataNew);
}

void QN8027::rdsBufSet(const char buf[RDSBUF_SIZE], uint16_t length)
{
	length -= 1;
	memcpy(rdsBuf, buf, length);
	memset(&rdsBuf[length], ' ', RDSBUF_SIZE - length);
	rdsBufPtr = 0;
	rdsBufLength = length;
	pageNumber = 0;
	rdsDisplayPtr = 0;
	pages = (uint16_t)((length >> 3)); // Division by 8 - fast math

}

void QN8027::begin(float fmFreq)
{
	Wire.begin();

	uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(fmFreq)));
	
	reset();

	_setFrequency(freqChannel);
}

void QN8027::begin(uint8_t sda, uint8_t scl)
{
	Wire.begin(sda, scl);

	uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(FREQ_DEFAULT)));
	
	reset();

	_setFrequency(freqChannel);

}

void QN8027::begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq)
{
	Wire.begin(sda, scl, i2cFreq);

	uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(fmFreq)));
	
	reset();

	_setFrequency(freqChannel);

	enable(true);

}

void QN8027::setTxPower(uint8_t pwr) // 20 - 75
{
	uint8_t data;
	if (pwr < 20)
		data = 20;
	else if (pwr > 75)
		data = 75;
	else 
		data = pwr;
		
	_writeRegister(QN8027_REG_PAC, data);
}

void QN8027::enable(boolean enable)
{
	_updateRegister(QN8027_REG_SYSTEM, (enable << QN8027_BIT_IDLE), QN8027_MASK_IDLE);
}

void QN8027::rdsEnable(boolean enable)
{
	_updateRegister(QN8027_REG_RDS, (enable << QN8027_BIT_RDSEN), QN8027_MASK_RDSEN);
}

void QN8027::_setFrequency(uint16_t freqChannel)
{
	_writeRegister(QN8027_REG_CH1, (uint8_t)freqChannel);
	_updateRegister(QN8027_REG_SYSTEM, (uint8_t)(freqChannel >> 8), QN8027_MASK_CH_HI);
}

void QN8027::setFrequency(float freq)
{
	uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(freq)));

	if (freqChannel > 640)
	{
		freqChannel = 640;
	}
	
	_setFrequency(freqChannel);
}

// If a user is time constrained they can optimize these delay times but it is not really necessary as you initialize the QN8027 only once
void QN8027::reset(void)
{
	_writeRegister(QN8027_REG_SYSTEM, 1 << QN8027_BIT_RESET);

	delay(80);

	_writeRegister(QN8027_REG_XTL, (((uint8_t)CLOCK_XTAL) << QN8027_BIT_XINJ) | 0x0f);

	delay(80);

	_writeRegister(QN8027_REG_VGA, (((uint8_t)CLOCKSPEED_12) << QN8027_BIT_XSEL) | (((uint8_t)GAIN_0) << QN8027_BIT_GVGA) | (((uint8_t)TXGAIN_2) << QN8027_BIT_GDB) | (((uint8_t)RIN_1) << QN8027_BIT_RIN)); //0b1010101 //4-2-2

	delay(80);

	_writeRegister(QN8027_REG_SYSTEM, 1 << QN8027_BIT_RECAL);

	delay(80);

	_writeRegister(QN8027_REG_RDS, 10); // Experimental - Sets N*0,35 kHz as a RDS Freq deviation

	delay(80);

	_writeRegister(QN8027_REG_SYSTEM, 0x00);

	delay(80);

	_writeRegister(QN8027_REG_GPLT, 0x39);

	delay(80);

}

void QN8027::getCurrentRds(char rdsData[9])
{
	char buf[9];
	buf[8] = '\0';
	memcpy(&buf[0], &rdsBuf[rdsDisplayPtr * 8], 8);
	memcpy(rdsData, buf, 9);
}

void QN8027::_writeRds()
{
	
	_writeRegister(QN8027_REG_RDSD0, 0b01000000); //0b00100000
	_writeRegister(QN8027_REG_RDSD1, 0b00000000); //0b00000000
	_writeRegister(QN8027_REG_RDSD2, 0b00001000); //0b00100000
	_writeRegister(QN8027_REG_RDSD3, (uint8_t)(0b01000000 | (pageNumber & 0b1111)));

	_writeRegister(QN8027_REG_RDSD4, (uint8_t)rdsBuf[rdsBufPtr]);
	_writeRegister(QN8027_REG_RDSD5, (uint8_t)rdsBuf[rdsBufPtr + 1]);
	_writeRegister(QN8027_REG_RDSD6, (uint8_t)rdsBuf[rdsBufPtr + 2]);
	_writeRegister(QN8027_REG_RDSD7, (uint8_t)rdsBuf[rdsBufPtr + 3]);
	
	//if (_readRegister(QN8027_REG_STATUS) & (1 << QN8027_BIT_RDS_UPD))
	{
	
		uint8_t tmp = _readRegister(QN8027_REG_SYSTEM);

		boolean rdsUpdated = (tmp & (1 << QN8027_BIT_RDSRDY)) ? false : true;
		if (!rdsUpdated)
		{
			_writeRegister(QN8027_REG_SYSTEM, (tmp & (0xff - (1 << QN8027_BIT_RDSRDY))));
		}
		else
		{
			_writeRegister(QN8027_REG_SYSTEM, tmp | (1 << QN8027_BIT_RDSRDY));
		}
	}
	//else
	{
		//Serial.printf("Not updating RDS, exit.\r\n");
	}

}


