#include "JY901.h"
#include "string.h"

CJY901 ::CJY901(HardwareSerial *serial)
{
	this->serial = serial;
}

void CJY901::begin(int baudrate) {
	serial->begin(baudrate);
	delay(1000);
	uint8_t unlock[] = {0xFF,0xF0,0xF0,0xF0,0xF0};
	serial->write(unlock, 5);
	serial->flush();
	delay(10);
	writeRegister(RSW, 0b0000000000010110);
	delay(10);
	writeRegister(RRATE, 0x09);
	delay(10);
	writeRegister(0x0b,0x00);
	delay(10);
	writeRegister(0x0c,0x00);
	delay(10);
	writeRegister(0x0d,0x00);
	delay(10);


	

	ucRxCnt = 0;
}

void CJY901 ::update()
{
	while (serial->available())
	{

		ucRxBuffer[ucRxCnt++] = serial->read();
		if (ucRxBuffer[0] != 0x55)
		{
			ucRxCnt = 0;
			continue;
		}
		if (ucRxCnt < 11)
		{
			continue;
		}
		else
		{
			switch (ucRxBuffer[1])
			{
			case 0x50:
				memcpy(&stcTime, &ucRxBuffer[2], 8);
				break;
			case 0x51:
				memcpy(&stcAcc, &ucRxBuffer[2], 8);
				break;
			case 0x52:
				memcpy(&stcGyro, &ucRxBuffer[2], 8);
				break;
			case 0x53:
				memcpy(&stcAngle, &ucRxBuffer[2], 8);
				break;
			case 0x54:
				memcpy(&stcMag, &ucRxBuffer[2], 8);
				break;
			case 0x55:
				memcpy(&stcDStatus, &ucRxBuffer[2], 8);
				break;
			case 0x56:
				memcpy(&stcPress, &ucRxBuffer[2], 8);
				break;
			case 0x57:
				memcpy(&stcLonLat, &ucRxBuffer[2], 8);
				break;
			case 0x58:
				memcpy(&stcGPSV, &ucRxBuffer[2], 8);
				break;
			}
			ucRxCnt = 0;
		}
	}
}

void CJY901::writeRegister(uint8_t address, uint16_t data) {
	uint8_t unlock[] = {0xFF,0xAA,0x69,0x88,0xB5};
	serial->write(unlock, 5);
	serial->flush();
	delay(100);

	uint8_t buffer[6];
	buffer[0] = 0xFF;
	buffer[1] = 0xAA;
	buffer[2] = address;
	buffer[3] = data & 0xFF;
	buffer[4] = (data >> 8) & 0xFF;

	serial->write(buffer, 5);
	serial->flush();

	serial->write(unlock, 5);
	serial->flush();
	delay(100);

	uint8_t save[] = {0xFF,0xAA,0x00,0x00,0x00};
	serial->write(save, 5);
	serial->flush();
	delay(100);
	
}
