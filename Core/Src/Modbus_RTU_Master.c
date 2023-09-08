#include "Modbus_RTU_Master.h"

extern uint8_t TxData[256];
extern uint8_t RxData[256];
extern uint16_t Data[256];
extern UART_HandleTypeDef huart1;

uint16_t startAddr = 0;  // start Register Address
uint16_t numRegs = 5;   // number to registers

#define SLAVE_ID 2

//#define Function_Code 0x01	// Function code for Read Coils
#define Function_Code 0x02	// Function code for Read Inputs
//#define Function_Code 0x03  // Function code for Read Holding Registers
//#define Function_Code 0x04	// Function code for Read Input Registers

void RequireData (void)
{
	TxData[0] = SLAVE_ID;  // Slave ID
	TxData[1] = Function_Code;  // function code
	//The Register or Coils address
	TxData[2] = (startAddr >> 8) & 0xFF;
	TxData[3] = startAddr & 0xFF;
	//no of Register or Coils
	TxData[4] = (numRegs >> 8) & 0xFF;
	TxData[5] = numRegs & 0xFF;

	SendData(TxData, 6);  // send data... CRC will be calculated in the function itself
}

void RecieveData (void)
{

	if ((Function_Code == 0x03) || (Function_Code == 0x04))
	{
		int indx = 0;

		while (indx <= numRegs)
		{
			Data[indx] = RxData[indx + 3] << 8 | RxData[indx + 4];
		}

		indx++;
	}
	else if((Function_Code == 0x01) || (Function_Code == 0x02))
	{
		uint8_t numByte = numRegs / 8;

		if ((numRegs % 8) != 0)
		{
			numByte++;
		}

		int indx = 0;

		while (indx <= numByte)
		{
			Data[indx] = RxData[indx + 3] << 8 | RxData[indx + 4];
		}

		indx++;
	}

}

void RecieveDataFromSlave (void)
{
	RequireData();
	RecieveData();

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
}
