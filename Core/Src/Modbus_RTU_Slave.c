#include "Modbus_RTU_Slave.h"

extern uint8_t RxData[256];
extern uint8_t TxData[256];
extern UART_HandleTypeDef huart1;

uint16_t Holding_Registers_Database[50]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   40001-40010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 40011-40020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 40021-40030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 40031-40040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 40041-40050
};

uint16_t Input_Registers_Database[50]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   30001-30010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 30011-30020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 30021-30030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 30031-30040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 30041-30050
};

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1] | 0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	SendData(TxData, 3);         // send Data... CRC will be calculated in the function
}

void CheckError (uint16_t StartAddr, uint16_t NumRegs)
{
	if ((NumRegs<1)||(NumRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
	}

	uint16_t EndAddr = StartAddr + NumRegs - 1;  // end Register
	if (EndAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
	}
}

uint8_t ReadHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);   // number to registers master has requested

	CheckError (startAddr, numRegs);
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs * 2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i = 0; i < numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr] >> 8) & 0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr]) & 0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	SendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t ReadInputRegs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4] << 8) | RxData[5]);   // number to registers master has requested

	CheckError (startAddr, numRegs);

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs * 2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i = 0; i < numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Input_Registers_Database[startAddr] >> 8) & 0xFF;  // extract the higher byte
		TxData[indx++] = (Input_Registers_Database[startAddr]) & 0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	SendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

void TransmitDataToMaster(void)
{
	if (RxData[0] == SLAVE_ID)
		{
			switch (RxData[1]){
			case 0x03:
				ReadHoldingRegs();
				break;
			case 0x04:
				ReadInputRegs();
				break;
			default:
				break;
			}
		}

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 8);
}
