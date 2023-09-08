#ifndef __MODBUS_RTU_SLAVE_H
#define __MODBUS_RTU_SLAVE_H

#include "Modbus_CRC.h"

#define SLAVE_ID 5

#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03

void TransmitDataToMaster(void);

#endif
