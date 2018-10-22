#include "i2c.h"



float temp_read(void)
{

	uint32_t temp = 0;

	temp= tempread();
	float tempf =((float)temp * 175.72);
	tempf = tempf/65536;
	tempf = tempf-46.85;
	return tempf;



}
