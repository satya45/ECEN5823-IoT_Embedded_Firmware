#include "i2c.h"



uint16_t temp_read(void)
{

	uint16_t temp = 0;

	temp = tempread();

	return temp;
}
