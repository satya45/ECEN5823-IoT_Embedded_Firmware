#include "em_i2c.h"

#define slaveAdd 0x40
#define tempMeasure 0xE3
#define event1 0x01


uint16_t schedule_event;

/*function declarations*/

void i2cinit(void);
uint32_t tempread(void);
float degree(float);
void i2cdisable(void);

