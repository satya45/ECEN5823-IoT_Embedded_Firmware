#include "em_i2c.h"

#define slaveAdd 0x40
#define tempMeasure 0xE3
#define event1 0x01


uint16_t schedule_event;

/*function declarations*/

void i2cinit(void);
uint16_t tempread(void);
int degree(int);
void i2cdisable(void);

