/*
 * sleep.h
 *
 *  Created on: Sep 14, 2018
 *      Author: satya
 *
 */
#include "em_emu.h"
#include "em_core.h"


#ifndef SRC_SLEEP_H_
#define SRC_SLEEP_H_

extern int sleep_block_counter;
extern int sleep_block[5];







void sleep(void);
extern void blockSleepMode(int);
extern void unblockSleepMode(int);












#endif /* SRC_SLEEP_H_ */
