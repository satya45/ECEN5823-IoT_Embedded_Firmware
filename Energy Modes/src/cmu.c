//***********************************************************************************
// Include files
//***********************************************************************************
#include "src/cmu.h"
#include "src/main.h"
#include "src/sleep.h"
#include "em_cmu.h"




//***********************************************************************************
// defined files
//***********************************************************************************
int sleep_block_counter;



//***********************************************************************************
// function prototypes
//***********************************************************************************
void cmu_init()
{

	CMU_ClockEnable(cmuClock_GPIO, true);
	if (sleep_block_counter <=3)
	{
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

	}
	else if (sleep_block_counter>3)
			{
				CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);	// Enable ULFRCO
				CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);	// Select ULFRCO for clock tree LFA

			}
	CMU_ClockEnable(cmuClock_HFPER,true); //enabling high freq peripheral clock
	CMU_ClockEnable(cmuClock_I2C0,true); //enabling clock to I2C0
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LFA, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);


}
