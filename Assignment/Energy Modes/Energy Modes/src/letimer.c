#include "letimer.h"
#include "src/gpio.h"
#include "em_core.h"
#include "src/sleep.h"
#include "src/cmu.h"
#include "src/main.h"
#include "math.h"
#include "main.h"

#define ledontime 0.500



int freq= 32768;
int comp0value;
volatile int mode;
int comp1value;

void LetimerSetup(void)
{


	LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);
const LETIMER_Init_TypeDef letimerinitialization =
{ 													//from silicon labs example

	  .enable         = false,                   /*  start counting when init completed - only with RTC compare match */
	  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */

	  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
	  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	  .out0Pol        = 0,                      /* Idle value for output 0. */
	  .out1Pol        = 0,                      /* Idle value for output 1. */
	  .ufoa0          = letimerUFOANone,      /* Pulse output on output 0 */
	  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
	  .repMode        = letimerRepeatFree       /* Repeat indefinitely */
	  };

	LETIMER_Init(LETIMER0, &letimerinitialization);
	while (LETIMER0->SYNCBUSY!=0);
	LETIMER0->IFC&=0x00000000;

if (sleep_block_counter<=3)
{
	int ticks;
		  ticks= freq * period;
		  comp0value=0;

	if (ticks > 65535)
			{
			int j;
				j = ticks/65535;
				j= pow(2,j);
				comp0value = ticks/j;
				comp1value= (freq/j)*ledontime;

				CMU_ClockDivSet(cmuClock_LETIMER0, j); // Divide clock by j

			}
			else if (ticks<=65535)
			{
				comp0value=ticks;
			}
}
else if (sleep_block_counter>3)
{
	int ticks;
		  ticks= 1000*period;
		  comp0value=0;
	if (ticks > 65535)
			{
			int j;
				j = ticks/65535;
				j= pow(2,j);
				comp0value = ticks/j;
				comp1value= (freq/j)*ledontime;
				CMU_ClockDivSet(cmuClock_LETIMER0, j); // Divide clock by j

			}
			else if (ticks<=65535)
			{
				comp0value=ticks;
				comp1value= (comp0value-(1000*ledontime));
			}
}

		LETIMER_CompareSet(LETIMER0, 0, comp0value);
		LETIMER_CompareSet(LETIMER0, 1, comp1value);


}



void LETIMER0_IRQHandler(void)
	{

	 CORE_ATOMIC_IRQ_DISABLE ();

	  int flags;
	  	flags = LETIMER_IntGet(LETIMER0);
	  	if(flags & LETIMER_IF_COMP1)
	  		   		{
	  		   			GPIO_PinOutSet(gpioPortF, 4);
	  		   			LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);
	  		   		}

	  	else  if(flags & LETIMER_IF_COMP0)
	   	{

	   		GPIO_PinOutClear(gpioPortF, 4);
	   		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);



	   	}







	   CORE_ATOMIC_IRQ_ENABLE();

	   //unblockSleepMode(mode);

	}
