#include "em_timer.h"
#include "src/timer0.h"
#include "em_core.h"

void TIMER0_IRQHandler(void)
{
  CORE_ATOMIC_IRQ_DISABLE();
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IFC_UF);
 // TIMER0->CMD= TIMER_CMD_STOP;
  CORE_ATOMIC_IRQ_ENABLE();
}





void timerinit()
{
TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = false,
	.prescale   = timerPrescale1024,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeDown,
    .oneShot    = true,

  };

TIMER_Init(TIMER0, &timerInit);

TIMER_IntEnable(TIMER0, TIMER_IF_UF);

/* Enable TIMER0 interrupt vector in NVIC */
NVIC_EnableIRQ(TIMER0_IRQn);
TIMER_Enable(TIMER0,true);
/* Set TIMER Top value */
TIMER_TopSet(TIMER0, 3000);
//TIMER0->CMD= TIMER_CMD_START;


}
