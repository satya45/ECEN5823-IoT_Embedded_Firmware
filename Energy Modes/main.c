
/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/


//emlib letimer header
#define EM 3 //Define the energy mode which you want to block. For example to run in EM2 define EM 3




#include "em_letimer.h"
#include "src/letimer.h"
#include "src/cmu.h"
#include "src/sleep.h"

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "src/gpio.h"
#include "em_device.h"
#include "em_core.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;


int sleep_block[5];


int sleep_block_counter;

int main(void)
{
	switch (EM){
	    case (0):
	    sleep_block_counter=0;
	    break;
	    case (1):
	    sleep_block_counter=1;
	    break;
	    case (2):
	    sleep_block_counter=2;
	    break;
	    case (3):
	    sleep_block_counter=3;
	    break;
	    case (4):
	    sleep_block_counter=4;

	//  default:


	}


  mode = EM;

  initMcu();
  initBoard();
  gpio_init();
  gecko_init(&config);


  cmu_init();
  LetimerSetup(); //call letimer initialization routine
  LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0 | LETIMER_IFC_UF);
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);  //comp1 interrupt enable
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);//Comp0 interrupt enable
  blockSleepMode(mode);
  NVIC_EnableIRQ(LETIMER0_IRQn);
  LETIMER_Enable(LETIMER0, true);


    //tp=period;


  while (1)
  {
	  //EMU_EnterEM3(false);

	  	sleep(); //call sleep routine

  }
  return 0;
}

