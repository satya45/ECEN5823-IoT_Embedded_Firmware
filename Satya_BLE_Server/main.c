/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Thermometer Example Application
 *
 * This Thermometer and OTA example allows the user to measure temperature
 * using the temperature sensor on the WSTK. The values can be read with the
 * Health Thermometer reader on the Blue Gecko smartphone app.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silicon Labs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "lcd_driver.h"
#include <stdio.h>
#include <retargetserial.h>

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#else
#error This sample app only works with a Silicon Labs Board
#endif

#include "em_emu.h"
#include "em_cmu.h"
#include "src/gpio.h"
#include "em_device.h"
#include "em_core.h"
#include "em_letimer.h"
#include "src/letimer.h"
#include "src/cmu.h"
#include "src/i2c.h"
#include "src/temp.h"
#include "src/gpio.h"
/*******************
 * ****************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

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

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */
float temperatureMeasure()
{

  uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
  uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
  float temperature;   /* Stores the temperature data read from the sensor in the correct format */
  uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */

  /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
  	 UINT8_TO_BITSTREAM(p, flags);
  	 float x = temp_read();

  	 //float y=degree(x);
 	 temperature=x;
 	 if (temperature <= 15)
 	 {
 		 GPIO_PinOutSet(gpioPortF,5);
 	 }




 	 uint32_t tmp =	FLT_TO_UINT32(temperature * 1000 , -3);
    /* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
    UINT32_TO_BITSTREAM(p, tmp);

    /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
     * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
     *  0xFF as connection ID will send indications to all connections. */
    gecko_cmd_gatt_server_send_characteristic_notification(
      0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
    //i2cdisable();
    return x;
  }

/**
 * @brief  Main function
 */
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  cmu_init();
  gpio_init();
  // Initialize stack
  gecko_init(&config);
  // Initialize the Temperature Sensor
  LetimerSetup();
  LCD_init("BLE Server");
  LCD_write("PB1 to del bond", LCD_ROW_ACTION);
  int16_t rssi,power;
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);
  	struct gecko_msg_system_get_bt_address_rsp_t *add;
    char address[40] = {0};
    char temp[32]={0};
  	bool read_y_n=false;
  while (1)
  {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
    	   printf("Boot\r\n");
//    	   while(GPIO_PinInGet(gpioPortF, 7)!=0);

    	   			  /* delete all bondings to force the pairing process */
    	   			  gecko_cmd_sm_delete_bondings();
    	   			  printf("Delete all bondings\n");
    	   			  LCD_write("Deleted All bondings", LCD_ROW_ACTION);
    	   			  gecko_cmd_sm_configure(0x0F,sm_io_capability_displayyesno);
    	   			  gecko_cmd_sm_set_bondable_mode(1);
    	   			  //gecko_cmd_le_gap_set_adv_parameters(160,160,7);
    	   			  add = gecko_cmd_system_get_bt_address();
    	   			  LCD_write("ADDR", LCD_ROW_BTADDR1);
    	   			  snprintf(address, 40, "%02x:%02x:%02x:%02x:%02x:%02x",
    	  				  add->address.addr[5],
    	  				  add->address.addr[4],
    	  				  add->address.addr[3],
    	  				  add->address.addr[2],
    	  				  add->address.addr[1],
    	  				  add->address.addr[0]
    	  	  	  );
    	   			  LCD_write(address, LCD_ROW_BTADDR2);
    	   			  /* Set advertising parameters. 100ms advertisement interval.
    	   			   * The first two parameters are minimum and maximum advertising interval, both in
    	   			   * units of (milliseconds * 1.6). */
    	   			  gecko_cmd_le_gap_set_advertise_timing(0, 400, 400, 0, 0);
    	   			  //gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
    	   			  /* Start general advertising and enable connections. */
    	   			  gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
    	   			  break;

      case gecko_evt_le_connection_opened_id:
    	  	  	  	   printf("Connection Awaited\r\n");
    	  	  	  	   LCD_write("Connection initiated", LCD_ROW_ACTION);
                       gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, 60,60,3,300);
                       uint32_t activeConnectionId = evt->data.evt_le_connection_opened.connection;
                       gecko_cmd_sm_increase_security(activeConnectionId);
                       break;


                 /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
      case gecko_evt_gatt_server_characteristic_status_id:
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
    	 // gecko_cmd_le_connection_get_rssi(evt-> data.evt_le_connection_opened.connection);

    	  if (evt-> data.evt_gatt_server_characteristic_status.status_flags == gatt_server_confirmation)
    	     	    	  {
    	     	    		  gecko_cmd_le_connection_get_rssi(evt-> data.evt_gatt_server_characteristic_status.connection);
    	     	    	  }
    	  	  	  	  	  break;

      	  case gecko_evt_le_connection_rssi_id:

        	      			rssi = evt->data.evt_le_connection_rssi.rssi;
        	      			if(rssi > -35)
        	      			power=260;
        	      		//gecko_cmd_system_set_tx_power(-260);
        	      			else if(rssi > -45)
        	      				power= -200;
        	      			//	gecko_cmd_system_set_tx_power(-200);
        	      			else if(rssi > -55)
        	      				power=-150;
        	      				//gecko_cmd_system_set_tx_power(-150);
        	      			else if(rssi > -65)
        	      				power= -50;
//        	      				gecko_cmd_system_set_tx_power(-50);
        	      			else if(rssi > -75)
        	      				power=0;
//        	      				gecko_cmd_system_set_tx_power(0);
        	      			else if(rssi > -85)
        	      				power=50;
//        	      				gecko_cmd_system_set_tx_power(50);
        	      			else
        	      				power=80;
//        	      				gecko_cmd_system_set_tx_power(80);
        	      			gecko_cmd_system_halt(1);
        	      			gecko_cmd_system_set_tx_power(power);
        	      			gecko_cmd_system_halt(0);
        	      			break;

       case gecko_evt_sm_passkey_display_id:

    	   	   	 printf("Passkey display\r\n");

                 printf("Enter this passkey on your phone:\r\n%d\r\n",evt->data.evt_sm_passkey_display.passkey);

                 break;



      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_system_external_signal_id:
    	  	  	  	  	  if (read_y_n==true)
    	  	  	  	  	  {
                    				i2cinit();
                    				float y=temperatureMeasure();
                    	        	snprintf(temp, 32, "%2.2f", y);
                    	        	LCD_write(temp, LCD_ROW_TEMPVALUE);

    	  	  	  	  	  }
                    	        	break;
      /*case gecko_evt_sm_passkey_request_id:
    	  printf("Passkey request\r\n");
    	  printf("Enter the passkey you see on the tablet\r\n");
    	  bool read_pk=true;
    	  break;*/


      case gecko_evt_sm_confirm_passkey_id:
    	  printf("Confirm passkey\r\n");
    	  char passkey[20]={0};
    	  printf("Are you able to see the same passkey on the tablet: %lu (y/n)?\r\n", evt->data.evt_sm_confirm_passkey.passkey);
    	  snprintf(passkey,sizeof(passkey),"%lu",evt->data.evt_sm_confirm_passkey.passkey);
    	  LCD_write(passkey,LCD_ROW_PASSKEY);
    	  LCD_write("Press PB0 to confirm", LCD_ROW_ACTION);
    	  while(GPIO_PinInGet(gpioPortF,6)!=0);
    	  LCD_write("",LCD_ROW_PASSKEY);
    	  gecko_cmd_sm_passkey_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
    	  printf("Passkey Confirmed\r\n");
    	  LCD_write("Passkey Confirmed", LCD_ROW_ACTION);
    	  break;

      case gecko_evt_sm_bonded_id:

    	  printf("Bonded");
    	  LCD_write("Bonded", LCD_ROW_ACTION);
    	  read_y_n = true;
    	  break;

     case gecko_evt_sm_bonding_failed_id:
    	  LCD_write("Bonding Failed", LCD_ROW_ACTION);
    	  break;

      case gecko_evt_le_connection_closed_id:
    	  LCD_write("Disconnected", LCD_ROW_ACTION);
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;



    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
