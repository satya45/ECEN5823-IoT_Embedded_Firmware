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
#include <math.h>
//Macros
#define UINT32_TO_FLT(data)       (((float)((int32_t)(data) & 0x00FFFFFFU)) * (float)pow(10, ((int32_t)(data) >> 24)))
#define MIN_INTERVAL (60)
#define MAX_INTERVAL (60)
#define LATENCY (3)
#define TIMEOUT (300)
#define PASSIVE (1)
#define LE1MPHY (5)
#define SCANINTERVAL (0x10)
#define SCANWINDOW (0x10)
#define CONFIGFLAGS (0x0F)
bd_addr addr= { .addr= {0xea, 0x2f, 0xef, 0x57, 0x0b, 0x00}}; //SLAVE ADDRESS
//bd_addr addr= { .addr= {0xd3, 0x7e, 0xa9, 0x9f, 0xfd, 0x90}}; //SLAVE ADDRESS
//bd_addr addr= { .addr= {0xb6, 0x80, 0xf1, 0x57, 0x0b, 0x00}}; //SLAVE ADDRESS

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
/*
 * @brief  Main function
 */
const uint8_t thermoService[2] = { 0x09, 0x18 };
const uint8_t thermoChar[2] = { 0x1c, 0x2a };
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
  LCD_init("BLE Client");
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);
  struct gecko_msg_system_get_bt_address_rsp_t *add;
  char address[40] = {0};
  float finaltemp;
  bool serv=false;
  bool charac=false;
  uint32_t service;
  uint16_t ch;
  uint8_t *temparray;
  uint8_t temp_c[5];

  while (1)
  {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      case gecko_evt_system_boot_id:
    	   printf("Boot\r\n");
    	   gecko_cmd_system_set_tx_power(0);
    	   gecko_cmd_sm_delete_bondings(1);
    	   printf("Delete all bondings\n");
    	   LCD_write("Deleted All bondings", LCD_ROW_ACTION);
    	   gecko_cmd_sm_configure(CONFIGFLAGS,sm_io_capability_displayyesno);
    	   gecko_cmd_sm_set_bondable_mode(1);
    	   gecko_cmd_le_gap_set_discovery_timing(LE1MPHY, SCANINTERVAL, SCANWINDOW);
    	   gecko_cmd_le_gap_set_discovery_type(LE1MPHY, PASSIVE);
    	   gecko_cmd_le_gap_connect(addr,le_gap_address_type_public, 0x01);
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
    	   break;

      case gecko_evt_le_connection_opened_id:
    	   printf("Connection Awaited\r\n");
    	   LCD_write("Connection initiated", LCD_ROW_ACTION);
    	   gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, MIN_INTERVAL,MAX_INTERVAL,LATENCY,TIMEOUT);
    	   uint32_t activeConnectionId = evt->data.evt_le_connection_opened.connection;
    	   gecko_cmd_sm_increase_security(activeConnectionId);
    	   break;



     case gecko_cmd_sm_bonding_confirm_id:
    	  LCD_write("Press PB0 to confirm", LCD_ROW_ACTION);
    	  gecko_cmd_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection,1);
    	  break;

      case gecko_evt_gatt_service_id:
    	  printf("Inside service id");
    	  serv=true;
    	  service= evt->data.evt_gatt_service.service;
    	  break;



      case gecko_evt_gatt_characteristic_id:
    	  printf("I am in Charac Id\n");
    	  charac=true;
    	  ch=evt->data.evt_gatt_characteristic.characteristic;
    	  break;


      case gecko_evt_gatt_characteristic_value_id:

    	  printf("I am in char value\n");
    	  gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
    	  temparray=&evt->data.evt_gatt_characteristic_value.value.data[0];
    	  memcpy(temp_c,temparray,5);
    	  finaltemp= UINT32_TO_FLT(*(uint32_t*)&temp_c[1]);
    	  char temper[32];
    	  snprintf(temper,32, "Temperature:%2.2fC", finaltemp);
    	  printf("%f", finaltemp);
    	  LCD_write(temper, LCD_ROW_TEMPVALUE);
    	  break;



      case gecko_evt_gatt_procedure_completed_id:
    	  if (serv==true)
    	      	  {
    	      		  serv=false;
    	      		  gecko_cmd_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
    	      		                                                            service,
    	      		                                                            2,
    	      		                                                            (const uint8_t*)thermoChar);
    	      	  }
    	  if (charac==true)
    	  	  {

    		  charac=false;
    		  gecko_cmd_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
    		                                                         ch,
    		                                                         gatt_indication);
    	  	  }

    	  break;


      case gecko_evt_sm_confirm_passkey_id:
    	  printf("Confirm passkey\r\n");
    	  char passkey[20];
    	  snprintf(passkey,sizeof(passkey),"%lu",evt->data.evt_sm_confirm_passkey.passkey);
    	  LCD_write(passkey,LCD_ROW_PASSKEY);
    	  LCD_write("Press PB0 to confirm", LCD_ROW_ACTION);
    	  while(GPIO_PinInGet(LED0_port,PB0	)!=0);
    	  LCD_write("",LCD_ROW_PASSKEY);
    	  gecko_cmd_sm_passkey_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
    	  printf("Passkey Confirmed\r\n");
    	  LCD_write("Passkey Confirmed", LCD_ROW_ACTION);
    	  break;

      case gecko_evt_sm_bonded_id:

    	  printf("Bonded");
    	  LCD_write("Bonded", LCD_ROW_ACTION);
    	  gecko_cmd_gatt_discover_primary_services_by_uuid(evt->data.evt_sm_bonded.connection,
    	                                                          2,
    	                                                          (const uint8_t*)thermoService);
    	  break;

     case gecko_evt_sm_bonding_failed_id:
    	  LCD_write("Bonding Failed", LCD_ROW_ACTION);
    	  gecko_cmd_le_gap_set_discovery_timing(0x05, 0x10,0x10);
    	  gecko_cmd_le_gap_set_discovery_type(0x05, 0x01);

    	  break;

      case gecko_evt_le_connection_closed_id:
    	  LCD_write("Disconnected", LCD_ROW_ACTION);
    	  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
    	  gecko_cmd_le_gap_connect(addr,le_gap_address_type_public, 0x01);
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu)
        {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else
        {
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
