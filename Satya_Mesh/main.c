/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs BT Mesh Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/
/* C Library Headers*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>



/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"



/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>
/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/*My Source Headers*/

#include "src/letimer.h"
#include "src/cmu.h"
#include "src/i2c.h"
#include "src/gpio.h"
#include <stdint.h>
#include "lcd_driver.h"

/*OOB AUTHENTICATION PARAMETERS*/
#define PK (0x00)
#define AM (0x03)
#define OA (0x03)
#define OS (0x04)
#define IA (0x00)
#define IS (0x04)
#define BITMASK (0x01)




/*LED STATE DEFINITIONS*/
#define LED_STATE_OFF (0)
#define LED_STATE_ON  (1)


/*LED MACROS*/
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_PORT 		gpioPortF
#define LED1_PIN		5
#define LED0_PIN 		4
#define LED_DEFAULT_STATE  0



/*BUTTON PORT*/
#define BUTTON_PORT	gpioPortF
#define PB0 (6)
#define PB1 (7)

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

/*SOFT TIMER ID's*/

#define TIMER_ID_FACTORY_RESET  (77)
#define TIMER_ID_RETRANS    	(10)
#define TIMER_ID_FRIEND_FIND 	(20)
#define TIMER_ID_RESTART    	(78)
#define TIMER_ID_SAVE_STATE   	(60)
#define TIMER_ID_SIGNAL 		(40)

static int ps_load(void);
static int ps_store(void);
static void ps_changed(void);



/*variables declaration*/
  static uint16 _elem_index=0xFFFF;
  uint8 request_count;
  uint8 switch_pos=0;
  static uint8 trid=0;
  uint8_t num_connections;
  uint8_t conn_handle=0xFF;
  static uint16 _my_address = 0;    /* Address of the Primary Element of the Node */
  static uint8 init_done = 0;
/*Function Prototypes*/
  void mag_init(void);
  void node_init(void);
  void lpn_init(void);
/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .max_timers = 16,
};

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);


/*Persistent Data Store related structure and Function*/
static PACKSTRUCT(struct lightbulb_state {
  // On/Off Server state
  uint8_t onoff_current;
  uint8_t onoff_target;
  uint8_t mgmn_thresh;
  uint8_t power_up;
}) lightbulb_state;




static int ps_store(void);
static int ps_load(void);
void lpn_init(void);
static void LED_set_state(int);


static errorcode_t onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = lightbulb_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = lightbulb_state.onoff_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
  }

  return e;
}




static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
  printf("ON/OFF request: requested state=<%s>, transition=%lu, delay=%u\r\n",
         request->on_off ? "ON" : "OFF", transition_ms, delay_ms);
  	  	  LED_set_state(1);
  if (lightbulb_state.onoff_current == request->on_off) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Turning lightbulb <%s>\r\n", request->on_off ? "ON" : "OFF");
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      lightbulb_state.onoff_current = request->on_off;
      lightbulb_state.onoff_target = request->on_off;
      if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
    	  LED_set_state(0);
      } else {
    	LED_set_state(1);
      }
    }
    ps_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
//    onoff_response(element_index, client_addr, appkey_index);
  } else {
    onoff_update(element_index);
  }
}


static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
  if (current->on_off.on != lightbulb_state.onoff_current) {
    printf("on-off state changed %u to %u\r\n", lightbulb_state.onoff_current, current->on_off.on);

    lightbulb_state.onoff_current = current->on_off.on;
    ps_changed();
  } else {
    printf("dummy onoff change - same state as before\r\n");
  }
}

static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           onoff_request,
                                           onoff_change);

}



void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "LPN node %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  LCD_write(name, LCD_ROW_BTADDR1);
}


void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  LCD_write("\n***\nFACTORY RESET\n***", LCD_ROW_ACTION);

  /* if connection is open then close it before rebooting */


  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

static void led_init()
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(LED_PORT,LED0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(LED_PORT,LED1_PIN, gpioModePushPull, 0);
}

static void pb_init()
{
	  GPIO_PinModeSet(BUTTON_PORT,PB0, gpioModeInputPull, true);
	  GPIO_PinModeSet(BUTTON_PORT,PB1, gpioModeInputPull, true);

}



/*CHANGE LED STATE FUNCTION*/
static void LED_set_state(int state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(LED_PORT, LED0_PIN);
      TURN_LED_OFF(LED_PORT, LED1_PIN);
      break;
    case LED_STATE_ON:
      TURN_LED_ON(LED_PORT, LED0_PIN);
      TURN_LED_ON(LED_PORT, LED1_PIN);
      break;
    default:
      break;
  }
}


int ps_store(void)
{
struct gecko_msg_flash_ps_save_rsp_t* pSave;

  pSave = gecko_cmd_flash_ps_save(0x4004, sizeof(struct lightbulb_state), (const uint8*)&lightbulb_state);

  if (pSave->result) {
    printf("lightbulb_state_store(): PS save failed, code %x\r\n", pSave->result);
    return(-1);
  }
  printf("Persistent Store successful\r\n");
  return 0;
}
static void ps_changed(void)
{
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000), TIMER_ID_SAVE_STATE, 1);
}


void server_publish(void)
{
	uint16 delay;
	delay = (request_count - 1) * 50;
	int r;
	r=gecko_cmd_mesh_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,_elem_index, mesh_generic_state_on_off)->result;
	if(r){
		printf("\r\nServer Publish result=%x", r);
	}
	else{
	printf("\r\n Server Publish Sent, delay = %d\r\n", delay);
	}
	if (request_count > 0) {
			    request_count--;
			  }
}

void server_to_client(void)
{

	request_count=3;
	lightbulb_state.onoff_target=0x01;

//	onoff_update(_elem_index);
	onoff_update_and_publish(_elem_index);

//	server_publish();
	gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50), TIMER_ID_RETRANS, 0);
}

int ps_load(void)
{
	struct gecko_msg_flash_ps_load_rsp_t* pLoad;

	  pLoad = gecko_cmd_flash_ps_load(0x4004);

	  if (pLoad->result) {
	    memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
	    lightbulb_state.onoff_current = 0x01;
	    lightbulb_state.onoff_target = 0x00;
	    lightbulb_state.mgmn_thresh= 0x00;
	    return -1;
	  }

	  memcpy(&lightbulb_state, pLoad->value.data, pLoad->value.len);
	  return 0;

}

/*Configuration of Magnetometer*/
void mag_init(void)
{

	write_byte(0x1E, 0x20, 0xD0);
	write_byte(0x1E, 0x21, 0x60); //Full Scale configuration
	write_byte(0x1E, 0x22, 0x00);
	write_byte(0x1E, 0x23, 0x08);
	write_byte(0x1E, 0x24, 0x00);
	write_byte(0x1E, 0x30, 0x08);
}

void node_init(void)
{

	mesh_lib_init(malloc, free, 8);
	lpn_init();
}

void lpn_init(void)
{
  uint16 res;
//   Initialize LPN functionality.
  res = gecko_cmd_mesh_lpn_init()->result;
  if (res) {
    printf("LPN init failed (0x%x)\r\n", res);
    return;
  }
 res = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
  if (res) {
    printf("LPN conf failed (0x%x)\r\n", res);
    return;
  }

  printf("trying to find friend...\r\n");
  res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (res != 0) {
    printf("ret.code %x\r\n", res);
  }
  memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
    if (ps_load() != 0) {
      printf("ps_load() failed, using defaults\r\n");
      goto publish;
    }
    else
    {
    	lightbulb_state.power_up=0x00;
    }

switch(lightbulb_state.power_up)
{
case MESH_GENERIC_ON_POWER_UP_STATE_OFF:
     printf("On power up state is OFF\r\n");
     lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
     lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
     LED_set_state(LED_STATE_OFF);
     break;
   case MESH_GENERIC_ON_POWER_UP_STATE_ON:
     printf("On power up state is ON\r\n");
     break;
   case MESH_GENERIC_ON_POWER_UP_STATE_RESTORE:
     printf("On power up state is RESTORE\r\n");
     GPIO_PinOutSet(LED0_port,LED0_pin);
     break;


    publish:
	ps_changed();
	init_models();
	onoff_update(_elem_index);
	init_done=1;
}
}

void onoff_transition_complete()
{
  // transition done -> set state, update and publish
  lightbulb_state.onoff_current = lightbulb_state.onoff_target;

  printf("transition complete. New state is %s\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");

  ps_changed();
  onoff_update_and_publish(_elem_index);
}




int main()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();
  cmu_init();
  gpio_init();
  pb_init();
  LCD_init("BLE LPN");
  led_init();
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);
  i2c_init();
  mag_init();
  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_endpoint_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
//  gecko_bgapi_class_sm_init();

  gecko_bgapi_class_mesh_node_init();
  gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
//  gecko_bgapi_class_mesh_proxy_client_init();
//  gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  gecko_bgapi_class_mesh_lpn_init();
  gecko_initCoexHAL();
  int j= ps_load();
  if(j==0)
  {
	  uint8_t mgmn_data= lightbulb_state.mgmn_thresh;
	  printf("Persistent Load Successful\r\n");
	  printf("\r\n Data= %x", mgmn_data);
  }
  else
	  printf("PS Load failed\r\n");
  data_receive=read_status(0x1E, WHO_AM_I);
  printf("\n\r My Address: %x ",data_receive);
  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  uint16_t result;
  switch (evt_id) {
  case gecko_evt_system_boot_id:
  printf("In BOOT ID");
  if (GPIO_PinInGet(BUTTON_PORT, PB0) == 0 || GPIO_PinInGet(BUTTON_PORT, PB1) == 0) {
         initiate_factory_reset();
       } else {


  //gecko_cmd_le_gap_set_advertising_timing(0, 1000*adv_interval_ms/625, 1000*adv_interval_ms/625, 0, 0);
  	  gecko_cmd_le_gap_set_mode(2, 2);

  	 struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
  	 set_device_name(&pAddr->address);
       }
    result = gecko_cmd_mesh_node_init_oob(PK, AM, OA, OS, IA, IS, BITMASK)->result;
  printf("\r\nResult is %x\r\n", result);
      break;

    case gecko_evt_mesh_node_display_output_oob_id:
    {
    	   printf("In OOB ID");
           struct gecko_msg_mesh_node_display_output_oob_evt_t *pOOB = (struct gecko_msg_mesh_node_display_output_oob_evt_t *)&(evt->data);
           printf("gecko_msg_mesh_node_display_output_oob_evt_t: action %d, size %d\r\n", pOOB ->output_action, pOOB->output_size);
           for(int i=0;i<pOOB->data.len;i++)
           {
               printf("%2.2x ", pOOB->data.data[i]);
           }
           printf("\r\n");
    }
    	break;


    case gecko_evt_mesh_node_model_config_changed_id:
        printf("model config changed\r\n");
        break;


    case gecko_evt_system_external_signal_id:

    	server_to_client();
//    	data_receive=read_status(0x1E, WHO_AM_I);
//    	printf("\n\r My Address: %x ",data_receive);
//    	data_receive=read_status(0x1E, 0x29);
//    	xAxis=data_receive;
//    	xAxis=(xAxis<< 8);
//    	data_receive=read_status(0x1E, 0x28);//x low
//    	xAxis= xAxis|(data_receive);
//    	float xAxisf= (xAxis*SENSITIVITY);
//    	printf("\tX Final: %1.2f",xAxisf);
//    	data_receive=read_status(0x1E, 0x2B);
//    	yAxis= data_receive;
//    	data_receive=read_status(0x1E, 0x2A);
//    	yAxis=(yAxis<< 8);
//    	yAxis= yAxis|(data_receive);
//    	float yAxisf= yAxis*SENSITIVITY;
//    	printf("\tY Final: %1.2f ",yAxisf);
//    	int16_t temp= temper();
//    	printf("\r\n Temperature is %d", temp);

    	break;

    case gecko_evt_mesh_node_initialized_id:
         printf("node initialized\r\n");

         gecko_cmd_mesh_generic_server_init();
         struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);
         if (pData->provisioned) {
           printf("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);
           _elem_index = 0;
           node_init();
           init_models();
           LetimerSetup();

         }
           else{
           printf("starting unprovisioned beaconing...\r\n");
           //gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
           gecko_cmd_mesh_node_start_unprov_beaconing(0x2);   // enable GATT provisioning bearer
           }
           break;


    case gecko_evt_mesh_generic_server_client_request_id:
          printf("evt gecko_evt_mesh_generic_server_client_request_id\r\n");
          printf("Model ID 0x%x", evt->data.evt_mesh_generic_server_client_request.model_id);

          mesh_lib_generic_server_event_handler(evt);
          break;



    case gecko_evt_hardware_soft_timer_id:
    	switch (evt->data.evt_hardware_soft_timer.handle) {
    	case TIMER_ID_FACTORY_RESET:
    		gecko_cmd_system_reset(0);
    		break;

    	case TIMER_ID_RETRANS:
    		printf("\r\n In Timer ID Retrans\r\n");
    		server_publish();
//    		send_onoff_request(1);   // param 1 indicates that this is a retransmission
    		// stop retransmission timer if it was the last attempt
    		if (request_count == 0) {
    			gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_RETRANS, 0);
    		}
    		break;

    	case TIMER_ID_FRIEND_FIND:
    	{
    		printf("trying to find friend...\r\n");
    		result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

    		if (result != 0) {
    			printf("ret.code %x\r\n", result);
    		}
    	}
                    break;

    	  case TIMER_ID_SAVE_STATE:
    	          ps_store();
    	          break;

          }
              break;



    case gecko_evt_mesh_node_provisioning_started_id:
          printf("Started provisioning\r\n");

          // start timer for blinking LEDs to indicate which node is being provisioned
           break;

    case gecko_evt_mesh_node_provisioned_id:
    	printf("Node Successfully provisioned");
    	node_init();
    	 LetimerSetup();
    	 _elem_index = 0;
    	break;


    case gecko_evt_mesh_node_key_added_id:
       printf("got new %s key with index %x\r\n", evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
              evt->data.evt_mesh_node_key_added.index);
       break;


    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
         LCD_write("", LCD_ROW_CONNECTION);
          lpn_init();
        }
      }

      break;

      break;

    case gecko_evt_le_connection_opened_id:
    	num_connections++;
         gecko_cmd_mesh_lpn_deinit(); //Turn of LPN Feature
         conn_handle=evt->data.evt_le_connection_opened.connection;
         LCD_write("Connected", LCD_ROW_ACTION);
         gecko_cmd_mesh_lpn_deinit();
         LCD_write("LPN OFF", LCD_ROW_ACTION);
         break;


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
    case gecko_evt_mesh_lpn_friendship_established_id:
          printf("friendship established\r\n");
//          DI_Print("LPN", DI_ROW_LPN);
          break;

        case gecko_evt_mesh_lpn_friendship_failed_id:
          printf("friendship failed\r\n");
          LCD_write("no friend", LCD_ROW_ACTION);
//           try again in 2 seconds
          result  = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000), TIMER_ID_FRIEND_FIND, 1)->result;
          if (result) {
            printf("timer failure?!  %x\r\n", result);
          }
          break;

        case gecko_evt_mesh_lpn_friendship_terminated_id:
          printf("friendship terminated\r\n");
          LCD_write("friend lost", LCD_ROW_ACTION);
          if (num_connections == 0) {
            // try again in 2 seconds
            result  = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000), TIMER_ID_FRIEND_FIND, 1)->result;
            if (result) {
              printf("timer failure?!  %x\r\n", result);
            }
          break;

        default:
   //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
 break;
  }
  }

}
