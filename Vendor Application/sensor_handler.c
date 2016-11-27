

#include <string.h>	// For memcpy/memset.
#include <stdlib.h>
#include <stdbool.h> //for boolean
#include <stdio_serial.h> // ASF module: Standard serial I/O (stdio) for debug output
#include "logger.h"
#include "persistent.h"
#include "persistent_addresses.h"
#include "cmb_communication.h" // TODO: remove refs to here. Use queued IO instead!
#include "cmb_queued_io.h"
#include "sensor_io.h"
#include "cmb_client.h"
#include "vendor_application.h"
#include "sensor_handler.h"
#include "task_scheduler.h"


#define INVALID_TAG 255 

static struct task_info sensor_print_task;
static void sensor_print_task_do(struct task_info* self);



struct sensor_event_callback_str
	{
	Sensor_type sensor_type;
	sensor_event_cb_t cb_fun;
	};

/* Sensor events from SensorIO module */
const struct sensor_event_callback_str sensor_event_callbacks[] =
	{
	    /* sensor_type      , cb_fun */
		 {SENSOR_ADC0  , sensor_hndl_adc0_event}
	    ,{SENSOR_ADC1  , sensor_hndl_adc1_event}
	    ,{SENSOR_MOTION, sensor_hndl_motion_event}
		,{SENSOR_SWITCH, sensor_hndl_button_event}
	};

#define SENSOR_EVENT_CALLBACK_STR_N (sizeof(sensor_event_callbacks) / sizeof(struct sensor_event_callback_str))


/* Local variables to store sensor values */
uint16_t cmb_sensor_motion_status = 0;
uint16_t cmb_sensor_button_status = 0;
uint16_t adc_ch0_voltage = 0; 
uint16_t adc_ch1_voltage = 0;


/************************************************************************/
/* LOCAL FUNCTION PROTOTYPES                                            */
/************************************************************************/

/************************************************************************/
/*  PUBLIC FUNCTIONS                                                    */
/************************************************************************/ 

/** sensor_handler_set_sensor_io_callback( void )
 * register the sensor io-events to sensor_handler event handler functions 
 */
void sensor_handler_set_sensor_io_callback( void ){
	
	/* variables */
	uint8_t idx;
	
	/* Code */
	
	/* Register callback handler for the sensor types.*/
	for(idx = 0; idx < SENSOR_EVENT_CALLBACK_STR_N; idx++ ){		
		sensor_io_event_set_callback( (sensor_event_callbacks[idx].cb_fun) ,( sensor_event_callbacks[idx].sensor_type ) );		
		}
	
	/* Start periodic sensor_print task (Joar)*/
	sensor_print_task.period_time = 100*(5); // 5 second read period
	sensor_print_task.repeats = 0;
	sensor_print_task.fun = sensor_print_task_do; // callback for ADC read task
	#ifdef SCHEDULER_ENABLE_DEBUG_TASK_LOG
	sensor_print_task.dbg_task_name = "adc_read_task";
	#endif
	scheduler_task_add(&sensor_print_task);
		
	log_puts(LL_SPAM, "Sensor printing task started...");
}

static void sensor_print_task_do(struct task_info* self){
	
	//uint16_t adc1_value;
	//uint16_t adc2_value;
	//uint16_t motion_status;
	
	//sensor_value_get_adc0_voltage(&adc1_value);
	//sensor_value_get_adc1_voltage(&adc2_value);
	//sensor_value_get_motion_status(&motion_status);

	log_printf(LL_DBG, "Some sensor values: %i \n", adc_ch0_voltage);
	log_printf(LL_DBG, "Some sensor values: %i \n", adc_ch1_voltage);
	log_printf(LL_DBG, "Some sensor values: %i \n", cmb_sensor_motion_status);
	
	/* ADC Send Vendor message */

	/* First initialize the payload */
	struct vnd_msg_adc_read_str vendor_msg_adc_event;

	vnd_msg_init_payload(  (struct vnd_msg_str*) &vendor_msg_adc_event );

	/* Fill routing information */
	vendor_msg_adc_event.sender = cmb_client_id; // my client ID
	vendor_msg_adc_event.receiver = VND_MSG_BROADCAST;
	vendor_msg_adc_event.vendor_opcode = VND_OPCODE_ADC_EVENT;

	/* Fill payload if any */
	vendor_msg_adc_event.adc0_read = adc_ch0_voltage;
	vendor_msg_adc_event.adc1_read = adc_ch1_voltage;

	/* Send the vendor message */
	(void) vnd_app_send_vendor_msg( CMB_VENDORMESSAGE0, (struct vnd_msg_str*)&vendor_msg_adc_event);
	
}


/** sensor_value_get_adc0_voltage(uint16_t* value)
 * Returns latests adc0 voltage 
 */
void sensor_value_get_adc0_voltage(uint16_t* value){
	*value = adc_ch0_voltage;
}


/** sensor_value_get_adc1_voltage(uint16_t* value)
 * Returns latests adc1 voltage 
 */
void sensor_value_get_adc1_voltage(uint16_t* value){
	*value = adc_ch1_voltage;
}


/** sensor_value_get_motion_status(uint16_t* value)
 * Returns latests motions status 
 */
void sensor_value_get_motion_status(uint16_t* value){
	*value = cmb_sensor_motion_status;
}


/************************************************************************/
/* Sensor IO Event handlers                                             */
/************************************************************************/

/** sensor_hndl_adc0_event(struct sensor_event* def)
 *  EXAMPLE ADC0 event handler
 */
void sensor_hndl_adc0_event(struct sensor_event* def){

	uint8_t tag=INVALID_TAG;	
	adc_ch0_voltage = def->value_adc;	
	
	get_tag_by_sensor_type(def->type, &tag);	
	log_printf(LL_SPAM, "adc0 voltage = %imV\n", adc_ch0_voltage );
	
	if (INVALID_TAG != tag){
		// Valid TAG found, send value to Server --> stored probably to Cloud
		uint8_t argc = 0;
		uint8_t arg_buf[sizeof(adc_ch0_voltage)];// 16bit value		
		argc = sizeof(arg_buf);
		
		memcpy(&arg_buf[0], &adc_ch0_voltage, sizeof(adc_ch0_voltage));
		
		/* send CAsambi message */
		(void) cmb_send_cmd_queued(CMB_SETSENSORVALUE, arg_buf, argc);
			
	}		
}


/** sensor_hndl_adc0_event(struct sensor_event* def)
 *  EXAMPLE ADC1 event handler
 */
void sensor_hndl_adc1_event(struct sensor_event* def){

	uint8_t tag=INVALID_TAG;
	adc_ch1_voltage = def->value_adc;
	
	get_tag_by_sensor_type(def->type, &tag);	
	log_printf(LL_SPAM, "adc1 voltage = %imV\n", adc_ch1_voltage );
	
	if (INVALID_TAG != tag){				
		uint8_t argc = 0;
		uint8_t arg_buf[sizeof(adc_ch1_voltage)];
		argc = sizeof(arg_buf); // 16bit
		
		memcpy(&arg_buf[0], &adc_ch1_voltage, sizeof(adc_ch1_voltage));
				
		/* send Casambi message */
		(void) cmb_send_cmd_queued(CMB_SETSENSORVALUE, arg_buf, argc);
		
	}
}

/** sensor_hndl_button_event(struct sensor_event* def)
 *  EXAMPLE button event handler
 */
void sensor_hndl_button_event(struct sensor_event* def)
	{
	if(def->type == SENSOR_SWITCH){log_puts(LL_SPAM,"sensor_hndl_button_event sensor type is SENSOR_SWITCH\n");}
	log_puts(LL_SPAM, "button event! ");
		
	
	/* 1. For testing purposes button press will trigger VENDOR PING to network */
	
	/* First initialize the payload */
	struct vnd_msg_str vendor_msg_ping;
	
	vnd_msg_init_payload( &vendor_msg_ping );
	
	/* Fill routing information */
	vendor_msg_ping.sender = cmb_client_id; // my client ID
	vendor_msg_ping.receiver = VND_MSG_BROADCAST;
	vendor_msg_ping.vendor_opcode = VND_OPCODE_PING;
	

	/* Send the vendor message */
	(void) vnd_app_send_vendor_msg( CMB_VENDORMESSAGE0, &vendor_msg_ping);
	
	}


/** sensor_hndl_motion_event(struct sensor_event* def)
 *  EXAMPLE motion sensor event handler
 */
void sensor_hndl_motion_event(struct sensor_event* def){
	
	uint8_t tag = INVALID_TAG;
	
	cmb_sensor_motion_status = (0x00FF & def->motion_detected);
	
	log_puts(LL_DBG, cmb_sensor_motion_status ? "motion detected" : "motion ended");
	
		
	/* 1. set light on/off to Server*/	
	uint8_t argc = 0;
	uint8_t arg_buf_1[1]; // 8bit value
	uint8_t level = ( cmb_sensor_motion_status == 1 ? 255 : 0 );
	
	memcpy( &arg_buf_1[0], &level, sizeof(level) );
	argc = sizeof(arg_buf_1);
	
	/* send Casambi message */
	(void) cmb_send_cmd_queued(CMB_SETLEVEL, arg_buf_1, argc);
	
	
	/* 2. Update sensor value to CLOUD and Casambi APP UI if TAG is valid*/
#if 0
	// NOTE: IF Sensor type is PULL, this is not needed, only for PUSH sensor.
	uint8_t arg_buf_2[sizeof(cmb_sensor_motion_status)+sizeof(tag)]; 
	argc = sizeof(arg_buf_2); 
	get_tag_by_sensor_type(def->type, &tag);
	arg_buf_2[0] = tag;		
	memcpy( &arg_buf_2[1], &cmb_sensor_motion_status, sizeof(cmb_sensor_motion_status) );
	/* send Casambi message */
	(void) cmb_send_cmd_queued(CMB_SETSENSORVALUE, arg_buf_2, argc);
#endif
		
	/* 3. Send Vendor message */
	
	/* First initialize the payload */
	struct vnd_msg_motion_detected_str vendor_msg_motion_event;
	
	vnd_msg_init_payload(  (struct vnd_msg_str*) &vendor_msg_motion_event );
	
	/* Fill routing information */
	vendor_msg_motion_event.sender = cmb_client_id; // my client ID
	vendor_msg_motion_event.receiver = VND_MSG_BROADCAST;
	vendor_msg_motion_event.vendor_opcode = VND_OPCODE_MOTION_EVENT;
	
	/* Fill payload if any */
	vendor_msg_motion_event.motion_detected = cmb_sensor_motion_status;
		
	/* Send the vendor message */
	(void) vnd_app_send_vendor_msg( CMB_VENDORMESSAGE0, (struct vnd_msg_str*)&vendor_msg_motion_event);
	
}
