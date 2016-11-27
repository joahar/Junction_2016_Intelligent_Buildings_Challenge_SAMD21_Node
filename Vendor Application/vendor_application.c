#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for boolean

#include "vendor_application.h"
#include "logger.h"
#include "task_scheduler.h"
#include "block_queue.h"
#include "cmb_client.h"

#include "cmb_queued_io.h"
#include "cmb_communication.h"
#include "vendor_message_defs.h"


/************************************************************************/
/* LOCAL FUNCTION PRTOTYPES                                             */
/************************************************************************/

/** vnd_msg_handle_ping()
 *  \Handler for PING vendor message
 *
 *  \casambi_opcode: Casambi command opcode. See 'cmb_opcodes.h'
 *  \vnd_msg_str:	 Generic struct for vendor messages
 *
 *  \return:  void
 */
static void vnd_msg_handle_ping(uint8_t casambi_opcode, struct vnd_msg_str* vendor_ping);


/************************************************************************/
/*  PUBLIC FUNCTIONS                                                    */
/************************************************************************/


/* vnd_app_client_init */
void vnd_app_client_init(void)
{
	//Init task scheduler if needed, e.g. start a task etc.
}


/*  vnd_msg_init_fillers */
void vnd_msg_init_payload(struct vnd_msg_str* vendor_msg) {
	
	for(int i=0; i < VND_MSG_FILLER_SZ; ++i ){
		vendor_msg->payload[i] = 0x00;
	}
}


/** vnd_app_handle_vendor_msg() : 
* EXAMPLE handler function for a received Casambi message
* Entry point for all received Vendor messages
*/
void vnd_app_handle_vendor_msg(uint8_t casambi_opcode, struct vnd_msg_str* input_vendor_msg) {
	
	// Define variables:
	uint8_t argc;
	uint8_t arg_buf_1[1];
	uint8_t level;
	
	if(input_vendor_msg == NULL) {
		return;
		}
	
	/* Check if the message shall be ignored based on routing info */
	if( ( cmb_client_id != input_vendor_msg->receiver && input_vendor_msg->receiver != VND_MSG_BROADCAST ) ) {
		log_printf(LL_SPAM, "my addr is %d, ignoring this message...\n", cmb_client_id);
		return;
		}	
	else if( cmb_client_id == input_vendor_msg->sender ) {
		log_puts(LL_SPAM, "Received my own message, ignored it\n");
		return;
		} 

	log_puts(LL_SPAM, "Handle vendor message\n");

	/* Handle the received vendor message */		
	switch(input_vendor_msg->vendor_opcode) {
		case VND_OPCODE_PING:	
			vnd_msg_handle_ping(casambi_opcode, input_vendor_msg);	
			break;
		
		case VND_OPCODE_PONG:
			// Add Handler for pong!
			log_printf(LL_SPAM, "VENDOR PONG received from Sender:%i\n", input_vendor_msg->sender);
			break;
		
		case VND_OPCODE_MOTION_EVENT:
			// Add Handler for pong!
			log_printf(LL_SPAM, "VENDOR MOTION EVENT received from Sender:%i\n", input_vendor_msg->sender);
			break;
			
		case VND_OPCODE_CONTROL_LIGHTS_EVENT:
			// Handler for controlling the lights
			/* 1. set light on/off to Server*/
			argc = 0;
			arg_buf_1[1]; // 8bit value
			//level = vnd_msg_control_lights_str->control_lights;
			level = input_vendor_msg->payload[0];
			
			memcpy( &arg_buf_1[0], &level, sizeof(level) );
			argc = sizeof(arg_buf_1);
			
			/* send Casambi message */
			(void) cmb_send_cmd_queued(CMB_SETLEVEL, arg_buf_1, argc);
			break;
		
			
		default:
			log_printf(LL_DBG, "Received vendor message with unsupported vendor_opcode:%i\n", input_vendor_msg->vendor_opcode);
			break;
	}
}

/** vnd_app_send_vendor_msg(): 
 * Send a vendor message to Casambi mesh network.  
 */
bool vnd_app_send_vendor_msg(uint8_t casambi_opcode, struct vnd_msg_str* output_vendor_msg)
{

	/* Variables */
	uint8_t argc=CMB_MSG_ARGV_SZ;
	uint8_t arg_buf[CMB_MSG_ARGV_SZ];

	/* Code */	
	assert(output_vendor_msg);
	
	if ( casambi_opcode != CMB_VENDORMESSAGE0 && casambi_opcode != CMB_VENDORMESSAGE1 &&
	casambi_opcode != CMB_VENDORGROUPMESSAGE0 && casambi_opcode !=CMB_VENDORGROUPMESSAGE1 ){
		log_puts(LL_DBG, "Vendor msg error: Invalid Casambi Opcode for Vendor message");
		return false;
	}
	memcpy(&arg_buf[0], (uint8_t*)output_vendor_msg, sizeof(struct vnd_msg_str));
	/* send Casambi message */
	(void) cmb_send_cmd_queued(casambi_opcode, arg_buf, argc);
		
	
	return true;
}


/************************************************************************/
/*  LOCAL FUNCTIONS                                                     */
/************************************************************************/

/** vnd_msg_handle_ping()
 * Sends a "vendor pong"-message back to Sender of received PING 
 */
static void vnd_msg_handle_ping(uint8_t casambi_opcode, struct vnd_msg_str* vendor_ping) {

	/* Variables */
	
	struct vnd_msg_str vendor_pong;
	
	/* Code */
		
	log_printf(LL_SPAM,"Vendor PING received: Casambi opcode=%i, sender=%i, receiver=%i\n", casambi_opcode, vendor_ping->sender, vendor_ping->receiver);
	vnd_msg_init_payload(&vendor_pong);
	
	/* Send response to Vendor PING */
	vendor_pong.sender = cmb_client_id; //my CLient ID
	vendor_pong.receiver = vendor_ping->sender; // pong is sent back to sender
	vendor_pong.vendor_opcode = VND_OPCODE_PONG;
		
	(void) vnd_app_send_vendor_msg(casambi_opcode, &vendor_pong);	
	log_printf(LL_SPAM,"Vendor PONG sent: Casambi opcode=%i, sender=%i, receiver=%i\n", casambi_opcode, vendor_pong.sender, vendor_pong.receiver);
}




