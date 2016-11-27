#ifndef VENDOR_APPLICATION_H
#define VENDOR_APPLICATION_H

#include <tc.h>     // ASF module: TC - Timer Counter (callback)
#include <tc_interrupt.h>
#include "vendor_message_defs.h"


/************************************************************************/
/* PUBLIC FUNCTION PRTOTYPES                                            */
/************************************************************************/

/** vnd_app_client_init()
 *  \Initializes the vendor application module
 *
 *  \Input params : void
 *
 *  \return: void
 */
void vnd_app_client_init(void);

/** vnd_app_send_vendor_msg()
 *  \Puts a Vendor message to output queue. 
 *  \Note: This function takes care of putting the vendor message structure inside the CAsambi message
 *
 *  \casambi_opcode: Casambi command opcode. See 'cmb_opcodes.h'
 *  \vnd_msg_str:	 Generic struct for vendor messages
 *
 *  \return:  true if message was put to queue
 *            false if message queue was full
 */
bool  vnd_app_send_vendor_msg (uint8_t casambi_opcode, struct vnd_msg_str* output_vendor_msg);

/** vnd_app_handle_vendor_msg()
 *  \Handler for vendor messages received from Casambi extension interface
 *
 *  \casambi_opcode: Casambi command opcode. See 'cmb_opcodes.h'
 *  \vnd_msg_str:	 Generic struct for vendor messages
 *
 *  \return:  void
 */
void vnd_app_handle_vendor_msg	(uint8_t casambi_opcode, struct vnd_msg_str* input_vendor_msg);


/** vnd_msg_init_payload()
 *  \initializes the payload part of Vendor messages with 0x00
 *
 *  \vnd_msg_str:	 Generic struct for vendor messages
 * 
 *  \return : void 
 */
void vnd_msg_init_payload(struct vnd_msg_str* vendor_msg);

#endif/* VENDOR_APPLICATION_H */