#ifndef CMB_MESSAGE_HANDLER_H
#define CMB_MESSAGE_HANDLER_H


#include <stdint.h>
#include "cmb_opcodes.h"



/* Client ID, used in VEndor messaging */
extern uint16_t	cmb_client_id;
extern uint16_t fw_version_id;

/* Return value for cmb_do_command() */
typedef enum casambi_msg_status
{
	CMB_COMMAND_OK,
	CMB_COMMAND_NOT_FOUND,
	CMB_COMMAND_FAILED
} casambi_msg_status;


/* return value enumeration for Casambi param and sensor handling */
typedef enum cmb_param_status
{
	CMB_PARAM_OK			= 0x00,
	CMB_PARAM_ID_NOT_FOUND	= 0x01,
	CMB_PARAM_ACCESS_DENIED	= 0x02,
	CMB_PARAM_SIZE_MISMATCH = 0x03,
	CMB_PARAM_INVALID_ADDR	= 0x04,
} cmb_param_status;



/************************************************************************/
/* PUBLIC FUNCTION PROTOTYPES                                            */
/************************************************************************/



/** cmb_do_command()
 *  \Handles all messages received from Casambi extension interface
 *
 *  \param opcode:	Casambi command opcode. See 'cmb_opcodes.h' for spefs
 *  \param args:	A table of arguments. Set to NULL if none are sent.
 *  \param argc:	Length of the argument table. Set to 0 if none are sent.
 *
 *  \return cmb_param_status
 */
casambi_msg_status cmb_do_command(uint8_t opcode, uint8_t* argv, uint8_t argc);


/** cmb_send_cmd_queued()
 *  \Puts Casambi message to output queue. 
 *
 *  \param opcode:	Casambi command opcode. See 'cmb_opcodes.h' for specs
 *  \param args:	A table of arguments. Set to NULL if none are sent.
 *  \param argc:	Length of the argument table. Set to 0 if none are sent.
 *
 *  \return true:		Command was added succesfully to queue
 *          false:		Error
 */
bool cmb_send_cmd_queued(uint8_t opcode, const uint8_t* args, uint8_t argc);


/** cmb_client_init()
 *  \Initializes the Casambi client module.
 *
 *	Input params: void
 *
 *  \return void
 */
void cmb_client_init(void);

/** get_tag_by_sensor_type()
 */
cmb_param_status get_tag_by_sensor_type ( uint8_t type, uint8_t *tag );

					  
#endif /* CMB_MESSAGE_HANDLER_H */