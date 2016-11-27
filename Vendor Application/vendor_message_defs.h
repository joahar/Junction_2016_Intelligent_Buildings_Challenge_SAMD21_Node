#ifndef VENDOR_MESSAGE_DEFS_H
#define VENDOR_MESSAGE_DEFS_H

#include <stdint.h>


/* enum for Vendor message opcodes!
 */
enum vnd_op_code
{
	VND_OPCODE_PING					= 0x01,
	VND_OPCODE_PONG					= 0x02,
	VND_OPCODE_MOTION_EVENT			= 0x03,
	VND_OPCODE_ADC0_EVENT			= 0x04,
	VND_OPCODE_ADC1_EVENT			= 0x05,
	VND_OPCODE_CONTROL_EVENT		= 0x06,
	VND_OPCODE_ADC_EVENT			= 0x07,
	VND_OPCODE_CONTROL_LIGHTS_EVENT = 0x08,
	
	VND_OPCODE_OP_MAX				= 0xFF
};

/* Vendor message header defines */
typedef	uint8_t vnd_id_t;
typedef	uint8_t vnd_op_t;								 
#define vnd_msg_header			vnd_id_t sender; vnd_id_t receiver; vnd_op_t vendor_opcode
#define VND_MSG_HEADER_SZ	3 //Header size is 3 bytes
#define VND_MSG_MAX_SZ		8 // Vendor message total size 8 bytes
#define VND_MSG_FILLER_SZ  (VND_MSG_MAX_SZ - VND_MSG_HEADER_SZ) // 5 bytes left for vendor message payload

/* Broadcast can be used in receiver field */
#define VND_MSG_BROADCAST			0


/************************************************************************/
/* Vendor messages  structures                                          */
/************************************************************************/

/* Generic structure for all vendor messages */	
struct vnd_msg_str {
	vnd_msg_header;
	uint8_t payload[VND_MSG_FILLER_SZ];
}__attribute__((__packed__));


/* Structure for Vendor message: MOTION_EVENT
 */
struct vnd_msg_motion_detected_str{
	vnd_msg_header;
	uint16_t motion_detected; // 1=true, 0=means motion lost
}__attribute__((__packed__));

/* Structure for Vendor message: ADC_EVENT
 */
struct vnd_msg_adc_read_str{
	vnd_msg_header;
	uint16_t adc0_read; // value
	uint16_t adc1_read; // value
}__attribute__((__packed__));

/* Structure for Vendor message: ADC0_EVENT
 */
struct vnd_msg_adc0_read_str{
	vnd_msg_header;
	uint16_t adc0_read; // value
}__attribute__((__packed__));

/* Structure for Vendor message: ADC1_EVENT
 */
struct vnd_msg_adc1_read_str{
	vnd_msg_header;
	uint16_t adc1_read; // value 
}__attribute__((__packed__));

/* Structure for Vendor message: CONTROL_EVENT
 */
struct vnd_msg_control_read_str{
	vnd_msg_header;
	uint16_t control_read; // value 
}__attribute__((__packed__));

/* Structure for Vendor message: CONTROL_LIGHTS_EVENT
 */
struct vnd_msg_control_lights_str{
	vnd_msg_header;
	uint8_t control_lights; // value 
}__attribute__((__packed__));

#endif /* VENDOR_MESSAGE_DEFS_H */