#ifndef CASAMBI_OPCODES_H
#define CASAMBI_OPCODES_H

/** cmb_opcodes.h
 *  \brief	List of the Casambi Extension Interface command opcodes.
 *			
 *			Notes:
 *				- last tested firmware version: 18.4
 *				- 'the App' refers to the Casambi mobile application for users.
 *				- 'server' refers to the Extension Interface Server (e.q. CBM-001 module)
 *				- 'client' refers to the device communicating with the server; usually the
 *				  one running the program compiled with this file.		
 */
enum cmb_op
{
	CMB_NO_OP				= 0x00,		/* No operation. Safe to send as a filler if needed.
										 * dir:		both
										 * args:	none */
	
    CMB_PING                = 0x01,     /* Ping. Response with a pong.
										 * dir:		both
                                         * args:	none */
	
    CMB_PONG                = 0x02,     /* Pong. Response to a ping.
										 * dir:		both
                                         * args:	none */
	
    CMB_INIT                = 0x03,     /* Functionality not described by Casambi.
										 * dir:		both
                                         * args:	none */
	
    CMB_BUTTON_PRESSED      = 0x10,     /* Dimming button pressed (dims/brightens light level
                                         * until button is released with 0x11).
										 * dir:		both
                                         * args:	none */
	
    CMB_BUTTON_RELEASED     = 0x11,     /* Dimming button released. 
										 * dir:		both
                                         * args:	none */
	
    CMB_TOGGLEONOROFF       = 0x12,     /* Turns the lamp on or off.
										 * dir:		C->S
                                         * args:	none */
	
    CMB_SETLEVEL            = 0x13,     /* Sets light level.
										 * dir:		C->S
                                         * args:	uint8 level */
	
	CMB_VENDORMESSAGE0		= 0x14,		/* Vendor message 0.
										 * dir:		both
										 * args:	0..8 opaque bytes */
	
	CMB_VENDORMESSAGE1		= 0x15,		/* Vendor message 1.
										 * dir:		both
										 * args:	0..8 opaque bytes */
	
	CMB_VENDORGROUPMESSAGE0	= 0x16,		/* Vendor group message 0.
										 * dir:		both
										 * args:	0..7 opaque bytes */
	
	CMB_VENDORGROUPMESSAGE1	= 0x17,		/* Vendor group message 1.
										 * dir:		both
										 * args:	0..7 opaque bytes */
	
	CMB_GETSENSORVALUE		= 0x18,		/* Get a sensor value. Server may query this in intervals or
										 * after user input in the App.
										 * dir:		S->C
										 * args:	uint8 tag */	
	
	CMB_SETSENSORVALUE		= 0x19,		/* Set a sensor value in the App.
										 * dir:		C->S
										 * args:	uint8 tag, uint8 values, ... 
										 *			value is 1..3 bytes, combined as little-endian. */	
		
	CMB_SETPARAMETERVALUE	= 0x1a,		/* Set a parameter value. Used to indicate user changed value in
										 * the App or as a response to a query from the server.
										 * dir:		both
										 * args:	uint8 tag, uint8 values, ...
										 *			value is 1..8 bytes, combined as little-endian. */	
	
	CMB_PARAMETERSCOMPLETE	= 0x1b,	    /* Indication from the server that all parameters are sent and that the
										 * given the values should be storaged.
										 * dir:		S->C
										 * args:	none */	
	
	CMB_SETFIRMWAREVERSION	= 0x1c,	    /* Sets the secondary firmware version. If set, the Casambi
										 * will advertise this secondary fw version too. This version
										 * can later be changed from the utility App too.
										 * given the values should be storaged.
										 * dir:		C->S
										 * args:	none */
	
	CMB_GETPARAMETERVALUE	= 0x1d,	    /* Get a parameter value. Should be responded by the client
										 * with SetParameterValue (0x1b) within 500ms.
										 * dir:		C->S
										 * args:	none */		
	CMB_OP_MAX
};

#endif /* CASAMBI_OPCODES_H */