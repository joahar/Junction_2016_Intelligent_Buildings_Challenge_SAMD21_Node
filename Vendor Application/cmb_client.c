/** cmb_message_handler.c
  *	\brief General Casambi client side logic.
 */

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


#define GET_PARAM_ARG_BUF_SZ 17

/************************************************************************/
/* LOCAL DEFINES FOR CASAMBI PARAMETERS                                 */
/************************************************************************/

/* Parameter definitions */
enum
{
	CMBP_READ		= 0x01,
	CMBP_WRITE		= 0x02
};

enum
{
	CMBP_VAR_LEN	= 0x10, // Indicates variable write length (e.q. string)
	CMBP_INTEGER	= 0x00,
	CMBP_FLOAT		= 0x01,
	CMBP_BINARY		= 0x02 | CMBP_VAR_LEN,
	CMBP_STRING		= 0x03 | CMBP_VAR_LEN,
	CMBP_VERSION	= 0x04
};

/* Casambi Parameter TAG enumeration, shall be in sync with Casambi module configuration, see CAsambi Admin-pages */
enum
{
	/* Param tags */
	CMB_P_TAG_CLIENT_ID		= 0x00,
	CMB_P_TAG_READ_ID		= 0x01,
	CMB_P_TAG_FW_ID			= 0x02,
	CMB_P_TAG_LAST_PARAM    = 0xFF
};

struct cmb_param_def
{
	uint8_t tag;
	uint8_t access	: 2;
	uint8_t type	: 6;
	void*	addr;
	uint8_t size;
	prst_addr_t	prst_addr;
};

/* CASAMBI PARAMETERS:  UPDATE THIS TABLE TO BE IN SYNC WITH THE CASAMBI MOULE CONFIGURATION
 */
const struct cmb_param_def cmb_parameters[] =
{
	/* uint8_t tag,			uint8_t access : 2,		uint8_t type : 6,	void* address		uint8_t size	*/             /* see persistent_addresses.h */
	{CMB_P_TAG_CLIENT_ID,	CMBP_READ|CMBP_WRITE,	CMBP_INTEGER,		&cmb_client_id,		(uint8_t)sizeof(cmb_client_id),		PADDR_CMB_CLIENT_ID}
   ,{CMB_P_TAG_READ_ID,		CMBP_READ,				CMBP_INTEGER,		&cmb_client_id,		(uint8_t)sizeof(cmb_client_id),		PADDR_CMB_CLIENT_ID}
   ,{CMB_P_TAG_FW_ID,		CMBP_READ,				CMBP_VERSION,		&fw_version_id,		(uint8_t)sizeof(fw_version_id),		PADDR_FW_VERSION_ID}
	
};

#define CMB_PARAMETERS_N (sizeof(cmb_parameters) / sizeof(struct cmb_param_def))


/************************************************************************/
/* LOCAL DEFINES FOR CASAMBI SENSORS                                    */
/************************************************************************/

/* Casambi Sensor TAG enumeration, shall be in sync with Casambi module configuration, see CAsambi Admin-pages*/
enum
{
	/* Sensor tags */
	CMB_S_TAG_MOTION_STATUS = 0x00,
	CMB_S_TAG_ADC0_VOLTAGE  = 0x01,
	CMB_S_TAG_ADC1_VOLTAGE  = 0x02,
	CMB_S_TAG_LAST_SENSOR   = 0xFF	
};

enum
{
	CMBS_CALLBACK	= 0x40,	/* Instead of a pointer to a location in memory, addr points to a callback fun with the type cmb_get_sensor_cb_t. */
	CMBS_INTEGER	= 0x01,
	CMBS_FLOAT		= 0x02
};

/* Function pointer callback definition for sensor get */
typedef void(*cmb_get_sensor_cb_t)(uint16_t* value );


struct cmb_sensor_def
{
	Sensor_type sensor_type_value;
	uint8_t		tag; // Casambi sensor TAG	
	uint8_t		type; 
	void*		addr;
	uint8_t		size;
};

const struct cmb_sensor_def cmb_sensors[] =
{
	/* sensor_type_value,       uint8_t tag,					uint8_t type,		void* addr   					       uint8_t size	*/
  //{SENSOR_MOTION,        CMB_S_TAG_MOTION_STATUS,		    CMBS_INTEGER,		&cmb_sensor_motion_status,		(uint8_t)sizeof(cmb_sensor_motion_status) }
	{SENSOR_MOTION,        CMB_S_TAG_MOTION_STATUS,		    CMBS_CALLBACK,		sensor_value_get_motion_status,	(uint8_t)sizeof(uint16_t) }
   ,{SENSOR_ADC0,          CMB_S_TAG_ADC0_VOLTAGE,		    CMBS_CALLBACK,	    sensor_value_get_adc0_voltage,   (uint8_t)sizeof(uint16_t) }
   ,{SENSOR_ADC1,          CMB_S_TAG_ADC1_VOLTAGE,		    CMBS_CALLBACK,	    sensor_value_get_adc1_voltage,   (uint8_t)sizeof(uint16_t) }
	   
};

#define CMB_SENSORS_N (sizeof(cmb_sensors) / sizeof(struct cmb_sensor_def))


/************************************************************************/
/* LOCAL FUNCTION PRTOTYPES                                             */
/************************************************************************/


/** param_def_search_by_tag()
 *	\brief		Search for a parameter definition by given tag.
 *
 *	\param defs:	Pointer to a table of parameter definitions.
 *	\param def_n:	Number of parameter definitions in the table.
 *	\param tag:		The parameter tag which definition is searched for.
 *
 *	\return		A pointer to a parameter definition or NULL if the definition was not found.
 */
static inline const struct cmb_param_def* param_def_search_by_tag(const struct cmb_param_def* defs, uint16_t def_n, int8_t tag);

static inline const struct cmb_sensor_def* sensor_def_search_by_tag(const struct cmb_sensor_def* defs, uint16_t def_n, int8_t tag);

static cmb_param_status cmb_get_parameter_value(uint8_t param_tag, uint8_t** argv, uint8_t* argc);

static cmb_param_status cmb_set_parameter_value(uint8_t param_tag, uint8_t* argv, uint8_t argc);

static cmb_param_status cmb_get_sensor_value(uint8_t tag, uint16_t* value);



/************************************************************************/
/*  PUBLIC FUNCTIONS                                                    */
/************************************************************************/

/** cmb_do_command() 
 * EXAMPLE of Extension interface message handler
 * Entry point for all Extension interface messages
 * Called when there is a new message in the Message QUEUE.
 */
casambi_msg_status cmb_do_command(uint8_t opcode, uint8_t* argv, uint8_t argc)
{
	
	switch(opcode)
	{
		/* ** Vendor layer message received ** */
		case CMB_VENDORMESSAGE0:
		case CMB_VENDORMESSAGE1:
		case CMB_VENDORGROUPMESSAGE0:
		case CMB_VENDORGROUPMESSAGE1:		
		{					
			if(argc < VND_MSG_HEADER_SZ || argc > VND_MSG_MAX_SZ) {
				break;
			}
			struct vnd_msg_str cmd;
			memcpy(&cmd, argv, argc);
			uint8_t casambi_opcode = opcode;	
			vnd_app_handle_vendor_msg(casambi_opcode, &cmd);
		}
		break;				

		/* ** Handle Server messages**  */			
		case CMB_GETPARAMETERVALUE:
			{
				if(argc > 0) {
					uint8_t* param_argv = NULL;
					uint8_t param_argc = 0;				
					/* Get the parameter! */
					if(cmb_get_parameter_value(argv[0], &param_argv, &param_argc) == CMB_PARAM_OK)
					{
						if(param_argv == NULL || param_argc + 1 > GET_PARAM_ARG_BUF_SZ) {
							return CMB_COMMAND_FAILED;
						}					
						uint8_t arg_buf[GET_PARAM_ARG_BUF_SZ];
						arg_buf[0] = argv[0];
						memcpy(&arg_buf[1], param_argv, param_argc);
						/* Put Casambi message to output Queue */
						(void) cmb_send_cmd_queued(CMB_SETPARAMETERVALUE, arg_buf, param_argc + 1);					
					}
				}
			}
			break;
				
		case CMB_GETSENSORVALUE:
			{
				if(argc > 0) {
					uint16_t sensor_value;
					log_printf(LL_SPAM,"GetSensorValue(tag=%i)\n", argv[0]);
					if(cmb_get_sensor_value(argv[0], &sensor_value) == CMB_PARAM_OK)
					{
						// If sensor value for given tag was found, send the sensor value to Server
						uint8_t arg_buf[3];// opcode + 16bit value
						arg_buf[0] = argv[0];
						memcpy(&arg_buf[1], &sensor_value, 2);
						/* Put Casambi message to output Queue */
						(void) cmb_send_cmd_queued(CMB_SETSENSORVALUE, arg_buf, 3);
					}
				}
			}
			break;
					
		case CMB_SETPARAMETERVALUE:
			{
				if(argc > 1) {
					cmb_set_parameter_value(argv[0], &argv[1], argc - 1);
				}
			}
			break;
					
		case CMB_PARAMETERSCOMPLETE:
			persistent_commit();
			log_puts(LL_SPAM, "Parameters complete");
			break;
								
		default:
			log_puts(LL_SPAM,"Casambi message ID not found!");
			return CMB_COMMAND_NOT_FOUND;
	}

	return CMB_COMMAND_OK;	
}


/** cmb_send_cmd_queued(uint8_t opcode, const uint8_t* args, uint8_t argc)
 * Sends a Casambi message to Server.
 */
bool cmb_send_cmd_queued(uint8_t opcode, const uint8_t* args, uint8_t argc){

	
	struct casambi_msg* cmb_cmd = cmb_msg_queue_alloc(&cmb_output_queue);
	// is the output buffer full?
	
	if(cmb_cmd == NULL) {
		log_puts(LL_DBG, "Error: output message buffer is full! skipping a message");
		return false;
		}
	cmb_cmd->opcode = opcode;
	
	memcpy(cmb_cmd->argv, args, argc);
	cmb_cmd->argc = argc;
	

return true;

}


/** cmb_client_init() 
 * Reads client id after reboot from persistent memory
 */
void cmb_client_init(void)
{
	uint16_t temp_id;
		
	persistent_read(PADDR_CMB_CLIENT_ID, &temp_id, sizeof(temp_id));
	if ( 0xFFFF == temp_id ){
		/* AFter FW update the address is 65535, write default! */
		persistent_write(PADDR_CMB_CLIENT_ID, &cmb_client_id , sizeof(cmb_client_id));		
		}
		
	persistent_read(PADDR_CMB_CLIENT_ID, &cmb_client_id, sizeof(cmb_client_id));
	log_printf(LL_DBG, "read client id: %d\n", cmb_client_id);
	
	
	
	/* Store current FW version to persistent memory */
	persistent_write( PADDR_FW_VERSION_ID, &fw_version_id, 1);
	
	/* Init vendor application module */
	vnd_app_client_init();
	

}


/** get_tag_by_sensor_type( uint8_t type, uint8_t *tag)
 * Returns Casambi TAG for sensor based on SensorType
 */
cmb_param_status get_tag_by_sensor_type( uint8_t type, uint8_t *tag){

	/* Variables */
	uint8_t defi;
	const struct cmb_sensor_def* def = NULL;

	/* Attributes */
	for(defi = 0; defi < CMB_SENSORS_N; ++defi) {
		def = &cmb_sensors[defi];
		if (type == def->sensor_type_value){
			*tag = def->tag;
			return (CMB_PARAM_OK);
		}
	
	}
	/* sensor_type not found! */
	return ( CMB_PARAM_ID_NOT_FOUND );
}

/************************************************************************/
/*  LOCAL FUNCTIONS                                                     */
/************************************************************************/

/** param_def_search_by_tag() 
 * Returns parameter def based on tag
 */
static inline const struct cmb_param_def* param_def_search_by_tag(const struct cmb_param_def* defs, uint16_t def_n, int8_t tag)
{
	uint16_t def_idx;
	for(def_idx = 0; def_idx < def_n; ++def_idx) {
		if(tag == defs[def_idx].tag) {
			return &defs[def_idx];
		}
	}
	
	return NULL;
}


/** cmb_get_parameter_value (uint8_t param_tag, uint8_t** argv, uint8_t* argc)
 * Handler for Casambi GetParameterValue message
 */
cmb_param_status cmb_get_parameter_value(uint8_t param_tag, uint8_t** argv, uint8_t* argc)
{
	// Search for the parameter definition by the parameter tag.
	const struct cmb_param_def* param_def = param_def_search_by_tag(cmb_parameters, CMB_PARAMETERS_N, param_tag);
	
	// If the parameter definition was not found, return error.
	if(param_def == NULL) {
		return CMB_PARAM_ID_NOT_FOUND;
	}

	// If the parameter does not have write access, return error.
	if(!(param_def->access & CMBP_READ)) {
		return CMB_PARAM_ACCESS_DENIED;
	}

	if(param_def->addr != NULL) {
		*argv = (uint8_t*)param_def->addr;
		
		if(param_def->type == CMBP_STRING) {
			size_t len = strlen(param_def->addr);
			*argc = (uint8_t)len;
		}
		else {
			*argc = param_def->size;
		}
	}
	else {
		return CMB_PARAM_INVALID_ADDR;
	}

	return CMB_PARAM_OK;
}


/** cmb_set_parameter_value(uint8_t param_tag, uint8_t* argv, uint8_t argc)
 * Handler for the Casambi SetParameterValue message
 */
cmb_param_status cmb_set_parameter_value(uint8_t param_tag, uint8_t* argv, uint8_t argc)
{
	// Search for the parameter definition by the parameter tag.
	const struct cmb_param_def* param_def = param_def_search_by_tag(cmb_parameters, CMB_PARAMETERS_N, param_tag);

	log_printf(LL_DBG, "param tag %d to be set\n", param_tag);

	// If the parameter definition was not found, return error.
	if(param_def == NULL) {
		log_puts(LL_DBG, "param id not found");
		return CMB_PARAM_ID_NOT_FOUND;
	}
	
	// If the parameter does not have write access, return error.
	if(!(param_def->access & CMBP_WRITE)) {
		log_puts(LL_DBG, "param access denied");
		return CMB_PARAM_ACCESS_DENIED;
	}
	
	// Parameter size matches, ok.
	if(argc == param_def->size) {
		log_puts(LL_DBG, "param size ok");
		memcpy(param_def->addr, argv, argc);
	}
	// Does the type allow variable length?
	else if(argc < param_def->size && param_def->type & CMBP_VAR_LEN) {
		log_puts(LL_DBG, "param var size ok");
		// Copy the user set bytes.
		memcpy(param_def->addr, argv, argc);
		
		// Zero the remaining bytes.
		memset(param_def->addr, 0, param_def->size - argc);
	}
	// Erroneous length, return error.
	else {
		log_printf(LL_DBG, "param size error: got %d, expected %d", argc, param_def->size);
		return CMB_PARAM_SIZE_MISMATCH;
	}
	
	// If the persistent address is set, store the new value.
	if(param_def->prst_addr != PADDR_INVALID) {
		log_printf(LL_DBG, "stored a param to %d\n", param_def->prst_addr);
		
		persistent_write(param_def->prst_addr, param_def->addr, argc);
		
		if(argc < param_def->size && param_def->type & CMBP_VAR_LEN) {
			persistent_set(param_def->prst_addr + argc, 0, param_def->size - argc);
		}
	}
	
	return CMB_PARAM_OK;
}


/** cmb_get_sensor_value(uint8_t tag, uint16_t* value)
 * HAndler for the Casambi GetSensorValue message
 */
cmb_param_status cmb_get_sensor_value(uint8_t tag, uint16_t* value) {
	// Search for the sensor definition by the sensor tag.
	const struct cmb_sensor_def* sensor_def = sensor_def_search_by_tag(cmb_sensors, CMB_SENSORS_N, tag);

	// If the sensor definition was not found, return error.
	if(sensor_def == NULL) {
		return CMB_PARAM_ID_NOT_FOUND;
	}
	
	// Sanity check the address
	if(sensor_def->addr != NULL) {
		// If pointer is a callback, call it.
		if(sensor_def->type & CMBS_CALLBACK) {
			((cmb_get_sensor_cb_t)sensor_def->addr)(value);			
		}
		// Read the pointed memory address
		else {
			*value = *(uint16_t*)sensor_def->addr;
		}
	}
	else {
		return CMB_PARAM_INVALID_ADDR;
	}
	
	return CMB_PARAM_OK;
}


/** sensor_def_search_by_tag(const struct cmb_sensor_def* defs, uint16_t def_n, int8_t tag)
 * Searches sensor def based on given tag. 
 */
static inline const struct cmb_sensor_def* sensor_def_search_by_tag(const struct cmb_sensor_def* defs, uint16_t def_n, int8_t tag)
{
	uint16_t def_idx;
	for(def_idx = 0; def_idx < def_n; ++def_idx) {
		if(tag == defs[def_idx].tag) {
			return &defs[def_idx];
		}
	}
	
	return NULL;
}
