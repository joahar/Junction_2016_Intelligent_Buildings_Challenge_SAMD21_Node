#ifndef SENSOR_HANDLER_H_
#define SENSOR_HANDLER_H_


/************************************************************************/
/* SENSOR DEFINITIONS                                                   */
/************************************************************************/


/************************************************************************/
/* PUBLIC FUNCTION PRTOTYPES                                             */
/************************************************************************/

/* Init sensor event callbacks */
void sensor_handler_set_sensor_io_callback( void );

/* Sensor Event handlers */
void sensor_hndl_motion_event(struct sensor_event* def);
void sensor_hndl_button_event(struct sensor_event* def);
void sensor_hndl_adc0_event(struct sensor_event* def);
void sensor_hndl_adc1_event(struct sensor_event* def);

/* Sensor value read callback used when GetSensorValue is sent by CBM-001 Server */
void sensor_value_get_adc0_voltage(uint16_t* value);
void sensor_value_get_adc1_voltage(uint16_t* value);
void sensor_value_get_motion_status(uint16_t* value);

#endif /* SENSOR_HANDLER_H_ */