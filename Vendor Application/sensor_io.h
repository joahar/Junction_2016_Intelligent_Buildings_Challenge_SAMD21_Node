#ifndef SENSOR_IO_H
#define SENSOR_IO_H


/* Select 1 or 2:
 * When 1 the ADC conversion is done from AIN[18], i.e. pin PA10
 * When 2 the ADC conversion is done from AIN[18] and AIN[19], i.e. PA10 and PA11.
 */
#define ADC_INPUT_CHANNELS 2

/* One sample is enough, ADC HW already takes average of 8 samples */
#define SAMPLES_PER_CHANNEL 1


/* Sensor types */
typedef uint8_t Sensor_type;
enum 
{
	SENSOR_SWITCH,		/* sensor on a digital pin */
	SENSOR_MOTION,		/* sensor with motion detection logic */
	SENSOR_ADC0,		/* sensor value on first ADC channel */
	SENSOR_ADC1,		/* sensor value on second ADC channel */	
	SENSOR_LAST_VALUE = 255
};


typedef uint8_t sensor_trigger;
enum
{
	TRIGGER_FALLING,
	TRIGGER_RISING,
	TRIGGER_BOTH,
	TRIGGER_HIGH,
	TRIGGER_LOW,
	TRIGGER_OVER_THRESHOLD,		/* arg0: ADC threshold value, arg1: hysteresis */
	TRIGGER_UNDER_THRESHOLD,	/* arg0: ADC threshold value, arg1: hysteresis */
	TRIGGER_DELTA,
	TRIGGER_NONE
};

typedef uint8_t sensor_pull;
enum
{
	PULL_NONE = 0,
	PULL_UP,
	PULL_DOWN
};

/* struct for delivering sensor data from sensor_io to sensor_handler
 */
struct sensor_event
{	
	uint8_t		line;
	Sensor_type type;
	uint8_t     value_switch;	 /* if type == SENSOR_SWITCH: 0 or 1. */
	uint8_t     motion_detected; /* if type == SENSOR_MOTION: 0 or 1. */	
	uint16_t	value_adc;
};


/* This is the generic sensor event callback type */
typedef void(*sensor_event_cb_t)(struct sensor_event*);



/************************************************************************/
/* PUBLIC FUNCTION PROTOTYPES                                           */
/************************************************************************/

void sensor_io_init(void);

void sensor_io_event_set_callback(sensor_event_cb_t cb_fun, Sensor_type type);

void sensor_io_event_enable_interrupts(void);
void sensor_io_event_disable_interrupts(void);


#endif /* SENSOR_IO_H */