#include <adc.h>	// ASF module: ADC - Analog-to-Digital Converter (Callback APIs)
#include <adc_callback.h>
#include <port.h>   // ASF module: PORT - GPIO Pin Control
#include <extint.h> // ASF module: EXTINT - External Interrupt
#include <asf.h>
#include "sensor_io.h"
#include "logger.h"
#include "task_scheduler.h"




/* Adjust this parameters to control how many seconds motion sensor 
 * re-detection is waited before motion lost is triggered
 * minimum resolution of the re-detection timer 
 */
#define REDETEC_TMR_IN_SEC 20

/* This defines the ADC read task period in seconds.
 */  
#define ADC_READ_PERIOD_IN_SEC 5


/************************************************************************/
/* LOCAL ADC conversion stuff                                           */
/************************************************************************/

#define ADC_RANGE_12BIT 4096
#define ADC_RANGE_11BIT 2048
#define ADC_RANGE_10BIT 1024
#define ADC_RANGE_9BIT  512
#define ADC_RANGE_8BIT  256

/* Note: ADC reference values below are in uV, i.e. 10^-6 */
#define ADC_VOLTAGE_REF_VALUE_INT1V ( 1000000 ) // When ADC_REFERENCE_INT1V without gain
#define ADC_VOLTAGE_REF_VALUE_INT1V_DIV2 ( 2000000 ) // When ADC_REFERENCE_INT1V is used with gain DIV2

/* When ADC_REFERENCE_INTVCC0 is used Vref is VDDANA/1,48V--> 3.3V/1.48V=2,22V  */
#define ADC_VOLTAGE_REF_VALUE_INTVCC0 (2229730) // ADC_REFERENCE_INTVCC0 without gain
#define ADC_VOLTAGE_REF_VALUE_INTVCC0_DIV2 (4459459) // ADC_REFERENCE_INTVCC0 without gain DIV2

#define MILLIVOLT_DIVIDER 1000

#define ADC_BUFFER ((SAMPLES_PER_CHANNEL)*(ADC_INPUT_CHANNELS))

/* table for ADC module to store ADC conversion results */
uint16_t adc_result_buffer[ADC_BUFFER];

static uint32_t adc_voltage_ref; // This is set in sensor_adc_init

static uint32_t adc_max_range; // Depends on selected ADC bits

struct adc_module adc_instance; 

/* Structure for configuring Switch and Event based interrupts */
struct sensor_io_def
{	
	const uint8_t	  line; /* the interrupt line that pin and mux use. see p.34 of Atmel SAM D21 datasheet for pin and line definitions. */
	const uint8_t	  pin; /* Note, when  sensor_type is ADC only the first line is mandatory, pin for SENSOR_ADC1 is only informative */
	const uint8_t	  mux;
	const uint8_t	  type; /* Sensor_type */
	const uint8_t	  trigger;
	const uint8_t	  pull;
	uint16_t		  arg0;
	uint16_t		  arg1;
	const uint16_t	  wait_redetection_ds;  /* in deciseconds (100 ms). if using wait re-detection with motion sensors, set trigger to TRIGGER_BOTH. */
	uint16_t		  redetection_left;
	uint16_t		  debounce_period_ms;	/* after every trigger, adds a debounce period during which triggers will be ignored. */
	sensor_event_cb_t event_cb_fun; /* callback function to be called on a sensor event */
};

/* *SENSOR IO PIN DEFINITIONS */
#define NOT_USED  255


static struct sensor_io_def sensor_io_defs[] = {
  /*  line     ,               pin       ,          mux        ,    type:4    ,   trigger      ,    pull  , arg0, arg1, wait_redetection_ds, redetection_left, debounce_period_ms , event_cb_fun */
   {   0	   , PIN_PA16A_EIC_EXTINT0   , MUX_PA16A_EIC_EXTINT0 , SENSOR_MOTION , TRIGGER_BOTH   , PULL_DOWN ,   0 ,   0 , REDETEC_TMR_IN_SEC ,       0         ,        0       , NULL   }
  ,{  15	   , PIN_PA15A_EIC_EXTINT15  , MUX_PA15A_EIC_EXTINT15, SENSOR_SWITCH , TRIGGER_FALLING, PULL_UP   ,   0 ,   0 ,       0            ,       0         ,        0       , NULL   }       
  ,{  NOT_USED , ADC_POSITIVE_INPUT_PIN18,           0           , SENSOR_ADC0   , TRIGGER_NONE   , PULL_NONE ,   0 ,   0 ,       0            ,       0         ,        0       , NULL   } 
  ,{  NOT_USED , ADC_POSITIVE_INPUT_PIN19,           0           , SENSOR_ADC1   , TRIGGER_NONE   , PULL_NONE ,   0 ,   0 ,       0            ,       0         ,        0       , NULL   }
  
};
 
const uint8_t sensor_io_def_count = sizeof(sensor_io_defs) / sizeof(struct sensor_io_def);


/************************************************************************/
/*  SENSOR IO module TASKs                                              */
/************************************************************************/

static struct task_info wait_redetection_task; /* TAsk to wait for motion re-detection after it has been lost */
static struct task_info adc_read_task; /* Periodic ADC conversion task */


/************************************************************************/
/* LOCAL FUNCTION PROTOTYPES                                            */
/************************************************************************/

/* NOTE: extint_pull defined in extint.h */

static enum extint_detect get_conf_detection_criteria(sensor_trigger trigger);

static enum extint_pull get_conf_gpio_pin_pull(sensor_pull pull);

static void wait_redetection_task_do(struct task_info* self);

static void adc_read_task_do(struct task_info* self);

static struct sensor_io_def* find_def_by_line(uint8_t line);

static void sensor_switch_interrupt(void);

static void sensor_motion_interrupt(void);

static void sensor_adc_interrupt(struct adc_module *const module);

static void sensor_digital_init(const struct sensor_io_def* def, void (*int_fun)(void));

static void sensor_digital_enable_interrupt(struct sensor_io_def* def);

static void sensor_digital_disable_interrupt(struct sensor_io_def* def);

static void sensor_adc_init(struct sensor_io_def* def);



/************************************************************************/
/*  PUBLIC FUNCTIONS                                                    */
/************************************************************************/

/** sensor_io_init()  
 *  Init Sensor related IOs & interrupts
 */
void sensor_io_init(void)
{
	/* init hardware and interrupts needed by sensor definitions */
	uint8_t defi;

	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		struct sensor_io_def* def = &sensor_io_defs[defi];
				
		switch(def->type) {
			case SENSOR_ADC0:
				sensor_adc_init(def);
				break;
			case SENSOR_ADC1:
			    //SENSOR_ADC0 init already 2 channels if needed
				break;
			case SENSOR_SWITCH:
				sensor_digital_init(def, sensor_switch_interrupt);
				break;
			case SENSOR_MOTION:
				sensor_digital_init(def, sensor_motion_interrupt);
				break;
			default: /* should not be reached */
				log_printf(LL_DBG, "sensor_io_init: unknown sensor type %d\n", def->type);
			break;
		}
	}
	
	/* init wait re-detection task for motion sensor */
	wait_redetection_task.period_time = 100; // 1sec time between between callback invocations, (10ms resolution)
	wait_redetection_task.fun = wait_redetection_task_do;
	#ifdef SCHEDULER_ENABLE_DEBUG_TASK_LOG
	wait_redetection_task.dbg_task_name = "wait_redetection_task";
	#endif
	

}


/* sensor_io_event_set_callback() 
 * Map sensor type to trigger callback function
 */
void sensor_io_event_set_callback(sensor_event_cb_t cb_fun, Sensor_type type )
{
	
	uint8_t defi;
	struct sensor_io_def* def = NULL;
	
	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		def = &sensor_io_defs[defi];
		
		if (type == def->type){
			def->event_cb_fun = cb_fun;
			break;
		}
	}
					
}


/** sensor_io_event_enable_interrupts() 
 * enables motion sensor and switch interrupts
 */
void sensor_io_event_enable_interrupts(void)
{
	uint8_t defi;
	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		struct sensor_io_def* def = &sensor_io_defs[defi];
		
		switch(def->type) {
			case SENSOR_ADC0:
			case SENSOR_ADC1:
				/* Not needed currently */				
				break;			
			case SENSOR_SWITCH:
			case SENSOR_MOTION:
				sensor_digital_enable_interrupt(def);
				break;				
			default: /* should not be reached */
				log_printf(LL_DBG, "sensor_io_event_enable_interrupts: unknown sensor type %d\n", def->type);
				break;
		}
	}
}


/** sensor_io_event_disable_interrupts() 
 * disables motion sensor and switch interrupts
 */
void sensor_io_event_disable_interrupts(void)
{
	uint8_t defi;
	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		struct sensor_io_def* def = &sensor_io_defs[defi];
		
		switch(def->type) {
			case SENSOR_ADC0:
			case SENSOR_ADC1:
				/* not needed currently */
				break;
			case SENSOR_SWITCH:
			case SENSOR_MOTION:
				sensor_digital_disable_interrupt(def);
				break;
			default: /* should not be reached */
				log_printf(LL_DBG, "sensor_io_event_disable_interrupts: unknown sensor type %d\n", def->type);
				break;
		}
	}
}



/************************************************************************/
/* LOCAL FUNCTIONS                                                      */
/************************************************************************/


/** get_conf_detection_criteria(sensor_trigger trigger)
 *  helper function for sensor hardware init 
 */
static enum extint_detect get_conf_detection_criteria(sensor_trigger trigger)
{
	switch(trigger) {
	case TRIGGER_FALLING:
		return EXTINT_DETECT_FALLING;
	case TRIGGER_RISING:
		return EXTINT_DETECT_RISING;
	case TRIGGER_BOTH:
		return EXTINT_DETECT_BOTH;
	case TRIGGER_HIGH:
		return EXTINT_DETECT_HIGH;
	case TRIGGER_LOW:
		return EXTINT_DETECT_LOW;
	default:
		return EXTINT_DETECT_NONE;
	}
}

/** get_conf_gpio_pin_pull(sensor_pull pull)
 *  helper function for sensor hardware init 
 */
static enum extint_pull get_conf_gpio_pin_pull(sensor_pull pull)
{
	switch(pull) {
	case PULL_UP:
		return EXTINT_PULL_UP;
	case PULL_DOWN:
		return EXTINT_PULL_DOWN;
	case PULL_NONE:
	default:
		return EXTINT_PULL_NONE;
	}
}


/** wait_redetection_task_do(struct task_info* self)
 *  Task for waiting if motion sensor re-detection happens in certain time.
 */
static void wait_redetection_task_do(struct task_info* self)
{
	int redetections_left = 0;

	/* behold the mighty levels of indentation */
	uint8_t defi;
	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		struct sensor_io_def* def = &sensor_io_defs[defi];
		
		if(def->redetection_left > 0) {
			--def->redetection_left;
			//log_printf(LL_SPAM, "re-detection callback task, left = %i\n", def->redetection_left);
			
			if(def->redetection_left == 0) {
				switch(def->type) {
				case SENSOR_MOTION:
					/* send an event signaling motion ending */
					if(NULL != def->event_cb_fun) {
						struct sensor_event event;
						event.line = def->line;
						//event.casambi_tag = def->tag;	
						event.type = SENSOR_MOTION;
						event.motion_detected = 0;
						def->event_cb_fun(&event);					
						}
					break;
				default:
					break;	
				}
			}
			else {
				redetections_left = 1;
			}
		}
	}
	
	/* if no re-detections left, de-schedule this task.
	   sensor interrupts will re-schedule this if needed. */
	if(redetections_left == 0) {
		scheduler_task_del(self);
	}
}


/**  find_def_by_line(uint8_t line) 
 * helper func that searches and returns first matching
 * sensor with given interrupt line. 
 */
static struct sensor_io_def* find_def_by_line(uint8_t line) 
{
	uint8_t defi;
	for(defi = 0; defi < sensor_io_def_count; ++defi) {
		struct sensor_io_def* def = &sensor_io_defs[defi];
		
		if(def->line == line) {
			return def;
		}
	}
	
	return NULL;
}


/** sensor_switch_interrupt(void)
 *  /Interrupt handler for switch event                                                     
 */
static void sensor_switch_interrupt(void)
{
	uint8_t line = extint_get_current_channel();
	struct sensor_io_def* def = find_def_by_line(line);
	
	/* if the callback is not set, no need to continue */
	if(def->event_cb_fun == NULL) {
		return;
		}
	
	if(def == NULL) {
		log_printf(LL_DBG, "sensor_io: interrupt line %d with no matching sensor definition, ignoring...\n", line);
		return;	
	}
	struct sensor_event event;
	event.value_switch = port_pin_get_input_level(def->pin);	
	event.line = line;
	event.type = SENSOR_SWITCH;
	def->event_cb_fun(&event);	
}


/** sensor_motion_interrupt(void)
 *  /Interrupt handler for motions sensor event                                                       
 */
static void sensor_motion_interrupt(void)
{
	
	struct sensor_event event;
	uint8_t line = extint_get_current_channel();
	struct sensor_io_def* def = find_def_by_line(line);

	/* if the callback is not set, no need to continue */
	if(NULL == def->event_cb_fun ) {
		return;
	}
	
	if(def == NULL) {
		log_printf(LL_DBG, "sensor_io: interrupt line %d with no matching sensor definition, ignoring...\n", line);
		return;
	}

	//log_puts(LL_SPAM," sensor_motion_interrupt()\n");
	
	/* read the pin */
	uint8_t pin_state = port_pin_get_input_level(def->pin);
	
	/* motion detected during re-detection timer.
	   disable re-detection timer and exit. */
	if(def->redetection_left > 0 && pin_state == 1) {
		def->redetection_left = 0;		
	}
	/* if the motion is lost and wait re-detection is defined,pio_pin_mux
	   schedule the re-detection task. */
	else if(def->wait_redetection_ds > 0 && pin_state == 0) {
		//log_puts(LL_SPAM,"Start: MotionSensor wait_redetection_task\n");
		def->redetection_left = def->wait_redetection_ds;
		scheduler_task_add(&wait_redetection_task);		
	}
	/* otherwise send a motion event. */
	else {
		//event.casambi_tag = def->tag;		
		event.line = line;
		event.type = SENSOR_MOTION;
		event.motion_detected = pin_state ? 1 : 0;
		def->event_cb_fun(&event);
		
	}
}

/** sensor_digital_init(const struct sensor_io_def* def, void (*int_fun)(void))
 *  /Interrupt init for motion sensor and switch                                                      
 */
static void sensor_digital_init(const struct sensor_io_def* def, void (*int_fun)(void))
{
	/* setup external interrupt */
    struct extint_chan_conf ec_conf;
	
    extint_chan_get_config_defaults(&ec_conf);

    ec_conf.gpio_pin            = def->pin;
    ec_conf.gpio_pin_mux        = def->mux;
    ec_conf.detection_criteria  = get_conf_detection_criteria(def->trigger);
    ec_conf.gpio_pin_pull       = get_conf_gpio_pin_pull(def->pull);
    ec_conf.filter_input_signal = true;

    extint_chan_set_config(def->line, &ec_conf);
	extint_register_callback(int_fun, def->line, EXTINT_CALLBACK_TYPE_DETECT);
}


/** sensor_digital_enable_interrupt(struct sensor_io_def* def)
 *  /Interrupt enable for motion sensor and switch                                                      
 */
static void sensor_digital_enable_interrupt(struct sensor_io_def* def)
{
	/* enable external interrupt */
	extint_chan_enable_callback(def->line, EXTINT_CALLBACK_TYPE_DETECT);
}


/** sensor_digital_disable_interrupt(struct sensor_io_def* def)
 *  /Interrupt disable for motion sensor and switch                                                      
 */
static void sensor_digital_disable_interrupt(struct sensor_io_def* def)
{
	/* disable external interrupt */
	extint_chan_disable_callback(def->line, EXTINT_CALLBACK_TYPE_DETECT);
}


/************************************************************************/
/* Local ADC IO related functions:                                      */
/************************************************************************/

/** sensor_adc_init(struct sensor_io_def* def)
 * Initializes the ADC module and starts a task to trigger 
 * periodic ADC conversions
 */
static void sensor_adc_init(struct sensor_io_def* def)
{
	
	/* Variables */
	
	//enum status_code status;
	struct adc_config config_adc;
	
	
	/* Code */

	/* 1. Configure ADC instance*/
	adc_get_config_defaults(&config_adc);
	
	/* Reference voltage */
	config_adc.reference = ADC_REFERENCE_INTVCC0; // 3,3V/1,48V = 2,22V	
	config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2; // Range is 0-4,44V, 12bits
	config_adc.reference_compensation_enable = true; // Compensate GAIN error
	adc_voltage_ref = ADC_VOLTAGE_REF_VALUE_INTVCC0_DIV2;

	/* Resolution */
	config_adc.resolution = ADC_RESOLUTION_CUSTOM;	
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_8;
	config_adc.divide_result = ADC_DIVIDE_RESULT_8;	
	adc_max_range = ADC_RANGE_12BIT;
#ifdef HW_AVG_DISABLED
	config_adc.resolution = ADC_RESOLUTION_12BIT;
	config_adc.accumulate_samples = ADC_ACCUMULATE_DISABLE;
	config_adc.divide_result = ADC_DIVIDE_RESULT_DISABLE;
#endif /* HW_AVG_DISABLED */

	/* Select First IO for ADC */
	config_adc.positive_input = def->pin; //ADC[18] is PA10
	config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	
	/* Enable pin scan if 2 ADC channels */
	if (2 == ADC_INPUT_CHANNELS ){
		config_adc.pin_scan.inputs_to_scan = 2;
		config_adc.pin_scan.offset_start_scan = 0; //AIN[18]=PA10, AIN[19]=PA11
		}
	
	
	adc_init(&adc_instance, ADC, &config_adc);		
	adc_enable(&adc_instance);
				
	/* Register ADC interrupt callback function */
	adc_register_callback(&adc_instance,sensor_adc_interrupt, ADC_CALLBACK_READ_BUFFER);
	
	
	/* enable callback function */
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
	// NOTE: MANDATORY Global system interrupt enable is done in the main.c

	
	/* 2. Start periodic ADC read task */		
	adc_read_task.period_time = 100*(ADC_READ_PERIOD_IN_SEC);
	adc_read_task.repeats = 0;
	adc_read_task.fun = adc_read_task_do; // callback for ADC read task
	#ifdef SCHEDULER_ENABLE_DEBUG_TASK_LOG
	adc_read_task.dbg_task_name = "adc_read_task";
	#endif
	scheduler_task_add(&adc_read_task);
		
	log_puts(LL_SPAM, "ADC conversion task started...");
	
}



/** adc_read_task_do(struct task_info* self)
 *  ADC task callback for triggering ADC conversion to start, period defined with ADC_READ_PERIOD_IN_SEC 
 */
static void adc_read_task_do(struct task_info* self){

	/* Variables */
	enum status_code status;
	
	/* Code */

	status = adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_BUFFER);
	
	if (status == STATUS_BUSY){
		log_puts(LL_DBG, "ADC STATUS BUSY\n");
		}
}


/** sensor_adc_interrupt(struct adc_module *const module)
 * ADC interrupt handler, called when ADc result is ready for reading 
 */
static void sensor_adc_interrupt(struct adc_module *const module){

	/* Variables */
	struct sensor_event event;	
	struct sensor_io_def* def = NULL;
	uint32_t idx = 0;
	uint8_t adc_ch_index = 0;
	
	/* Code */
	for(idx = 0; idx < sensor_io_def_count; ++idx){
		def = &sensor_io_defs[idx];
	
		if (def->type == SENSOR_ADC0 || def->type == SENSOR_ADC1 ){
			//Trigger event ind for ADC0 if callback configured.
			/* Note: This works currently only when samples_count = 1 */ 
			if ( def->type == SENSOR_ADC1 )
				{
				adc_ch_index = 1;
				}
			/* Read ADC result and trigger callback, ADC result is converted to mV */		
			event.value_adc = (uint16_t)( ( (adc_voltage_ref / adc_max_range ) * adc_result_buffer[adc_ch_index] )/ MILLIVOLT_DIVIDER );
			
			event.type = def->type;
			if (NULL != def->event_cb_fun){
				def->event_cb_fun(&event);
				}
			else{
				/* Trace if callback is not set */
				if ( def->type == SENSOR_ADC0){ log_puts(LL_DBG,"sensor_adc_interrupt(SENSOR_ADC0):Sensor event callback missing\n");}
				if ( def->type == SENSOR_ADC1){ log_puts(LL_DBG,"sensor_adc_interrupt(SENSOR_ADC1):Sensor event callback missing\n");}
			}
		}
	}	
}
