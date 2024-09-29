

#include "hal/nrf_pwm.h"
#include <nrfx_example.h>
#include <nrfx_pwm.h>
#include <stdbool.h>
#include <stdint.h>
//#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "drivers/nrfx_errors.h"
#include <nrfx_glue.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
//#include "saadc_examples_common.h"
#include <nrfx_log.h>


#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 7
#define PWM_INST_IDX 0
#define ARRAYSIZE 192
//#define CH0_AIN ANALOG_INPUT_TO_SAADC_AIN(ANALOG_INPUT_A0)
/* BLUETOOTH SETUP */
/** @brief Temperature Sensor Characteristic UUID. */
#define BT_UUID_TEMPSENSOR_VAL BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_TEMP_MYSENSOR_VAL \
	BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_TEMPSENSE BT_UUID_DECLARE_128(BT_UUID_TEMPSENSOR_VAL)
#define BT_UUID_TEMP_MYSENSOR       BT_UUID_DECLARE_128(BT_UUID_TEMP_MYSENSOR_VAL)
//Bluetooth Naming Structure
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
/* PWM SETUP*/
/**
 * @brief Symbol specifying number of times that each duty cycle is to be repeated (after being
 *        played once) and is strictly correlated with the "breath" effect speed.
 */
#define VALUE_REPEATS 0UL
/** @brief Symbol specifying number of loops (breaths) to be performed. */
#define NUM_OF_LOOPS 1UL
/**
 * @brief Symbol specifying number of playbacks to be performed. In this example couple of
 *        playbacks might be considered as one loop.
 */
#define PLAYBACK_COUNT 1UL

/* 
*   @brief Temperature Sensor Notification Properties to be updated and stored
 */
static bool notify_sensor_enabled = false;
static void tempsense_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
    // Get the value for the the notifciation status and store it in the global variable
	notify_sensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}


struct bt_conn *my_conn = NULL;
//Advertising Packet
static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x1a, 0x18),
        //BT_DATA(BT_DATA_SVC_DATA16, 0x18, 0x1a), // UUID for Temperature Sensor
       //BT_DATA(BT_, 0x18,0x1a), // UUID for Temperature Sensor
};
int temperature = 0;
static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
            (BT_LE_ADV_OPT_CONNECTABLE |
            BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
            BT_GAP_ADV_FAST_INT_MIN_1, /* 0x30 units, 48 units, 30ms */
            BT_GAP_ADV_FAST_INT_MAX_1, /* 0x60 units, 96 units, 60ms */
            NULL); /* Set to NULL for undirected advertising */
//Scan Response Packet
static const struct bt_data sd[] = {
        //BT_DATA(BT_DATA_UUID16_ALL,0x1a,0x18),
       //BT_DATA_BYTES(BT_DATA_UUID16_ALL,0x2a,0x6e),
       //BT_DATA(BT_DATA_SVC_DATA16, 0x18, 0x1a), // UUID for Temperature Sensor
};

BT_GATT_SERVICE_DEFINE(tempsensor_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	/*BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_MYSENSOR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),*/
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(tempsense_ccc_mysensor_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);

//Bluetooth connection callback's
void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        //LOG_ERR("Connection error %d", err);
        return;
    }
    //LOG_INF("Connected");
    my_conn = bt_conn_ref(conn);
    return;
    /* STEP 3.2  Turn the connection status LED on */
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    //LOG_INF("Disconnected. Reason %d", reason);
    bt_conn_unref(my_conn);
    return;
    /* STEP 3.3  Turn the connection status LED off */
}



struct bt_conn_cb connection_callbacks = {
    .connected              = on_connected,
    .disconnected           = on_disconnected,
};

int tempsensor_send_sensor_notify(uint32_t sensor_value)
{
	if (!notify_sensor_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &tempsensor_svc.attrs[2], 
			      &sensor_value,
			      sizeof(sensor_value));
}


//create a global PWM instance
nrfx_pwm_t pwm_instance1 = NRFX_PWM_INSTANCE(PWM_INST_IDX);
typedef struct {
    int temp_reading;
} SensorReading;
K_MSGQ_DEFINE(device_message_queue, sizeof(SensorReading), 16, 4);
// Global PWM configuration setup
// 1 PWM period is 1.25us, clco8k period is 62.5ns
// automatic stepping
nrfx_pwm_config_t config1 = {    
    .output_pins = {LED1_PIN},
    .pin_inverted = {1},
    .irq_priority = 7,
    .base_clock = NRF_PWM_CLK_16MHz,
    .count_mode = NRF_PWM_MODE_UP,
    .top_value = 20,
    .load_mode = NRF_PWM_LOAD_COMMON,
    .step_mode = NRF_PWM_STEP_AUTO,
    .skip_gpio_cfg = 0
};

// Mutex definition of variable synchronisation/ control over critical code section timing
K_MUTEX_DEFINE(key);
nrf_pwm_values_common_t pwm_val []=
{
    0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 13, 6, 13, 6, 13, 6, 6, 13, 13, 13, 13, 6, 13
};
static nrf_pwm_values_common_t reset_seq [55] = {0};
nrf_pwm_values_common_t pwm_array_val [192] = {0};
nrf_pwm_values_common_t pwm_array_val2 [192] = {0};

// Led Array Build Function:
/*
Inputs:
    nrf_pwm__values_common_t *array: Pointer to array[0] to store signal PWM duty cycles as one big block to send as a continouse signal
    int lednumber: number where it sits in the LED strip 0-7 to place the colour sequnce in teh correct order
    nrf_pwm__values_common_t *coloursignal: pointer to coloursignal[0] which is the 24 element array that holds the colour value for 1 LED
Outputs:
    none
Description:
    concatenates each LED colour setup array as one big block of a pwm duty cycle sequence to be sent as a continous signal due to the WS2812 signalling
*/
void configurePPI(void){
    uint8_t pwm_stop_ppi_channel;
    nrfx_err_t err = nrfx_gppi_channel_alloc(&pwm_stop_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }
    nrfx_gppi_channel_endpoints_setup(pwm_stop_ppi_channel,nrfx_pwm_event_address_get(&pwm_instance1, NRF_PWM_EVENT_SEQEND0),
                                  nrf_pwm_task_address_get(pwm_instance1.p_reg,  NRF_PWM_TASK_STOP));

    nrfx_gppi_channels_enable(BIT(pwm_stop_ppi_channel));
}
void ledarraybuild(nrf_pwm_values_common_t *array,int lednumber,nrf_pwm_values_common_t *coloursignal){
    int base;
    base = 24*lednumber;
    for(int i = 0; i < 24; i++){
        array[base + i] = coloursignal[i];
    }
    return;
}

// Colour Sequence Function:
/*
Inputs:
    nrf_pwm__values_common_t *val: Pointer to val[0], which is a 24 element array of uint16_t values to store individual duty cycles in
    uint8_t red: Red component of a colour RGB hex code for ease of use
    uint8_t green: Green component of a colour RGB hex code for ease of use
    uint8_t blue: Blue component of a colour RGB hex code for ease of use
    
Outputs:
    none
Description:
    Adds appropriate duty cycle values in an array to represent a colour for one LED from a hex code
    to match the WS2812 signalling protocol. This is inverted as it appears the PWM signal has an 
    an inversion at some point. High should be VH for ~800ns, Low should be VH for ~400ns, this is designated
    from the hex code with the correct ordering as well.
*/
void colour_sequence(nrf_pwm_values_common_t *val, uint8_t red, uint8_t green, uint8_t blue){

    for(int i = 7; i >= 0; i--){
        //redblock
        if((red&(0x80>>i))){
            val[i + 8] = 5;
        } else {
            val[i + 8] = 15;
        }
    
        // blue block
        if((blue&(0x80>>i))){
            val[i + 16] = 5;
        } else {
            val[i + 16] = 15; 
        }
        // green block
        if((green&(0x80>>i))){
            val[i] = 5;
        } else {
            val[i] = 15;
        }
    }
    return;
}

//once PWM has a loop event this is triggered to setup a looped PWM.-> we need to change this to not be looping
//currently set to only loop once
static void pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
{
    nrfx_pwm_t * inst = p_context;

}


static void ledupdate(uint32_t highcolour, uint32_t lowcolour, int level){
    static nrf_pwm_values_common_t pwm_array_val [192] = {0};
    nrf_pwm_sequence_t const seq_set =
    {
        .values = {pwm_array_val},
        .length = NRFX_ARRAY_SIZE(pwm_array_val),
        .repeats = VALUE_REPEATS,
        .end_delay = 0
    };
    //PWM0->PSEL
    for(int i = 0; i < 8; i++){
        colour_sequence(pwm_val, (uint8_t)((highcolour&(0xFF000000))>>24), (uint8_t)((highcolour&(0x00FF0000))>>16), (uint8_t)((highcolour&(0x0000FF00))>>8));
        
        //colour_sequence(pwm_val, 0x01, 0x0F, 0x01);
        ledarraybuild(pwm_array_val, i, pwm_val);
    }
    for(int i = 0; i < level; i++){
        colour_sequence(pwm_val, (uint8_t)((lowcolour&0xFF000000)>>24), (uint8_t)((lowcolour&0x00FF0000)>>16), (uint8_t)((lowcolour&0x0000FF00)>>8));
        //colour_sequence(pwm_val,0x0F, 0x00, 0x00);
        ledarraybuild(pwm_array_val, i, pwm_val);
    } 

    nrfx_pwm_simple_playback(&pwm_instance1, &seq_set, PLAYBACK_COUNT, NRFX_PWM_FLAG_LOOP);

    
    //nrfx_pwm_stop(&pwm_instance, true);
    //nrfx_pwm_sequence_update(&pwm_instance, 0,&seq_set);
    return;
}


//Thread to change the colour based on the ADC value/ Temperature
void thread0(void)
{
    #if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX), 0, 0);
    #endif
    nrfx_err_t status;
    (void) status;
    int temp;
    k_mutex_lock(&key, K_FOREVER);
    configurePPI();
    k_mutex_unlock(&key);

    //NRFX_EXAMPLE_LOG_INIT();

    //NRFX_LOG_INFO("Starting nrfx_pwm example for sequence loaded in common mode.");
    //NRFX_EXAMPLE_LOG_PROCESS();
    //colour_sequence(pwm_val, 0x00, 0x00, 0xFF);
    
    //nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN);
    status = nrfx_pwm_init(&pwm_instance1, &config1, pwm_handler, &pwm_instance1);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    NRF_PWM0->PSEL.OUT[0] = 0x0000000C;// fix for bug which moves the PIN to 0xD
    k_mutex_lock(&key, K_FOREVER);
    //LOG_HEXDUMP_INF(pwm_array_val, sizeof(pwm_array_val),"Thread0 Data");
    /* nrfx_pwm_simple_playback(&pwm_instance, &seq_set1, PLAYBACK_COUNT, NRFX_PWM_FLAG_LOOP);
    k_busy_wait(20000);*/
    int i = 0;
    //ledupdate(0xFF, 0X00, (4));
    k_mutex_unlock(&key);
 
    // at the moment set not to change as the LEDS can't update -> focus on getting ADC to work.
	while (1) {
        i++;
        nrfx_err_t ret = k_msgq_get(&device_message_queue, &temp, K_FOREVER);
        if (ret){
            LOG_ERR("Return value from k_msgq_get = %d",ret);
        }
        k_mutex_lock(&key, K_FOREVER);
        ledupdate(0x0000FF00, 0xFF000000, (temp*8)/40);
        k_msleep(50);
        
        k_mutex_unlock(&key);
        k_msleep(100);
        //k_busy_wait(20000);

	}
}

static int16_t sample;
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
//Thread to configure ADC and to filter + convert the sensor value to a temperature.
void thread1(void)
{   
    
    //status value for errors
            /* STEP 5.1 - Connect ADC interrupt to nrfx interrupt handler */
        k_mutex_lock(&key, K_FOREVER);
        #if defined(__ZEPHYR__)
        IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);
        #endif
        /* STEP 5.2 - Connect ADC interrupt to nrfx interrupt handler */
        nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
        if (err != NRFX_SUCCESS) 
        {
                printk("nrfx_saadc_mode_trigger error: %08x", err);
                return;
        }
        k_mutex_unlock(&key);
        /* STEP 5.3 - Configure the SAADC channel */
        channel.channel_config.gain = NRF_SAADC_GAIN1_6;
        err = nrfx_saadc_channels_config(&channel, 1);
        if (err != NRFX_SUCCESS) 
        {
		    printk("nrfx_saadc_channels_config error: %08x", err);
	        return;
	    }

        /* STEP 5.4 - Configure nrfx_SAADC driver in simple and blocking mode */
        err = nrfx_saadc_simple_mode_set(BIT(0),
                                         NRF_SAADC_RESOLUTION_12BIT,
                                         NRF_SAADC_OVERSAMPLE_DISABLED,
                                         NULL);
        if (err != NRFX_SUCCESS) {
                printk("nrfx_saadc_simple_mode_set error: %08x", err);
                return;
        }
        
        /* STEP 5.5 - Set buffer where sample will be stored */
        err = nrfx_saadc_buffer_set(&sample, 1);
        if (err != NRFX_SUCCESS) {
                printk("nrfx_saadc_buffer_set error: %08x", err);
                return;
        }
        k_mutex_lock(&key, K_FOREVER);
        bt_conn_cb_register(&connection_callbacks);


        err = bt_enable(NULL);
        if (err) {	
            LOG_ERR("Bluetooth init failed (err %d)\n", err);	
            return;
            }
            LOG_INF("Bluetooth initialized\n");
        //wait_xtal_to_start();
        k_mutex_unlock(&key);
        err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
            if (err) {		
                LOG_ERR("Advertising failed to start (err %d)\n", err);		
                return;	
            }	
        while(1){
            nrfx_err_t err = nrfx_saadc_mode_trigger();
            if (err != NRFX_SUCCESS) {
                    printk("nrfx_saadc_mode_trigger error: %08x", err);
                    return;
            }
            k_mutex_lock(&key,K_FOREVER);
            /* STEP 7.3 - Calculate and print voltage */
            int voltage = ((600*6) * sample) / ((1<<12));
            uint32_t temp = (voltage - 500)/19.5;
            temperature = temp;
            nrfx_err_t ret = k_msgq_put(&device_message_queue,&temp, K_FOREVER);
            if (ret){
                LOG_ERR("Return value from k_msgq_put = %d",ret);
            }
            tempsensor_send_sensor_notify(temp);
            //bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
            k_mutex_unlock(&key);
            k_msleep(500);
        }

    }
   K_THREAD_DEFINE(thread0_id, STACKSIZE, thread0, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
   K_THREAD_DEFINE(thread1_id, STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD1_PRIORITY, 0, 0);
