/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_ppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
//LOG_MODULE_REGISTER(nrfx_sample, LOG_LEVEL_INF);


#define SAADC_CHANNEL_NUM 	0
#define SAMPLES_IN_BUFFER 	8
#define SAMPLE_RATE 		1000000UL

int16_t saadc_buf1[SAMPLES_IN_BUFFER];
int16_t saadc_buf2[SAMPLES_IN_BUFFER];

nrfx_timer_t timer1 = NRFX_TIMER_INSTANCE(1);

static int16_t averagevalue;

void saadc_evt_handler(nrfx_saadc_evt_t const * p_event)
{
	nrfx_err_t err = NRFX_SUCCESS;
	static uint8_t counter = 0;
	if(p_event->type == NRFX_SAADC_EVT_DONE)
	{		
		averagevalue = 0;//reset temperature value
		printk("SAADC sampled: \n");
		for(uint8_t i = 0; i < p_event->data.done.size; i++)
		{
			printk("%hi\n", p_event->data.done.p_buffer[i]);
			averagevalue += p_event->data.done.p_buffer[i];
			//sum all temperature components
		}
		averagevalue = averagevalue/8;
	}														
	else if(p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
	{
		printk("SAADC calibrated.\n");
	}
	else if(p_event->type == NRFX_SAADC_EVT_BUF_REQ)
	{
		printk("SAADC buffer requested\n");
		counter++;

		if(counter%2)
		{
			err = nrfx_saadc_buffer_set(saadc_buf1, SAMPLES_IN_BUFFER);
			if(err != NRFX_SUCCESS)
			{
				printk("Error! Could not set buffer2: %d\n", err);
			}
		}
		else
		{
			err = nrfx_saadc_buffer_set(saadc_buf2, SAMPLES_IN_BUFFER);
			if(err != NRFX_SUCCESS)
			{
				printk("Error! Could not set buffer1: %d\n", err);
			}
		}
		
	}
	else if(p_event->type == NRFX_SAADC_EVT_FINISHED)
	{
		printk("SAADC finished sampling\n\n");
	}
}

void timer1_evt_handler(nrf_timer_event_t event_type, void * p_context)
{
	// Nothing to do here, the timer's IRQ is not enabled
}

void timer_init(void)
{
	nrfx_err_t err = NRFX_SUCCESS;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

	err = nrfx_timer_init(&timer1, &timer_cfg, timer1_evt_handler);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not initialize TIMER1: %d\n", err);
	}

	nrfx_timer_extended_compare(&timer1, NRF_TIMER_CC_CHANNEL0, SAMPLE_RATE, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);	
}

void saadc_init(void)
{
	nrfx_err_t err = NRFX_SUCCESS;
	nrfx_saadc_channel_t saadc_channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, SAADC_CHANNEL_NUM);
	nrfx_saadc_adv_config_t saadc_adv_cfg = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	saadc_adv_cfg.start_on_end = true;
	
	err = nrfx_saadc_init(IRQ_PRIO_LOWEST);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not initialize SAADC: %d\n", err);
	}

	err = nrfx_saadc_offset_calibrate(NULL);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not calibrate offset: %d\n", err);
	}

	err = nrfx_saadc_channel_config(&saadc_channel);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not configure SAADC channels: %d\n", err);
	}	

	err = nrfx_saadc_advanced_mode_set((1 << SAADC_CHANNEL_NUM), NRF_SAADC_RESOLUTION_10BIT, &saadc_adv_cfg, saadc_evt_handler);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not set advanced SAADC mode: %d\n", err);
	}

	err = nrfx_saadc_buffer_set(saadc_buf1, SAMPLES_IN_BUFFER);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not set buffer1: %d\n", err);
	}

	err = nrfx_saadc_buffer_set(saadc_buf2, SAMPLES_IN_BUFFER);
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could not set buffer2: %d\n", err);
	}

	err = nrfx_saadc_mode_trigger();
	if(err != NRFX_SUCCESS)
	{
		printk("Error! Could trigger mode: %d\n", err);
	}	
}
void tempcalc (void){
	
	return;
}


int main(void)
{
	printk("nrfx_saadc sample on %s\n", CONFIG_BOARD);

	nrfx_err_t err;

	/* Connect SAADC IRQ to nrfx_saadc_irq_handler */
	IRQ_CONNECT(SAADC_IRQn, IRQ_PRIO_LOWEST, nrfx_isr, nrfx_saadc_irq_handler, 0);

	timer_init();
	saadc_init();

	/* Allocate a PPI channel. */
	nrf_ppi_channel_t channel;
	err = nrfx_ppi_channel_alloc(&channel);

	if (err != NRFX_SUCCESS) {
		printk("PPI channel allocation error: %08x", err);
		return 0;
	}

	/* Configure endpoints of the channel so that the TIMER1 CAPTURE0
	 * event is connected with the SAADC SAMPLE task. This means that each time
	 * TIMER1 reaches it's set compare value, the SAADC will sample all 
	 * enabled channel once.
	 */
	nrfx_gppi_channel_endpoints_setup(channel,
		nrfx_timer_event_address_get(&timer1, NRF_TIMER_EVENT_COMPARE0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));


	err = nrfx_ppi_channel_enable(channel);

	if (err != NRFX_SUCCESS) {
		printk("Failed to enable PPI channel, error: %08x", err);
		return 0;
	}
	printk("PPI configured\n");

	nrfx_timer_enable(&timer1);
	printk("TIMER1 started\n");

	while(1)
	{
		k_msleep(1000);
	}
	return 0;
}
