#include "radar.h"

#include "arm_math.h"
#include "arm_const_structs.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "waveform.h"
#include "constants.h"
#include "main.h"
#include "serial_communication.h"
#include "cobs_encoding.h"

#define GRAPHS_NUM 4

volatile ALIGN_32BYTES (__IO uint16_t   g_video[SAMPLES]);
volatile ALIGN_32BYTES (__IO uint16_t   g_video_1[SAMPLES]);
volatile ALIGN_32BYTES (__IO float data_output[GRAPHS_NUM][GRAPH_SAMPLES]);

float teste_tx2[] = {1.0,2.0,3.0};

float *g_fft_result;
float *g_fft_mag;
float *f_adc_samples;

uint64_t _radar_start_time = 0;

enum STATE_MACHINE
{
	STANDBY,
	ACQUISITIONING,
	PROCESSING,
	OUTPUT_PROCESSING,
	SEND_DATA,
	RECEIVE_DATA,
	NUM_OF_STATES
};

enum
{
	UP_CHIRP,
	DOWN_CHIRP
};

enum OUTPUT
{
	FMCW_ECHO,
	FMCW_REF
};

void dsp()
{
	arm_status status;
	arm_rfft_fast_instance_f32 fft_instance;
	uint8_t ifftFlag = 0;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_fast_init_f32(&fft_instance, SAMPLES);

	for(int i = 0; i < SAMPLES; i++)
	{
		f_adc_samples[i] = (float) g_video[i];
	}

	arm_rfft_fast_f32(&fft_instance, f_adc_samples, g_fft_result, ifftFlag);
	arm_cmplx_mag_f32(g_fft_result,g_fft_mag,HALF_SAMPLES);
}

uint64_t get_1ms_tick_counter()
{
	return _1ms_tick_acc;
}

void update_state(enum STATE_MACHINE *st)
{
	uint64_t time_now = get_1ms_tick_counter();
	int state_int = (int) *st;
	if((time_now - _radar_start_time) >= 50)
	{
		_radar_start_time = get_1ms_tick_counter();
		state_int = (state_int + 1) % NUM_OF_STATES;
		*st = (enum STATE_MACHINE) state_int;
	}
}

void radar_routine()
{
	static bool new_data = false;
	static bool reload   = false;
	static enum STATE_MACHINE state = STANDBY;
	const int output_mem_size = GRAPHS_NUM*GRAPH_SAMPLES*sizeof(float);

	if(state == STANDBY)
	{

	}
	else if(state == ACQUISITIONING)
	{

	}
	else if(state == PROCESSING)
	{
		dsp();
	}
	else if(state == OUTPUT_PROCESSING)
	{
		for(int i=0, k=0; i < GRAPH_SAMPLES; i++, k+=4)
		{
			data_output[0][i] = (float) g_video[k];
		}

		for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
		{
			data_output[1][i] = (float) g_video_1[k];
		}

		for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=2)
		{
			data_output[2][i] = g_fft_mag[i];
		}
		data_output[2][0] = 0;

		for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=2)
		{
			data_output[3][i] = g_fft_mag[i + (GRAPH_SAMPLES - 1)];
		}
		data_output[3][0] = 0;
	}
	else if(state == SEND_DATA)
	{
		char teste_tx[] = "Oi Mundo\n";
		//float teste_tx2[] = {1.0,2.0,3.0};

		uart_tx_data(teste_tx2,12);

#if 0
		uart_tx_data(data_output,output_mem_size);
		SCB_InvalidateDCache_by_Addr((uint32_t*) &g_video, SAMPLES * sizeof(float));
		SCB_InvalidateDCache_by_Addr((uint32_t*) &data_output, output_mem_size);
#endif
/*
		if(new_data == true)
		{
			//udp_send_data((void*)data_output, output_mem_size);
			SCB_InvalidateDCache_by_Addr((uint32_t*) &g_video, SAMPLES * sizeof(float));
			SCB_InvalidateDCache_by_Addr((uint32_t*) &data_output, output_mem_size);
			new_data = false;
		}
*/
	}
	else if(state == RECEIVE_DATA)
	{
		//uart_rx_data
	}
	update_state(&state);
}

void init_hardware()
{
	g_fft_result  = (float *) malloc(sizeof(float) * SAMPLES);
	if(g_fft_result == NULL)
	{
		while(1);
	}

	g_fft_mag 	  = (float *) malloc(sizeof(float) * SAMPLES);
	if(g_fft_mag == NULL)
	{
		while(1);
	}

	f_adc_samples = (float *) malloc(sizeof(float) * SAMPLES);
	if(f_adc_samples == NULL)
	{
		while(1);
	}

	init_dac();
	init_adc();
	init_timers();

	//set_out_freq(4000,FMCW_ECHO);
	//reload_timer_period(TIM4, 10000);
	//reload_timer_period(TIM3, 50000);
}
// INP15 -> A0 = Sinal do radar -> ADC trigado pelo TIMER 3
// Canal do DAC Channel 2(PA5) ligado ao A0 -> DAC trigado pelo TIMER 4

void init_timers()
{
	if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Base_Start(&htim4) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Base_Start(&htim8) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
}

void init_dac()
{
	HAL_StatusTypeDef ret;
	ret = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)waveform, DAC_SAMPLES,DAC_ALIGN_12B_R);
	if(ret != HAL_OK)
	{
		Error_Handler();
	}

	ret = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2,(uint32_t *)waveform, DAC_SAMPLES,DAC_ALIGN_12B_R);
	if(ret != HAL_OK)
	{
		Error_Handler();
	}
}

void init_adc()
{
	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*) g_video, SAMPLES) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*) g_video_1, SAMPLES) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
	}
	else if(hadc == &hadc3)
	{
	}
}

void set_out_freq(float frequency, enum OUTPUT output)
{
	float freq_dac = 75e6;
	uint32_t timer_period = 1;

	timer_period = (uint32_t) (freq_dac / ((float)(DAC_SAMPLES * frequency)));

	if(output == FMCW_ECHO)
	{
		reload_timer_period(TIM4, timer_period);
	}
	else if(output == FMCW_REF)
	{
		reload_timer_period(TIM8, timer_period);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{

}

