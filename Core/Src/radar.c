#include "radar.h"

#include "tim.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "adc.h"
#include "dac.h"

#include "waveform.h"
#include "constants.h"
#include "main.h"
#include "serial_communication.h"
#include "cobs_encoding.h"

#define GRAPHS_NUM 4

ALIGN_32BYTES (__IO uint16_t   dma_buffer_0[SAMPLES]);
ALIGN_32BYTES (__IO uint16_t   dma_buffer_1[SAMPLES]);

ALIGN_32BYTES (__IO uint16_t   g_video[SAMPLES]);
ALIGN_32BYTES (__IO uint16_t   g_video_1[SAMPLES]);
ALIGN_32BYTES (__IO float data_output[GRAPHS_NUM][GRAPH_SAMPLES]);

typedef struct DATA_PACKET
{
    char init_of_packet[4];
    uint8_t code;
    uint8_t reserved[3];
    float data_output[GRAPHS_NUM][GRAPH_SAMPLES];
    char end_of_packet[4];
};

struct DATA_PACKET g_data_packet = {.init_of_packet = "ini", .code = 0x80,  .end_of_packet = "end"};

float *g_fft_result;
float *g_fft_mag;
float *f_adc_samples;

uint64_t _radar_start_time = 0;
static bool new_data = false;

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

void toggle_radar_ref(enum FMCW_REF ref)
{
	static bool pin_value_PG2 = true;
	static bool pin_value_PG3 = false;

	if(ref == REF_PG2)
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, pin_value_PG2);
		pin_value_PG2 = !pin_value_PG2;
	}
	else if(ref == REF_PG3)
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, pin_value_PG3);
		pin_value_PG3 = !pin_value_PG3;
	}
}

void radar_routine()
{
#if 0
	for(int i=0, k=0; i < GRAPH_SAMPLES; i++, k+=4)
	{
		g_data_packet.data_output[0][i] = (float) g_video[k];
	}
	SCB_InvalidateDCache_by_Addr((uint32_t*) g_video, sizeof(g_video));

	for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
	{
		g_data_packet.data_output[1][i] = (float) g_video_1[k];
	}
	SCB_InvalidateDCache_by_Addr((uint32_t*) g_video_1, sizeof(g_video_1));

#endif

	dsp();

	for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
	{
		g_data_packet.data_output[0][i] = g_fft_mag[i];
	}
	g_data_packet.data_output[0][0] = 0;

	for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
	{
		g_data_packet.data_output[1][i] = g_fft_mag[i + (GRAPH_SAMPLES - 1)];
	}
	g_data_packet.data_output[1][0] = 0;

	for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
	{
		g_data_packet.data_output[2][i] = g_fft_mag[i + ((GRAPH_SAMPLES* 2) - 1)];
	}
	g_data_packet.data_output[2][0] = 0;

	for(int i=0,k=0; i < GRAPH_SAMPLES; i++,k+=4)
	{
		g_data_packet.data_output[3][i] = g_fft_mag[i +  + ((GRAPH_SAMPLES* 3) - 1)];
	}
	g_data_packet.data_output[3][0] = 0;

	uint64_t time_now = get_1ms_tick_counter();
	static uint64_t last_time = 0;
	uint64_t time_elapsed = time_now - last_time;
	if(time_elapsed > 100)
	{
		if(new_data == true)
		{
			int tx_ret = uart_tx_data(&g_data_packet.init_of_packet, sizeof(g_data_packet));
			SCB_InvalidateDCache_by_Addr((uint32_t*) &g_data_packet, sizeof(g_data_packet));

			if(tx_ret != 0)
			{
				//HAL != 0
			}
			new_data = false;
		}
		last_time = get_1ms_tick_counter();
	}
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

	set_out_freq((float)16.0, FMCW_ECHO);
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
	ret = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)dac_waveform, DAC_SAMPLES,DAC_ALIGN_12B_R);
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

void set_out_freq(float frequency, enum OUTPUT output)
{
	const float timer_freq = 200e6;

	uint32_t dac_samples_size = sizeof(waveform)/sizeof(uint16_t);
	uint32_t timer_period;

	if(output == FMCW_ECHO)
	{
		timer_period = (uint32_t) (timer_freq / ((float)(dac_samples_size * frequency)));
		reload_timer_period(TIM4, timer_period);
	}
	else if(output == FMCW_REF)
	{
		timer_period = (uint32_t) (timer_freq / ((float)(DAC_CH2_SAMPLES * frequency)));
		reload_timer_period(TIM8, timer_period);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	toggle_radar_ref(REF_PG2);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	toggle_radar_ref(REF_PG3);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
		new_data = true;

	}
	else if(hadc == &hadc3)
	{
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static float frequency = 4000;

	if(GPIO_Pin == GPIO_PIN_13)
	{
		frequency = frequency + (16*100);
		set_out_freq(frequency, FMCW_ECHO);
	}
}
