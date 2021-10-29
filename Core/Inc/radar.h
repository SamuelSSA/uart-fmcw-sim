/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADAR_H
#define __RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

enum OUTPUT
{
	FMCW_ECHO,
	FMCW_REF
};

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

enum FMCW_REF
{
	REF_PG2,
	REF_PG3
};


void init_hardware();
void init_timers();
void init_dac();
void init_adc();

void dsp();
void update_state(enum STATE_MACHINE *st);
void radar_routine();
void set_out_freq(float frequency, enum OUTPUT output);
void toggle_radar_ref(enum FMCW_REF);

uint64_t get_1ms_tick_counter();

uint64_t _1ms_tick_acc;
extern uint64_t _radar_start_time;


#endif /* __RADAR_H */
