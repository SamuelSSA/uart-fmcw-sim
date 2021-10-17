/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADAR_H
#define __RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

void radar_routine();
void init_hardware();

void init_timers();
void init_dac();
void init_adc();

uint64_t _1ms_tick_acc;
extern uint64_t _radar_start_time;


#endif /* __RADAR_H */
