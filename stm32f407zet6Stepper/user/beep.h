#ifndef __BEEP_H__
#define __BEEP_H__
#include "main.h"
extern uint8_t beep_long_flag;
extern uint8_t beep_short_flag;
void set_beep_short_flag(void);
void set_beep_long_flag(void);
void beep_short(void);
void beep_long(void);
#endif
