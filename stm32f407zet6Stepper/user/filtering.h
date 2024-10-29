#ifndef _FILTERING_H_
#define _FILTERING_H_

#include "main.h"

float clampFilter(float NewValue, float lastValue, float maxDifference);
float FirstOrderLagFilter(float NewValue, float lastValue, float alpha);
#endif