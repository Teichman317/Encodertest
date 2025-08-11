/*
 * Encoder.h
 *
 *  Created on: Aug 10, 2025
 *      Author: Sim
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"


void Encoder_Init(void);
void Encoder_Update(void);
int16_t Encoder_GetPosition(void);

#endif // ENCODER_H
