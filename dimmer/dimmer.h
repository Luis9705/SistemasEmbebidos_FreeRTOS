/*
 * dimmer.h
 *
 *  Created on: May 2, 2020
 *      Author: FLL1GA
 */
/// @file dimmer.h
/// @brief dimmer header file.
#ifndef DIMMER_H_
#define DIMMER_H_

#include <stdint.h>

void dimmer_setup(void);
void dimmer_update_percentage(uint16_t percentage);

#endif /* DIMMER_H_ */
