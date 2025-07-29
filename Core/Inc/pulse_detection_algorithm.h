/*
 * pulse_detection_algorithm.h
 *
 *  Created on: Apr 9, 2025
 *      Author: 14382
 */

#ifndef INC_PULSE_DETECTION_ALGORITHM_H_
#define INC_PULSE_DETECTION_ALGORITHM_H_

bool checkForBeat(int32_t sample);
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

#endif /* INC_PULSE_DETECTION_ALGORITHM_H_ */
