/*
 * VL53L0X.h
 *
 *  Created on: 2016/08/30
 *      Author: Yuki
 */

#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "required_version.h"

void print_pal_error(VL53L0X_Error Status);
void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice);
VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev);

/*VL53L0X_SINGLRANGE_MODE*/
//#define		VL53L0X_SINGLERANGE_NORMAL
//#define		VL53L0X_SINGLERANGE_LONG_RANGE
//#define		VL53L0X_SINGLERANGE_HIGH_SPEED
#define		VL53L0X_SINGLERANGE_HIGH_ACCURACY

#if !defined(VL53L0X_SINGLERANGE_NORMAL)\
	&& !defined (VL53L0X_SINGLERANGE_LONG_RANGE)\
	&& !defined (VL53L0X_SINGLERANGE_HIGH_SPEED)\
	&& !defined (VL53L0X_SINGLERANGE_HIGH_ACCURACY)
#error "Define according the VL53L0X_SINGLRANGE_MODE"
#endif

#endif /* VL53L0X_H_ */
