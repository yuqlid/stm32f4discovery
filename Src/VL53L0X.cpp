/*
 * VL53L0X.cpp
 *
 *  Created on: 2016/08/30
 *      Author: Yuki
 */

#include "VL53L0X.h"
//#include "diag/Trace.h"
//#include "stm32f4xx.h"

void print_pal_error(VL53L0X_Error Status){
	char buf[VL53L0X_MAX_STRING_LENGTH];
	VL53L0X_GetPalErrorString(Status, buf);
	printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
	char buf[VL53L0X_MAX_STRING_LENGTH];
	uint8_t RangeStatus;

	/*
	 * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
	 */

	RangeStatus = pRangingMeasurementData->RangeStatus;

	VL53L0X_GetRangeStatusString(RangeStatus, buf);
	printf("Range Status: %i : %s\n", RangeStatus, buf);

}


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}


VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
	int i;
	FixPoint1616_t LimitCheckCurrent;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	if(Status == VL53L0X_ERROR_NONE)
	{
		printf ("Call of VL53L0X_StaticInit\n");
		Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
		print_pal_error(Status);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
		printf ("Call of VL53L0X_PerformRefCalibration\n");
		Status = VL53L0X_PerformRefCalibration(pMyDevice,
				&VhvSettings, &PhaseCal); // Device Initialization
		print_pal_error(Status);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{
		printf ("Call of VL53L0X_PerformRefSpadManagement\n");
		Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
				&refSpadCount, &isApertureSpads); // Device Initialization
		printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
		print_pal_error(Status);
	}

	if(Status == VL53L0X_ERROR_NONE)
	{

		// no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
		printf ("Call of VL53L0X_SetDeviceMode\n");
		Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		print_pal_error(Status);
	}

	// Enable/Disable Sigma and Signal check
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

#if defined(VL53L0X_SINGLERANGE_NORMAL)
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
		(FixPoint1616_t)(1.5*0.023*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
			Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
					30000);
		}
#elif defined(VL53L0X_SINGLERANGE_LONG_RANGE)
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		(FixPoint1616_t)(0.1*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		(FixPoint1616_t)(60*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
				33000);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}
#elif defined(VL53L0X_SINGLERANGE_HIGH_SPEED)
	if (Status == VL53L0X_ERROR_NONE) {
			Status = VL53L0X_SetLimitCheckValue(pMyDevice,
					VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		(FixPoint1616_t)(0.25*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		(FixPoint1616_t)(32*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
				20000);
	}
#elif defined(VL53L0X_SINGLERANGE_HIGH_ACCURACY)
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		(FixPoint1616_t)(0.25*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
		(FixPoint1616_t)(18*65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
				200000);
	}
#endif

	/*
	 *  Step  4 : Test ranging mode
	 */

#if	defined(VL53L0X_SINGLERANGE_NORMAL)
	printf("Scan_Normal_Mode\n");
#elif	defined(VL53L0X_SINGLERANGE_LONG_RANGE)
	printf("Scan_Long_Range_Mode\n");
#elif	defined(VL53L0X_SINGLERANGE_HIGH_SPEED)
	printf("Scan_High_Speed_Mode\n");
#elif	defined(VL53L0X_SINGLERANGE_HIGH_ACCURACY)
	printf("Scan_High_Accuracy_Mode\n");
#endif

	if(Status == VL53L0X_ERROR_NONE)
	{
		//for(i=0;i<10;i++){
		while(1){
			//HAL_Delay(100);
			printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
			Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
				 &RangingMeasurementData);

			print_pal_error(Status);
			print_range_status(&RangingMeasurementData);

			VL53L0X_GetLimitCheckCurrent(pMyDevice,
				 VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

			printf("RANGE IGNORE THRESHOLD: %i.%i\n\n", (int)(LimitCheckCurrent/65536), (int)(LimitCheckCurrent%65536));


			if (Status != VL53L0X_ERROR_NONE) break;

			printf("Measured distance: %i[mm]\n", RangingMeasurementData.RangeMilliMeter);
		}
	}
	return Status;
}





