#include "mbed.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n\r", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n\r", RangeStatus, buf);

}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev)
{
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

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev)
{
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

VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice = &MyDevice;

void init_sensor()
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;

    printf("VL53L0X API Simple Ranging Example\r\n");

    // Initialize Comms
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;

    printf("Init comms\r\n");

    if(Status == VL53L0X_ERROR_NONE) {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    printf("VL53L0X API Version: %d.%d.%d (revision %d)\r\n", pVersion->major, pVersion->minor ,pVersion->build, pVersion->revision);

    int addr;

    addr = VL53L0X_scan();
    printf("Device found at: %i\r\n", addr);
    //uint8_t data;
    //data=0;
    if(Status == VL53L0X_ERROR_NONE) {
        printf ("Call of VL53L0X_DataInit\r\n");
        uint16_t osc_calibrate_val=0;
        Status = VL53L0X_RdWord(&MyDevice, VL53L0X_REG_OSC_CALIBRATE_VAL,&osc_calibrate_val);
        printf("%i\r\n",osc_calibrate_val);
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE) {
            printf("VL53L0X_GetDeviceInfo:\r\n");
            printf("Device Name : %s\r\n", DeviceInfo.Name);
            printf("Device Type : %s\r\n", DeviceInfo.Type);
            printf("Device ID : %s\r\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\r\n", DeviceInfo.ProductRevisionMajor);
            printf("ProductRevisionMinor : %d\r\n", DeviceInfo.ProductRevisionMinor);

            if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                printf("Error expected cut 1.1 but found cut %d.%d\r\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        print_pal_error(Status);
    }

    Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE) {
        printf ("Call of VL53L0X_StaticInit\r\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        // StaticInit will set interrupt by default
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) {
        printf ("Call of VL53L0X_PerformRefCalibration\r\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
                                               &VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) {
        printf ("Call of VL53L0X_PerformRefSpadManagement\r\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                 &refSpadCount, &isApertureSpads); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) {

        printf ("Call of VL53L0X_SetDeviceMode\r\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }
}

uint32_t take_measurement()
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;

    if(Status == VL53L0X_ERROR_NONE) {
        //printf ("Call of VL53L0X_StartMeasurement\r\n");
        Status = VL53L0X_StartMeasurement(pMyDevice);
        print_pal_error(Status);
    }

    uint32_t measurement;
    if(Status == VL53L0X_ERROR_NONE) {

        Status = WaitMeasurementDataReady(pMyDevice);

        if(Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);

            measurement = pRangingMeasurementData->RangeMilliMeter;
//            printf("In loop measurement %lu: %d\n\r", measurement, pRangingMeasurementData->RangeMilliMeter);
            
            // Clear the interrupt
            VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            VL53L0X_PollingDelay(pMyDevice);
        } else {
            return -1;
        }
    }

    if(Status == VL53L0X_ERROR_NONE) {
        //printf ("Call of VL53L0X_StopMeasurement\r\n");
        Status = VL53L0X_StopMeasurement(pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE) {
        //printf ("Wait Stop to be competed\r\n");
        Status = WaitStopCompleted(pMyDevice);
    }

    if(Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_ClearInterruptMask(pMyDevice,
                                            VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return measurement;
}
