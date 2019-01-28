#pragma once

#include <string>
#include <iostream>

#include "Helpers.h"
#include "Timer.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define VL53L0X_SETTLE_TIMEDELAY 0.01 // seconds
#define VL53L0X_DEFAULT_I2C_ADDR 0x29

class Sensor_VL53L0X 
{
  public:

  #define VERSION_REQUIRED_MAJOR 1
  #define VERSION_REQUIRED_MINOR 0
  #define VERSION_REQUIRED_BUILD 1
 
  void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    //printf("API Status: %i : %s\n", Status, buf);
  }

  int _i2c_init(VL53L0X_Dev_t *DeviceP, char *i2c_dev, int i2c_addr)
  {
    DeviceP->fd = VL53L0X_i2c_init(i2c_dev, i2c_addr);  
    if(DeviceP->fd < 0)
    {
        //printf("_i2c_init failed on %s at addr=0x%02X\n", i2c_dev, i2c_addr);
        return -1;
    }
    //printf("i2c_init successful on bus %s at fd=%d and addr=0x%02X\n", i2c_dev, DeviceP->fd, i2c_addr);
    return 1;
  }

  bool Init(int I2cAddr)
  {
    // Initialize Comms
    BlockingTimer Timer;

    // We need to use a temp i2c device to get comms started using the default addess
    VL53L0X_Dev_t TempDevice;

    // Set the i2c address of our default object to the device default
    TempDevice.I2cDevAddr = VL53L0X_DEFAULT_I2C_ADDR;

    // Set the desired i2c address for the device we are initializing
    MyDevice.I2cDevAddr = I2cAddr;

    // Setup comms to the temporary device
    if(_i2c_init(&TempDevice, "/dev/i2c-2", TempDevice.I2cDevAddr) < 0) 
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    // Using the temp device, send the command to change its adress
    //   NOTE: The address used by the ST library here is an 8-bit address, not 7. Hence the *2;
    if((Status = VL53L0X_SetDeviceAddress(&TempDevice, MyDevice.I2cDevAddr * 2)) != VL53L0X_ERROR_NONE)
    {   
        std::cout << "VL53L0X_SetDeviceAddress failed" << std::endl;
        print_pal_error(Status);
        return false;
    } 

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    // Now we need to initialize comms to the i2c device we'll use
    if(_i2c_init(&MyDevice, "/dev/i2c-2", MyDevice.I2cDevAddr) < 0) 
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
  
    if((Status = VL53L0X_DataInit(&MyDevice)) != VL53L0X_ERROR_NONE)
    {
        //printf("KO - VL53L0X_DataInit \n");
         // Data initialization
        print_pal_error(Status);
        return false;
    }

    if((Status = VL53L0X_GetVersion(&Version)) != VL53L0X_ERROR_NONE)
    {
        std::cout << "VL53L0X_GetVersion failed" << std::endl;
        return false;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( Version.major != VERSION_REQUIRED_MAJOR ||
            Version.minor != VERSION_REQUIRED_MINOR ||
            Version.build != VERSION_REQUIRED_BUILD )
        {
            //printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
            //    Version.major, Version.minor, Version.build, Version.revision,
            //    VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
            return false;
        }
    }
    else 
    {
        //printf("Bad VL53L0X_GetVersion \n");
        return false;
    }

    if((Status = VL53L0X_get_device_info(&MyDevice, &DeviceInfo)) == VL53L0X_ERROR_NONE)
    {
        /*printf("VL53L0X_GetDeviceInfo:\n");
        printf("Device Name : %s\n", DeviceInfo.Name);
        printf("Device Type : %s\n", DeviceInfo.Type);
        printf("Device ID : %s\n", DeviceInfo.ProductId);
        printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
        printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);
        */
        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) 
        {
            //printf("Error expected cut 1.1 but found cut %d.%d\n",DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
            Status = VL53L0X_ERROR_NOT_SUPPORTED;
            print_pal_error(Status);
            return false;
        }
        //printf("VL53L0X Micro Lidar Init successful on i2c Addr=0x%02X\n", MyDevice.I2cDevAddr);
    }
    else
    {
        print_pal_error(Status);
        return false;
    }

    Reset();
    
    return true;
  }

  void Reset()
  { 
    int i;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
  
    //void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
    //char buf[VL53L0X_MAX_STRING_LENGTH];
    //uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    //RangeStatus = pRangingMeasurementData->RangeStatus;

    //VL53L0X_GetRangeStatusString(RangeStatus, buf);
    //printf("Range Status: %i : %s\n", RangeStatus, buf);
    
    if(Status == VL53L0X_ERROR_NONE)
    {
      printf ("Call of VL53L0X_StaticInit %d\n", MyDevice.fd);
      Status = VL53L0X_StaticInit(&MyDevice); // Device Initialization
      print_pal_error(Status);
    }
  
    if(Status == VL53L0X_ERROR_NONE)
    {
      printf ("Call of VL53L0X_PerformRefCalibration\n");
      Status = VL53L0X_PerformRefCalibration(&MyDevice,&VhvSettings, &PhaseCal); // Device Initialization
      print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
      printf ("Call of VL53L0X_PerformRefSpadManagement\n");
      Status = VL53L0X_PerformRefSpadManagement(&MyDevice,&refSpadCount, &isApertureSpads); // Device Initialization
      printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
      print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
      // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
      printf ("Call of VL53L0X_SetDeviceMode\n");
      Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
      print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check

 /*   if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetSequenceStepEnable(MyDevice,VL53L0X_SEQUENCESTEP_DSS, 1);
    }*/	

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.1*65536));
	  }			
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(60*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice,33000);
	  } 

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
  }

  int MeasureMM()
  {
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    if(Status == VL53L0X_ERROR_NONE || 1)
    {
      //printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
      Status = VL53L0X_PerformSingleRangingMeasurement(&MyDevice,&RangingMeasurementData);

      print_pal_error(Status);
      //print_range_status(&RangingMeasurementData);

      if (Status == VL53L0X_ERROR_NONE)
      {
        if(RangingMeasurementData.RangeMilliMeter < 1000)
          return RangingMeasurementData.RangeMilliMeter;
        else
          return 9999;  
      } 
      else  
        return 9001;
      
    }
    return 9002; 
  }

  VL53L0X_Error Status; // = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t MyDevice;
  VL53L0X_Version_t                  Version;
  VL53L0X_DeviceInfo_t                DeviceInfo;
  //VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

  int32_t status_int;
};
