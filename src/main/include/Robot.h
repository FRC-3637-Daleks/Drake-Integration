/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

class Sensor_VL53L0X 
{
  public:

  #define VERSION_REQUIRED_MAJOR 1
  #define VERSION_REQUIRED_MINOR 0
  #define VERSION_REQUIRED_BUILD 1
 
  void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    ////printf("API Status: %i : %s\n", Status, buf);
  }

  void Init(int I2cAddr)
  {
        // Initialize Comms
    pMyDevice = new VL53L0X_Dev_t;
    pMyDevice->I2cDevAddr      = I2cAddr;  //0x29;

    pVersion = new VL53L0X_Version_t;

    pMyDevice->fd = VL53L0X_i2c_init("/dev/i2c-2", pMyDevice->I2cDevAddr);//choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3
    if (pMyDevice->fd < 0) 
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        printf ("Failed to init\n");
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD )
        {
            printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }


    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(pMyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            printf("VL53L0X_GetDeviceInfo:\n");
            printf("Device Name : %s\n", DeviceInfo.Name);
            printf("Device Type : %s\n", DeviceInfo.Type);
            printf("Device ID : %s\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
        	printf("Error expected cut 1.1 but found cut %d.%d\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        print_pal_error(Status);
    }
    Status = VL53L0X_ERROR_NONE;
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit: %d\n", pMyDevice->fd);
        Status = VL53L0X_DataInit(pMyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
     

      Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);
      if(Status == VL53L0X_ERROR_NONE)
      {
          printf("VL53L0X_GetDeviceInfo: %d\n",pMyDevice->fd);
          printf("Device Name : %s\n", DeviceInfo.Name);
          printf("Device Type : %s\n", DeviceInfo.Type);
          printf("Device ID : %s\n", DeviceInfo.ProductId);
          printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
          printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);
      }
      if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) 
      {
        printf("Error expected cut 1.1 but found cut %d.%d\n",
                      DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
        Status = VL53L0X_ERROR_NOT_SUPPORTED;
            
      }
      print_pal_error(Status);
    }
 
    Reset();
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
      printf ("Call of VL53L0X_StaticInit %d\n", pMyDevice->fd);
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

 /*   if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetSequenceStepEnable(pMyDevice,VL53L0X_SEQUENCESTEP_DSS, 1);
    }*/	

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.1*65536));
	  }			
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(60*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
        		33000);
	  } 

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
  }

  int MeasureMM()
  {
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    if(Status == VL53L0X_ERROR_NONE || 1)
    {
      //printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
      Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

     print_pal_error(Status);
      //print_range_status(&RangingMeasurementData);


      if (Status == VL53L0X_ERROR_NONE)
      {
        if(RangingMeasurementData.RangeMilliMeter < 1000)//printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
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
  VL53L0X_Dev_t *pMyDevice;
  //VL53L0X_Version_t                   Version;
  VL53L0X_Version_t                  *pVersion;
   VL53L0X_DeviceInfo_t                DeviceInfo;
  //VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

  int32_t status_int;
};
