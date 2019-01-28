
#pragma once

#include "VL53L0X.h"
#include "frc/DigitalOutput.h"

#define MAX_MICROLIDAR_DEVICE_COUNT 20
class MicroLidar
{
public:

    enum MeasurementMode { SINGLE_MEASURE_MODE=0, CONTINUOUS_MEASURE_MODE };

    MicroLidar(std::string I2C_Bus, MeasurementMode MeasureMode);
    ~MicroLidar();
    
    // Add a single device
    void Add(int DigGpioIdx);
    // Init all sensors that were Add'ed
    void InitSensors();
    // Start taking measurements
    void StartMeasurements();
    // Stop Measurememnts
    void StopMeasurements();
    // Get the measurement for a single device
    int GetMeasurement(int DigGpioIdx);

private:
    void SetAllGpio(bool State);
    void SetOneGpio(int DeviceIdx, bool State);
    void ReadOne(int DigGpioIdx);
    void ReadAll();

    MeasurementMode MeasureMode;
    // The number of devices that were Add'ed
    int DeviceCount;
    std::string I2CBusName;
    // The index into Devices for each that have been added
    // I.E. DeviceList[0] is set to the DigGpioIdx for the first sensor that was Add'ed
    int DeviceList[MAX_MICROLIDAR_DEVICE_COUNT];
    // Devices is indexed by GPIO. Devices[2] correspomds to device connected to Gpio2
    VL53L0X *Devices[MAX_MICROLIDAR_DEVICE_COUNT];
    frc::DigitalOutput *DigGpio[MAX_MICROLIDAR_DEVICE_COUNT];
};

