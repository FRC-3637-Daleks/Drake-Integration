/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "frc/SerialPort.h"
#include "frc/AnalogPotentiometer.h"
#include "frc/DigitalOutput.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

frc::AnalogPotentiometer *pot;
frc::AnalogPotentiometer *lineSensor;
//frc::DigitalOutput *led;
//frc::DigitalOutput *led10;
#define LIDAR_COUNT 6
Sensor_VL53L0X *mLidar[LIDAR_COUNT];
frc::DigitalOutput *lidarSHDN[LIDAR_COUNT];

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  pot = new frc::AnalogPotentiometer(0, 300, 0);
  lineSensor = new frc::AnalogPotentiometer(1, 10, 0);

  for (int i = 0; i < LIDAR_COUNT - 2; i++) {
    lidarSHDN[i] = new frc::DigitalOutput(i);
    lidarSHDN[i]->Set(0);
  }
  //led =   new frc::DigitalOutput(0);
  //led10 = new frc::DigitalOutput(11);
  //led10->Set(1);
  //lidarSHDN0 = new frc::DigitalOutput(0);
  //lidarSHDN1 = new frc::DigitalOutput(1);
  //lidarSHDN0->Set(0);

  /*for(int i=0; i<LIDAR_COUNT-2; i++)
  {
    lidarSHDN[i]->Set(1);
    mLidar[i] = new Sensor_VL53L0X;
    mLidar[i]->Init(0x29+i);
  }*/
  lidarSHDN[1]->Set(1);
  mLidar[1] = new Sensor_VL53L0X;
  mLidar[1]->Init(0x30);
  /*lidarSHDN[1]->Set(1);
  mLidar[1] = new Sensor_VL53L0X;
  mLidar[1]->Init(0x30);*/
    //Status = rangingTest(pMyDevice);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  static int state=0;
  //std::cout << "Measurement= " << mLidar[1]->MeasureMM() << ", " << /*mLidar[1]->MeasureMM()*/"99" << std::endl;
  //std::cout << "POT = " << pot->Get() << std::endl;
/*  if (lineSensor->Get() < 1.6 && lineSensor->Get() > 1.58)
  {
    std::cout << "Line" << std::endl;
    led->Set(1);
  }
  else if (lineSensor->Get() <= 1.58)
  {
    std::cout << "Probably Line" << std::endl;
    led->Set(1);
  }
  else
  {
    std::cout << "No Line" << std::endl;
    led->Set(0);
  }*/
  //if(state++>9) state=0;
  //led10->Set(state>4?1:0);
  //std::cout << "Line Sensor = " << lineSensor->Get() << std::endl;

  
/*  Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
      &RangingMeasurementData);
  if(RangingMeasurementData.RangeMilliMeter < 8000)
    printf("Measured distance: %i\n", RangingMeasurementData.RangeMilliMeter);
*/
//    Status = rangingTest(pMyDevice);
/*    if(Status == VL53L0X_ERROR_NONE)
    {
      std::cout << "Distance = " << Status << std::endl;
    }
*/
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
