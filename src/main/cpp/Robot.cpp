/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>

#include "Robot.h"
#include "frc/SerialPort.h"
#include "frc/AnalogPotentiometer.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalInput.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "frc/shuffleboard/ShuffleboardTab.h"
//#include "frc/smartdashboard/SendableChooser.h"
#include "frc/Servo.h"
#include "frc/smartdashboard/SendableBuilder.h"
#include "C:\FRC2019\Timed\Drake-Integration\build\tmp\expandedArchives\ntcore-cpp-2019.1.1-headers.zip_de8bb156e7c972a7498927ac56d827f7\networktables\NetworkTableEntry.h"

#include "Helpers.h"
#include "Helper_VL53L0X.h"

frc::AnalogPotentiometer *pot;
frc::DigitalInput *lineSensor;
//frc::DigitalOutput *led;
//frc::DigitalOutput *led10;
#define LIDAR_COUNT 6
Sensor_VL53L0X *mLidar[LIDAR_COUNT];
frc::DigitalOutput *lidarSHDN[LIDAR_COUNT];
frc::Shuffleboard *shuffleBoard;
//frc::ShuffleboardTab *shuffleboardTab;
//frc::SendableChooser *sendableChooser;
frc::Servo *servo0;
nt::NetworkTableEntry *servoNTE;

void Robot::RobotInit() {
  //sendableChooser->InitSendable();
  //sendableChooser->AddOption("Pizza", "meow");
  
  BlockingTimer Timer;

  

  printf("\nRobo Init\n\n");
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  lineSensor = new frc::DigitalInput(8);
  servo0 = new frc::Servo(0);
  pot = new frc::AnalogPotentiometer(0, servo0->GetMaxAngle() - servo0->GetMinAngle(), servo0->GetMinAngle());
  servoNTE = new nt::NetworkTableEntry();
  //shuffleboardTab = new frc::ShuffleboardTab();

  //shuffleboardTab = shuffleBoard->GetTab("Servo");
  frc::ShuffleboardTab shuffleboardTab = frc::Shuffleboard::GetTab("Servo");
  //servoNTE = shuffleboardTab->Add("Servo Angle", 1);

  // Crete the GPIO objects
  for(int i = 0; i < LIDAR_COUNT; i++) {
    lidarSHDN[i] = new frc::DigitalOutput(i);
  }
  // Set them all LOW
  for(int i = 0; i < LIDAR_COUNT; i++) {
    lidarSHDN[i]->Set(0);
  }

  // Chill
  Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

  // Bring all HIGH
  for(int i = 0; i < LIDAR_COUNT; i++) {
    lidarSHDN[i]->Set(1);
  }

  // Chill
  Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
  // Put all LOW again
  for (int i = 0; i < LIDAR_COUNT; i++) {
    lidarSHDN[i]->Set(0);
  }

  // Chill
  Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
  // Now initialize one by one
  for (int i = 0; i < LIDAR_COUNT; i++) {
    std::cout << "Initializing MicroLidar: " << i << " ... ";
    lidarSHDN[i]->Set(1);
    mLidar[i] = new Sensor_VL53L0X;
    if(!mLidar[i]->Init(0x2A+i))
    {
      std::cout << "FAILED!" << std::endl;
    } else {
      std::cout << "OK!" << std::endl;
    }
  }


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
  //servoNTE = shuffleBoard->add("Servo", 1);

  servo0->SetAngle(pot->Get());
  //frc::SendableBuilder::GetEntry("Servo Angle");
  frc::SmartDashboard::PutNumber("Servo 0 Angle:", servo0->GetAngle());
  frc::SmartDashboard::PutNumber("Potentiometer Angle:", pot->Get());

  //servoNTE->GetValue();

  frc::SmartDashboard::PutNumber("Lidar Sensor: 0", mLidar[0]->MeasureMM());
  frc::SmartDashboard::PutNumber("Lidar Sensor: 1", mLidar[1]->MeasureMM());
  frc::SmartDashboard::PutNumber("Lidar Sensor: 2", mLidar[2]->MeasureMM());
  frc::SmartDashboard::PutNumber("Lidar Sensor: 3", mLidar[3]->MeasureMM());
  frc::SmartDashboard::PutNumber("Lidar Sensor: 4", mLidar[4]->MeasureMM());
  frc::SmartDashboard::PutNumber("Lidar Sensor: 5", mLidar[5]->MeasureMM());

  frc::SmartDashboard::PutBoolean("Line Sensor:", !(lineSensor->Get()));
//  std::cout << "Measurement= " ;
//  for (int i = 0; i < LIDAR_COUNT; i++) 
//  {
//    std::cout << "Measurement " << i << " " << mLidar[i]->MeasureMM() << std::endl; 
//    printf(" %4d ", mLidar[i]->MeasureMM()); 
//  }
//  std::cout << std::endl;

  //printf(" Measure= %4d\n", mLidar[3]->MeasureMM());

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
  // m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  //std::cout << "Auto selected: " << m_autoSelected << std::endl;

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
