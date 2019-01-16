# C++ Instructions

## Install
Online and offline installations will set up the ADIS16470 library for use in any C++ robot project, but the online installation will automatically update the software if a new version is released. See the bullets below for instructions on setting the driver up in your environment. Both installation methods assume you've installed FRC Visual Studio Code. 

### Online Install
- Open FRC Visual Studio Code
- Click the WPILib command pallete icon
- Select "Manage Vendor Libraries" in the menu
- Choose "Install New Libraries (Online)"
- Paste the following link: [http://maven.highcurrent.io/vendordeps/ADIS16470.json](http://maven.highcurrent.io/vendordeps/ADIS16470.json)

### Offline Install
- Download the latest release zip from the [releases](https://github.com/juchong/ADIS16470-RoboRIO-Driver/releases) page on Github. The zip will be named `adis16470_roborio-[releaseversion].zip`.
- Close all instances of FRC Visual Studio Code
- If using Windows, extract the zip you downloaded to `C:\Users\Public\frc2019`. If using Linux or Mac, extract the zip to `~/home/frc2019/`.
- Open FRC Visual Studio Code
- Click the WPILib command pallete icon
- Select "Manage Vendor Libraries" in the menu
- Choose "Install New Libraries (Offline)"
- Check ADIS16470, then click "OK".

## Usage
Make a new instance of the driver, and use it however you like. For example:

```cpp
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Modified by Juan Chong - juan.chong@analog.com                             */
/*----------------------------------------------------------------------------*/

#include <cmath>

#include <frc/Joystick.h>
#include <frc/Spark.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ADIS16470_IMU.h>

/**
 * This is a sample program to demonstrate how to use the ADIS16470 IMU sensor 
 * to make a robot drive straight. This program uses a joystick to drive forwards 
 * and backwards while the gyro is used for direction keeping.
 */
class Robot : public frc::TimedRobot {
 public:
  
  void RobotInit() override {

  }

  /**
   * The motor speed is set from the joystick while the DifferentialDrive
   * turning value is assigned from the error between the setpoint and the gyro
   * angle.
   */
  void TeleopPeriodic() override {

    double turningValue = (kAngleSetpoint - m_imu.GetAngleZ()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = std::copysign(turningValue, m_joystick.GetY());
    m_robotDrive.ArcadeDrive(m_joystick.GetY(), turningValue);

  }

 private:
  static constexpr double kAngleSetpoint = 0.0;
  static constexpr double kP = 0.005;  // Proportional turning constant

  static constexpr int kLeftMotorPort = 0;
  static constexpr int kRightMotorPort = 1;
  static constexpr int kJoystickPort = 0;

  frc::Spark m_left{kLeftMotorPort};
  frc::Spark m_right{kRightMotorPort};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  frc::Joystick m_joystick{kJoystickPort};
  frc::ADIS16470_IMU m_imu{};
  // Note: Either configuration is valid.
  //frc::ADIS16470_IMU m_imu{frc::ADIS16470_IMU::kZ, SPI::Port::kOnboardCS0};
};

int main() { return frc::StartRobot<Robot>(); }

```

