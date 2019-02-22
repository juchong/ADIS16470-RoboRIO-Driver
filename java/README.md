# Java Instructions

## Install
Online and offline installations will set up the ADIS16470 library for use in any Java robot project, but the online installation will automatically update the software if a new version is released. See the bullets below for instructions on setting the driver up in your environment. Both installation methods assume you've installed FRC Visual Studio Code. 

### Online Install
- Open FRC Visual Studio Code
- Click the WPILib command pallete icon
- Select "Manage Vendor Libraries" in the menu
- Choose "Install New Libraries (Online)"
- Paste the following link: [http://maven.highcurrent.io/vendordeps/ADIS16470.json](http://maven.highcurrent.io/vendordeps/ADIS16470.json)

### Offline Install
- Download the latest release zip from the [releases](https://github.com/juchong/ADIS16470-RoboRIO-Driver/releases) page in this GitHub repository. The zip will be named `adis16470_roborio-[releaseversion].zip`.
- Close all instances of FRC Visual Studio Code
- If using Windows, extract the zip you downloaded to `C:\Users\Public\frc2019`. If using Linux or Mac, extract the zip to `~/home/frc2019/`.
- Open FRC Visual Studio Code
- Click the WPILib command pallete icon
- Select "Manage Vendor Libraries" in the menu
- Choose "Install New Libraries (Offline)"
- Check ADIS16470, then click "OK".

## Usage
Make a new instance of the driver, and use it however you like. For example:
```java
import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
  public static final ADIS16470_IMU imu = new ADIS16470_IMU();
  
  @Override
  public void robotPeriodic() { 
    SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
    SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
    SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
    
    SmartDashboard.putNumber("Accel-X", imu.getAccelX());
    SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
    SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
    
    SmartDashboard.putNumber("Pitch", imu.getPitch());
    SmartDashboard.putNumber("Roll", imu.getRoll());
    SmartDashboard.putNumber("Yaw", imu.getYaw());
    
    SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
    SmartDashboard.putNumber("Temperature: ", imu.getTemperature()); 
  }
}
```
