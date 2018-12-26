# LabVIEW Instructions

## Self-Extracting Installation
This driver assumes you have a working 2018 LabVIEW Professional installation and 2019.b3 or greater FRC software package already installed on your system. The self-extracting installer can be downloaded from the releases page [here](https://github.com/juchong/ADIS16470-RoboRIO-Driver/releases). 

Once downloaded, run the installer with administrator privileges. All the files required to integrate the IMU driver with Labview will be extracted from the installer. LabVIEW driver examples will also be installed to make testing your sensor easy!

## Manual Installation
After cloning or downloading the repository, copy the `ADIS16470 IMU` folder into the folder where your LabVIEW robot project is stored. Once copied, drag the `ADIS16470 IMU` folder into the project explorer. 

## Usage
The LabVIEW driver can be used like any other sensor. If the **installer** was used, all .vi's can be found in the pallet shown below:

![ADIS16470 LabVIEW Pallet](https://raw.githubusercontent.com/juchong/ADIS16470-RoboRIO-Driver/master/Reference/470_pallet.JPG)

If the driver was added to your project **manually**, all user .vi's should be located in your project like this:

![ADIS16470 LabVIEW Project Explorer](https://raw.githubusercontent.com/juchong/ADIS16470-RoboRIO-Driver/master/Reference/RobotProject.JPG)

## Examples
If the **installer** was used, the examples can be accessed by following the steps below:

Navigate to `Help > Find Examples...` in the main window.

![ADIS16470 Main Menu](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/MainMenu.PNG)

In the NI Example Finder, navigate to `FRC Robotics > Sensors > ADIS16470 IMU.lvproj` and double click. 

![ADIS16470 Find Examples](https://raw.githubusercontent.com/juchong/ADIS16470-RoboRIO-Driver/master/Reference/470_ExampleFinder.JPG)

The example project will open. Select `ADIS16470 IMU Example.vi` and run the example. If the sensor was successfully detected, the front panel should look similar to this:

![ADIS16470 Front Panel](https://raw.githubusercontent.com/juchong/ADIS16470-RoboRIO-Driver/master/Reference/470_FrontPanel.JPG)
