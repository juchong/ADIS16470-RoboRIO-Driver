/*----------------------------------------------------------------------------*/
/* ADIS16470 RoboRIO Driver (c) by Juan Chong
/*
/* The ADIS16470 RoboRIO Driver is licensed under a
/* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
/*
/* You should have received a copy of the license along with this
/* work. If not, see <http://creativecommons.org/licenses/by-nc-sa/4.0/>.                             */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <frc/DigitalOutput.h>
#include <frc/DigitalSource.h>
#include <frc/DigitalInput.h>
#include <frc/interfaces/Gyro.h>
#include <frc/SPI.h>
#include <wpi/sendable/SendableHelper.h>
#include <networktables/NTSendable.h>
#include <wpi/mutex.h>
#include <wpi/condition_variable.h>

namespace frc {

/* ADIS16470 Calibration Time Enum Class */
enum class ADIS16470CalibrationTime { 
  _32ms = 0,
  _64ms = 1,
  _128ms = 2,
  _256ms = 3,
  _512ms = 4,
  _1s = 5,
  _2s = 6,
  _4s = 7,
  _8s = 8,
  _16s = 9,
  _32s = 10,
  _64s = 11
};

/* ADIS16470 Register Map Declaration */
static constexpr uint8_t FLASH_CNT      =   0x00;  //Flash memory write count
static constexpr uint8_t DIAG_STAT      =   0x02;  //Diagnostic and operational status
static constexpr uint8_t X_GYRO_LOW     =   0x04;  //X-axis gyroscope output, lower word
static constexpr uint8_t X_GYRO_OUT     =   0x06;  //X-axis gyroscope output, upper word
static constexpr uint8_t Y_GYRO_LOW     =   0x08;  //Y-axis gyroscope output, lower word
static constexpr uint8_t Y_GYRO_OUT     =  	0x0A;  //Y-axis gyroscope output, upper word
static constexpr uint8_t Z_GYRO_LOW     = 	0x0C;  //Z-axis gyroscope output, lower word
static constexpr uint8_t Z_GYRO_OUT     =   0x0E;  //Z-axis gyroscope output, upper word
static constexpr uint8_t X_ACCL_LOW     =   0x10;  //X-axis accelerometer output, lower word
static constexpr uint8_t X_ACCL_OUT     =   0x12;  //X-axis accelerometer output, upper word
static constexpr uint8_t Y_ACCL_LOW     =   0x14;  //Y-axis accelerometer output, lower word
static constexpr uint8_t Y_ACCL_OUT     =   0x16;  //Y-axis accelerometer output, upper word
static constexpr uint8_t Z_ACCL_LOW     =   0x18;  //Z-axis accelerometer output, lower word
static constexpr uint8_t Z_ACCL_OUT     =   0x1A;  //Z-axis accelerometer output, upper word
static constexpr uint8_t TEMP_OUT       =   0x1C;  //Temperature output (internal, not calibrated)
static constexpr uint8_t TIME_STAMP     =   0x1E;  //PPS mode time stamp
static constexpr uint8_t X_DELTANG_LOW  =   0x24;  //X-axis delta angle output, lower word
static constexpr uint8_t X_DELTANG_OUT  =   0x26;  //X-axis delta angle output, upper word
static constexpr uint8_t Y_DELTANG_LOW  =   0x28;  //Y-axis delta angle output, lower word
static constexpr uint8_t Y_DELTANG_OUT  =   0x2A;  //Y-axis delta angle output, upper word
static constexpr uint8_t Z_DELTANG_LOW  =   0x2C;  //Z-axis delta angle output, lower word
static constexpr uint8_t Z_DELTANG_OUT  =   0x2E;  //Z-axis delta angle output, upper word
static constexpr uint8_t X_DELTVEL_LOW  =   0x30;  //X-axis delta velocity output, lower word
static constexpr uint8_t X_DELTVEL_OUT  =   0x32;  //X-axis delta velocity output, upper word
static constexpr uint8_t Y_DELTVEL_LOW  =   0x34;  //Y-axis delta velocity output, lower word
static constexpr uint8_t Y_DELTVEL_OUT  =   0x36;  //Y-axis delta velocity output, upper word
static constexpr uint8_t Z_DELTVEL_LOW  =   0x38;  //Z-axis delta velocity output, lower word
static constexpr uint8_t Z_DELTVEL_OUT  =   0x3A;  //Z-axis delta velocity output, upper word
static constexpr uint8_t XG_BIAS_LOW    =   0x40;  //X-axis gyroscope bias offset correction, lower word
static constexpr uint8_t XG_BIAS_HIGH   =   0x42;  //X-axis gyroscope bias offset correction, upper word
static constexpr uint8_t YG_BIAS_LOW    =   0x44;  //Y-axis gyroscope bias offset correction, lower word
static constexpr uint8_t YG_BIAS_HIGH 	=   0x46;  //Y-axis gyroscope bias offset correction, upper word
static constexpr uint8_t ZG_BIAS_LOW    =   0x48;  //Z-axis gyroscope bias offset correction, lower word
static constexpr uint8_t ZG_BIAS_HIGH   =   0x4A;  //Z-axis gyroscope bias offset correction, upper word
static constexpr uint8_t XA_BIAS_LOW    =   0x4C;  //X-axis accelerometer bias offset correction, lower word
static constexpr uint8_t XA_BIAS_HIGH   =   0x4E;  //X-axis accelerometer bias offset correction, upper word
static constexpr uint8_t YA_BIAS_LOW    =   0x50;  //Y-axis accelerometer bias offset correction, lower word
static constexpr uint8_t YA_BIAS_HIGH   =   0x52;  //Y-axis accelerometer bias offset correction, upper word
static constexpr uint8_t ZA_BIAS_LOW    =   0x54;  //Z-axis accelerometer bias offset correction, lower word
static constexpr uint8_t ZA_BIAS_HIGH   =   0x56;  //Z-axis accelerometer bias offset correction, upper word
static constexpr uint8_t FILT_CTRL      =   0x5C;  //Filter control
static constexpr uint8_t MSC_CTRL       =   0x60;  //Miscellaneous control
static constexpr uint8_t UP_SCALE       =   0x62;  //Clock scale factor, PPS mode
static constexpr uint8_t DEC_RATE       =   0x64;  //Decimation rate control (output data rate)
static constexpr uint8_t NULL_CNFG      =   0x66;  //Auto-null configuration control
static constexpr uint8_t GLOB_CMD       =   0x68;  //Global commands
static constexpr uint8_t FIRM_REV       =   0x6C;  //Firmware revision
static constexpr uint8_t FIRM_DM        =   0x6E;  //Firmware revision date, month and day
static constexpr uint8_t FIRM_Y         =   0x70;  //Firmware revision date, year
static constexpr uint8_t PROD_ID        =   0x72;  //Product identification 
static constexpr uint8_t SERIAL_NUM     =   0x74;  //Serial number (relative to assembly lot)
static constexpr uint8_t USER_SCR1      =   0x76;  //User scratch register 1 
static constexpr uint8_t USER_SCR2      =   0x78;  //User scratch register 2 
static constexpr uint8_t USER_SCR3      =   0x7A;  //User scratch register 3 
static constexpr uint8_t FLSHCNT_LOW    =   0x7C;  //Flash update count, lower word 
static constexpr uint8_t FLSHCNT_HIGH   =   0x7E;  //Flash update count, upper word 

/* ADIS16470 Auto SPI Data Packets */
static constexpr uint8_t m_autospi_x_packet [16] = {
X_DELTANG_OUT, 
FLASH_CNT, 
X_DELTANG_LOW, 
FLASH_CNT, 
X_GYRO_OUT,
FLASH_CNT, 
Y_GYRO_OUT, 
FLASH_CNT, 
Z_GYRO_OUT, 
FLASH_CNT, 
X_ACCL_OUT, 
FLASH_CNT, 
Y_ACCL_OUT, 
FLASH_CNT,
Z_ACCL_OUT,
FLASH_CNT
};

static constexpr uint8_t m_autospi_y_packet [16] = {
Y_DELTANG_OUT, 
FLASH_CNT, 
Y_DELTANG_LOW, 
FLASH_CNT, 
X_GYRO_OUT,
FLASH_CNT, 
Y_GYRO_OUT, 
FLASH_CNT, 
Z_GYRO_OUT, 
FLASH_CNT, 
X_ACCL_OUT, 
FLASH_CNT, 
Y_ACCL_OUT, 
FLASH_CNT,
Z_ACCL_OUT,
FLASH_CNT
};

static constexpr uint8_t m_autospi_z_packet [16] = {
Z_DELTANG_OUT, 
FLASH_CNT, 
Z_DELTANG_LOW, 
FLASH_CNT, 
X_GYRO_OUT,
FLASH_CNT, 
Y_GYRO_OUT, 
FLASH_CNT, 
Z_GYRO_OUT, 
FLASH_CNT, 
X_ACCL_OUT, 
FLASH_CNT, 
Y_ACCL_OUT, 
FLASH_CNT,
Z_ACCL_OUT,
FLASH_CNT
};

/* ADIS16470 Constants */
const double delta_angle_sf = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
const double rad_to_deg = 57.2957795;
const double deg_to_rad = 0.0174532;
const double grav = 9.81;

/**
 * Use DMA SPI to read rate and acceleration data from the ADIS16470 IMU and return the
 * robot's heading relative to a starting position and instant measurements
 *
 * The ADIS16470 gyro angle outputs track the robot's heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of rotation returned by 
 * the IMU. When the class is instantiated, a short calibration routine is performed where the 
 * IMU samples the gyros while at rest to determine the initial offset. This is subtracted from 
 * each sample to determine the heading.
 *
 * This class is for the ADIS16470 IMU connected via the primary SPI port available on the RoboRIO.
 */

class ADIS16470_IMU : public Gyro,
                 public nt::NTSendable,
                 public wpi::SendableHelper<ADIS16470_IMU> {
 public:

 enum IMUAxis { kX, kY, kZ };

  /**
  * @brief Default constructor. Uses CS0 on the 10-pin SPI port, the yaw axis is set to the IMU Z axis,
  * and calibration time is defaulted to 1 second.
  */
  ADIS16470_IMU();

  /**
   * @brief Customizable constructor. Allows the SPI port and CS to be customized, the yaw axis used for GetAngle()
   * is adjustable, and initial calibration time can be modified.
   * 
   * @param yaw_axis Selects the "default" axis to use for GetAngle() and GetRate()
   * 
   * @param port The SPI port and CS where the IMU is connected.
   * 
   * @param cal_time The calibration time that should be used on start-up.
   * 
   * @param cal_on_start Calibrate the sensor every time robot code starts. 
   */
  explicit ADIS16470_IMU(IMUAxis yaw_axis, SPI::Port port, ADIS16470CalibrationTime cal_time, bool cal_on_start);

  /**
   * @brief Destructor. Kills the acquisiton loop and closes the SPI peripheral.
   */
  ~ADIS16470_IMU();

  ADIS16470_IMU(ADIS16470_IMU&&) = default;
  ADIS16470_IMU& operator=(ADIS16470_IMU&&) = default;

  int ConfigDecRate(uint16_t reg);

  /**
   * @brief Switches the active SPI port to standard SPI mode, writes the command to activate the new null configuration, and re-enables auto SPI.
   */
  void Calibrate() override;

  /**
   * @brief Switches the active SPI port to standard SPI mode, writes a new value to the NULL_CNFG register in the IMU, and re-enables auto SPI.
   */
  int ConfigCalTime(ADIS16470CalibrationTime new_cal_time);

  /**
   * @brief Resets (zeros) the xgyro, ygyro, and zgyro angle integrations. 
   *
   * Resets the gyro accumulations to a heading of zero. This can be used if 
   * the "zero" orientation of the sensor needs to be changed in runtime.
   */
  void Reset() override;

  /**
  * @brief Writes the current IMU configuration (including the initial calibration) to flash.
  * 
  * @return A boolean indicating the success or failure of writing the current settings to the IMU flash.
  */
  int WriteIMUSettingsToFlash();

  /**
  * @brief Reads the contents of a specified register location over SPI. 
  *
  * @param reg An unsigned, 8-bit register location.
  * 
  * @return An unsigned, 16-bit number representing the contents of the requested register location.
  */
  uint16_t ReadRegister(uint8_t reg);

  /**
  * @brief Writes an unsigned, 16-bit value to two adjacent, 8-bit register locations over SPI.
  *
  * @param reg An unsigned, 8-bit register location.
  * 
  * @param val An unsigned, 16-bit value to be written to the specified register location.
  */
  void WriteRegister(uint8_t reg, uint16_t val);

  /**
   * @brief Returns the current integrated angle for the axis specified. 
   *
   * The angle is based on the current accumulator value corrected by
   * offset calibration and built-in IMU calibration. The angle is continuous, 
   * that is it will continue from 360->361 degrees. This allows algorithms 
   * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
   * from 360 to 0 on the second time around. The axis returned by this 
   * function is adjusted based on the configured yaw_axis.
   *
   * @return the current heading of the robot in degrees. This heading is based
   *         on integration of the returned rate from the gyro.
   */
  double GetAngle() const override;

  double GetRate() const;

  double GetGyroInstantX() const;

  double GetGyroInstantY() const;

  double GetGyroInstantZ() const;

  double GetAccelInstantX() const;

  double GetAccelInstantY() const;

  double GetAccelInstantZ() const;
  
  double GetXComplementaryAngle() const;

  double GetYComplementaryAngle() const;

  double GetXFilteredAccelAngle() const;

  double GetYFilteredAccelAngle() const;

  IMUAxis GetYawAxis() const;

  int SetYawAxis(IMUAxis yaw_axis);

  // IMU yaw axis
  IMUAxis m_yaw_axis;

  void InitSendable(nt::NTSendableBuilder& builder) override;

 private:

  /**
  * @brief Switches to standard SPI operation. Primarily used when exiting auto SPI mode.
  *
  * @return A boolean indicating the success or failure of setting up the SPI peripheral in standard SPI mode.
  */
  bool SwitchToStandardSPI();

  /**
  * @brief Switches to auto SPI operation. Primarily used when exiting standard SPI mode.
  * 
  * @return A boolean indicating the success or failure of setting up the SPI peripheral in auto SPI mode.
  */
  bool SwitchToAutoSPI();

  /**
  * @brief Main acquisition loop. Typically called asynchronously and free-wheels while the robot code is active. 
  */
  void Acquire();

  void Close();

  // Integrated gyro value
  double m_integ_angle = 0.0;

  // Instant raw outputs
  double m_gyro_x, m_gyro_y, m_gyro_z, m_accel_x, m_accel_y, m_accel_z = 0.0;

  // Complementary filter variables
  double m_tau = 1.0;
  double m_dt, m_alpha = 0.0;
  double m_compAngleX, m_compAngleY, m_accelAngleX, m_accelAngleY = 0.0;

  //Complementary filter functions
  double FormatFastConverge(double compAngle, double accAngle);

  double FormatRange0to2PI(double compAngle);

  double FormatAccelRange(double accelAngle, double accelZ);

  double CompFilterProcess(double compAngle, double accelAngle, double omega);

  // State and resource variables
  volatile bool m_thread_active = false;
  volatile bool m_first_run = true;
  volatile bool m_thread_idle = false;
  volatile bool m_needs_flash = false;
  volatile bool m_calibrate_every_start = true;
  volatile bool m_needs_flash = false;
  bool m_auto_configured = false;
  SPI::Port m_spi_port;
  uint16_t m_calibration_time;
  SPI *m_spi = nullptr;
  DigitalInput *m_auto_interrupt = nullptr;
  double m_scaled_sample_rate = 2500.0; // Default sample rate setting
  
  std::thread m_acquire_task;

  mutable wpi::mutex m_mutex;

};

} //namespace frc