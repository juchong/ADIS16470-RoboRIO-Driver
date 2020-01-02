/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Modified by Juan Chong - juan.chong@analog.com                             */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <frc/DigitalOutput.h>
#include <frc/DigitalSource.h>
#include <frc/DigitalInput.h>
#include <frc/GyroBase.h>
#include <frc/SPI.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <wpi/mutex.h>
#include <wpi/condition_variable.h>

namespace frc {

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

class ADIS16470_IMU : public GyroBase {
 public:

 enum IMUAxis { kX, kY, kZ };

  /**
  * IMU constructor on onboard SPI CS0.
  */
  ADIS16470_IMU();

  /**
   * IMU constructor on the specified SPI port.
   * 
   * @param yaw_axis Selects the "default" axis to use for GetAngle() and GetRate()
   * @param port The SPI port where the IMU is connected.
   */
  explicit ADIS16470_IMU(IMUAxis yaw_axis, SPI::Port port, ADIS16470CalibrationTime cal_time);

  ~ADIS16470_IMU();

  ADIS16470_IMU(ADIS16470_IMU&&) = default;
  ADIS16470_IMU& operator=(ADIS16470_IMU&&) = default;

  bool Recalibrate();
  bool Reconfigure(ADIS16470CalibrationTime new_cal_time);

  /**
   * Reset the gyro.
   *
   * Resets the gyro accumulations to a heading of zero. This can be used if 
   * there is significant drift in the gyro and it needs to be recalibrated
   * after running.
   */
  void Reset() override;

  /**
   * Return the actual angle in degrees that the robot is currently facing.
   *
   * The angle is based on the current accumulator value corrected by
   * offset calibration and built-in IMU calibration. The angle is continuous, 
   * that is it will continue from 360->361 degrees. This allows algorithms 
   * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
   * from 360 to 0 on the second time around. The axis returned by this 
   * function is adjusted fased on the configured yaw_axis.
   *
   * @return the current heading of the robot in degrees. This heading is based
   *         on integration of the returned rate from the gyro.
   */
  double GetAngle() const override;

  /**
   * Return the IMU X-axis integrated angle in degrees.
   *
   * The angle is based on the current accumulator value corrected by
   * offset calibration and built-in IMU calibration. The angle is continuous, 
   * that is it will continue from 360->361 degrees. This allows algorithms 
   * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
   * from 360 to 0 on the second time around. 
   *
   * @return the current accumulated value of the X-axis in degrees. 
   */
  double GetAngleX() const;

  /**
   * Return the IMU Y-axis integrated angle in degrees.
   *
   * The angle is based on the current accumulator value corrected by
   * offset calibration and built-in IMU calibration. The angle is continuous, 
   * that is it will continue from 360->361 degrees. This allows algorithms 
   * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
   * from 360 to 0 on the second time around. 
   *
   * @return the current accumulated value of the Y-axis in degrees. 
   */
  double GetAngleY() const;

  /**
   * Return the IMU Z-axis integrated angle in degrees.
   *
   * The angle is based on the current accumulator value corrected by
   * offset calibration and built-in IMU calibration. The angle is continuous, 
   * that is it will continue from 360->361 degrees. This allows algorithms 
   * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
   * from 360 to 0 on the second time around. 
   *
   * @return the current accumulated value of the Z-axis in degrees. 
   */
  double GetAngleZ() const;

  void InitSendable(SendableBuilder& builder) override;

 private:

  bool SwitchToStandardSPI();
  bool SwitchToAutoSPI();

  // IMU yaw axis
  IMUAxis m_yaw_axis;

  uint16_t ReadRegister(uint8_t reg);
  void WriteRegister(uint8_t reg, uint16_t val);
  void Acquire();

  // Integrated gyro values
  double m_integ_gyro_x = 0.0;
  double m_integ_gyro_y = 0.0;
  double m_integ_gyro_z = 0.0;

  std::atomic_bool m_freed;
  SPI::Port m_spi_port;
  uint16_t m_calibration_time;
  SPI *m_spi = nullptr;
  DigitalInput *m_auto_interrupt;
  
  
  std::thread m_acquire_task;

  mutable wpi::mutex m_mutex;

  // Samples FIFO.  We make the FIFO 2 longer than it needs
  // to be so the input and output never overlap (we hold a reference
  // to the output while the lock is released).
  static constexpr int kSamplesDepth = 10;
  Sample m_samples[kSamplesDepth + 2];
  wpi::mutex m_samples_mutex;
  wpi::condition_variable m_samples_not_empty;

};

} //namespace frc