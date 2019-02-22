/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Modified by Juan Chong - juan.chong@analog.com                             */
/*----------------------------------------------------------------------------*/

#include <string>
#include <iostream>
#include <cmath>

#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include <frc/DriverStation.h>
#include <frc/ErrorBase.h>
#include <adi/ADIS16470_IMU.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/Timer.h>
#include <frc/WPIErrors.h>
#include <hal/HAL.h>

static constexpr double kCalibrationSampleTime = 5.0; // Calibration time in seconds (regardless of sps)
static constexpr double kDegreePerSecondPerLSB = 1.0/10.0;
static constexpr double kGPerLSB = 1.0/800.0;
static constexpr double kDegCPerLSB = 1.0/10.0;
static constexpr double kDegCOffset = 0.0;

static constexpr uint8_t kGLOB_CMD = 0x68;
static constexpr uint8_t kRegDEC_RATE = 0x64;
static constexpr uint8_t kRegFILT_CTRL = 0x5C;
static constexpr uint8_t kRegMSC_CTRL = 0x60;
static constexpr uint8_t kRegPROD_ID = 0x72;

double timestamp_old = 0;

static inline uint16_t ToUShort(const uint8_t* buf) {
  return ((uint16_t)(buf[0]) << 8) | buf[1];
}

static inline int16_t ToShort(const uint8_t* buf) {
  return (int16_t)(((uint16_t)(buf[0]) << 8) | buf[1]);
}

using namespace frc;

/**
 * Constructor.
 */
ADIS16470_IMU::ADIS16470_IMU() : ADIS16470_IMU(kZ, SPI::Port::kOnboardCS0) {}

ADIS16470_IMU::ADIS16470_IMU(IMUAxis yaw_axis, SPI::Port port) : m_yaw_axis(yaw_axis), m_spi(port) {

  // Set general SPI settings
  m_spi.SetClockRate(1000000);
  m_spi.SetMSBFirst();
  m_spi.SetSampleDataOnTrailingEdge();
  m_spi.SetClockActiveLow();
  m_spi.SetChipSelectActiveLow();

  ReadRegister(kRegPROD_ID); // Dummy read

  // Validate the product ID
  if (ReadRegister(kRegPROD_ID) != 16982) {
    DriverStation::ReportError("Could not find ADIS16470!");
    return;
  }
  else {
    DriverStation::ReportError("ADIS16470 IMU Detected. Starting calibration.");
  }

  // Set IMU internal decimation to 200 SPS
  WriteRegister(kRegDEC_RATE, 0x0009);

  // Enable Data Ready (LOW = Good Data), gSense Compensation, PoP
  WriteRegister(kRegMSC_CTRL, 0x00C1);

  // Configure IMU internal Bartlett filter
  WriteRegister(kRegFILT_CTRL, 0x0000);

  // Configure interrupt on SPI CS1
  DigitalInput *m_interrupt = new DigitalInput(26);

  // Configure DMA SPI
  m_spi.InitAuto(8200);
  m_spi.SetAutoTransmitData(kGLOB_CMD,21);

  // Kick off DMA SPI (Note: Device configration impossible after SPI DMA is activated)
  m_spi.StartAutoTrigger(*m_interrupt,true,false);

  // Start acquisition and calculation threads
  m_freed = false;
  m_acquire_task = std::thread(&ADIS16470_IMU::Acquire, this);

  // Execute offset calibration on start-up
  Calibrate();

  Wait(0.500); 

  // Re-initialize accumulations after calibration
  Reset();
  
  // Let the user know the IMU was initiallized successfully
  DriverStation::ReportWarning("ADIS16470 IMU Successfully Initialized!");

  // Report usage and post data to DS
  HAL_Report(HALUsageReporting::kResourceType_ADIS16470, 0);
  SetName("ADIS16470", 0);
}

void ADIS16470_IMU::Calibrate() {
  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_accum_count = 0;
    m_accum_gyro_x = 0.0;
    m_accum_gyro_y = 0.0;
    m_accum_gyro_z = 0.0;
  }

  Wait(kCalibrationSampleTime);
  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_gyro_offset_x = m_accum_gyro_x / m_accum_count;
    m_gyro_offset_y = m_accum_gyro_y / m_accum_count;
    m_gyro_offset_z = m_accum_gyro_z / m_accum_count;
  }
}

uint16_t ADIS16470_IMU::ReadRegister(uint8_t reg) {
  uint8_t buf[2];
  buf[0] = reg & 0x7f;
  buf[1] = 0;

  m_spi.Write(buf, 2);
  m_spi.Read(false, buf, 2);

  return ToUShort(buf);
}

void ADIS16470_IMU::WriteRegister(uint8_t reg, uint16_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  buf[1] = val & 0xff;
  m_spi.Write(buf, 2);
  buf[0] = 0x81 | reg;
  buf[1] = val >> 8;
  m_spi.Write(buf, 2);
}

void ADIS16470_IMU::Reset() {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_integ_gyro_x = 0.0;
  m_integ_gyro_y = 0.0;
  m_integ_gyro_z = 0.0;
}

ADIS16470_IMU::~ADIS16470_IMU() {
  m_spi.StopAuto();
  m_freed = true;
  m_samples_not_empty.notify_all();
  if (m_acquire_task.joinable()) m_acquire_task.join();
}

void ADIS16470_IMU::Acquire() {
  uint32_t buffer[2000];
  double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp, status, counter;
  int data_count = 0;
  int data_remainder = 0;
  int data_to_read = 0;
  uint8_t data_subset[24];
  uint32_t timestamp_new = 0;

  while (!m_freed) {

	  // Waiting for the buffer to fill...
	  Wait(.020); // A delay greater than 50ms could potentially overflow the local buffer (depends on sensor sample rate)
    
    std::fill_n(buffer, 2000, 0);  // Clear buffer
	  data_count = m_spi.ReadAutoReceivedData(buffer,0,0); // Read number of bytes currently stored in the buffer
	  data_remainder = data_count % 23; // Check if frame is incomplete
    data_to_read = data_count - data_remainder;  // Remove incomplete data from read count
	  m_spi.ReadAutoReceivedData(buffer,data_to_read,0); // Read data from DMA buffer

    /* // DEBUG: Print buffer size and contents to terminal
    std::cout << "Start - " << data_count << "," << data_remainder << "," << data_to_read << std::endl;
    for (int m = 0; m < data_to_read - 1; m++ )
    {
      std::cout << buffer[m] << ",";
    }
    std::cout << " " << std::endl;
    std::cout << "End" << std::endl; */

	  for (int i = 0; i < data_to_read; i += 23) { // Process each set of 22 bytes + timestamp (23 total)

		  for (int j = 1; j < 23; j++) { 
			  data_subset[j - 1] = buffer[i + j];  // Split each set of 22 bytes into a sub-array for processing
      }

		  // DEBUG: Plot sub-array data in terminal
 		  /*std::cout << ToUShort(&data_subset[0]) << "," << ToUShort(&data_subset[2]) << "," << ToUShort(&data_subset[4]) <<
		  "," << ToUShort(&data_subset[6]) << "," << ToUShort(&data_subset[8]) << "," << ToUShort(&data_subset[10]) << "," <<
		  ToUShort(&data_subset[12]) << "," << ToUShort(&data_subset[14]) << "," << ToUShort(&data_subset[16]) << "," <<
		  ToUShort(&data_subset[18]) << "," << ToUShort(&data_subset[20]) << std::endl; */

      // Calculate checksum on each data packet
      uint16_t imu_checksum = 0;
      uint16_t calc_checksum = 0;
		  for(int k = 2; k < 20; k++ ) { // Cycle through STATUS, XYZ GYRO, XYZ ACCEL, TEMP, COUNTER (Ignore DUT Checksum)
			  calc_checksum += data_subset[k];
		  }

		  imu_checksum = ToUShort(&data_subset[20]); // Extract DUT checksum from data buffer

      // Compare calculated vs read CRC. Don't update outputs or dt if CRC-16 is bad
      if (calc_checksum == imu_checksum) {
        
        // Calculate delta-time (dt) using FPGA timestamps
        timestamp_new = buffer[i];  // Extract timestamp from buffer
        dt = (timestamp_new - timestamp_old)/1000000; // Calculate dt and convert us to seconds
        timestamp_old = timestamp_new; // Store new timestamp in old variable for next cycle

        gyro_x = ToShort(&data_subset[4]) * kDegreePerSecondPerLSB;
        gyro_y = ToShort(&data_subset[6]) * kDegreePerSecondPerLSB;
        gyro_z = ToShort(&data_subset[8]) * kDegreePerSecondPerLSB;
        accel_x = ToShort(&data_subset[10]) * kGPerLSB;
        accel_y = ToShort(&data_subset[12]) * kGPerLSB;
        accel_z = ToShort(&data_subset[14]) * kGPerLSB;
        temp = ToShort(&data_subset[16]) * kDegCPerLSB + kDegCOffset;
        status = ToUShort(&data_subset[2]);
        counter = ToUShort(&data_subset[18]);

        // Update global state
        {
          std::lock_guard<wpi::mutex> sync(m_mutex);
          m_gyro_x = gyro_x;
          m_gyro_y = gyro_y;
          m_gyro_z = gyro_z;
          m_accel_x = accel_x;
          m_accel_y = accel_y;
          m_accel_z = accel_z;
          m_temp = temp;
          m_status = status;
          m_counter = counter;

          // Accumulate gyro for offset calibration
          ++m_accum_count;
          m_accum_gyro_x += gyro_x;
          m_accum_gyro_y += gyro_y;
          m_accum_gyro_z += gyro_z;
          
          // Accumulate gyro for angle integration
          m_integ_gyro_x += (gyro_x - m_gyro_offset_x) * dt;
          m_integ_gyro_y += (gyro_y - m_gyro_offset_y) * dt;
          m_integ_gyro_z += (gyro_z - m_gyro_offset_z) * dt;
        }
      }
      else {
        // Print notification when checksum fails and bad data is rejected
        std::cout << "IMU Data Checksum Invalid - " << calc_checksum << "," << imu_checksum << " - Data Ignored and Integration Time Adjusted" << std::endl;
      }
	  }
  }
}

double ADIS16470_IMU::GetAngle() const {
  switch (m_yaw_axis) {
    case kX: 
      return GetAngleX();
    case kY: 
      return GetAngleY();
    case kZ: 
      return GetAngleZ();
  }
}

double ADIS16470_IMU::GetRate() const {
  switch (m_yaw_axis) {
    case kX: 
      return GetRateX();
    case kY: 
      return GetRateY();
    case kZ: 
      return GetRateZ();
  }
}

double ADIS16470_IMU::GetAngleX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_x;
}

double ADIS16470_IMU::GetAngleY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_y;
}

double ADIS16470_IMU::GetAngleZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_z;
}

double ADIS16470_IMU::GetRateX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_x;
}

double ADIS16470_IMU::GetRateY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_y;
}

double ADIS16470_IMU::GetRateZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_z;
}

double ADIS16470_IMU::GetAccelX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_x;
}

double ADIS16470_IMU::GetAccelY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_y;
}

double ADIS16470_IMU::GetAccelZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_z;
}

double ADIS16470_IMU::Getdt() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return dt;
}

double ADIS16470_IMU::GetTemperature() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_temp;
}

double ADIS16470_IMU::GetStatus() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_status;
}

double ADIS16470_IMU::GetCounter() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_counter;
}

void ADIS16470_IMU::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("ADIS16470 IMU");
  auto gyroX = builder.GetEntry("GyroX").GetHandle();
  auto gyroY = builder.GetEntry("GyroY").GetHandle();
  auto gyroZ = builder.GetEntry("GyroZ").GetHandle();
  auto accelX = builder.GetEntry("AccelX").GetHandle();
  auto accelY = builder.GetEntry("AccelY").GetHandle();
  auto accelZ = builder.GetEntry("AccelZ").GetHandle();
  auto angleX = builder.GetEntry("AngleX").GetHandle();
  auto angleY = builder.GetEntry("AngleY").GetHandle();
  auto angleZ = builder.GetEntry("AngleZ").GetHandle();
  auto temp = builder.GetEntry("Temperature").GetHandle();
  auto dt = builder.GetEntry("dt").GetHandle();
  auto status = builder.GetEntry("Status").GetHandle();
  auto counter = builder.GetEntry("Counter").GetHandle();
  builder.SetUpdateTable([=]() {
    nt::NetworkTableEntry(gyroX).SetDouble(GetRateX());
    nt::NetworkTableEntry(gyroY).SetDouble(GetRateY());
    nt::NetworkTableEntry(gyroZ).SetDouble(GetRateZ());
    nt::NetworkTableEntry(accelX).SetDouble(GetAccelX());
    nt::NetworkTableEntry(accelY).SetDouble(GetAccelY());
    nt::NetworkTableEntry(accelZ).SetDouble(GetAccelZ());
    nt::NetworkTableEntry(angleX).SetDouble(GetAngleX());
    nt::NetworkTableEntry(angleY).SetDouble(GetAngleY());
    nt::NetworkTableEntry(angleZ).SetDouble(GetAngleZ());
    nt::NetworkTableEntry(temp).SetDouble(GetTemperature());
    nt::NetworkTableEntry(dt).SetDouble(Getdt());
    nt::NetworkTableEntry(status).SetDouble(GetStatus());
    nt::NetworkTableEntry(counter).SetDouble(GetCounter());
  });
}
