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

static constexpr uint8_t kGLOB_CMD = 0x68;
static constexpr uint8_t kRegDEC_RATE = 0x64;
static constexpr uint8_t kRegFILT_CTRL = 0x5C;
static constexpr uint8_t kRegMSC_CTRL = 0x60;
static constexpr uint8_t kRegNULL_CNFG = 0x66;
static constexpr uint8_t kRegPROD_ID = 0x72;

static inline int32_t ToInt(const uint32_t *buf){
  return (int32_t)( (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3] );
}

using namespace frc;

/**
 * Constructor.
 */
ADIS16470_IMU::ADIS16470_IMU() : ADIS16470_IMU(kZ, SPI::Port::kOnboardCS0, ADIS16470CalibrationTime::_4s) {}

ADIS16470_IMU::ADIS16470_IMU(IMUAxis yaw_axis, SPI::Port port, ADIS16470CalibrationTime cal_time) : 
                m_yaw_axis(yaw_axis), 
                m_spi_port(port),
                m_calibration_time((uint16_t)cal_time) {

  // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
  // Relies on the RIO hardware by default configuring an output as low
  // and configuring an input as high Z. The 10k pull-up resistor internal to the
  // IMU then forces the reset line high for normal operation.
  DigitalOutput *m_reset_out = new DigitalOutput(27);  // Drive SPI CS2 (IMU RST) low
  Wait(0.01);  // Wait 10ms
  delete m_reset_out;
  /*DigitalInput *m_reset_in = */new DigitalInput(27);  // Set SPI CS2 (IMU RST) high
  Wait(0.5); // Wait 500ms for reset to complete

  // Do this for configuration
  if(!SwitchToStandardSPI()){
    return;
  }

  // Set IMU internal decimation to 400 SPS
  WriteRegister(kRegDEC_RATE, 0x0004);
  // Enable Data Ready (LOW = Good Data), gSense Compensation, PoP
  WriteRegister(kRegMSC_CTRL, 0x00C1);
  // Configure IMU internal Bartlett filter
  WriteRegister(kRegFILT_CTRL, 0x0000);
  // Configure calibration time
  WriteRegister(kRegNULL_CNFG, m_calibration_time | 0x700);

  // Wait for samples to accumulate (110% of required time)
  Wait(pow(2, m_calibration_time) / 2000 * 64 * 1.1);

  // Start acquisition
  SwitchToAutoSPI();
  m_freed = false;
  m_acquire_task = std::thread(&ADIS16470_IMU::Acquire, this);

  // Re-initialize accumulations after calibration
  Reset();

  // Let the user know the IMU was initiallized successfully
  DriverStation::ReportWarning("ADIS16470 IMU Successfully Initialized!");

  // Drive "Ready" LED low
  /*auto *m_status_led = */new DigitalOutput(28);  // Set SPI CS3 (IMU Ready LED) low

  // Report usage and post data to DS
  HAL_Report(HALUsageReporting::kResourceType_ADIS16470, 0);
  SetName("ADIS16470", 0);
}

bool ADIS16470_IMU::SwitchToStandardSPI(){

  if (m_spi != nullptr)
    delete m_spi;

  // Set general SPI settings
  m_spi = new SPI(m_spi_port);
  m_spi->SetClockRate(1000000);
  m_spi->SetMSBFirst();
  m_spi->SetSampleDataOnTrailingEdge();
  m_spi->SetClockActiveLow();
  m_spi->SetChipSelectActiveLow();

  ReadRegister(kRegPROD_ID); // Dummy read

  // Validate the product ID
  uint16_t prod_id = ReadRegister(kRegPROD_ID);
  if (prod_id != 16982 && prod_id != 16470) {
    DriverStation::ReportError("Could not find ADIS16470!");
    return false;
  }
  DriverStation::ReportWarning("ADIS16470 IMU Detected. Starting calibration.");
  return true;
}

bool ADIS16470_IMU::SwitchToAutoSPI(){

  // Initialize standard SPI first
  if(m_spi == nullptr){
    if(!SwitchToStandardSPI()){
      return false;
    }
  }

  // Configure interrupt on SPI CS1
  m_auto_interrupt = new DigitalInput(26);

  // Configure DMA SPI
  m_spi->InitAuto(8200);
  m_spi->SetAutoTransmitData(kGLOB_CMD, 21);
  m_spi->ConfigureAutoStall(static_cast<HAL_SPIPort>(m_spi_port), 5, 1000, 1);

  // Kick off DMA SPI (Note: Device configration impossible after SPI DMA is activated)
  m_spi->StartAutoTrigger(*m_auto_interrupt, true, false);

  return true;
}

bool ADIS16470_IMU::Reconfigure(ADIS16470CalibrationTime new_cal_time) { 
  if(!SwitchToStandardSPI())
    return false;
  m_calibration_time = (uint16_t)new_cal_time;
  WriteRegister(kRegNULL_CNFG, m_calibration_time | 0x700);
  SwitchToAutoSPI();
  return true;
}

bool ADIS16470_IMU::Recalibrate() {
  if(!SwitchToStandardSPI())
    return false;
  WriteRegister(kGLOB_CMD, 0x0001);
  SwitchToAutoSPI();
  return true;
}

uint16_t ADIS16470_IMU::ReadRegister(uint8_t reg) {
  uint8_t buf[2];
  buf[0] = reg & 0x7f;
  buf[1] = 0;

  m_spi->Write(buf, 2);
  m_spi->Read(false, buf, 2);

  return ToUShort(buf);
}

void ADIS16470_IMU::WriteRegister(uint8_t reg, uint16_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  buf[1] = val & 0xff;
  m_spi->Write(buf, 2);
  buf[0] = 0x81 | reg;
  buf[1] = val >> 8;
  m_spi->Write(buf, 2);
}

void ADIS16470_IMU::Reset() {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_integ_gyro_x = 0.0;
  m_integ_gyro_y = 0.0;
  m_integ_gyro_z = 0.0;
}

ADIS16470_IMU::~ADIS16470_IMU() {
  m_spi->StopAuto();
  m_freed = true;
  m_samples_not_empty.notify_all();
  if (m_acquire_task.joinable()) m_acquire_task.join();
}

void ADIS16470_IMU::Acquire() {

  // X, Y, Z values are 32-bit split across 4 indices in the buffer
  // AutoSPI puts one byte in each index. Each index is 32-bits because the timestamp is an unsigned 32-bit int

  // Data order: [timestamp, request_1, request_2, x_1, x_2, x_3, x_4, y_1, y_2, y_3, y_4, z_1, z_2, z_3, z_4]
  // request = initial request = not used
  // x = delta x (32-bit x_1 = highest bit)
  // y = delta y (32-bit y_1 = highest bit)
  // z = delta z (32-bit z_1 = highest bit)

  const int dataset_len = 14; // Excluding timestamp

  // This buffer can contain many datasets
  uint32_t buffer[dataset_len * 30];

  uint32_t previous_timestamp = 0;

  while (!m_freed) {

    // Sleep loop for 10ms (wait for data)
	  Wait(.01);

    std::fill_n(buffer, 2000, 0);  // Clear buffer
	  data_count = m_spi->ReadAutoReceivedData(buffer, 0, 0_s); // Read number of bytes currently stored in the buffer
	  data_remainder = data_count % dataset_len; // Check if frame is incomplete
    data_to_read = data_count - data_remainder;  // Remove incomplete data from read count
	  m_spi->ReadAutoReceivedData(buffer, data_to_read, 0_s); // Read data from DMA buffer (only complete sets)

    // Could be multiple data sets in the buffer. Handle each one.
    for (int i = 0; i < data_to_read; i += dataset_len) {
      // Timestamp is at buffer[i]

      double delta_x = ToInt(&buffer[i + 3]) * (2160.0 / 2147483648.0) / (10000 * (buffer[i] - previous_timestamp)) / 4;
      double delta_y  = ToInt(&buffer[i + 7]) * (2160.0 / 2147483648.0) / (10000 * (buffer[i] - previous_timestamp)) / 4;
      double delta_z = ToIng(&buffer[i + 11]) * (2160.0 / 2147483648.0) / (10000 * (buffer[i] - previous_timestamp)) / 4;
      previous_timestamp = buffer[i];

      // Update global state
      {
        std::lock_guard<wpi::mutex> sync(m_mutex);
        // Accumulate gyro for angle integration
        m_integ_gyro_x += delta_x;
        m_integ_gyro_y += delta_y;
        m_integ_gyro_z += delta_z;
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
    default:
      return 0.0;
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

void ADIS16470_IMU::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("ADIS16470 IMU");
  auto gyroX = builder.GetEntry("GyroX").GetHandle();
  auto gyroY = builder.GetEntry("GyroY").GetHandle();
  auto gyroZ = builder.GetEntry("GyroZ").GetHandle();
  builder.SetUpdateTable([=]() {
    nt::NetworkTableEntry(angleX).SetDouble(GetAngleX());
    nt::NetworkTableEntry(angleY).SetDouble(GetAngleY());
    nt::NetworkTableEntry(angleZ).SetDouble(GetAngleZ());
  });
}
