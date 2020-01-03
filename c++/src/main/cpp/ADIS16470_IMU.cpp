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

#include <adi/ADIS16470_IMU.h>

#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include <frc/DriverStation.h>
#include <frc/ErrorBase.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/Timer.h>
#include <frc/WPIErrors.h>
#include <hal/HAL.h>

/* Helpful conversion functions */
static inline int32_t ToInt(const uint32_t *buf){
  return (int32_t)( (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3] );
}

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

  // Configure standard SPI
  if(!SwitchToStandardSPI()){
    return;
  }

  // Set IMU internal decimation to 400 SPS
  WriteRegister(DEC_RATE, 0x0004);
  // Set data ready polarity (HIGH = Good Data), gSense Compensation, PoP
  WriteRegister(MSC_CTRL, 0x00C1);
  // Configure IMU internal Bartlett filter
  WriteRegister(FILT_CTRL, 0x0000);
  // Configure continuous bias calibration time based on user setting
  WriteRegister(NULL_CNFG, m_calibration_time | 0x700);

  // Wait for samples to accumulate internal to the IMU (110% of user-defined time)
  Wait(pow(2, m_calibration_time) / 2000 * 64 * 1.1);

  // Write offset calibration command to IMU
  WriteRegister(GLOB_CMD, 0x0001);

  // Configure and enable auto SPI
  SwitchToAutoSPI();

  // Kick off acquire thread
  m_freed = false;
  m_acquire_task = std::thread(&ADIS16470_IMU::Acquire, this);

  // Let the user know the IMU was initiallized successfully
  DriverStation::ReportWarning("ADIS16470 IMU Successfully Initialized!");

  // Drive SPI CS3 (IMU ready LED) low (active low)
  new DigitalOutput(28); 

  // Report usage and post data to DS
  HAL_Report(HALUsageReporting::kResourceType_ADIS16470, 0);
  SetName("ADIS16470", 0);
}

/**
  * @brief Switches to standard SPI operation. Primarily used when exiting auto SPI mode.
  *
  * @return A boolean indicating the success or failure of setting up the SPI peripheral in standard SPI mode.
  *
  * This function switches the active SPI port to standard SPI and is used primarily when 
  * exiting auto SPI. Exiting auto SPI is required to read or write using SPI since the 
  * auto SPI configuration, once active, locks the SPI message being transacted. This function
  * also verifies that the SPI port is operating in standard SPI mode by reading back the IMU
  * product ID. 
 **/
bool ADIS16470_IMU::SwitchToStandardSPI(){

  if (m_spi != nullptr)
    delete m_spi;

  // Set general SPI settings
  m_spi = new SPI(m_spi_port);
  m_spi->SetClockRate(2000000);
  m_spi->SetMSBFirst();
  m_spi->SetSampleDataOnTrailingEdge();
  m_spi->SetClockActiveLow();
  m_spi->SetChipSelectActiveLow();

  ReadRegister(PROD_ID); // Dummy read

  // Validate the product ID
  uint16_t prod_id = ReadRegister(PROD_ID);
  if (prod_id != 16982 && prod_id != 16470) {
    DriverStation::ReportError("Could not find ADIS16470!");
    return false;
  }
  DriverStation::ReportWarning("ADIS16470 IMU Detected. Starting calibration delay.");
  return true;
}

/**
  * @brief Switches to auto SPI operation. Primarily used when exiting standard SPI mode.
  *
  * @return A boolean indicating the success or failure of setting up the SPI peripheral in auto SPI mode.
  *
  * This function switches the active SPI port to auto SPI and is used primarily when 
  * exiting standard SPI. Auto SPI is required to asynchronously read data over SPI as it utilizes
  * special FPGA hardware to react to an external data ready (GPIO) input. Data captured using auto SPI
  * is buffered in the FPGA and can be read by the CPU asynchronously. Standard SPI transactions are
  * impossible on the selected SPI port once auto SPI is enabled. The stall settings, GPIO interrupt pin,
  * and data packet settings used in this function are hard-coded to work only with the ADIS16470 IMU.
 **/
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
  m_spi->SetAutoTransmitData(m_autospi_packet, 0);
  //m_spi->ConfigureAutoStall(static_cast<HAL_SPIPort>(m_spi_port), 5, 1000, 1);
  m_spi->ConfigureAutoStall(HAL_SPI_kOnboardCS0, 5, 1000, 1);

  // Kick off DMA SPI (Note: Device configration impossible after SPI DMA is activated)
  m_spi->StartAutoTrigger(*m_auto_interrupt, true, false);

  return true;
}

/**
  * @brief Switches the active SPI port to standard SPI mode, writes a new value to the NULL_CNFG register in the IMU, and re-enables auto SPI.
  *
  * @param new_cal_time Calibration time to be set.
  * 
  * @return A boolean indicating the success or failure of writing the new NULL_CNFG setting and returning to auto SPI mode.
  *
  * This function enters standard SPI mode, writes a new NULL_CNFG setting to the IMU, and re-enters auto SPI mode. 
  * This function does not include a blocking sleep, so the user must keep track of the elapsed offset calibration time
  * themselves. After waiting the configured calibration time, the Calibrate() function should be called to activate the new
  * offset calibration. 
 **/
bool ADIS16470_IMU::Reconfigure(ADIS16470CalibrationTime new_cal_time) { 
  if(!SwitchToStandardSPI())
    return false;
  m_calibration_time = (uint16_t)new_cal_time;
  WriteRegister(NULL_CNFG, m_calibration_time | 0x700);
  SwitchToAutoSPI();
  return true;
}

/**
  * @brief Switches the active SPI port to standard SPI mode, writes the command to activate the new null configuration, and re-enables auto SPI.
  *
  * This function enters standard SPI mode, writes 0x0001 to the GLOB_CMD register (thus making the new offset active in the IMU), and 
  * re-enters auto SPI mode. This function does not include a blocking sleep, so the user must keep track of the elapsed offset calibration time
  * themselves. 
 **/
void ADIS16470_IMU::Calibrate() {
  if(!SwitchToStandardSPI())
    return;
  WriteRegister(GLOB_CMD, 0x0001);
  SwitchToAutoSPI();
}

/**
  * @brief Reads the contents of a specified register location over SPI. 
  *
  * @param reg An unsigned, 8-bit register location.
  * 
  * @return An unsigned, 16-bit number representing the contents of the requested register location.
  *
  * This function reads the contents of an 8-bit register location by transmitting the register location
  * byte along with a null (0x00) byte using the standard WPILib API. The response (two bytes) is read 
  * back using the WPILib API and joined using a helper function. This function assumes the controller 
  * is set to standard SPI mode.
 **/
uint16_t ADIS16470_IMU::ReadRegister(uint8_t reg) {
  uint8_t buf[2];
  buf[0] = reg & 0x7f;
  buf[1] = 0;

  m_spi->Write(buf, 2);
  m_spi->Read(false, buf, 2);

  return ToUShort(buf);
}

/**
  * @brief Writes an unsigned, 16-bit value to two adjacent, 8-bit register locations over SPI.
  *
  * @param reg An unsigned, 8-bit register location.
  * 
  * @param val An unsigned, 16-bit value to be written to the specified register location.
  *
  * This function writes an unsigned, 16-bit value into adjacent 8-bit addresses via SPI. The upper
  * and lower bytes that make up the 16-bit value are split into two unsined, 8-bit values and written
  * to the upper and lower addresses of the specified register value. Only the lower (base) address
  * must be specified. This function assumes the controller is set to standard SPI mode.
 **/
void ADIS16470_IMU::WriteRegister(uint8_t reg, uint16_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  buf[1] = val & 0xff;
  m_spi->Write(buf, 2);
  buf[0] = 0x81 | reg;
  buf[1] = val >> 8;
  m_spi->Write(buf, 2);
}

/**
  * @brief Resets (zeros) the xgyro, ygyro, and zgyro angle integrations. 
  *
  * This function resets (zeros) the accumulated (integrated) angle estimates for the xgyro, ygyro, and zgyro outputs.
 **/
void ADIS16470_IMU::Reset() {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_integ_gyro_x = 0.0;
  m_integ_gyro_y = 0.0;
  m_integ_gyro_z = 0.0;
}

ADIS16470_IMU::~ADIS16470_IMU() {
  m_spi->StopAuto();
  m_freed = true;
  if (m_acquire_task.joinable()) m_acquire_task.join();
}

/**
  * @brief Main acquisition loop. Typically called asynchronously and free-wheels while the robot code is active. 
  *
  * This is the main acquisiton loop for the IMU. During each iteration, data read using auto SPI is 
  * extracted from the FPGA FIFO, split, scaled, and integrated. 
  * 
  * Each X, Y, and Z value is 32-bits split across 4 indices (bytes) in the buffer.
  * Auto SPI puts one byte in each index location. Each index is 32-bits wide because the timestamp is an unsigned 32-bit int.
  * The timestamp is always located at the beginning of the frame. Two indices (request_1 and request_2 below) are always
  * invalid (garbage) and can be disregarded.
  *
  * Data order: [timestamp, request_1, request_2, x_1, x_2, x_3, x_4, y_1, y_2, y_3, y_4, z_1, z_2, z_3, z_4]
  * x = delta x (32-bit x_1 = highest bit)
  * y = delta y (32-bit y_1 = highest bit)
  * z = delta z (32-bit z_1 = highest bit)
 **/
void ADIS16470_IMU::Acquire() {
  // Set data packet length
  const int dataset_len = 15; // 14 data points + timestamp
  const int num_buffers = 30;

  // This buffer can contain many datasets
  uint32_t buffer[dataset_len * num_buffers];
  int data_count, data_remainder, data_to_read = 0;
  uint32_t previous_timestamp = 0;
  double delta_x, delta_y, delta_z;

  while (!m_freed) {

    // Sleep loop for 10ms (wait for data)
	  Wait(.01);

	  data_count = m_spi->ReadAutoReceivedData(buffer, 0, 0_s); // Read number of bytes currently stored in the buffer
	  data_remainder = data_count % dataset_len; // Check if frame is incomplete. Add 1 because of timestamp
    data_to_read = data_count - data_remainder;  // Remove incomplete data from read count
	  m_spi->ReadAutoReceivedData(buffer, data_to_read, 0_s); // Read data from DMA buffer (only complete sets)

    /*
    // DEBUG: Print buffer size and contents to terminal
    std::cout << "Start - " << data_count << "," << data_remainder << "," << data_to_read << std::endl;
    for (int m = 0; m < data_to_read - 1; m++ )
    {
      std::cout << buffer[m] << ",";
    }
    std::cout << " " << std::endl;
    std::cout << "End" << std::endl;
    std::cout << "Reading " << data_count << " bytes." << std::endl;
    */
	  
    // Could be multiple data sets in the buffer. Handle each one.
    for (int i = 0; i < data_to_read; i += dataset_len) {
      // Timestamp is at buffer[i]
      // Scale 32-bit data and adjust for IMU decimation seting (2000 / 4 + 1 = 400SPS)
      delta_x = (ToInt(&buffer[i + 3]) * (2160.0 / 2147483648.0)) / ((10000 / (buffer[i] - previous_timestamp)) / 4);
      delta_y = (ToInt(&buffer[i + 7]) * (2160.0 / 2147483648.0)) / ((10000 / (buffer[i] - previous_timestamp)) / 4);
      delta_z = (ToInt(&buffer[i + 11]) * (2160.0 / 2147483648.0)) / ((10000 / (buffer[i] - previous_timestamp)) / 4);
      previous_timestamp = buffer[i];

      /*
      // DEBUG: Print timestamp and delta values
      std::cout << previous_timestamp << "," << delta_x << "," << delta_y << "," << delta_z << std::endl;
      */

      {
        std::lock_guard<wpi::mutex> sync(m_mutex);
        // Accumulate gyro for angle integration
        m_integ_gyro_x += delta_x;
        m_integ_gyro_y += delta_y;
        m_integ_gyro_z += delta_z;
      }

      // DEBUG: Print accumulated values
      //std::cout << m_integ_gyro_x << "," << m_integ_gyro_y << "," << m_integ_gyro_z << std::endl;

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

double ADIS16470_IMU::GetRate() const {
  return 0;
}

void ADIS16470_IMU::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("ADIS16470 IMU");
  auto gyroX = builder.GetEntry("GyroX").GetHandle();
  auto gyroY = builder.GetEntry("GyroY").GetHandle();
  auto gyroZ = builder.GetEntry("GyroZ").GetHandle();
  builder.SetUpdateTable([=]() {
    nt::NetworkTableEntry(gyroX).SetDouble(GetAngleX());
    nt::NetworkTableEntry(gyroY).SetDouble(GetAngleY());
    nt::NetworkTableEntry(gyroZ).SetDouble(GetAngleZ());
  });
}
