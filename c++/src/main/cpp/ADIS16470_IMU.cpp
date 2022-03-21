/*----------------------------------------------------------------------------*/
/* ADIS16470 RoboRIO Driver (c) by Juan Chong
/*
/* The ADIS16470 RoboRIO Driver is licensed under a
/* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
/*
/* You should have received a copy of the license along with this
/* work. If not, see <http://creativecommons.org/licenses/by-nc-sa/4.0/>.                             */
/*----------------------------------------------------------------------------*/

#include <string>
#include <iostream>
#include <cmath>

#include <adi/ADIS16470_IMU.h>

#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include <frc/DriverStation.h>
#include <networktables/NTSendableBuilder.h>
#include <frc/Timer.h>
#include <frc/Errors.h>
#include <hal/HAL.h>

/* Helpful conversion functions */
static inline int32_t ToInt(const uint32_t *buf){
  return (int32_t)( (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3] );
}

static inline uint16_t BuffToUShort(const uint32_t* buf) {
  return ((uint16_t)(buf[0]) << 8) | buf[1];
}

static inline int16_t BuffToShort(const uint32_t* buf) {
  return ((int16_t)(buf[0]) << 8) | buf[1];
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
ADIS16470_IMU::ADIS16470_IMU() : ADIS16470_IMU(kZ, SPI::Port::kOnboardCS0, ADIS16470CalibrationTime::_1s, true) {}

ADIS16470_IMU::ADIS16470_IMU(IMUAxis yaw_axis, SPI::Port port, ADIS16470CalibrationTime cal_time, bool cal_on_start) : 
                m_yaw_axis(yaw_axis), 
                m_spi_port(port),
                m_calibration_time((uint16_t)cal_time),
                m_calibrate_every_start(cal_on_start) {

  // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
  // Relies on the RIO hardware by default configuring an output as low
  // and configuring an input as high Z. The 10k pull-up resistor internal to the
  // IMU then forces the reset line high for normal operation.
  DigitalOutput *m_reset_out = new DigitalOutput(27);  // Drive SPI CS2 (IMU RST) low
  Wait(0.01_s);  // Wait 10ms
  delete m_reset_out;
  new DigitalInput(27);  // Set SPI CS2 (IMU RST) high
  Wait(0.5_s); // Wait 500ms for reset to complete

  // Configure standard SPI
  if(!SwitchToStandardSPI()){
    return;
  }

  // Set IMU internal decimation to 4 (output data rate of 2000 SPS / (4 + 1) = 400Hz), output bandwidth = 200Hz
  if(ReadRegister(DEC_RATE) != 0x0004){
    WriteRegister(DEC_RATE, 0x0004);
    m_needs_flash = true;
    DriverStation::ReportWarning("ADIS16470: DEC_RATE register configuration inconsistent! Scheduling flash update...");
  }

  // Set data ready polarity (HIGH = Good Data), Disable gSense Compensation and PoP
  if(ReadRegister(MSC_CTRL) != 0x0001){
    WriteRegister(MSC_CTRL, 0x0001); 
    m_needs_flash = true;
    DriverStation::ReportWarning("ADIS16470: MSC_CTRL register configuration inconsistent! Scheduling flash update...");
  }
 
  // Disable IMU internal Bartlett filter (200Hz bandwidth is sufficient)
  if(ReadRegister(FILT_CTRL) != 0x0000{
    WriteRegister(FILT_CTRL, 0x0000);
    m_needs_flash = true;
    DriverStation::ReportWarning("ADIS16470: FILT_CTRL register configuration inconsistent! Scheduling flash update...");
  }

  // If any registers on the IMU don't match the config, trigger a flash update
  if(m_needs_flash){
    DriverStation::ReportWarning("ADIS16470: Register configuration changed! Starting IMU flash update.");
    WriteRegister(GLOB_CMD, 0x0008);
    // Wait long enough for the flash update to finish (72ms minimum as per the datasheet)
    Wait(0.3);
    DriverStation::ReportWarning("ADIS16470: Done!");
    m_needs_flash = false;
  }

  // Configure continuous bias calibration time based on user setting
  WriteRegister(NULL_CNFG, m_calibration_time | 0x700);

  // Notify DS that IMU calibration delay is active
  FRC_ReportError(warn::Warning, "{}", "ADIS16470 IMU Detected. Starting initial calibration delay.");

  // Wait for samples to accumulate internal to the IMU (110% of user-defined time)
  Wait(pow(2, m_calibration_time) / 2000 * 64 * 1.1_s);

  // Write offset calibration command to IMU
  WriteRegister(GLOB_CMD, 0x0001);

  // Configure and enable auto SPI
  if(!SwitchToAutoSPI()) {
    return;
  }

  // Let the user know the IMU was initiallized successfully
  FRC_ReportError(warn::Warning, "{}", "ADIS16470 IMU Successfully Initialized!");

  // Drive SPI CS3 (IMU ready LED) low (active low)
  new DigitalOutput(28); 

  // Report usage and post data to DS
  HAL_Report(HALUsageReporting::kResourceType_ADIS16470, 0);
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
  // Check to see whether the acquire thread is active. If so, wait for it to stop producing data.
  if (m_thread_active) {
    m_thread_active = false;
    while (!m_thread_idle) {
      Wait(0.01_s);
    }
    std::cout << "Paused the IMU processing thread successfully!" << std::endl;
    // Maybe we're in auto SPI mode? If so, kill auto SPI, and then SPI.
    if (m_spi != nullptr && m_auto_configured) {
      m_spi->StopAuto();
      // We need to get rid of all the garbage left in the auto SPI buffer after stopping it.
      // Sometimes data magically reappears, so we have to check the buffer size a couple of times
      //  to be sure we got it all. Yuck.
      uint32_t trashBuffer[200];
      Wait(0.1_s);
      int data_count = m_spi->ReadAutoReceivedData(trashBuffer, 0, 0_s);
      while (data_count > 0) {
        /* Receive data, max of 200 words at a time (prevent potential segfault) */
        m_spi->ReadAutoReceivedData(trashBuffer, std::min(data_count, 200), 0_s);
        /*Get the reamining data count */
        data_count = m_spi->ReadAutoReceivedData(trashBuffer, 0, 0_s);
      }
      std::cout << "Paused the auto SPI successfully!" << std::endl;
      }
  }
  // There doesn't seem to be a SPI port active. Let's try to set one up
  if (m_spi == nullptr) {
    std::cout << "Setting up a new SPI port." << std::endl;
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
      FRC_ReportError(err::Error, "{}", "Could not find ADIS16470!");
      Close();
      return false;
    }
    return true;
  }
  else {
    // Maybe the SPI port is active, but not in auto SPI mode? Try to read the product ID.
    ReadRegister(PROD_ID); // Dummy read
    uint16_t prod_id = ReadRegister(PROD_ID);
    if (prod_id != 16982 && prod_id != 16470) {
      FRC_ReportError(err::Error, "{}", "Could not find ADIS16470!");
      Close();
      return false;
    }
    else {
      return true;
    }
  }
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

  // No SPI port has been set up. Go set one up first.
  if(m_spi == nullptr){
    if(!SwitchToStandardSPI()){
      FRC_ReportError(err::Error, "{}", "Failed to start/restart auto SPI");
      return false;
    }
  }
  // Only set up the interrupt if needed.
  if (m_auto_interrupt == nullptr) {
    m_auto_interrupt = new DigitalInput(26);
  }
  // The auto SPI controller gets angry if you try to set up two instances on one bus.
  if (!m_auto_configured) {
    m_spi->InitAuto(8200);
    m_auto_configured = true;
  }
  // Do we need to change auto SPI settings?
  switch (m_yaw_axis) {
  case kX:
    m_spi->SetAutoTransmitData(m_autospi_x_packet, 2);
    break;
  case kY:
    m_spi->SetAutoTransmitData(m_autospi_y_packet, 2);
    break;
  default:
    m_spi->SetAutoTransmitData(m_autospi_z_packet, 2);
    break;
  }
  // Configure auto stall time  
  m_spi->ConfigureAutoStall(HAL_SPI_kOnboardCS0, 5, 1000, 1);
  // Kick off DMA SPI (Note: Device configration impossible after SPI DMA is activated)
  // DR High = Data good (data capture should be triggered on the rising edge)
  m_spi->StartAutoTrigger(*m_auto_interrupt, true, false);
  // Check to see if the acquire thread is running. If not, kick one off.
  if(!m_thread_idle) {
    m_first_run = true;
    m_thread_active = true;
    m_acquire_task = std::thread(&ADIS16470_IMU::Acquire, this);
    std::cout << "New IMU Processing thread activated!" << std::endl;
  }
  else {
    m_first_run = true;
    m_thread_active = true;
    std::cout << "Old IMU Processing thread re-activated!" << std::endl;
  }
  // Looks like the thread didn't start for some reason. Abort.
  /*
  if(!m_thread_idle) {
    FRC_ReportError(err::Error, "{}", "Failed to start/restart the acquire() thread.");
    Close();
    return false;
  }
  */
  return true;
}

/**
  * @brief Switches the active SPI port to standard SPI mode, writes a new value to the NULL_CNFG register in the IMU, and re-enables auto SPI.
  *
  * @param new_cal_time Calibration time to be set.
  * 
  * @return An int indicating the success or failure of writing the new NULL_CNFG setting and returning to auto SPI mode. 0 = Success, 1 = No Change, 2 = Failure
  *
  * This function enters standard SPI mode, writes a new NULL_CNFG setting to the IMU, and re-enters auto SPI mode. 
  * This function does not include a blocking sleep, so the user must keep track of the elapsed offset calibration time
  * themselves. After waiting the configured calibration time, the Calibrate() function should be called to activate the new
  * offset calibration. 
 **/
int ADIS16470_IMU::ConfigCalTime(ADIS16470CalibrationTime new_cal_time) { 
  if(m_calibration_time == (uint16_t)new_cal_time)
    return 1;
  if(!SwitchToStandardSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure standard SPI.");
    return 2;
  }
  m_calibration_time = (uint16_t)new_cal_time;
  WriteRegister(NULL_CNFG, m_calibration_time | 0x700);
  if(!SwitchToAutoSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure auto SPI.");
    return 2;
  }
  return 0;
}

/**
  * @brief Switches the active SPI port to standard SPI mode, writes a new value to the DECIMATE register in the IMU, and re-enables auto SPI.
  *
  * @param reg Decimation value to be set.
  * 
  * @return An int indicating the success or failure of writing the new DECIMATE setting and returning to auto SPI mode. 0 = Success, 1 = No Change, 2 = Failure
  *
  * This function enters standard SPI mode, writes a new DECIMATE setting to the IMU, adjusts the sample scale factor, and re-enters auto SPI mode. 
 **/
int ADIS16470_IMU::ConfigDecRate(uint16_t reg) { 
  uint16_t m_reg = reg;
  if(!SwitchToStandardSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure standard SPI.");
    return 2;
  }
  if(m_reg > 1999) {
    FRC_ReportError(err::Error, "{}", "Attempted to write an invalid decimation value.");
    m_reg = 1999;
  }
  m_scaled_sample_rate = (((m_reg + 1.0)/2000.0) * 1000000.0);
  WriteRegister(DEC_RATE, m_reg);
  if(!SwitchToAutoSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure auto SPI.");
    return 2;
  }
  return 0;
}

/**
  * @brief Writes the current IMU configuration (including the initial calibration) to flash.
  *
  * This function triggers an IMU subroutine (GLOB_CMD = 0x0008) that writes the current IMU register configuration
  * to the IMU's onboard ROM. Once the configuration is stored to the IMU's onboard flash, the IMU will automatically
  * boot with the previous configuration.
 **/
int ADIS16470_IMU::WriteIMUSettingsToFlash() {
  if(!SwitchToStandardSPI()) {
    DriverStation::ReportError("Failed to configure/reconfigure standard SPI.");
    return 2;
  }
  WriteRegister(GLOB_CMD, 0x0008);
  wait(1.0);
  if(!SwitchToAutoSPI()) {
    DriverStation::ReportError("Failed to configure/reconfigure auto SPI.");
    return 2;
  }
  return 0;
}

/**
  * @brief Switches the active SPI port to standard SPI mode, writes the command to activate the new null configuration, and re-enables auto SPI.
  *
  * This function enters standard SPI mode, writes 0x0001 to the GLOB_CMD register (thus making the new offset active in the IMU), and 
  * re-enters auto SPI mode. This function does not include a blocking sleep, so the user must keep track of the elapsed offset calibration time
  * themselves. 
 **/
void ADIS16470_IMU::Calibrate() {
  if(!SwitchToStandardSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure standard SPI.");
  }
  WriteRegister(GLOB_CMD, 0x0001);
  if(!SwitchToAutoSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure auto SPI.");
  }
}

int ADIS16470_IMU::SetYawAxis(IMUAxis yaw_axis) {
  if(m_yaw_axis == yaw_axis)
    return 1; 
  if(!SwitchToStandardSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure standard SPI.");
    return 2;
  }
  m_yaw_axis = yaw_axis;
  if(!SwitchToAutoSPI()) {
    FRC_ReportError(err::Error, "{}", "Failed to configure/reconfigure auto SPI.");
    return 2;
  }
  return 0;
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
  m_integ_angle = 0.0;
}

void ADIS16470_IMU::Close() {
  if (m_thread_active) {
    m_thread_active = false;
    if (m_acquire_task.joinable()) m_acquire_task.join();
  }
  if (m_spi != nullptr) {
    if (m_auto_configured) {
      m_spi->StopAuto();
    }
  delete m_spi;
  m_auto_configured = false;
    if (m_auto_interrupt != nullptr) {
      delete m_auto_interrupt;
      m_auto_interrupt = nullptr;
    }
    m_spi = nullptr;
  }
  std::cout << "Finished cleaning up after the IMU driver." << std::endl;
}

ADIS16470_IMU::~ADIS16470_IMU() {
  Close();
}

/**
  * @brief Main acquisition loop. Typically called asynchronously and free-wheels while the robot code is active. 
  *
  * This is the main acquisiton loop for the IMU. During each iteration, data read using auto SPI is 
  * extracted from the FPGA FIFO, split, scaled, and integrated. Each X, Y, and Z value is 32-bits split 
  * across 4 indices (bytes) in the buffer. Auto SPI puts one byte in each index location. 
  * Each index is 32-bits wide because the timestamp is an unsigned 32-bit int.
  * The timestamp is always located at the beginning of the frame. Two indices (request_1 and request_2 below)
  * are always invalid (garbage) and can be disregarded.
  *
  * Data order: [timestamp, request_1, request_2, x_1, x_2, x_3, x_4, y_1, y_2, y_3, y_4, z_1, z_2, z_3, z_4]
  * x = delta x (32-bit x_1 = highest bit)
  * y = delta y (32-bit y_1 = highest bit)
  * z = delta z (32-bit z_1 = highest bit)
  * 
  * Complementary filter code was borrowed from https://github.com/tcleg/Six_Axis_Complementary_Filter
 **/
void ADIS16470_IMU::Acquire() {
  // Set data packet length
  const int dataset_len = 19; // 18 data points + timestamp

  /* Fixed buffer size */
  const int BUFFER_SIZE = 4000;

  // This buffer can contain many datasets
  uint32_t buffer[BUFFER_SIZE];
  int data_count = 0;
  int data_remainder = 0;
  int data_to_read = 0;
  uint32_t previous_timestamp = 0;
  double delta_angle = 0.0;
  double gyro_x = 0.0;
  double gyro_y = 0.0;
  double gyro_z = 0.0;
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
  double gyro_x_si = 0.0;
  double gyro_y_si = 0.0;
  //double gyro_z_si = 0.0;
  double accel_x_si = 0.0;
  double accel_y_si = 0.0;
  double accel_z_si = 0.0;
  double compAngleX = 0.0;
  double compAngleY = 0.0;
  double accelAngleX = 0.0;
  double accelAngleY = 0.0;

  while (true) {

    // Sleep loop for 10ms (wait for data)
	  Wait(0.01_s);

    if (m_thread_active) {

      m_thread_idle = false;

      data_count = m_spi->ReadAutoReceivedData(buffer, 0, 0_s); // Read number of bytes currently stored in the buffer
      data_remainder = data_count % dataset_len; // Check if frame is incomplete. Add 1 because of timestamp
      data_to_read = data_count - data_remainder;  // Remove incomplete data from read count
      /* Want to cap the data to read in a single read at the buffer size */
      if(data_to_read > BUFFER_SIZE)
      {
          FRC_ReportError(warn::Warning, "{}", "ADIS16470 data processing thread overrun has occurred!");
          data_to_read = BUFFER_SIZE - (BUFFER_SIZE % dataset_len);
      }
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
        m_dt = (buffer[i] - previous_timestamp) / 1000000.0;
        /* Get delta angle value for selected yaw axis and scale by the elapsed time (based on timestamp) */
        delta_angle = (ToInt(&buffer[i + 3]) * delta_angle_sf) / (m_scaled_sample_rate / (buffer[i] - previous_timestamp));
        gyro_x = (BuffToShort(&buffer[i + 7]) / 10.0);
        gyro_y = (BuffToShort(&buffer[i + 9]) / 10.0);
        gyro_z = (BuffToShort(&buffer[i + 11]) / 10.0);
        accel_x = (BuffToShort(&buffer[i + 13]) / 800.0);
        accel_y = (BuffToShort(&buffer[i + 15]) / 800.0);
        accel_z = (BuffToShort(&buffer[i + 17]) / 800.0);

        // Convert scaled sensor data to SI units
        gyro_x_si = gyro_x * deg_to_rad;
        gyro_y_si = gyro_y * deg_to_rad;
        //gyro_z_si = gyro_z * deg_to_rad;
        accel_x_si = accel_x * grav;
        accel_y_si = accel_y * grav;
        accel_z_si = accel_z * grav;

        // Store timestamp for next iteration
        previous_timestamp = buffer[i];

        /*
        // DEBUG: Print timestamp and delta values
        std::cout << previous_timestamp << "," << delta_x << "," << delta_y << "," << delta_z << std::endl;
        */

        m_alpha = m_tau / (m_tau + m_dt);

        if (m_first_run) {
          accelAngleX = atan2f(accel_x_si, sqrtf((accel_y_si * accel_y_si) + (accel_z_si * accel_z_si)));
          accelAngleY = atan2f(accel_y_si, sqrtf((accel_x_si * accel_x_si) + (accel_z_si * accel_z_si)));
          compAngleX = accelAngleX;
          compAngleY = accelAngleY;
        }
        else {
          // Process X angle
          accelAngleX = atan2f(accel_x_si, sqrtf((accel_y_si * accel_y_si) + (accel_z_si * accel_z_si)));
          accelAngleY = atan2f(accel_y_si, sqrtf((accel_x_si * accel_x_si) + (accel_z_si * accel_z_si)));
          accelAngleX = FormatAccelRange(accelAngleX, accel_z_si);
          accelAngleY = FormatAccelRange(accelAngleY, accel_z_si);
          compAngleX = CompFilterProcess(compAngleX, accelAngleX, -gyro_y_si);
          compAngleY = CompFilterProcess(compAngleY, accelAngleY, gyro_x_si);
        }

        // DEBUG: Print accumulated values
        //std::cout << m_compAngleX << "," << m_compAngleY << std::endl;

        {
          std::lock_guard<wpi::mutex> sync(m_mutex);
          /* Push data to global variables */
          if(m_first_run) {
            /* Don't accumulate first run. previous_timestamp will be "very" old and the integration will end up way off */
            m_integ_angle = 0.0;
          }
          else {
            m_integ_angle += delta_angle;
          }
          m_gyro_x = gyro_x;
          m_gyro_y = gyro_y;
          m_gyro_z = gyro_z;
          m_accel_x = accel_x;
          m_accel_y = accel_y;
          m_accel_z = accel_z;
          m_compAngleX = compAngleX * rad_to_deg;
          m_compAngleY = compAngleY * rad_to_deg;
          m_accelAngleX = accelAngleX * rad_to_deg;
          m_accelAngleY = accelAngleY * rad_to_deg;
        }
        m_first_run = false;
      }
    }
    else {
        m_thread_idle = true;
        data_count = 0;
        data_remainder = 0;
        data_to_read = 0;
        previous_timestamp = 0.0;
        delta_angle = 0.0;
        gyro_x = 0.0;
        gyro_y = 0.0;
        gyro_z = 0.0;
        accel_x = 0.0;
        accel_y = 0.0;
        accel_z = 0.0;
        gyro_x_si = 0.0;
        gyro_y_si = 0.0;
        //gyro_z_si = 0.0;
        accel_x_si = 0.0;
        accel_y_si = 0.0;
        accel_z_si = 0.0;
        compAngleX = 0.0;
        compAngleY = 0.0;
        accelAngleX = 0.0;
        accelAngleY = 0.0;
    }
  }
}

/* Complementary filter functions */
double ADIS16470_IMU::FormatFastConverge(double compAngle, double accAngle) {
  if(compAngle > accAngle + M_PI) {
    compAngle = compAngle - 2.0 * M_PI;
  }
  else if (accAngle > compAngle + M_PI) {
    compAngle = compAngle + 2.0 * M_PI;
  }
  return compAngle;  
}

double ADIS16470_IMU::FormatRange0to2PI(double compAngle) {
  while(compAngle >= 2 * M_PI) {
    compAngle = compAngle - 2.0 * M_PI;
  }
  while(compAngle < 0.0) {
    compAngle = compAngle + 2.0 * M_PI;
  }
  return compAngle;
}

double ADIS16470_IMU::FormatAccelRange(double accelAngle, double accelZ) {
  if(accelZ < 0.0) {
    accelAngle = M_PI - accelAngle;
  }
  else if(accelZ > 0.0 && accelAngle < 0.0) {
    accelAngle = 2.0 * M_PI + accelAngle;
  }
  return accelAngle;
}

double ADIS16470_IMU::CompFilterProcess(double compAngle, double accelAngle, double omega) {
  compAngle = FormatFastConverge(compAngle, accelAngle);
  compAngle = m_alpha * (compAngle + omega * m_dt) + (1.0 - m_alpha) * accelAngle;
  compAngle = FormatRange0to2PI(compAngle);
  if(compAngle > M_PI) {
    compAngle = compAngle - 2.0 * M_PI;
  }
  return compAngle;
}

/**
  * @brief Returns the current integrated angle for the axis specified. 
  *
  * @param m_yaw_axis An enum indicating the axis chosen to act as the yaw axis.
  * 
  * @return The current integrated angle in degrees.
  *
  * This function returns the most recent integrated angle for the axis chosen by m_yaw_axis. 
  * This function is most useful in situations where the yaw axis may not coincide with the IMU
  * Z axis. 
 **/
double ADIS16470_IMU::GetAngle() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_angle;
}

double ADIS16470_IMU::GetRate() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
    switch (m_yaw_axis) {
    case kX:
      return m_gyro_x;
    case kY:
      return m_gyro_y;
    case kZ:
      return m_gyro_z;
    default:
      return 0.0;
  }
}

ADIS16470_IMU::IMUAxis ADIS16470_IMU::GetYawAxis() const {
  return m_yaw_axis;
}

double ADIS16470_IMU::GetGyroInstantX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_x;
}

double ADIS16470_IMU::GetGyroInstantY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_y;
}

double ADIS16470_IMU::GetGyroInstantZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_z;
}

double ADIS16470_IMU::GetAccelInstantX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_x;
}

double ADIS16470_IMU::GetAccelInstantY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_y;
}

double ADIS16470_IMU::GetAccelInstantZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_z;
}

double ADIS16470_IMU::GetXComplementaryAngle() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_compAngleX;
}

double ADIS16470_IMU::GetYComplementaryAngle() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_compAngleY;
}

double ADIS16470_IMU::GetXFilteredAccelAngle() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accelAngleX;
}

double ADIS16470_IMU::GetYFilteredAccelAngle() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accelAngleY;
}

/**
  * @brief Builds a Sendable object to push IMU data to the driver station.
  *
  * This function pushes the most recent angle estimates for all axes to the driver station.
 **/
void ADIS16470_IMU::InitSendable(nt::NTSendableBuilder& builder) {
  builder.SetSmartDashboardType("ADIS16470 IMU");
  auto yaw_angle = builder.GetEntry("Yaw Angle").GetHandle();
  builder.SetUpdateTable([=]() {
    nt::NetworkTableEntry(yaw_angle).SetDouble(GetAngle());
  });
}
