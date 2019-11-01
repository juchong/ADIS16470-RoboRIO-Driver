/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.analog.adis16470.frc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is for the ADIS16470 IMU that connects to the RoboRIO SPI port.
 */
@SuppressWarnings("unused")
public class ADIS16470_IMU extends GyroBase implements Gyro, PIDSource, Sendable {
	public static final double kCalibrationSampleTime = 5.0; // Calibration time in seconds (regardless of sps)
  public static final double kDegreePerSecondPerLSB = 1.0/10.0;
  public static final double kGPerLSB = 1.0/800.0;
  public static final double kDegCPerLSB = 1.0/10.0;
  public static final double kDegCOffset = 0.0;

  public static final int kGLOB_CMD = 0x68;
  public static final int kRegDEC_RATE = 0x64;
  public static final int kRegFILT_CTRL = 0x5C;
  public static final int kRegMSC_CTRL = 0x60;
  public static final int kRegPROD_ID = 0x72;

  public enum Axis { kX, kY, kZ }

  // AHRS yaw axis
  private Axis m_yaw_axis;

  // gyro offset
  private double m_gyro_offset_x = 0.0;
  private double m_gyro_offset_y = 0.0;
  private double m_gyro_offset_z = 0.0;

  // last read values (post-scaling)
  private double m_gyro_x = 0.0;
  private double m_gyro_y = 0.0;
  private double m_gyro_z = 0.0;
  private double m_accel_x = 0.0;
  private double m_accel_y = 0.0;
  private double m_accel_z = 0.0;
  private double m_temp = 0.0;
  private double m_status = 0.0;
  private double m_counter = 0.0;
  private double dt = 0.0;

  // accumulated gyro values (for offset calculation)
  private int m_accum_count = 0;
  private double m_accum_gyro_x = 0.0;
  private double m_accum_gyro_y = 0.0;
  private double m_accum_gyro_z = 0.0;

  // integrated gyro values
  private double m_integ_gyro_x = 0.0;
  private double m_integ_gyro_y = 0.0;
  private double m_integ_gyro_z = 0.0;

  // last sample time
  private double m_last_sample_time = 0.0;

  private AtomicBoolean m_freed = new AtomicBoolean(false);

  private SPI m_spi;
  private DigitalInput m_interrupt;
  private DigitalOutput m_reset_out;
  private DigitalInput m_reset_in;
  private DigitalOutput m_status_led;

  // Sample from the IMU
  private static class Sample {
    public double gyro_x;
    public double gyro_y;
    public double gyro_z;
    public double accel_x;
    public double accel_y;
    public double accel_z;
    public double mag_x;
    public double mag_y;
    public double mag_z;
    public double baro;
    public double temp;
    public double dt;

    // Swap axis as appropriate for yaw axis selection
    public void adjustYawAxis(Axis yaw_axis) {
      switch (yaw_axis) {
        case kX: {
          // swap X and Z
          double tmp;
          tmp = accel_x;
          accel_x = accel_z;
          accel_z = tmp;
          tmp = mag_x;
          mag_x = mag_z;
          mag_z = tmp;
          tmp = gyro_x;
          gyro_x = gyro_z;
          gyro_z = tmp;
          break;
        }
        case kY: {
          // swap Y and Z
          double tmp;
          tmp = accel_y;
          accel_y = accel_z;
          accel_z = tmp;
          tmp = mag_y;
          mag_y = mag_z;
          mag_z = tmp;
          tmp = gyro_y;
          gyro_y = gyro_z;
          gyro_z = tmp;
          break;
        }
        case kZ:
        default:
          // no swap required
          break;
      }
    }
  }

  // Sample FIFO
  private static final int kSamplesDepth = 10;
  private final Sample[] m_samples;
  private final Lock m_samples_mutex;
  private final Condition m_samples_not_empty;
  private int m_samples_count = 0;
  private int m_samples_take_index = 0;
  private int m_samples_put_index = 0;
  private boolean m_calculate_started = false;

  // Previous timestamp
  long timestamp_old = 0;

  private static class AcquireTask implements Runnable {
    private ADIS16470_IMU imu;
    public AcquireTask(ADIS16470_IMU imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.acquire();
    }
  }
  private Thread m_acquire_task;

  public ADIS16470_IMU(){
    this(Axis.kZ, SPI.Port.kOnboardCS0);
  }

  /**
   * @param yaw_axis Which axis is Yaw
   * @param port SPI port to use
   */
  public ADIS16470_IMU(Axis yaw_axis, SPI.Port port) {
    m_yaw_axis = yaw_axis;
    
    // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
    // Relies on the RIO hardware by default configuring an output as low
    // and configuring an input as high Z. The 10k pull-up resistor internal to the 
    // IMU then forces the reset line high for normal operation. 
    m_reset_out = new DigitalOutput(27);  // Drive SPI CS2 (IMU RST) low
    Timer.delay(0.01);  // Wait 10ms
    m_reset_out.close();
    m_reset_in = new DigitalInput(27);  // Set SPI CS2 (IMU RST) high
    Timer.delay(0.5); // Wait 500ms for reset to complete

    m_spi = new SPI(port);
    m_spi.setClockRate(1000000);
    m_spi.setMSBFirst();
    m_spi.setSampleDataOnTrailingEdge();
    m_spi.setClockActiveLow();
    m_spi.setChipSelectActiveLow();
    
    readRegister(kRegPROD_ID); // dummy read
    
    // Validate the product ID
    if (readRegister(kRegPROD_ID) != 16982) {
      m_spi.close();
      m_spi = null;
      m_samples = null;
      m_samples_mutex = null;
      m_samples_not_empty = null;
      DriverStation.reportError("could not find ADIS16470", false);
      return;
    }

    // Set IMU internal decimation to 200 SPS
    writeRegister(kRegDEC_RATE, 0x0009);

    // Enable Data Ready (LOW = Good Data), gSense Compensation, PoP
    writeRegister(kRegMSC_CTRL, 0x00C1);

    // Configure IMU internal Bartlett filter
    writeRegister(kRegFILT_CTRL, 0x0000);

    // Create data acq FIFO.  We make the FIFO 2 longer than it needs
    // to be so the input and output never overlap (we hold a reference
    // to the output while the lock is released).
    m_samples_mutex = new ReentrantLock();
    m_samples_not_empty = m_samples_mutex.newCondition();
    
    m_samples = new Sample[kSamplesDepth + 2];
    for (int i=0; i<kSamplesDepth + 2; i++) {
      m_samples[i] = new Sample();
    }
    
    // Configure interrupt on SPI CS1
    m_interrupt = new DigitalInput(26);

    // Configure DMA SPI
    m_spi.initAuto(8200);
    m_spi.setAutoTransmitData(new byte[]{kGLOB_CMD},21);

    // Kick off DMA SPI (Note: Device configration impossible after SPI DMA is activated)
    m_spi.startAutoTrigger(m_interrupt,true,false);
    
    m_freed.set(false);
    m_acquire_task = new Thread(new AcquireTask(this));
    m_acquire_task.setDaemon(true);
    m_acquire_task.start();
    
    // Execute offset calibration on start-up
    calibrate();

    Timer.delay(0.500); 

    // Re-initialize accumulations after calibration
    reset();
    
    // Let the user know the IMU was initiallized successfully
    DriverStation.reportWarning("ADIS16470 IMU Successfully Initialized!", false);

    // Drive "Ready" LED low
    m_status_led = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low

    // Report usage and post data to DS
    HAL.report(tResourceType.kResourceType_ADIS16470, 0);
    setName("ADIS16470", 0);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void calibrate() {
    if (m_spi == null) return;

    Timer.delay(0.1);

    synchronized (this) {
      m_accum_count = 0;
      m_accum_gyro_x = 0.0;
      m_accum_gyro_y = 0.0;
      m_accum_gyro_z = 0.0;
    }

    Timer.delay(kCalibrationSampleTime);

    synchronized (this) {
      m_gyro_offset_x = m_accum_gyro_x / m_accum_count;
      m_gyro_offset_y = m_accum_gyro_y / m_accum_count;
      m_gyro_offset_z = m_accum_gyro_z / m_accum_count;
    }
  }

  static int ToUShort(ByteBuffer buf) {
	  return (buf.getShort(0)) & 0xFFFF;
  }
  static int ToUShort(int... data) {
	  ByteBuffer buf = ByteBuffer.allocateDirect(data.length);
	  for(int d : data) {
		  buf.put((byte)d);
	  }
	  return ToUShort(buf);
  }
  
  public static long ToULong(int sint) {
		return sint & 0x00000000FFFFFFFFL;
	}

  private static int ToShort(int... buf) {
      return (short)(((short)buf[0]) << 8 | buf[1]);
  }
  static int ToShort(ByteBuffer buf) {
	  return ToShort(buf.get(0), buf.get(1));
  }
  
  private int readRegister(int reg) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    buf.order(ByteOrder.BIG_ENDIAN);
    buf.put(0, (byte) (reg & 0x7f));
    buf.put(1, (byte) 0);

    m_spi.write(buf, 2);
    m_spi.read(false, buf, 2);

    return ToUShort(buf);
  }

  private void writeRegister(int reg, int val) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    // low byte
    buf.put(0, (byte)((0x80 | reg) | 0x10));
    buf.put(1, (byte) (val & 0xff));
    m_spi.write(buf, 2);
    // high byte
    buf.put(0, (byte) (0x81 | reg));
    buf.put(1, (byte) (val >> 8));
    m_spi.write(buf, 2);
  }

  /**
   * {@inheritDoc}
   */
  public void reset() {
    synchronized (this) {
      m_integ_gyro_x = 0.0;
      m_integ_gyro_y = 0.0;
      m_integ_gyro_z = 0.0;
    }
  }

  /**
   * Delete (free) the spi port used for the IMU.
   */
  @Override
  public void close() {
    m_freed.set(true);
    if (m_samples_mutex != null) {
      m_samples_mutex.lock();
      try {
        m_samples_not_empty.signal();
      } finally {
        m_samples_mutex.unlock();
      }
    }
    try {
      if (m_acquire_task != null) {
        m_acquire_task.join();
      }
    } catch (InterruptedException e) {
    }
    if (m_interrupt != null) {
      m_interrupt.close();
      m_interrupt = null;
    }
    if (m_spi != null) {
      m_spi.close();
      m_spi = null;
    }
  }

  private void acquire() {
    ByteBuffer readBuf = ByteBuffer.allocateDirect(64000);
    readBuf.order(ByteOrder.LITTLE_ENDIAN);
    double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp, status, counter;
    int data_count = 0;
    int array_offset = 0;
    int imu_checksum = 0;
    double dt = 0; // This number must be adjusted if decimation setting is changed. Default is 1/102.4 SPS
    int data_subset[] = new int[23];
    long timestamp_new = 0;
    int data_to_read = 0;
    
    while (!m_freed.get()) {
      // Waiting for the buffer to fill...
  	  Timer.delay(.020); // A delay less than 10ms could potentially overflow the local buffer

  	  data_count = m_spi.readAutoReceivedData(readBuf,0,0); // Read number of bytes currently stored in the buffer
  	  array_offset = data_count % 92; // Look for "extra" data This is 92 not 23 like in C++ b/c everything is 32-bits and takes up 4 bytes in the buffer
  	  data_to_read = data_count - array_offset; // Discard "extra" data
      m_spi.readAutoReceivedData(readBuf,data_to_read,0); // Read data from DMA buffer
  	  for(int i = 0; i < data_to_read; i += 92) { // Process each set of 23 bytes (timestamp + 23 data) * 4 (32-bit ints)
		    for(int j = 1; j < 24; j++) { // Split each set of 23 bytes into a sub-array for processing
          int at = (i + 4 * (j));
          data_subset[j - 1] = readBuf.getInt(at);
        }
        
        // Calculate checksum
        int calc_checksum = 0;
        for(int k = 2; k < 20; k++ ) { // Cycle through STATUS, XYZ GYRO, XYZ ACCEL, TEMP, COUNTER (Ignore DUT Checksum)
          calc_checksum += data_subset[k];
        }

        // This is the data needed for Checksum
        ByteBuffer bBuf = ByteBuffer.allocateDirect(2);
        bBuf.put((byte)data_subset[20]); 
        bBuf.put((byte)data_subset[21]);
        //System.out.println(data_subset[20] + "," + data_subset[21]);
        imu_checksum = ToUShort(bBuf);

        //System.out.println("IMU Checksum: " + imu_checksum);

        // Compare calculated vs read CRC. Don't update outputs if CRC-16 is bad
        if(calc_checksum == imu_checksum) {
          // Calculate delta-time (dt) using FPGA timestamps
          timestamp_new = ToULong(readBuf.getInt(i * 4));
          dt = (timestamp_new - timestamp_old)/1000000.0; // Calculate dt and convert us to seconds
          timestamp_old = timestamp_new; // Store new timestamp in old variable for next cycle

          gyro_x = ToShort(data_subset[4], data_subset[5]) * kDegreePerSecondPerLSB;
          gyro_y = ToShort(data_subset[6], data_subset[7]) * kDegreePerSecondPerLSB;
          gyro_z = ToShort(data_subset[8], data_subset[9]) * kDegreePerSecondPerLSB;
          accel_x = ToShort(data_subset[10], data_subset[11]) * kGPerLSB;
          accel_y = ToShort(data_subset[12], data_subset[13]) * kGPerLSB;
          accel_z = ToShort(data_subset[14], data_subset[15]) * kGPerLSB;
          temp = ToShort(data_subset[16], data_subset[17]) * kDegCPerLSB + kDegCOffset;
          status = ToUShort(data_subset[2], data_subset[3]);
          counter = ToUShort(data_subset[18], data_subset[19]);        

          // Update global state
          synchronized(this){
            m_gyro_x = gyro_x;
            m_gyro_y = gyro_y;
            m_gyro_z = gyro_z;
            m_accel_x = accel_x;
            m_accel_y = accel_y;
            m_accel_z = accel_z;
            m_temp = temp;
            m_status = status;
            m_counter = counter;
            this.dt = dt;

            ++m_accum_count;
            m_accum_gyro_x += gyro_x;
            m_accum_gyro_y += gyro_y;
            m_accum_gyro_z += gyro_z;

            m_integ_gyro_x += (gyro_x - m_gyro_offset_x) * dt;
            m_integ_gyro_y += (gyro_y - m_gyro_offset_y) * dt;
            m_integ_gyro_z += (gyro_z - m_gyro_offset_z) * dt;
            
          }
        }else{
          System.out.println("Invalid checksum");
        }
	    }
    }
  }

  /**
   * {@inheritDoc}
   */
  public double getAngle() {
    switch (m_yaw_axis) {
      case kX: 
        return getAngleX();
      case kY: 
        return getAngleY();
      case kZ: 
        return getAngleZ();
    }
    return 0.0;
  }

  /**
   * {@inheritDoc}
   */
  public double getRate() {
    switch (m_yaw_axis) {
      case kX: 
        return getRateX();
      case kY: 
        return getRateY();
      case kZ: 
        return getRateZ();
    }
    return 0.0;
  }

  public synchronized double getAngleX() {
    return m_integ_gyro_x;
  }

  public synchronized double getAngleY() {
    return m_integ_gyro_y;
  }

  public synchronized double getAngleZ() {
    return m_integ_gyro_z;
  }

  public synchronized double getRateX() {
    return m_gyro_x;
  }

  public synchronized double getRateY() {
    return m_gyro_y;
  }

  public synchronized double getRateZ() {
    return m_gyro_z;
  }

  public synchronized double getAccelX() {
    return m_accel_x;
  }

  public synchronized double getAccelY() {
    return m_accel_y;
  }

  public synchronized double getAccelZ() {
    return m_accel_z;
  }

  public synchronized double getTemperature() {
    return m_temp;
  }

  public synchronized double getStatus() {
    return m_status;
  }

  public synchronized double getCounter(){
    return m_counter;
  }

  public synchronized double getdt(){
    return dt;
  }
  
  /**
   * {@inheritDoc}
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ADIS16470 IMU");
    NetworkTableInstance tb = builder.getEntry("GyroX").getInstance();
    int gyroX = builder.getEntry("GyroX").getHandle();
    int gyroY = builder.getEntry("GyroY").getHandle();
    int gyroZ = builder.getEntry("GyroZ").getHandle();
    int accelX = builder.getEntry("AccelX").getHandle();
    int accelY = builder.getEntry("AccelY").getHandle();
    int accelZ = builder.getEntry("AccelZ").getHandle();
    int angleX = builder.getEntry("AngleX").getHandle();
    int angleY = builder.getEntry("AngleY").getHandle();
    int angleZ = builder.getEntry("AngleZ").getHandle();
    int temp = builder.getEntry("Temperature").getHandle();
    int dt = builder.getEntry("dt").getHandle();
    int status = builder.getEntry("Status").getHandle();
    int counter = builder.getEntry("Counter").getHandle();
    builder.setUpdateTable(new Runnable(){
      @Override
      public void run(){
          new NetworkTableEntry(tb, gyroX).setDouble(getRateX());
          new NetworkTableEntry(tb, gyroY).setDouble(getRateY());
          new NetworkTableEntry(tb, gyroZ).setDouble(getRateZ());
          new NetworkTableEntry(tb, accelX).setDouble(getAccelX());
          new NetworkTableEntry(tb, accelY).setDouble(getAccelY());
          new NetworkTableEntry(tb, accelZ).setDouble(getAccelZ());
          new NetworkTableEntry(tb, angleX).setDouble(getAngleX());
          new NetworkTableEntry(tb, angleY).setDouble(getAngleY());
          new NetworkTableEntry(tb, angleZ).setDouble(getAngleZ());
          new NetworkTableEntry(tb, temp).setDouble(getTemperature());
          new NetworkTableEntry(tb, dt).setDouble(getdt());
          new NetworkTableEntry(tb, status).setDouble(getStatus());
          new NetworkTableEntry(tb, counter).setDouble(getCounter());
      }
    });
  }  
}
