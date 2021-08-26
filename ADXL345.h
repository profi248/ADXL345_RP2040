/**************************************************************************/
/*!
    @file     ADXL345.h
    @author   K. Townsend (Adafruit Industries)
    @author   profi248
    
    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.
    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
    
    @section license License
    BSD (see license.txt)
*/
/**************************************************************************/

#ifndef ADXL345_h
#define ADXL345_h

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADXL345_DEFAULT_ADDRESS (0x53) ///< Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define ADXL345_REG_DEVID (0x00) ///< Device ID
#define ADXL345_REG_THRESH_TAP (0x1D) ///< Tap threshold
#define ADXL345_REG_OFSX (0x1E) ///< X-axis offset
#define ADXL345_REG_OFSY (0x1F) ///< Y-axis offset
#define ADXL345_REG_OFSZ (0x20) ///< Z-axis offset
#define ADXL345_REG_DUR (0x21) ///< Tap duration
#define ADXL345_REG_LATENT (0x22) ///< Tap latency
#define ADXL345_REG_WINDOW (0x23) ///< Tap window
#define ADXL345_REG_THRESH_ACT (0x24) ///< Activity threshold
#define ADXL345_REG_THRESH_INACT (0x25) ///< Inactivity threshold
#define ADXL345_REG_TIME_INACT (0x26) ///< Inactivity time
#define ADXL345_REG_ACT_INACT_CTL \
    (0x27) ///< Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF (0x28) ///< Free-fall threshold
#define ADXL345_REG_TIME_FF (0x29) ///< Free-fall time
#define ADXL345_REG_TAP_AXES (0x2A) ///< Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS (0x2B) ///< Source for single/double tap
#define ADXL345_REG_BW_RATE (0x2C) ///< Data rate and power mode control
#define ADXL345_REG_POWER_CTL (0x2D) ///< Power-saving features control
#define ADXL345_REG_INT_ENABLE (0x2E) ///< Interrupt enable control
#define ADXL345_REG_INT_MAP (0x2F) ///< Interrupt mapping control
#define ADXL345_REG_INT_SOURCE (0x30) ///< Source of interrupts
#define ADXL345_REG_DATA_FORMAT (0x31) ///< Data format control
#define ADXL345_REG_DATAX0 (0x32) ///< X-axis data 0
#define ADXL345_REG_DATAX1 (0x33) ///< X-axis data 1
#define ADXL345_REG_DATAY0 (0x34) ///< Y-axis data 0
#define ADXL345_REG_DATAY1 (0x35) ///< Y-axis data 1
#define ADXL345_REG_DATAZ0 (0x36) ///< Z-axis data 0
#define ADXL345_REG_DATAZ1 (0x37) ///< Z-axis data 1
#define ADXL345_REG_FIFO_CTL (0x38) ///< FIFO control
#define ADXL345_REG_FIFO_STATUS (0x39) ///< FIFO status
/*=========================================================================*/

/*=========================================================================*/

/*=========================================================================
    CONSTANTS
    -----------------------------------------------------------------------*/
#define ADXL345_MG2G_MULTIPLIER (0.004) ///< 4mg per lsb
/*=========================================================================*/

/**
 * @brief Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth
*/
typedef enum {
    ADXL345_DATARATE_3200_HZ = 0b1111,   ///< 1600Hz Bandwidth   140µA IDD
    ADXL345_DATARATE_1600_HZ = 0b1110,   ///<  800Hz Bandwidth    90µA IDD
    ADXL345_DATARATE_800_HZ  = 0b1101,   ///<  400Hz Bandwidth   140µA IDD
    ADXL345_DATARATE_400_HZ  = 0b1100,   ///<  200Hz Bandwidth   140µA IDD
    ADXL345_DATARATE_200_HZ  = 0b1011,   ///<  100Hz Bandwidth   140µA IDD
    ADXL345_DATARATE_100_HZ  = 0b1010,   ///<   50Hz Bandwidth   140µA IDD
    ADXL345_DATARATE_50_HZ   = 0b1001,   ///<   25Hz Bandwidth    90µA IDD
    ADXL345_DATARATE_25_HZ   = 0b1000,   ///< 12.5Hz Bandwidth    60µA IDD
    ADXL345_DATARATE_12_5_HZ = 0b0111,   ///< 6.25Hz Bandwidth    50µA IDD
    ADXL345_DATARATE_6_25HZ  = 0b0110,   ///< 3.13Hz Bandwidth    45µA IDD
    ADXL345_DATARATE_3_13_HZ = 0b0101,   ///< 1.56Hz Bandwidth    40µA IDD
    ADXL345_DATARATE_1_56_HZ = 0b0100,   ///< 0.78Hz Bandwidth    34µA IDD
    ADXL345_DATARATE_0_78_HZ = 0b0011,   ///< 0.39Hz Bandwidth    23µA IDD
    ADXL345_DATARATE_0_39_HZ = 0b0010,   ///< 0.20Hz Bandwidth    23µA IDD
    ADXL345_DATARATE_0_20_HZ = 0b0001,   ///< 0.10Hz Bandwidth    23µA IDD
    ADXL345_DATARATE_0_10_HZ = 0b0000    ///< 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;

/**
 * @brief Used to denote types of interrupts
*/
typedef enum {
    ADXL345_INT_DATA_READY = 0b10000000, ///< Data-ready
    ADXL345_INT_SINGLE_TAP = 0b01000000, ///< Single tap
    ADXL345_INT_DOUBLE_TAP = 0b00100000, ///< Double tap
    ADXL345_INT_ACTIVITY   = 0b00010000, ///< Activity
    ADXL345_INT_INACTIVITY = 0b00001000, ///< Inactivity
    ADXL345_INT_FREE_FALL  = 0b00000100, ///< Free fall
    ADXL345_INT_WATERMARK  = 0b00000010, ///< Watermark
    ADXL345_INT_OVERRUN    = 0b00000001  ///< Overrun
} interrupt_t;

/**
 * @brief  Used to denote interrupt pins
 */
typedef enum {
    ADXL345_INT1 = 0b0, ///< Interrupt pin 1
    ADXL345_INT2 = 0b1  ///< Interrupt pin 2
} interruptPin_t;

/**
 * @brief  Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range
 */
typedef enum {
    ADXL345_RANGE_16_G = 0b11,  ///< +/- 16g
    ADXL345_RANGE_8_G  = 0b10,  ///< +/- 8g
    ADXL345_RANGE_4_G  = 0b01,  ///< +/- 4g
    ADXL345_RANGE_2_G  = 0b00   ///< +/- 2g (default value)
} range_t;



// ported from Adafruit unified library
#define SENSORS_GRAVITY_EARTH (9.80665F)
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)

typedef struct {
    float x;
    float y;
    float z;
} sensors_vec_t;

typedef struct {
    int32_t       version;
    int32_t       sensor_id;
    int32_t       type;
    int32_t       reserved0;
    int32_t       timestamp;
    sensors_vec_t acceleration;
} sensors_event_t;

typedef struct {
    char    name[12];
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    float   max_value;
    float   min_value;
    float   resolution;
    int32_t min_delay;
} sensor_t;

typedef enum {
    SENSOR_TYPE_ACCELEROMETER = (1)
} sensors_type_t;

// I2CDevice drop-in replacement
class I2CDevice {
    uint8_t     m_addr;
    i2c_inst_t* m_i2c_bus;
    uint8_t     m_sda_pin;
    uint8_t     m_scl_pin;

public:
    I2CDevice(uint8_t addr, i2c_inst_t* i2c_bus, uint8_t sda_pin, uint8_t scl_pin) 
        : m_addr(addr),
         m_i2c_bus(i2c_bus),
         m_sda_pin(sda_pin),
         m_scl_pin(scl_pin) {};

    bool begin();
    void read(uint8_t* buf, size_t len);
    void write(const uint8_t* buf, size_t len);
};

/**
 * @brief Class to interact with the ADXL345 accelerometer
 *
 */
class ADXL345 {
public:
    ADXL345(int32_t sensorID = -1);
    bool begin(uint8_t addr = ADXL345_DEFAULT_ADDRESS,
        i2c_inst_t* i2c_bus = i2c_default,
        uint8_t sda_pin = PICO_DEFAULT_I2C_SDA_PIN,
        uint8_t scl_pin = PICO_DEFAULT_I2C_SCL_PIN);
    void setRange(range_t range);
    range_t getRange(void);
    void setDataRate(dataRate_t dataRate);
    dataRate_t getDataRate(void);
    bool getEvent(sensors_event_t*);
    void getSensor(sensor_t*);
    void setInterrupt(interrupt_t interrupt, bool state);
    void setInterruptMap(interrupt_t interrupt, interruptPin_t pin);
    uint8_t getInterruptSources(void);
    bool isInterruptSource(interrupt_t interrupt, uint8_t sources_bitmap);

    void setFreefallInterruptTreshold(uint16_t threshold_mg);
    void setFreefallInterruptTime(uint16_t time_ms);

    uint8_t getDeviceID(void);
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    int16_t read16(uint8_t reg);

    int16_t getX(void), getY(void), getZ(void);

private:
    I2CDevice* i2c_dev = nullptr; ///< Pointer to I2C bus interface

    int32_t _sensorID;
    range_t _range;
};

#endif // ADXL345_h
