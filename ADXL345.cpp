/**************************************************************************/
/*!
    @file     ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @author   profi248
*/
/**************************************************************************/

#include <cmath>
#include <cstring>

#include "ADXL345.h"

/**************************************************************************/
/*!
    @brief  Initializes I2C device
*/
/**************************************************************************/
bool I2CDevice::begin()
{
    i2c_init(m_i2c_bus, 100 * 1000);
    gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(m_sda_pin);
    gpio_pull_up(m_scl_pin);

    return true;
}

/**************************************************************************/
/*!
    @brief Reads a buffer from a I2C bus
    @param buffer The buffer to read into
    @param len The number of bytes to read 
*/
/**************************************************************************/
void I2CDevice::read(uint8_t* buf, size_t len)
{
    i2c_read_blocking(m_i2c_bus, m_addr, buf, len, true);
}

/**************************************************************************/
/*!
    @brief Writes a buffer to a I2C bus
    @param buffer The buffer to write to
    @param len The number of bytes to write
*/
/**************************************************************************/
void I2CDevice::write(const uint8_t* buf, size_t len)
{
    i2c_write_blocking(m_i2c_bus, m_addr, buf, len, true);
}

/**************************************************************************/
/*!
    @brief  Writes one byte to the specified destination register
    @param reg The address of the register to write to
    @param value The value to set the register to
*/
/**************************************************************************/
void ADXL345::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = { reg, value };
    i2c_dev->write(buffer, 2);
}

/**************************************************************************/
/*!
    @brief Reads one byte from the specified register
    @param reg The address of the register to read from
    @returns The single byte value of the requested register
*/
/**************************************************************************/
uint8_t ADXL345::readRegister(uint8_t reg)
{
    uint8_t buffer[1] = { i2c_dev ? reg : reg | 0x80 };
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 1);
    return buffer[0];
}

/**************************************************************************/
/*!
    @brief Reads two bytes from the specified register
    @param reg The address of the register to read from
    @return The two bytes read from the sensor starting at the given address
*/
/**************************************************************************/
int16_t ADXL345::read16(uint8_t reg)
{
    uint8_t buffer[2] = { i2c_dev ? reg : reg | 0x80 | 0x40, 0 };
    i2c_dev->write(buffer, 1);
    i2c_dev->read(buffer, 2);
    return uint16_t(buffer[1]) << 8 | uint16_t(buffer[0]);
}

/**************************************************************************/
/*!
    @brief  Reads the device ID (can be used to check connection)
    @return The Device ID of the connected sensor
*/
/**************************************************************************/
uint8_t ADXL345::getDeviceID(void)
{
    // Check device ID register
    return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
    @return The raw `int16_t` unscaled x-axis acceleration value
*/
/**************************************************************************/
int16_t ADXL345::getX(void)
{
    return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
    @return The raw `int16_t` unscaled y-axis acceleration value
*/
/**************************************************************************/
int16_t ADXL345::getY(void)
{
    return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
    @return The raw `int16_t` unscaled z-axis acceleration value
*/
/**************************************************************************/
int16_t ADXL345::getZ(void)
{
    return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
    @param sensorID A unique ID to use to differentiate the sensor from others
*/
/**************************************************************************/
ADXL345::ADXL345(int32_t sensorID)
{
    _sensorID = sensorID;
    _range = ADXL345_RANGE_2_G;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param i2caddr The I2C address to begin communication with
    @return true: success false: a sensor with the correct ID was not found
*/
/**************************************************************************/
bool ADXL345::begin(uint8_t i2caddr, i2c_inst_t* i2c_bus, uint8_t sda_pin, uint8_t scl_pin)
{

    i2c_dev = new I2CDevice(i2caddr, i2c_bus, sda_pin, scl_pin);
    if (!i2c_dev->begin())
        return false;

    /* Check connection */
    uint8_t deviceid = getDeviceID();
    if (deviceid != 0xE5) {
        /* No ADXL345 detected ... return false */
        return false;
    }

    // Enable measurements
    writeRegister(ADXL345_REG_POWER_CTL, 0x08);

    return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
    @param range The new `range_t` to set the accelerometer to
*/
/**************************************************************************/
void ADXL345::setRange(range_t range)
{
    /* Read the data format register to preserve bits */
    uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

    /* Update the data rate */
    format &= ~0x0F;
    format |= range;

    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(ADXL345_REG_DATA_FORMAT, format);

    /* Keep track of the current range (to avoid readbacks) */
    _range = range;
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
    @return The current `range_t` value
*/
/**************************************************************************/
range_t ADXL345::getRange(void)
{
    /* Read the data format register to preserve bits */
    return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
    @param dataRate The `dataRate_t` to set
*/
/**************************************************************************/
void ADXL345::setDataRate(dataRate_t dataRate)
{
    /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
    writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
    @return The current data rate
*/
/**************************************************************************/
dataRate_t ADXL345::getDataRate(void)
{
    return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param event Pointer to the event object to fill
    @return true: success
*/
/**************************************************************************/
bool ADXL345::getEvent(sensors_event_t* event)
{
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp = 0;
    event->acceleration.x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    event->acceleration.y = getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    event->acceleration.z = getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

    return true;
}

/**************************************************************************/
/*!
    @brief Set interrupt enabled/disabled
    @param interrupt Interrupt type
    @param enable true to enable, false to disable
*/
/**************************************************************************/
void ADXL345::setInterrupt(interrupt_t interrupt, bool state)
{
    uint8_t reg = readRegister(ADXL345_REG_INT_ENABLE);

    if (state)
        reg |= interrupt;
    else
        reg &= ~interrupt;

    writeRegister(ADXL345_REG_INT_ENABLE, reg);
}

/**************************************************************************/
/*!
    @brief Set interrupt pin to either INT1 or INT2
    @param interrupt Interrupt type
    @param pin Interrupt pin
*/
/**************************************************************************/
void ADXL345::setInterruptMap(interrupt_t interrupt, interruptPin_t pin)
{
    uint8_t reg = readRegister(ADXL345_REG_INT_MAP);

    if (pin == ADXL345_INT2)
        reg |= interrupt;
    else
        reg &= ~interrupt;

    writeRegister(ADXL345_REG_INT_MAP, reg);
}

/**************************************************************************/
/*!
    @brief Get interrupt source(s) and clear latched motion detection intrrupts
    @return Bitmap with triggered interrupts set to 1
 */
/**************************************************************************/

uint8_t ADXL345::getInterruptSources()
{
    return readRegister(ADXL345_REG_INT_SOURCE);
}

/**************************************************************************/
/*!
    @brief Is given interrupt a source?
    @param interrupt Interrupt type
    @param sources_bitmap Interrupt sources bitmap
    @return true if interrupt is a source, false otherwise
 */
/**************************************************************************/
bool ADXL345::isInterruptSource(interrupt_t interrupt, uint8_t sources_bitmap)
{
    return (sources_bitmap & interrupt) != 0;
}

/**************************************************************************/
/*!
    @brief Set force treshold for triggering free fall detection
    @param threshold_mg threshold value in mg
    @note Threshold is rounded to nearest multiple of 62.5 mg
    @note Threshold is clamped to a maximum of 15937.5 mg
*/
/**************************************************************************/
void ADXL345::setFreefallInterruptTreshold(uint16_t threshold_mg)
{
    if (threshold_mg < 0)
        threshold_mg = 0;

    if (threshold_mg > 15937.5)
        threshold_mg = 15937.5;

    uint8_t threshold = (uint8_t) round(threshold_mg / 62.5);
    writeRegister(ADXL345_REG_THRESH_FF, threshold);
}

/**************************************************************************/
/*!
    @brief Set duration required to trigger free fall interrupt
    @param time_ms duration in ms
    @note Duration is rounded to the nearest multiple of 5ms
    @note Duration is clamped to a maximum of 1275ms
*/
/**************************************************************************/
void ADXL345::setFreefallInterruptTime(uint16_t time_ms)
{
    if (time_ms > 1275)
        time_ms = 1275;

    uint8_t time = (uint8_t) round(time_ms / 5);
    writeRegister(ADXL345_REG_TIME_FF, time);
}

/**************************************************************************/
/*!
    @brief Fill a `sensor_t` struct with information about the sensor

    @param sensor Pointer to a `sensor_t` struct to fill
*/
/**************************************************************************/
void ADXL345::getSensor(sensor_t* sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "ADXL345", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->max_value = -156.9064F; /* -16g = 156.9064 m/s^2  */
    sensor->min_value = 156.9064F; /*  16g = 156.9064 m/s^2  */
    sensor->resolution = 0.03923F; /*  4mg = 0.0392266 m/s^2 */
}
