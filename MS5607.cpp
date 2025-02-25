#include "MS5607.h"

#include <math.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

void MS5607::init() {
  i2c_init(i2c0, 100 * 1000);

  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);

  read_prom();
}

int MS5607::reset() {
  uint8_t cmd = MS5607_RESET_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;
  else
    read_prom();
  return 0;
}

int MS5607::read_prom() {
  uint8_t cmd = MS5607_READ_C1_CMD;
  uint8_t buf[2] = {0, 0};

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c1 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C2_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c2 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C3_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c3 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C4_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c4 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C5_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c5 = ((uint16_t)buf[0] << 8) | buf[1];

  cmd = MS5607_READ_C6_CMD;

  if (i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false) ==
      PICO_ERROR_GENERIC)
    return 1;

  if (i2c_read_blocking(i2c, MS5607_ADDR, buf, 2, false) == PICO_ERROR_GENERIC)
    return 1;

  c6 = ((uint16_t)buf[0] << 8) | buf[1];

  return 0;
}

int MS5607::conversion() {
  uint8_t cmd = MS5607_PRESS_CONV_CMD;
  uint8_t buf[3] = {0, 0, 0};

  i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false);
  i2c_read_blocking(i2c, MS5607_ADDR, &buf[0], 3, false);

  d1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  cmd = MS5607_TEMP_CONV_CMD;

  i2c_write_blocking(i2c, MS5607_ADDR, &cmd, 1, false);
  i2c_read_blocking(i2c, MS5607_ADDR, &buf[0], 3, false);

  d2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

  return 0;
}

void MS5607::get_press_temp(uint32_t *pressure, uint32_t *temperature) {
  int32_t press = 0;
  int32_t temp = 0;
  int32_t t2 = 0;
  int64_t off2 = 0;
  int64_t sens2 = 0;

  // Step 1: Read calibration data from PROM
  // Already did this in the init()

  // Step 2: Read pressure and temperature from the MS5607
  conversion();

  // The rest of this function mostly looks like random math, it is actually
  // the compensation calculations outline in the datasheet for the device!

  // Step 3: Calculate temperature
  dT = d2 - ((int64_t)c5 << 8);

  // temp is e.g. 2000 = 20.00 deg C
  temp = (int64_t)2000 + ((dT * (int64_t)c6) >> 23);

  // Step 4: Calculate temperature compensated pressure
  off = ((int64_t)c2 * (int64_t)131072) + (((int64_t)c4 * dT) / (int64_t)64);
  sens = ((int64_t)c1 * (int64_t)65536) + (((int64_t)c3 * dT) / (int64_t)128);

  // Second order compensation
  if (temp < (int)2000) {

    t2 = ((dT * dT) / ((int64_t)2147483648));
    off2 = (61 * ((temp - 2000) * (temp - 2000)) / 16);
    sens2 = (2 * ((temp - 2000) ^ 2));

    if (temp < -15) {

      off2 = (off2 + (15 * ((temp + 1500) * (temp + 1500))));
      sens2 = (sens2 + (8 * ((temp + 1500) * (temp + 1500))));
    }

    temp = temp - t2;
    off = off - off2;
    sens = sens - sens2;
  }

  press = (((d1 * sens) / (2097152)) - off) / (32768);

  // in "centi-celsius" e.g. 2000 = 20.00 deg C
  *temperature = temp;

  // in "centi-millibar" e.g. 110002 = 1100.02 mbar
  *pressure = press;
}
