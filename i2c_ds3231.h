// Copyright Jim Merkle, 11/19/2024
// File: i2c_ds3231.h
// I2C and DS3231 routines
//
// Notes:
//
#ifndef _i2c_ds3231_h_
#define _i2c_ds3231_h_


#ifdef __cplusplus
extern "C" {
#endif

#define I2C_ADDRESS_MIN	0x03
#define I2C_ADDRESS_MAX 0x77


#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000                                              \
  946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

//=================================================================================================
// I2C helper functions
//=================================================================================================
int i2c_validate_address(uint16_t i2c_address);
int i2c_write_read(uint8_t i2c_address, uint8_t * pwrite, uint8_t wr_count, uint8_t * pread, uint8_t rd_count);

//=================================================================================================
// DS3231 constants, functions
//=================================================================================================
/** Constants */
#define I2C_ADDRESS_DS3231  0x68

#ifndef DATE_TIME_
// Structure to hold/record time read from to written to an external RTC module (DS3231)
typedef struct {
	uint8_t yOff; ///< Year offset from 2000
	uint8_t m;    ///< Month 1-12
	uint8_t d;    ///< Day 1-31
	uint8_t hh;   ///< Hours 0-23
	uint8_t mm;   ///< Minutes 0-59
	uint8_t ss;   ///< Seconds 0-59
}DATE_TIME;
#define DATE_TIME_
#endif

void init_ds3231(void);
void write_ds3231(const DATE_TIME * dt);
void read_ds3231(DATE_TIME * dt);
uint8_t ds3231_read_status(void);
uint8_t ds3231_read_osf(void);
void ds3231_clearOSF(void);

//=================================================================================================
// AT24C32 support defines & functions
//=================================================================================================
#define AT24C32_SIZE_BYTES        4096
#define AT24C32_PAGE_SIZE_BYTES   32
//#define AT24C32_I2C_ADDRESS       0x57 // 7-bit address
int read_at24c32(uint8_t device_i2c_address, uint16_t memory_address, uint8_t *buffer, uint16_t count);
int fill_at24c32(uint8_t device_i2c_address, uint8_t fill_data_byte);
int write_byte_at24c32(uint8_t device_i2c_address, uint16_t memory_address, uint8_t data_byte);
int at24c32_is_busy(uint8_t device_i2c_address);
int cl_dump_at24c32(void);
int cl_fill_at24c32(void);
int cl_write_at24c32(void);

//=================================================================================================
// Command line functions
//=================================================================================================
int cl_i2c_scan(void);
int cl_time(void);
int cl_date(void);
int cl_ds3231_dump(void);
int cl_sqw_test(void);
int cl_alarm(void);
int cl_i2c_read_write(void);

//=================================================================================================
// TM1637 functions
//=================================================================================================
void init_tm1637(void);
void update_clock(void);

#ifdef __cplusplus
}
#endif

#endif // _i2c_ds3231_h_
