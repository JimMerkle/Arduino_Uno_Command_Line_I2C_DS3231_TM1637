// Copyright Jim Merkle, 11/19/2024
// File: i2c_ds3231.cpp
// I2C and DS3231 routines
//
// Notes:
//
#include <arduino.h>
#include <Wire.h>
#include "command_line.h"
#include "i2c_ds3231.h"
#include <TM1637Display.h>
#include "RTClib.h" // use local RTClib library

// Command line method to perform a scan of the I2C bus
int cl_i2c_scan(void)
{
  printf("I2C Scan - scanning I2C addresses 0x%02X - 0x%02X\n",I2C_ADDRESS_MIN,I2C_ADDRESS_MAX);
  // Display Hex Header
  printf("    "); for(int i=0;i<=0x0F;i++) printf(" %0X ",i);
  // Walk through address range 0x00 - 0x77, but only test 0x03 - 0x77
  for(uint8_t addr=0;addr<=I2C_ADDRESS_MAX;addr++) {
    // If address defines the beginning of a row, start a new row and display row text
    if(!(addr%16)) printf("\n%02X: ",addr);
    // Check I2C addresses in the range 0x03-0x7F
    if(addr < I2C_ADDRESS_MIN || addr > I2C_ADDRESS_MAX) {
      printf("   "); // out of range
      continue;
    }
    // Perform I2C device detection
    // The i2c_scanner uses the return value of the Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if(0 == error)
      printf("%02X ",addr);
    else
      printf("-- ");
  } // for-loop
  printf("\n");
  return 0;
}


// I2C helper function that validates I2C address is within range
// If I2C address is within range, return 0, else display error and return -1.
int i2c_validate_address(uint16_t i2c_address)
{
	if(i2c_address < I2C_ADDRESS_MIN || i2c_address > I2C_ADDRESS_MAX) {
		printf("Address out of range. Expect 0x%02X to 0x%02X\n",I2C_ADDRESS_MIN,I2C_ADDRESS_MAX); // out of range
		return -1;
	}
	return 0; // success
}

// I2C helper function that begins by writing zero or more bytes, followed by reading zero or more bytes.
// Return 0 for success
int i2c_write_read(uint8_t i2c_address, uint8_t * pwrite, uint8_t wr_count, uint8_t * pread, uint8_t rd_count)
{
	// Validate I2C address
	int rc=i2c_validate_address(i2c_address);
	if(rc) return rc;

	// If there are bytes to write, write them
	if(pwrite && wr_count) {
    //printf("%s Writing\n",__func__);
		Wire.beginTransmission(i2c_address);
    Wire.write(pwrite, wr_count);
    Wire.endTransmission(); // send stop condition
	} // if bytes to write

	// If there are bytes to read, read them
	if(pread && rd_count) {
    //printf("%s Reading\n",__func__);
		Wire.requestFrom(i2c_address, rd_count, (uint8_t)1); // send stop condition
    while(rd_count) {
      if(Wire.available()) {
        *pread = Wire.read(); // read I2C byte to pointer accress
        pread++;    // update pointer
        rd_count--; // decrement counter
      }
    } // while(rd_count)
	} // if bytes to read
	return 0;
}

/*====================================================================================================
| DS3231 Index Registers (See DS3231.pdf, Figure 1, Timekeeping Registers)
|=====================================================================================================
| INDEX | BIT 7 | BIT 6 | BIT 5 | BIT 4  | BIT 3 | BIT 2 | BIT 1 | BIT 0 | FUNCTION    |   RANGE     |
|  00h  |   0   |      10 Seconds        |            Seconds            |  Seconds    |   00-59     |
|  01h  |   0   |      10 Minutes        |            Minutes            |  Minutes    |   00-59     |
|  02h  |   0   | 12/24 | PM/AM |10 Hour |              Hour             |  Hours      |1-12 + PM/AM |
|       |       |       |20 Hour|        |                               |             |   00-23     |
|  03h  |   0   |   0   |   0   |   0    |   0   |      Day              | Day of Week |1-7 User Def |
|  04h  |   0   |   0   |    10 Date     |              Date             |  Date       |   01-31     |
|  05h  |Century|   0   |   0   |10 Month|             Month             |Month/Century|01-12+Century|
|  06h  |           10 Year              |              Year             |  Year       |   00-99     |
|  07h  | A1M1  |      10 Seconds        |            Seconds            |Alarm 1 Sec  |   00-59     |
|  08h  | A1M2  |      10 Minutes        |            Minutes            |Alarm 1 Min  |   00-59     |
|  09h  | A1M3  | 12/24 | PM/AM |10 Hour |              Hour             |Alarm 1 Hours|1-12 + PM/AM |
|       |       |       |20 Hour|        |                               |             |             |
|  0Ah  | A1M4  | DY/DT |    10 Date     |              Day              |Alarm 1 Day  |    1-7      |
|       |       |       |                |              Date             |Alarm 1 Date |    1-31     |
|  0Bh  | A2M2  |      10 Minutes        |            Minutes            |Alarm 2 Min  |   00-59     |
|  0Ch  | A2M3  | 12/24 | PM/AM |10 Hour |              Hour             |Alarm 2 Hours|1-12 + AM/PM |
|       |       |       |20 Hour|        |                               |             |   00-23     |
|  0Dh  | A2M4  | DY/DT |    10 Date     |              Day              |Alarm 2 Day  |    1-7      |
|       |       |       |                |              Date             |Alarm 2 Date |    1-31     |
|  0Eh  | EOSC  | BBSQW | CONV  |  RS2   |  RS1  | INTCN |  A2IE | A1IE  |  Control    |     —       |
|  0Fh  |  OSF  |   0   |   0   |   0    |EN32kHz|  BSY  |  A2F  | A1F   |Contrl/Status|     —       |
|  10h  | SIGN  | DATA  | DATA  | DATA   | DATA  | DATA  | DATA  | DATA  |Aging Offset |     —       |
|  11h  | SIGN  | DATA  | DATA  | DATA   | DATA  | DATA  | DATA  | DATA  |MSB of Temp  |     —       |
|  12h  | DATA  | DATA  |   0   |   0    |   0   |   0   |   0   |   0   |LSB of Temp  |     —       |
|====================================================================================================*/
// Notes:
// The numeric values stored to / read from the DS3231 for seconds, minutes, hours, day, date, month, year,
// use BCD encoding.

//=================================================================================================
// DS3231 routines
//=================================================================================================
// DS3231 registers use BCD encoding for time/date storage.
// This requires BCD to BIN and BIN to BCD functions to convert back and forth
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }

#if 0
// Values to write to DS3231 for a "reset" condition
const DATE_TIME dt_reset = {
	0,  //< Year offset from 2000
	1,  //< Month 1-12
	1,	//< Day 1-31
	0,  //< Hours 0-23
	0,  //< Minutes 0-59
	0   //< Seconds 0-59
};
#endif

// The Time/Date registers are located at index 00h - 06h
// Use a "generic I2C API" to write index registers 000h - 06h
// Write the DS3231 time/date registers given a DATE_TIME structure
void write_ds3231(const DATE_TIME * dt)
{
	uint8_t reg_data[8]; // [0] index, [1] - [7] time and date registers
	// Convert DATE_TIME format into DS3231 register values
	reg_data[0] = 0x00; // begin writing at index 0
	reg_data[1] = bin2bcd(dt->ss);
	reg_data[2] = bin2bcd(dt->mm);
	reg_data[3] = bin2bcd(dt->hh); // bit 6 should be low (We can force it...)
	reg_data[4] = 0; // day of the week (don't care)
	reg_data[5] = bin2bcd(dt->d);
	reg_data[6] = bin2bcd(dt->m); // remove century bit
	reg_data[7] = bin2bcd(dt->yOff);

	i2c_write_read(I2C_ADDRESS_DS3231, reg_data, sizeof(reg_data), NULL, 0);
}

// Return the value of the status register (index 0Fh)
uint8_t ds3231_read_status(void)
{
  // Read value of status register into reg_data
	uint8_t index = 0x0F;
	uint8_t reg_data;
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), &reg_data, sizeof(reg_data));
  return reg_data;
}

void ds3231_clearOSF(void)
{
	// Clear the status register (index 0Fh) OSF bit
  printf("Clear OSF\n");
	uint8_t index_status[2] = {0x0F, 0x00}; // index of status register and value to write to it
	i2c_write_read(I2C_ADDRESS_DS3231, index_status, sizeof(index_status), NULL, 0);
}

// The Time/Date registers are located at index 00h - 06h
// Use a "generic I2C API" to read index registers 00h - 06h
// Read the DS3231 time/date registers into a DATE_TIME structure
void read_ds3231(DATE_TIME * dt)
{
	uint8_t index = 0;
	uint8_t reg_data[7];
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), reg_data, sizeof(reg_data));
	// Convert DS3231 register data into DATE_TIME format
	dt->ss = bcd2bin(reg_data[0]);
	dt->mm = bcd2bin(reg_data[1]);
	dt->hh = bcd2bin(reg_data[2]); // bit 6 should be low (We can force it...)
	dt->d  = bcd2bin(reg_data[4]);
	dt->m  = bcd2bin(reg_data[5] & 0x1F); // remove century bit
	dt->yOff = bcd2bin(reg_data[6]);
}

#if 0
// Write the DS3231 Control register, get RTC counting
// Does NOT clear Oscillator Stop Flag (OSF)
// Set time to reset value if clock was stopped
void init_ds3231(void)
{
	uint8_t indx_control[2] = {0x0E, 0b00000000};  // Index, Control
  i2c_write_read(I2C_ADDRESS_DS3231, indx_control, sizeof(indx_control), NULL, 0);
	// Read status register - is OSF set?
	uint8_t status_reg = ds3231_read_status();
	// If OSF (BIT 7) set, write initial values to RTC registers
	if(status_reg & 0x80) {
    printf("OSF is SET, writing reset time and date values to DS3231\n");
		write_ds3231(&dt_reset);
	}
}
#endif

// Command line method to read / set the time
int cl_time(void)
{
	DATE_TIME dt;
	if(4 == argc) {
	  read_ds3231(&dt); // read in time and date values into DATE_TIME structure
		// Set the time using arguments at index 1 <hours>, 2 <minutes>, 3 <seconds>
		dt.hh = strtol(argv[1], NULL, 10); // user will use decimal
		dt.mm = strtol(argv[2], NULL, 10);
		dt.ss = strtol(argv[3], NULL, 10);
		// Write new time values to DS3231
		write_ds3231(&dt);
    // Reset the OSF bit
    ds3231_clearOSF();
	}

	// Always read the DS3231 and display the time
	read_ds3231(&dt);
	printf("%02u:%02u:%02u\n",dt.hh,dt.mm,dt.ss);
	return 0;
}

// Command line method to read / set the date
int cl_date(void)
{
	DATE_TIME dt;
	read_ds3231(&dt); // read in time and date values into DATE_TIME structure

	if(4 == argc) {
		// Set the date using arguments at index 1 <month>, 2 <day>, 3 <year>
		dt.m = strtol(argv[1], NULL, 10); // user will use decimal
    dt.d = strtol(argv[2], NULL, 10);
		uint16_t year = strtol(argv[3], NULL, 10);
		if(year >= 2000) year -= 2000; // convert to offset
		dt.yOff = (uint8_t)year;
		// Write new date values to DS3231
		write_ds3231(&dt);
	}

	// Always read the DS3231 and display the date MMDDYYYY
	read_ds3231(&dt);
	printf("%02u/%02u/%04u\r\n",dt.m,dt.d,dt.yOff + 2000);
	return 0;
}

// Dump the values of the 19 DS3231 index registers 00h - 12h
// Use a "generic I2C API" to read index registers 00h - 12h
// Read the DS3231 time/date registers into a DATE_TIME structure
int cl_ds3231_dump(void)
{
	uint8_t index = 0;
	uint8_t reg_data[19];
  const char * reg_name[]={"Seconds","Minutes","Hours","WeekDay","Date","Month","Year",
    "Alarm1 Sec","Alarm1 Min","Alarm1 Hr","Alarm1 Day-Date",
    "Alarm2 Min","Alarm2 Hr","Alarm2 Day-Date",
    "Control","Cntrl/Status","Aging Offset","MSB of Temp","LSB of Temp"};
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), reg_data, sizeof(reg_data));
  printf("Indx Data   Register name\n");
  for(unsigned i=0;i<19;i++) {
    printf("%02X   0x%02X   %s\n",i,reg_data[i],reg_name[i]);
  }
	printf("\n");
  return 0;
}

// Test the INT/SQW output
// Allow the user to enable one of several test functions via an argument
// 0: 1Hz, 1: 1024Hz, 2: 4096Hz, 3:8192Hz
int cl_sqw_test(void)
{
  if(argc > 1) {
    uint8_t RSx = (uint8_t) strtol(argv[1], NULL, 10); // assume decimal input
    RSx &= 3; // keep only lower 2 bits
    printf("%s: input: %u\n",__func__,RSx);
    RSx <<=3; // move the 2 bits into BIT4:BIT3 position (The value of RSx, written to Control Register (0Eh), will now enable SQW signal)
  	uint8_t index_data[2] = {0x0E,0x1C}; // write default (POR) value, turning off SQW output
    i2c_write_read(I2C_ADDRESS_DS3231, index_data, sizeof(index_data), NULL, 0);
    index_data[1] = RSx; // desired register - index 0Eh value
    i2c_write_read(I2C_ADDRESS_DS3231, index_data, sizeof(index_data), NULL, 0);
    printf("0X%02X written to index 0Eh\n",RSx);
  }
  else {
    // no arguments
#if 0
    DATE_TIME dt;
    printf("Setting alarm for 3 seconds from now...\n");
    // Get current time and convert it to unix seconds count
    read_ds3231(&dt); // fill in DATE_TIME structure
    uint32_t ut_now = rtc2unix(&dt);
    uint32_t ut_future = ut_now + 3;
    // Convert back into DATE-TIME format
    unix2rtc(&dt, ut_future); // DATE_TIME is now 3 seconds in the future
#else
    printf("%s: Turn off SQW output\n",__func__);
    uint8_t index_data[2] = {0x0E,0x1C}; // write default (POR) value, turning off SQW output
    i2c_write_read(I2C_ADDRESS_DS3231, index_data, sizeof(index_data), NULL, 0);
#endif
  }

  return 0;
}


// Configure SQW/INT pin for alarm interrupt output)
// Spin, watching the A1F flash
// Example serial output:
//   Display status of A1F flag...
//   Press any key to exit...
//   000000000000000000000011111111111111111111111111   (22 '0's, 200ms delay each)
// This command works great, and now resets like it should
int cl_alarm(void)
{
	// Begin by disabling the alarm - Read Control register, index 0Eh
	uint8_t index = 0x0E;
	uint8_t control_reg;
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), &control_reg, sizeof(control_reg));

	control_reg &= 0xFC; // Clear alarm enable bits,
	control_reg |= 4;    // Set INTCN to 1 (Let's use SQW/INT pin for alarm interrupt output)
	uint8_t index_control[3] = {index,control_reg,0};

	// Write modified control register value back
	i2c_write_read(I2C_ADDRESS_DS3231, index_control, sizeof(index_control), NULL, 0);

	// Clear any alarm bits in the status register (write 0s to them)
	// Moved this functionality into the write above (the 3rd byte)
	//uint8_t index_status[2]={0x0F, 0x00};
	//i2c_write_read(I2C_ADDRESS_DS3231, index_status, sizeof(index_status), NULL, 0);

    DATE_TIME dt;
    printf("Setting alarm for 5 seconds from now...\n");
    // Get current time and convert it to unix seconds count
    read_ds3231(&dt); // fill in DATE_TIME structure
    uint32_t ut_now = rtc2unix(&dt);
    uint32_t ut_future = ut_now + 5; // 5 seconds from now
    // Convert back into DATE-TIME format
    unix2rtc(&dt, ut_future); // DATE_TIME is now 3 seconds in the future

    // If we wish to alarm on a second boundary, we need to use Alarm1 that includes a seconds register
    // Looking at Table 2. Alarm Mask Bits, we want A1M4:A1M1 set as 0b1000, Alarm when hours, minutes, and seconds match
    uint8_t index_7_8_9_A[5]={7, bin2bcd(dt.ss), bin2bcd(dt.mm), bin2bcd(dt.hh), bin2bcd(dt.d)|0x80}; // Set A1M4 bit

	// Load Alarm1 registers
	i2c_write_read(I2C_ADDRESS_DS3231, index_7_8_9_A, sizeof(index_7_8_9_A), NULL, 0);

	// Enable Alarm1 interrupt
	index_control[1]|=1;
	i2c_write_read(I2C_ADDRESS_DS3231, index_control, sizeof(index_control), NULL, 0);

	// Dump registers as we begin to wait
	cl_ds3231_dump();

	// While we wait, show the status of A1F
	index = 0x0F;
	uint8_t status;
	printf("Display status of A1F flag...\nPress any key to exit...\n");

	while(EOF == __io_getchar() ) {
		i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), &status, sizeof(status));
		//printf("\b \b%c",(status&1)?'1':'0');
		printf("%c",(status&1)?'1':'0');
		delay(200); // display a '1' or '0' every 200ms
	}
	printf("\n");

    return 0;
}

// Create a set of I2C waveforms showing reading a register and then writing that register
int cl_i2c_read_write(void)
{
  printf("%s: reading and then writing\n",__func__);
  uint8_t index = 0x0E; // control register
	uint8_t control_reg;
  // Read contents of control register
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), &control_reg, sizeof(control_reg));
  printf("Control register value: 0x%02X\n",control_reg);
  // Write the contents of control register back
  uint8_t write_data[2]={index,control_reg};
  i2c_write_read(I2C_ADDRESS_DS3231,write_data, sizeof(write_data), NULL, 0);
  return 0;
}

//=================================================================================================
// AT24C43 Functions
//=================================================================================================
// Search the 7 possible addresses for the AT24C32.  If found, return the address, else return 0.
// The AT24C32 can reside at addresses 0x50 - 0x57
uint8_t find_at24c32(void)
{
  for(uint8_t i2c_address = 0x50; i2c_address <= 0x57; i2c_address++) {
    // Use similar code used by i2cscanner...
    Wire.beginTransmission(i2c_address);
    uint8_t error = Wire.endTransmission();
    if(0 == error) {
      printf("AT24C32 found, 0x%02X\n",i2c_address);
      return i2c_address; // ACK received, return this address
    }
  }
  printf("No AT24C32 found!\n");
  return 0;
}

// Write one byte of data to AT24C32, beginning at 'memory_address'
// Return 1(true) for success, or 0(false) for failure
// According to the datasheet, timing parameter tWR, need to allow 10ms between writes
// If faster writing is desired, use page writes, or monitor acknowledge bit for writing complete
int write_byte_at24c32(uint8_t device_i2c_address, uint16_t memory_address, uint8_t data_byte)
{
  if(memory_address >= AT24C32_SIZE_BYTES) {
    printf("%s: memory_address too large!\n",__func__);
    return 0;
  }
  // To perform a write, we begin by writing device_address, two memory address bytes, and the desired data byte (4 bytes total)
  // memory address is provided high byte first
  uint8_t memory_address_data[3] = {memory_address >> 8, memory_address, data_byte}; // low byte is cropped by compiler
  i2c_write_read(device_i2c_address, memory_address_data, sizeof(memory_address_data), NULL, 0);
  return 1;  
}

// Check if the AT24C32 is busy, or if it's available for a write
// Returns 1(true) is busy writing, 0(false) not busy, device is available for a write
int at24c32_is_busy(uint8_t device_i2c_address)
{
    // Use similar code used by i2cscanner...
    Wire.beginTransmission(device_i2c_address);
    uint8_t error = Wire.endTransmission();
    if(0 == error)
      return error;
    else
      return 1;
}

// Using page writes, fill the memory with a desired data value
// Return 1(true) for success, or 0(false) for failure
// Monitor acknowledge bit for writing complete
// NOTE!!! The Wire library's buffer is only 32 bytes in size.  Need to limit data transactions to 32.
// Using at24c32_is_busy() reduces this function's time to 805ms.
#define BYTES_PER_PAGE_WRITE 16
int fill_at24c32(uint8_t device_i2c_address, uint8_t fill_data_byte)
{
  uint32_t entry_millis = millis();
  uint8_t memory_address_data[BYTES_PER_PAGE_WRITE+2];
  //uint8_t memory_address_data[2];
  memset(memory_address_data,fill_data_byte,sizeof(memory_address_data));
  // According to the data sheet, a page write should be the fastest way to fill the device with data
  for(uint16_t memory_address=0;memory_address < AT24C32_SIZE_BYTES;memory_address+=BYTES_PER_PAGE_WRITE) {
    // memory address is provided high byte first
    memory_address_data[0] = memory_address >> 8;
    memory_address_data[1] = memory_address; // low byte is cropped by compiler
    i2c_write_read(device_i2c_address, memory_address_data, sizeof(memory_address_data), NULL, 0);
#if 1
  // Page write sent, poll device, waiting for acknowledge bit to be low
    uint32_t initial_millis = millis();
    // Enter spin-wait for EEPROM
    while(1) {
      if(!at24c32_is_busy(device_i2c_address)) break;
      // write should take 160ms max (16 * 10ms each)
      if(millis() - initial_millis >= 200) {
        printf("%s: write timeout!\n",__func__);
        return 0;
      }
    }
#else
    delay(320);
    printf(".");
#endif
  } // for-loop - write a page
  uint32_t delta = millis() - entry_millis;
  printf("%s duration: %lu ms\n",__func__,delta);
  return 1;  
}

#define BYTES_PER_LINE 16
// Dump the contents of the AT24C32
int cl_dump_at24c32(void)
{
  uint8_t i2c_address = find_at24c32();
  if(0 == i2c_address) return 0; // not found

  uint8_t buff[BYTES_PER_LINE]; // 16 hex bytes per line
  // According to the data sheet, a page write should be the fastest way to fill the device with data
  for(uint16_t memory_address=0;memory_address < AT24C32_SIZE_BYTES;memory_address+=BYTES_PER_LINE) {
    // Read a line of data
    read_at24c32(i2c_address, memory_address, buff, BYTES_PER_LINE);
    printf("%04X: ",memory_address);
    for(uint16_t i=0;i<BYTES_PER_LINE;i++) printf("%02X ",buff[i]);
    printf("\n");
  }
  return 0;
}

// Command line method to fill the EEPROM with a value
int cl_fill_at24c32(void)
{
  uint8_t i2c_address = find_at24c32();
  if(0 == i2c_address) return 0; // not found

  uint8_t value = (uint8_t) strtol(argv[1], NULL, 0); // allow user to use decimal or hex
  printf("%s: filling EEPROM with 0x%02X\n",__func__,value);
  return fill_at24c32(i2c_address, value);
}

// Read one or more bytes of data from AT24C32, beginning at 'memory_address'
// Return 1(true) for success, or 0(false) for failure
int read_at24c32(uint8_t device_i2c_address, uint16_t memory_address, uint8_t *buffer, uint16_t count)
{
  if((memory_address + count) > AT24C32_SIZE_BYTES) {
    printf("%s: memory_address + count too large!\n",__func__);
    return 0;
  }
  // To initiate a read, we begin by writing device_address, and two memory address bytes (3 bytes total)
  // This is followed by reading 'count' data bytes
  // memory address is provided high byte first
  uint8_t memory_address_buffer[2] = {memory_address >> 8, memory_address}; // low byte is cropped by compiler
  i2c_write_read(device_i2c_address, memory_address_buffer, sizeof(memory_address_buffer), buffer, count);
  return 1;  
}

// Collect string passed in - write to memory address 0
int cl_write_at24c32(void) {
  uint8_t i2c_address = find_at24c32();
  if(0 == i2c_address) return 0; // not found

  int length = strlen(argv[1]); // does not include null termination
  uint16_t memory_address = 0;
  for(;memory_address <= length;memory_address++) {
    write_byte_at24c32(i2c_address, memory_address, (uint8_t)(argv[1][memory_address]));
    delay(10); // Max value for tWR
  }
  return 0;
}


//=================================================================================================
// TM1637 Defines - functions
//=================================================================================================
#define TM1637_CLK  14  // A0
#define TM1637_DIO  15  // A1
#define colonMask   0b01000000

// Constructor for the TM1637 - Doesn't write to the pins
TM1637Display display(TM1637_CLK, TM1637_DIO, 100);

// Initialize the TM1637 for clock usage
// Display 00:00 on the display
void init_tm1637(void)
{
	// Initialize - configure the GPIO pins
	//display.configure_gpio_pins();
	display.setBrightness(0x0f);
	//display.clear();
	display.showNumberDecEx(0, colonMask, true, 2, 0);
	display.showNumberDec(0, true, 2, 2);
}

// Check time, update clock if needed, else just return
// This gets called very frequently.  Only "do something" every second, else just return
void update_clock(void)
{
	static uint8_t previous_minutes = 100; // intentionally an invalid value
	DATE_TIME dt;
  // Read RTC into DATE_TIME structure
  read_ds3231(&dt);
  // If the minutes value changes, update the display
  if(dt.mm != previous_minutes) {
    if(dt.hh>12) dt.hh-=12; // convert 24hr display to 12hr
    // if hours (dt.hh) is zero, the colon doesn't display.  Display 12 instead.
    display.showNumberDecEx(dt.hh?dt.hh:12, colonMask, false, 2, 0); 
    display.showNumberDec(dt.mm, true, 2, 2);
    previous_minutes = dt.mm;
  }
}

