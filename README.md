# Arduino_Uno_Command_Line_I2C_DS3231_TM1637

## Libraries
    
    Using Arduino Wire Class
	"TM1637" library by Avishay Orpaz, vesion 1.2.0, "TM1637Display.h" / "TM1637Display.cpp"

## DS3231 I2C Connections
    
    DS3231_I2C:SCL - Arduino A5 (PC5)   
    DS3231_I2C:SDA - Arduino A4 (PC4)

## TM1637 Connections:
    
    TM1637_CLK: A0 (PC0)  14
    TM1637_IO:  A1 (PC1)  15

## Command Line's three major components:
    
    * cmd_table[] - array of COMMAND_ITEM data structures, containing:
       command name
       comment concerning the command (usage)
       minimal argument count (usually just 1)
       pointer to function to execute the command
    
    * cl_parseArgcArgv() - break the command string, with arguments, into "words"
    
    * cl_process_buffer() - search cmd_table for "command" supplied, and if found,
       determine if enough parameters are provided.  Call associated function.

## I2C helper function, i2c_write_read()
***int i2c_write_read(uint8_t i2c_address, uint8_t * pwrite, uint8_t wr_count, uint8_t * pread, uint8_t rd_count);***
    
    Implementing this helper function allows easy code sharing with other development environments.
    As an example, any I2C code that uses this function can also work with STM32Cube programs.

## DS3231 RTC Support
    
    * init_ds3231()
    * read_ds3231()
    * write_ds3231()
    * cl_time()  - Command line example that reads (or sets) the RTC time
    * cl_date()  - Command line example that reads (or sets) the RTC date

## Notes
    
    The goal of this project is to provide a printf(), command-line framework,
    allowing many "commands" to be created, each supporting optional arguments.
    See function, cl_add(), which demonstrates a simple multi-argument command.
