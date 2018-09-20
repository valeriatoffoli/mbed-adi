// --------------------------------------------------------------------------------------------------------
//
//  September 2018
//  Author: Valeria Toffoli, Rohan Gurav 
//
// --------------------------------------------------------------------------------------------------------
//
//  ADXL355.h
//
// --------------------------------------------------------------------------------------------------------
// 
//  This library provides all the functions necessary to interface the ADXL355 with EV-COG-AD3029 or  
//  EV-COG-AD4050 Board. Functions for SPI configuration, reads and writes,and scaling are included. 
//  This library may be used for the entire ADXL35x family of devices 
//  with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// --------------------------------------------------------------------------------------------------------


#ifndef ADXL355_H_
#define ADXL355_H_

class ADXL355
{
public: 
    // -------------------------- //
    // CONST AND VARIABLES        // 
    // -------------------------- //
    const static float t_sens = -9.05;    
    const static float t_bias = 1852;    
    float axis355_sfactor;
    float axis357_sfactor;
    typedef struct {
        // scale factor - we do not consider cross sensitivity
        float x_sfactor;
        float y_sfactor;
        float z_sfactor;
        // 0g offset
        float x_offset;
        float y_offset;
        float z_offset;
    } calib_data_t;  
    calib_data_t axis355_cal;    
    // -------------------------- //
    // REGISTERS                  // 
    // -------------------------- //
    typedef enum {
        DEVID_AD = 0x00,
        DEVID_MST = 0x01,
        PARTID = 0x02,
        REVID = 0x03,
        STATUS = 0x04,
        FIFO_ENTRIES = 0x05,
        TEMP2 = 0x06,
        TEMP1 = 0x07,
        XDATA3 = 0x08,
        XDATA2 = 0x09,
        XDATA1 = 0x0A,
        YDATA3 = 0x0B,
        YDATA2 = 0x0C,
        YDATA1 = 0x0D,
        ZDATA3 = 0x0E,
        ZDATA2 = 0x0F,
        ZDATA1 = 0x10,
        FIFO_DATA = 0x11,
        OFFSET_X_H = 0x1E,
        OFFSET_X_L = 0x1F,
        OFFSET_Y_H = 0x20,
        OFFSET_Y_L = 0x21,
        OFFSET_Z_H = 0x22,
        OFFSET_Z_L = 0x23,
        ACT_EN = 0x24,
        ACT_THRESH_H = 0x25,
        ACT_THRESH_L = 0x26,
        ACT_COUNT = 0x27,
        FILTER = 0x28,
        FIFO_SAMPLES = 0x29,
        INT_MAP = 0x2A,
        SYNC = 0x2B,
        RANGE = 0x2C,
        POWER_CTL = 0x2D,
        SELF_TEST = 0x2E,
        RESET = 0x2F
    } ADXL355_register_t;
    ADXL355_register_t regist;
    // -------------------------- //
    // REGISTERS - DEFAULT VALUES //
    // -------------------------- //
    // Modes - POWER_CTL  
    typedef enum {
        DRDY_OFF = 0x04,
        TEMP_OFF = 0x02,
        STANDBY = 0x01,
        MEASUREMENT = 0x00
    } ADXL355_modes_t;    
    // Activate Threshold - ACT_EN  
    typedef enum {
        ACT_Z = 0x04,
        ACT_Y = 0x02,
        ACT_X = 0x01
    } ADXL355_act_ctl_t;
    // High-Pass and Low-Pass Filter - FILTER 
    typedef enum {
        HPFOFF = 0x00,
        HPF247 = 0x10,
        HPF62 = 0x20,
        HPF15 = 0x30,
        HPF3 = 0x40,
        HPF09 = 0x50,
        HPF02 = 0x60,
        ODR4000HZ = 0x00,
        ODR2000HZ = 0x01,
        ODR1000HZ = 0x02,
        ODR500HZ = 0x03,
        ODR250HZ = 0x04,
        ODR125Hz = 0x05,
        ODR62HZ = 0x06,
        ODR31Hz = 0x07,
        ODR15Hz = 0x08,
        ODR7Hz = 0x09,
        ODR3HZ = 0x0A
    } ADXL355_filter_ctl_t;
    // External timing register - INT_MAP 
    typedef enum {
        OVR_EN = 0x04,
        FULL_EN = 0x02,
        RDY_EN = 0x01
    } ADXL355_intmap_ctl_t;
    // External timing register - SYNC 
    typedef enum {
        EXT_CLK = 0x04,
        INT_SYNC = 0x00,
        EXT_SYNC_NO_INT = 0x01,
        EXT_SYNC_INT = 0x02
    } ADXL355_sync_ctl_t; 
    // polarity and range - RANGE 
    typedef enum {
        RANGE2G = 0x01, // ADXL355
        RANGE4G = 0x02,
        RANGE8G = 0x03,
        RANGE10 = 0x01, // ADXL357
        RANGE20 = 0x02,
        RANGE40 = 0x03
    } ADXL355_range_ctl_t;
    // self test interrupt - INT 
    typedef enum {
        ST2 = 0x02,
        ST1 = 0x01
    } ADXL355_int_ctl_t;
    // -------------------------- //
    // FUNCTIONS                  //  
    // -------------------------- //
    // SPI configuration & constructor 
    ADXL355(PinName cs_pin , PinName MOSI , PinName MISO , PinName SCK );
    void frequency(int hz);
    // Low level SPI bus comm methods 
    void reset(void);
    void write_reg(ADXL355_register_t reg, uint8_t data);
    void write_reg_u16(ADXL355_register_t reg, uint16_t data);
    uint8_t read_reg(ADXL355_register_t reg);
    uint16_t read_reg_u16(ADXL355_register_t reg);
    uint32_t read_reg_u20(ADXL355_register_t reg);
    // ADXL general register R/W methods 
    void set_power_ctl_reg(uint8_t data);
    void set_filter_ctl_reg(ADXL355_filter_ctl_t hpf, ADXL355_filter_ctl_t odr);
    void set_clk(ADXL355_sync_ctl_t data);
    void set_device(ADXL355_range_ctl_t range);
    uint8_t read_status();
    // ADXL X/Y/Z/T scanning methods   
    uint32_t scanx();
    uint32_t scany();
    uint32_t scanz();
    uint16_t scant();
    // ADXL activity methods 
    void set_activity_axis(ADXL355_act_ctl_t axis, uint8_t count,uint8_t data_h, uint8_t data_l);
    void set_inactivity();
    bool get_int2();
    // ADXL FIFO methods 
    uint8_t fifo_read_nr_of_entries();
    void fifo_setup(uint8_t nr_of_entries);
    uint32_t fifo_read_u32();
    uint64_t fifo_scan();
    // ADXL conversion
    float tempScale(uint16_t data);
    float accelScale(uint32_t data);
    // ADXL calibration
    void calib2point(float pos, float neg, int axis);
    void calib4point(float angle0, float angle90, float angle180, float angle270, int axis);
    // ADXL angle measurement
    float single_axis(uint32_t x);
    float dual_axis(uint32_t x, uint32_t y);
    float tri_axis1(uint32_t x, uint32_t y, uint32_t z);
    float tri_axis2(uint32_t x, uint32_t y, uint32_t z);
private:
    // SPI adxl355;                 
    SPI adxl355; DigitalOut cs;
    const static uint8_t _DEVICE_AD = 0xAD;     // contect of DEVID_AD (only-read) register 
    const static uint8_t _RESET = 0x52;         // reset code 
    const static uint8_t _DUMMY_BYTE = 0xAA;    // 10101010
    const static uint8_t _WRITE_REG_CMD = 0x00; // write register
    const static uint8_t _READ_REG_CMD = 0x01;  // read register
    const static uint8_t _READ_FIFO_CMD = 0x23; // read FIFO
    const static uint8_t _SPI_MODE = 0;         // timing scheme
};
 
#endif