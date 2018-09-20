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

#include <stdint.h>
#include <math.h>
#include "mbed.h"
#include "ADXL355.h"

/* Constructor with configurable of the SPI port */
ADXL355::ADXL355(PinName cs_pin, PinName MOSI, PinName MISO, PinName SCK): adxl355(MOSI, MISO, SCK), cs(cs_pin)
{
    cs = 1;
    adxl355.format(8,_SPI_MODE);
    adxl355.lock();
    axis355_sfactor = 3.9e-6;
    axis357_sfactor = 19.5e-6;
    axis355_cal.x_sfactor = 3.9e-6;
    axis355_cal.y_sfactor = 3.9e-6;
    axis355_cal.z_sfactor = 3.9e-6;
    axis355_cal.x_offset = 0;
    axis355_cal.y_offset = 0;
    axis355_cal.z_offset = 0;
}

/* Set SPI clock speed: from 100kHz to 10MHz */
void ADXL355::frequency(int hz)
{
    adxl355.frequency(hz);
}

/* Reset of the device and its register */
void ADXL355::reset(void)
{
    adxl355.format(8, _SPI_MODE);
    cs = false;
    // Writing Code 0x52 (representing the letter, R, in ASCII or unicode) to this register immediately resets the ADXL362.
    write_reg(RESET, _RESET);
    cs = true;
    axis355_sfactor = 3.9e-6;
    axis357_sfactor = 19.5e-6;
    axis355_cal.x_sfactor = 3.9e-6;
    axis355_cal.y_sfactor = 3.9e-6;
    axis355_cal.z_sfactor = 3.9e-6;
    axis355_cal.x_offset = 0;
    axis355_cal.y_offset = 0;
    axis355_cal.z_offset = 0;
}

/* Writes one byte of data to the specified register over SPI */
void ADXL355::write_reg(ADXL355_register_t reg, uint8_t data)
{
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(static_cast<uint8_t>(reg<<1) | _WRITE_REG_CMD);
    adxl355.write(data);
    cs = true;
}

/* Writes 2 bytes (one word) in two sequential registers over SPI */
void ADXL355::write_reg_u16(ADXL355_register_t reg, uint16_t data)
{
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(static_cast<uint8_t>(reg<<1) | _WRITE_REG_CMD);
    adxl355.write(static_cast<uint8_t>(data & 0xff));
    adxl355.write(static_cast<uint8_t>((data & 0xff00) >> 8));
    cs = true;
}

/* Reads one byte of data to the specified register over SPI */
uint8_t ADXL355::read_reg(ADXL355_register_t reg)
{
    uint8_t ret_val;
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(static_cast<uint8_t>(reg<<1) | _READ_REG_CMD);
    ret_val = adxl355.write(_DUMMY_BYTE);
    cs = true;
    return ret_val;
}

/* Reads 2 bytes (one word) in two sequential registers over SPI */
uint16_t ADXL355::read_reg_u16(ADXL355_register_t reg){
    uint16_t ret_val = 0;
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(static_cast<uint8_t>(reg<<1) | _READ_REG_CMD);
    ret_val = adxl355.write(_DUMMY_BYTE);
    ret_val = (ret_val<<8) | adxl355.write(_DUMMY_BYTE);
    cs = true;
    return ret_val;
}

/* Reads acceleration data - 20bit, left justified and formatted as 2s complement */
uint32_t ADXL355::read_reg_u20(ADXL355_register_t reg){
    uint32_t ret_val = 0;
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write((reg<<1) | _READ_REG_CMD);
    ret_val = 0x0f & adxl355.write(_DUMMY_BYTE);
    ret_val = (ret_val<<8) | adxl355.write(_DUMMY_BYTE);
    ret_val = (ret_val<<4) | (adxl355.write(_DUMMY_BYTE)>>4);
    cs = true;
    return ret_val;
}

/* Sets the CTL registers */
void ADXL355::set_power_ctl_reg(uint8_t data){
     write_reg(POWER_CTL, data);
}
void ADXL355::set_filter_ctl_reg(ADXL355_filter_ctl_t hpf, ADXL355_filter_ctl_t odr){
    write_reg(FILTER, static_cast<uint8_t>(hpf|odr));
}

/* Sets internal or external clk*/
void ADXL355::set_clk(ADXL355_sync_ctl_t data) {
    write_reg(SYNC, static_cast<uint8_t>(data));
}
/* Sets device range and scale factor*/
void ADXL355::set_device(ADXL355_range_ctl_t range) {
    write_reg(RANGE, static_cast<uint8_t>(range));
    switch(range){
        case 0x01:
            axis355_sfactor = 3.9e-6;
            axis357_sfactor = 19.5e-6;
            break;
        case 0x02:
            axis355_sfactor = 7.8e-6;
            axis357_sfactor = 39e-6;
            break;
        case 0x03:
            axis355_sfactor = 15.6e-6;
            axis357_sfactor = 78e-6;
            break;
        }
}

/* Reads the STATUS registers */
uint8_t ADXL355::read_status(){
    return read_reg(STATUS);
}

/* Reads the DATA registers */
uint32_t ADXL355::scanx(){
    return read_reg_u20(XDATA3);
}
uint32_t ADXL355::scany(){
    return read_reg_u20(YDATA3);
}
uint32_t ADXL355::scanz(){
    return read_reg_u20(ZDATA3);
}
uint16_t ADXL355::scant(){
    return read_reg_u16(TEMP2);
}

/* Enables and Sets activity detection */
void ADXL355::set_activity_axis(ADXL355_act_ctl_t axis, uint8_t count,uint8_t data_h, uint8_t data_l) {
    uint16_t ret_val = static_cast<uint16_t>((data_h<<8)|data_l);
    write_reg_u16(ACT_THRESH_H, ret_val);
    write_reg(ACT_COUNT, count);
    write_reg(ACT_EN, axis);
}

/* Disables activity detection and resets the registers*/
void ADXL355::set_inactivity() {
    write_reg(ACT_EN, 0x00);
    write_reg_u16(ACT_THRESH_H, 0x00);
    write_reg(ACT_COUNT, 0x00);
}

/* Reads the FIFO configurations */
uint8_t ADXL355::fifo_read_nr_of_entries(){
    return read_reg(FIFO_ENTRIES);
}
void ADXL355::fifo_setup(uint8_t nr_of_entries){
    if (nr_of_entries > 0x60) {
        nr_of_entries = nr_of_entries;
    }
    write_reg(FIFO_SAMPLES, nr_of_entries);
}    

/* Reads a FIFO entry */
uint32_t ADXL355::fifo_read_u32() {
    uint32_t ret_val = 0;
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(_READ_FIFO_CMD);
    ret_val = adxl355.write(_DUMMY_BYTE);
    ret_val = (ret_val<<8) | static_cast<uint8_t>(adxl355.write(_DUMMY_BYTE));
    ret_val = (ret_val<<4) | static_cast<uint8_t>(adxl355.write(_DUMMY_BYTE)>>4);
    cs = true;
    return ret_val;
    }

/* Reads 3 bytyes from the FIFO */    
uint64_t ADXL355::fifo_scan() {
    uint64_t ret_val = 0;
    uint32_t x = 0, y = 0, z = 0, dummy;
    adxl355.format(8, _SPI_MODE);
    cs = false;
    adxl355.write(_READ_FIFO_CMD);
    for(uint8_t i = 0; i < 3; i++) {
        dummy = adxl355.write(_DUMMY_BYTE);
        dummy = (dummy<<8) | static_cast<uint8_t>(adxl355.write(_DUMMY_BYTE));
        dummy = (dummy<<4) | static_cast<uint8_t>(adxl355.write(_DUMMY_BYTE)>>4);
        dummy = dummy & 0xffff;
        switch(i) {
            case 0: // x
                x = dummy;
                break;
            case 1: // y
                y = dummy;
                break;
            case 2: // z
                z = dummy;
                break;
        }
    } 
    cs = true;
    // format (24)xx(24)yy(24)zz
    ret_val = static_cast<uint64_t> (x) << 48;
    ret_val |= static_cast<uint64_t>(y) << 24;
    ret_val |= static_cast<uint64_t>(z) ;
    return ret_val;
}

/*  Converts accelerometer data output from the read_reg_u32() function. [g]  */ 
float ADXL355::accelScale(uint32_t data){
    // If a positive value, return it
    if ((data & 0x80000) == 0)
    {
        return (float(data)*axis357_sfactor);
    }
    // Otherwise perform the 2's complement math on the value
    return (float((~(data - 0x01)) & 0xfffff) * -1 *axis357_sfactor);
}

/*  Converts temperature data output from the read_reg_u16() function. [degrees Celcius] */
float ADXL355::tempScale(uint16_t data){
    float result;
    result = 25+float(data-1852)/(-9.05);
    return result;
}

/* Performs 2-point calibration: +1g and -1g */ 
void ADXL355::calib2point(float pos, float neg, int axis){
    float m, b;
    m = 2/(pos-neg);
    b = -(pos+neg)/(pos-neg);
    if (axis == 0) {axis355_cal.x_sfactor = axis355_cal.x_sfactor/m; axis355_cal.x_offset = axis355_cal.x_offset-b;}
    else if (axis == 1) {axis355_cal.y_sfactor = axis355_cal.y_sfactor/m; axis355_cal.y_offset = axis355_cal.y_offset-b;}
    else if (axis == 2) {axis355_cal.z_sfactor = axis355_cal.z_sfactor/m; axis355_cal.z_offset = axis355_cal.z_offset-b;}
    else {}
    }
 
/* Performs 4-point calibration based on: https://www.eetimes.com/document.asp?doc_id=1274067&page_number=3*/   
void ADXL355::calib4point(float angle0, float angle90, float angle180, float angle270, int axis){float m, b;
    m = (angle90-angle270)/2;
    b = (angle0-angle180)/2;
    if (axis == 0) {axis355_cal.x_sfactor = axis355_cal.x_sfactor/m; axis355_cal.x_offset = axis355_cal.x_offset-b;}
    else if (axis == 1) {axis355_cal.y_sfactor = axis355_cal.y_sfactor/m; axis355_cal.y_offset = axis355_cal.y_offset-b;}
    else if (axis == 2) {axis355_cal.z_sfactor = axis355_cal.z_sfactor/m; axis355_cal.z_offset = axis355_cal.z_offset-b;}
    else {}
}
    
/* Estimates the angles based on acceletometer data: single axis */
float ADXL355::single_axis(uint32_t x) {
        float X = accelScale(x);
        float Y;
        //int a=4;
        Y = floor(asin(X)*100)/100;
        //void arm_cmplx_mag_f32  (double *Y, double *X, int32_t a);       
        Y = floor(((57.2957f)*(Y))*100)/100;
        return Y;                    
}
/* Estimates the angles based on acceletometer data: dual axis */
float ADXL355::dual_axis(uint32_t x, uint32_t y){
        float Y;
        Y = 57.2957f * (atan(float(x)/float(y)));
        Y = floor(Y*100)/100;
        return Y;    
}
/* Estimates the angles based on acceletometer data: triaxil */
float ADXL355::tri_axis1(uint32_t x, uint32_t y, uint32_t z){
        float Y;
        float X;
        X = float(x)/(sqrt(pow(float(y),2)+pow(float(z),2)));
        Y= atan(X);
        Y = floor(Y*57.2957*100)/100;
}
float ADXL355::tri_axis2(uint32_t x, uint32_t y, uint32_t z){
        float Y;
        float X;
        X = (sqrt(pow(float(y),2)+pow(float(z),2)))/float(x);
        Y= atan(X);
        Y = floor(Y*57.2957*100)/100;
}