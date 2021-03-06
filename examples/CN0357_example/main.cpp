/**
*   @file     main.cpp
*   @brief    Main file for the CN0357-example project
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: www.analog.com/EVAL-CN0357-ARDZ
* More: https://wiki.analog.com/resources/tools-software/mbed-drivers-all

********************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************/
#include "mbed.h"
#include "CN0357.h"

/** @mainpage
 * CN0357 single-supply, low noise, portable gas detector circuit using an
 * electrochemical sensor. The Alphasense CO-AX carbon monoxide sensor is used
 * in this example. Electrochemical sensors offer several advantages for
 * instruments that detect or measure the concentration of many toxic gases.
 * Most sensors are gas specific and have usable resolutions under one part per
 * million (ppm) of gas concentration.
 *
 * EVAL-CN0357 example program for MBED platforms. To achieve connection between
 * the shield and the ST Nucleo, the following connections need to be made:
 *
 * - ICSP pin 1 -> pin 12 of the Arduino header
 * - ICSP pin 3 -> pin 13 of the Arduino header
 * - ICSP pin 4 -> pin 11 of the Arduino header
 * - if VIN is not supplied, 5V to VIN
 *
 * @page CN0357 CN0357 Analog Devices wiki page
 * @brief link->
 * https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/hardware/cn0357
 * @page AD7790 AD7790 datasheet
 * @brief link ->
 * http://www.analog.com/media/en/technical-documentation/data-sheets/AD7790.pdf
 * @page AD5270 AD5270 datasheet
 * @brief link ->
 * http://www.analog.com/media/en/technical-documentation/data-sheets/AD5270_5271.pdf
 *
 */
const float SENSOR_RANGE = 2000; ///< gas sensor range in PPM
const float SENSOR_SENSITIVITY = (65 * pow(10, -9)); ///< gas sensor sensitivity in A/ppm - 65 nA/ppm

Serial pc(USBTX, USBRX); ///< Serial interface to the pc

/**
 @brief Displays CN0357 circuit readings and data to the UART

 @param ui16Data - ADC data register value to be displayed
 @param fData1   - ADC input voltage reading to be displayed
 @param fdata2   - Gas Concentration reading to be displayed

 **/
void display_data(uint8_t ui8Status_Reg, uint16_t ui16Data, float fData1, float fdata2)
{
    pc.printf("\r\nStatus Register value: 0x%x", ui8Status_Reg);
    pc.printf("\r\nADC Data Register Value = %#08x", ui16Data); /** Send valid ADC data register value*/
    pc.printf("\r\nADC Input Voltage input = %f V", fData1); /** Send valid voltage input value */
    pc.printf("\r\nGas Concentration = %f PPM", fdata2); /** Send valid gas concentration value */

    pc.printf("\r\n");
}

/**
 * Project entry-point - initializes the CN0357 shield, reads the data when the ADC is ready and outputs the sensor
 * value in PPM
 */

#define SINGLE_CONVERSION
//#define CONTINOUS_CONVERSION

int main()
{
    /* Main variables */
    CN0357 cn0357;
    uint8_t ui8Status_Reg = 0;
#ifdef SINGLE_CONVERSION
    cn0357.init(SENSOR_RANGE, SENSOR_SENSITIVITY);
#elif defined CONTINOUS_CONVERSION
    cn0357.init(SENSOR_RANGE, SENSOR_SENSITIVITY, CN0357::INTERNAL_AD7790, 0x00, 0x07);
#else
#error define SINGLE_CONVERSION or CONTINOUS_CONVERSION, but not both
#endif


    /* Infinite loop */
    while (1) {
        wait_ms(1000);
#ifdef CONTINOUS_CONVERSION
        ui8Status_Reg = cn0357.read_adc_status(); //  Read ADC Status Register        //

        if (ui8Status_Reg != 0x08) { //  Checks if ADC data is available
            pc.printf("\r\nStatus Register value: 0x%x", ui8Status_Reg);
        } else
#endif
        {
            uint16_t ui16Adcdata = cn0357.read_sensor();
            float fAdcVoltage    = cn0357.data_to_voltage(ui16Adcdata); //  Convert ADC data to voltage
            float fConcentration = cn0357.calc_ppm(fAdcVoltage); //  Convert voltage to Gas concentration
            display_data(ui8Status_Reg, ui16Adcdata, fAdcVoltage, fConcentration); //  Display data thru UART
        }
    }


    /* Infinite loop, never returns. */
}

