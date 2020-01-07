////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTPressureBMP280.h"

RTPressureBMP280::RTPressureBMP280(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
}

RTPressureBMP280::~RTPressureBMP280()
{
}

bool RTPressureBMP280::pressureInit()
{
    unsigned char result;
    unsigned char data[BMP280_CALIB_DATA_SIZE];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // check ID of chip

    if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_ID, 1, &result, "Failed to read BMP280 id"))
        return false;

    if (result != BMP280_ID) {
        HAL_ERROR1("Incorrect BMP280 id %d\n", result);
        return false;
    }

    // get calibration data

    if (!m_settings->HALRead(m_pressureAddr, BMP280_DIG_T1_LSB_ADDR, BMP280_CALIB_DATA_SIZE, data, "Failed to read BMP280 calibration data"))
        return false;

    m_T1 = (uint16_t) (((uint16_t) data[BMP280_DIG_T1_MSB_POS] << 8) | ((uint16_t) data[BMP280_DIG_T1_LSB_POS]));
    m_T2 = (int16_t) (((int16_t) data[BMP280_DIG_T2_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_T2_LSB_POS]));
    m_T3 = (int16_t) (((int16_t) data[BMP280_DIG_T3_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_T3_LSB_POS]));
    m_P1 = (uint16_t) (((uint16_t) data[BMP280_DIG_P1_MSB_POS] << 8) | ((uint16_t) data[BMP280_DIG_P1_LSB_POS]));
    m_P2 = (int16_t) (((int16_t) data[BMP280_DIG_P2_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P2_LSB_POS]));
    m_P3 = (int16_t) (((int16_t) data[BMP280_DIG_P3_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P3_LSB_POS]));
    m_P4 = (int16_t) (((int16_t) data[BMP280_DIG_P4_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P4_LSB_POS]));
    m_P5 = (int16_t) (((int16_t) data[BMP280_DIG_P5_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P5_LSB_POS]));
    m_P6 = (int16_t) (((int16_t) data[BMP280_DIG_P6_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P6_LSB_POS]));
    m_P7 = (int16_t) (((int16_t) data[BMP280_DIG_P7_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P7_LSB_POS]));
    m_P8 = (int16_t) (((int16_t) data[BMP280_DIG_P8_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P8_LSB_POS]));
    m_P9 = (int16_t) (((int16_t) data[BMP280_DIG_P9_MSB_POS] << 8) | ((int16_t) data[BMP280_DIG_P9_LSB_POS]));

    m_state = BMP280_STATE_IDLE;
    m_oss = m_settings->m_pressureRate; // BMP280_SCO_PRESSURECONV_ULP;
    return true;
}

bool RTPressureBMP280::pressureRead(RTIMU_DATA& data)
{
    data.pressureValid = false;
    data.temperatureValid = false;
    data.temperature = 0;
    data.pressure = 0;

    if (m_state == BMP280_STATE_IDLE) {
        // start a temperature conversion
        if (!m_settings->HALWrite(m_pressureAddr, BMP280_REG_CONTROL, BMP280_SCO_TEMPCONV, "Failed to start temperature conversion")) {
            return false;
        } else {
            m_state = BMP280_STATE_TEMPERATURE;
        }
    }

    readPressureAndTemperature();

    if (m_validReadings) {
        data.pressureValid = true;
        data.temperatureValid = true;
        data.temperature = m_temperature;
        data.pressure = m_pressure;
        // printf("P: %f, T: %f\n", m_pressure, m_temperature);
    }
    return true;
}


void RTPressureBMP280::readPressureAndTemperature()
{
    uint8_t data[6];

    switch (m_state) {
        case BMP280_STATE_IDLE:
        break;

        case BMP280_STATE_TEMPERATURE:
        if (!m_settings->HALRead(m_pressureAddr, BMP280_STATUS, 1, data, "Failed to read BMP280 temp conv status")) {
            break;
        }
        if ((data[0] & 0x20) == 0x20)
            break;                                      // conversion not finished
        if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_TEMPDATA, 2, data, "Failed to read BMP280 temp conv result")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }
        //m_rawTemperature = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];
        m_rawTemperature = (int32_t) ((((int32_t) (data[3])) << 12) | (((int32_t) (data[4])) << 4) | (((int32_t) (data[5])) >> 4));

        data[0] = 0x34 + (m_oss << 6);
        if (!m_settings->HALWrite(m_pressureAddr, BMP280_STATUS, 1, data, "Failed to start pressure conversion")) {
            m_state = BMP280_STATE_IDLE;
            break;
        }
        m_state = BMP280_STATE_PRESSURE;
        break;

        case BMP280_STATE_PRESSURE:
            if (!m_settings->HALRead(m_pressureAddr, BMP280_STATUS, 1, data, "Failed to read BMP280 pressure conv status")) {
                break;
            }
            if ((data[0] & 0x20) == 0x20)
                break;                                      // conversion not finished
            
            if (!m_settings->HALRead(m_pressureAddr, BMP280_REG_PRESSUREDATA, 2, data, "Failed to read BMP280 temp conv result")) {
                m_state = BMP280_STATE_IDLE;
                break;
            }
            //m_rawPressure = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];
            m_rawPressure = (int32_t) ((((uint32_t) (data[0])) << 12) | (((uint32_t) (data[1])) << 4) | ((uint32_t) data[2] >> 4));

            if (!m_settings->HALRead(m_pressureAddr, BMP280_PRES_XLSB, 1, data, "Failed to read BMP280 XLSB")) {
                m_state = BMP280_STATE_IDLE;
                break;
            }

            int8_t rslt;
            double pres;
            double tmp;
            rslt = get_comp_temp_double(&tmp, m_rawTemperature);
            rslt = get_comp_pres_double(&pres, m_rawPressure);

            m_state = BMP280_STATE_IDLE;

            m_pressure = (RTFLOAT) pres;
            m_temperature = (RTFLOAT) tmp;

            m_validReadings = true;


        break;
    }
}


int8_t RTPressureBMP280::get_comp_pres_double(double *pressure, uint32_t uncomp_pres)
{
    double var1, var2;
    int8_t rslt = 0;
    
    var1 = ((double) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)  m_P6) / 32768.0;
    var2 = var2 + var1 * ((double)  m_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)  m_P4) * 65536.0);
    var1 = (((double) m_P3) * var1 * var1 / 524288.0 + ((double) m_P2) * var1) /
            524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)  m_P1);

    *pressure = 1048576.0 - (double)uncomp_pres;
    if (var1 < 0 || var1 > 0)
    {
        *pressure = (*pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double) m_P9) * (*pressure) * (*pressure) / 2147483648.0;
        var2 = (*pressure) * ((double) m_P8) / 32768.0;
        *pressure = *pressure + (var1 + var2 + ((double) m_P7)) / 16.0;
    }
    else
    {
        *pressure = 0;
        rslt = -18; // error code BMP280_E_DOUBLE_COMP_PRESS
    }
    

    return rslt;
}

int8_t RTPressureBMP280::get_comp_temp_double(double *temperature, int32_t uncomp_temp)
{
    double var1, var2;
    int8_t rslt = 0;

    var1 = (((double) uncomp_temp) / 16384.0 - ((double) m_T1) / 1024.0) * ((double) m_T2);
    var2 = ((((double) uncomp_temp) / 131072.0 - ((double) m_T1) / 8192.0) * (((double) uncomp_temp) / 131072.0 - ((double) m_T1) / 8192.0)) * ((double) m_T3);
    t_fine = (int32_t) (var1 + var2);
    *temperature = ((var1 + var2) / 5120.0);

    return rslt;
}


