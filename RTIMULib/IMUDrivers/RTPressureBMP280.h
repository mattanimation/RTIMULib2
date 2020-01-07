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

#ifndef _RTPRESSUREBMP280_H_
#define _RTPRESSUREBMP280_H_

#include "RTPressure.h"

//  State definitions

#define BMP280_STATE_IDLE               0
#define BMP280_STATE_TEMPERATURE        1
#define BMP280_STATE_PRESSURE           2

#define SLEEP_MODE		        	0
#define FORCED_MODE		        	1
#define NORMAL_MODE		        	3

//  Conversion reg defs

#define BMP280_SCO_TEMPCONV             0x27                // temperature conversion
#define BMP280_SCO_SKIPEPD              0                   // skipped
#define BMP280_SCO_PRESSURECONV_ULP     1                   // ultra low power pressure conversion
#define BMP280_SCO_PRESSURECONV_LP      2                   // low power pressure conversion
#define BMP280_SCO_PRESSURECONV_STD     3                   // standard pressure conversion
#define BMP280_SCO_PRESSURECONV_HR      4                   // high res pressure conversion
#define BMP280_SCO_PRESSURECONV_UHR     5                   // ultra high res pressure conversion

#define SKIPPED				        0
#define ULTRA_LOW_POWER			    1
#define LOW_POWER		        	2
#define STANDARD_RESOLUTION	    	3
#define HIGH_RESOLUTION		    	4
#define ULTRA_HIGH_RESOLUTION		5

#define LEVEL0                      0
#define LEVEL1                      1
#define LEVEL2		        		2
#define LEVEL3		        		3
#define LEVEL4		        		4
#define LEVEL5			        	5


/*! @name Calibration parameters' relative position */
#define BMP280_DIG_T1_LSB_POS                0
#define BMP280_DIG_T1_MSB_POS                1
#define BMP280_DIG_T2_LSB_POS                2
#define BMP280_DIG_T2_MSB_POS                3
#define BMP280_DIG_T3_LSB_POS                4
#define BMP280_DIG_T3_MSB_POS                5
#define BMP280_DIG_P1_LSB_POS                6
#define BMP280_DIG_P1_MSB_POS                7
#define BMP280_DIG_P2_LSB_POS                8
#define BMP280_DIG_P2_MSB_POS                9
#define BMP280_DIG_P3_LSB_POS                10
#define BMP280_DIG_P3_MSB_POS                11
#define BMP280_DIG_P4_LSB_POS                12
#define BMP280_DIG_P4_MSB_POS                13
#define BMP280_DIG_P5_LSB_POS                14
#define BMP280_DIG_P5_MSB_POS                15
#define BMP280_DIG_P6_LSB_POS                16
#define BMP280_DIG_P6_MSB_POS                17
#define BMP280_DIG_P7_LSB_POS                18
#define BMP280_DIG_P7_MSB_POS                19
#define BMP280_DIG_P8_LSB_POS                20
#define BMP280_DIG_P8_MSB_POS                21
#define BMP280_DIG_P9_LSB_POS                22
#define BMP280_DIG_P9_MSB_POS                23
#define BMP280_CALIB_DATA_SIZE               24

class RTIMUSettings;

class RTPressureBMP280 : public RTPressure
{
public:
    RTPressureBMP280(RTIMUSettings *settings);
    ~RTPressureBMP280();

    virtual const char *pressureName() { return "BMP280"; }
    virtual int pressureType() { return RTPRESSURE_TYPE_BMP280; }
    virtual bool pressureInit();
    virtual bool pressureRead(RTIMU_DATA& data);

private:
    void readPressureAndTemperature();
    int8_t get_comp_pres_double(double *pressure, uint32_t uncomp_pres);
    int8_t get_comp_temp_double(double *temperature, int32_t uncomp_temp);

    unsigned char m_pressureAddr;                           // I2C address
    RTFLOAT m_pressure;                                     // the current pressure
    RTFLOAT m_temperature;                                  // the current temperature

    // This is the calibration data read from the sensor
    // temp coeffs
    int16_t m_T1;
    int16_t m_T2;
    int16_t m_T3;

    //pressure coeffs
    uint16_t m_P1;
    uint16_t m_P2;
    uint16_t m_P3;
    int16_t m_P4;
    int16_t m_P5;
    int16_t m_P6;
    int16_t m_P7;
    int16_t m_P8;
    int16_t m_P9;
    int32_t t_fine;

    int m_state;
    int m_oss;

    uint32_t m_rawPressure;
    uint32_t m_rawTemperature;

    bool m_validReadings;
};

#endif // _RTPRESSUREBMP280_H_

