/**
 * MassAirflowSensor.h - Library for mass airflow sensors.
 */

#ifndef _MASSAIRFLOWSENSOR_H_
#define _MASSAIRFLOWSENSOR_H_

#include "Arduino.h"
#include "crc8.h"

class MassAirflowSensor
{
public:
    /* typedef return values of the sensor API */
    enum eRetVal { SENSOR_SUCCESS = 0, SENSOR_FAIL, SENSOR_CRC_ERROR, SENSOR_CMD_ERROR, SENSOR_RXCNT_ERROR, SENSOR_PARAM_ERROR };

    /* typedef floating point debug values of the sensor */
    enum eFloatValType { SENSOR_FLOAT_V = 0, SENSOR_FLOAT_VO, SENSOR_FLOAT_DP, SENSOR_FLOAT_FL, SENSOR_FLOAT_VOFF };

    MassAirflowSensor(uint8_t nDeviceAddress);
    uint8_t getDeviceAddress(void);
    eRetVal readMeasurement(float* pfFlow, uint16_t* pnRaw, bool bSendMeasCmd);
    eRetVal readFloat(float* pfFloat, eFloatValType eType, bool bSendMeasCmd);
    eRetVal readSerialNumber(int32_t* pnSerialNo);
    eRetVal sendSoftResetCmd(void);


private:
    eRetVal sendStartMeasurementCmd(void);
    eRetVal readMeasurementValue(uint16_t* pnVal);
    eRetVal readFloatValue(float* pnVal);
    eRetVal convertToFlow(uint16_t* pnVal, float* pfFlow);
    eRetVal sendReadSerialNumberCmd(void);
    eRetVal readSerialNumberValue(int32_t* pnSerialNo);
    eRetVal sendStartMeasurementRawVCmd(void);
    eRetVal sendStartMeasurementRawVoCmd(void);
    eRetVal sendStartMeasurementRawDpCmd(void);
    eRetVal sendStartMeasurementRawFlCmd(void);
    eRetVal sendStartMeasurementVoffCmd(void);
    crc_t calcCrc(const unsigned char *pData, size_t nDataLen);

    uint8_t m_nDeviceAddress;

};

#endif /* _MASSAIRFLOWSENSOR_H_ */

