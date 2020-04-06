#include "MassAirflowSensor.h"

#include <Wire.h>

#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
#else
    #define debugPrint    
    #define debugPrintln  
#endif

MassAirflowSensor::MassAirflowSensor(uint8_t nDeviceAddress) :
    m_nDeviceAddress(nDeviceAddress)
{
}

MassAirflowSensor::eRetVal MassAirflowSensor::readMeasurement(float* pfFlow, uint16_t* pnRaw, bool bSendMeasCmd)
{
    static eRetVal eStatus;
    static uint16_t nVal;

    if( bSendMeasCmd )
    {
        if( SENSOR_SUCCESS != sendStartMeasurementCmd() )
        {
            return SENSOR_CMD_ERROR;
        }
    }

    eStatus = readMeasurementValue(&nVal);
    if( eStatus == SENSOR_SUCCESS )
    {
        if( NULL != pnRaw )
        {
            *pnRaw = nVal;
        }
        convertToFlow(&nVal, pfFlow);
    }

    return eStatus;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendStartMeasurementCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress); // transmit to device
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readMeasurementValue(uint16_t* pnVal)
{
    static char sHexBuf[12]; /* string buffer for debug output via UART */
    static uint8_t nMeasRx[3];
    uint8_t nReceived = 0;
    int16_t nVal = 0;

    Wire.requestFrom(m_nDeviceAddress, (uint8_t)3); // request 3 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        uint8_t cRxByte = Wire.read(); // receive a byte as character
        if( nReceived < 3 )
        {
            // buffer raw bytes and received CRC here
            nMeasRx[nReceived] = cRxByte;
        }
        ++nReceived;
    }

    if( 3 != nReceived )
    {
        *pnVal = 0xDEAD;
        debugPrint("[ERROR] Received ");
        debugPrint( nReceived );
        debugPrintln(" instead of 3 bytes.");
        return SENSOR_RXCNT_ERROR;
    }

    // calculate CRC here and check with received CRC
    crc_t cCalculatedCrc = calcCrc(&nMeasRx[0], 2);

    if( nMeasRx[2] != (uint8_t)cCalculatedCrc )
    {
        debugPrintln("[ERROR] CRC does not match.");
        return SENSOR_CRC_ERROR;
    }

    // convert to int16_t and prepare as return value
    nVal = ((uint16_t)nMeasRx[0] << 8) | (uint16_t)nMeasRx[1];

    *pnVal = nVal;
    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::convertToFlow(uint16_t* pnVal, float* pfFlow)
{
    static const uint16_t fOffsetFlow = 32000.0f;
    static const float fScaleFactor = 140.0f;

    if( NULL == pnVal || NULL == pfFlow )
    {
        return SENSOR_PARAM_ERROR;
    }
    *pfFlow = ((float)(*pnVal) - fOffsetFlow) / fScaleFactor;

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readSerialNumber(int32_t* pnSerialNo)
{
    if( SENSOR_SUCCESS != sendReadSerialNumberCmd() )
    {
        return SENSOR_CMD_ERROR; 
    }
    if( SENSOR_SUCCESS != readSerialNumberValue(pnSerialNo) )
    {
        return SENSOR_FAIL;
    }

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendReadSerialNumberCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress); // transmit to device
    Wire.write(0x31);
    Wire.write(0xAE);
    Wire.endTransmission(); // stop transmitting

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readSerialNumberValue(int32_t* pnSerialNo)
{
    uint8_t nReceived = 0;
    uint32_t nSerialNo = 0;
    char sHexBuf[6]; /* string buffer for debug output via UART */

    Wire.requestFrom(m_nDeviceAddress, (uint8_t)6); // request 6 bytes from slave device

    debugPrint("Raw serial number data: ");
    while( Wire.available() ) // slave may send less than requested
    {
        uint8_t cRxByte = Wire.read(); // receive a byte as character
        // TODO: do something with the serial number here!
        //       i.e. prepare 32 bit return value and check CRC

        if( (nReceived >= 0 && nReceived <= 1) || (nReceived >= 3 && nReceived <= 4) )
        {
            nSerialNo = nSerialNo << 8;
            nSerialNo |= cRxByte;
        }
        ++nReceived;

        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        sHexBuf[5] = 0;
        debugPrint(sHexBuf);
    }
    debugPrintln("");

    if( 6 != nReceived )
    {
        *pnSerialNo = 0xDEADBEEF;
        return SENSOR_FAIL;
    }

    *pnSerialNo = nSerialNo;
    return SENSOR_SUCCESS;
}

crc_t MassAirflowSensor::calcCrc(const unsigned char *pData, size_t nDataLen)
{
    crc_t nCrc = crc_init();
    nCrc = crc_update(nCrc, pData, nDataLen);
    nCrc = crc_finalize(nCrc);

    return nCrc;
}
