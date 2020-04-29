#include "MassAirflowSensor.h"

#include <Wire.h>

//#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
#else
    #define debugPrint    
    #define debugPrintln  
#endif

union singlePrecFloat {
    uint8_t raw[4];
    float fFloatVal;
};

MassAirflowSensor::MassAirflowSensor(uint8_t nDeviceAddress) :
    m_nDeviceAddress(nDeviceAddress)
{
}

MassAirflowSensor::eRetVal MassAirflowSensor::readMeasurement(float* pfFlow, uint16_t* pnRaw, bool bSendMeasCmd)
{
    static eRetVal eStatus;
    static uint16_t nVal;

#ifdef DEBUG
    debugPrint("readMeasurement(");
    debugPrint("pfFlow=");
    debugPrint((long unsigned int)pfFlow);
    debugPrint(", pnRaw=");
    debugPrint((long unsigned int)pnRaw);
    debugPrint(", bSendMeasCmd=");
    debugPrint(bSendMeasCmd);
    debugPrintln(")");
#endif

    if( 0 == pfFlow )
    {
        return SENSOR_PARAM_ERROR; 
    }
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
        eStatus = convertToFlow(&nVal, pfFlow);
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
//  For the Sensirion SFM3000:
    static const uint16_t fOffsetFlow = 32000.0f;
    static const float fScaleFactor = 140.0f;

//  For the Sensirion SFM3200-AW:
//    static const uint16_t fOffsetFlow = 32768.0f;
//    static const float fScaleFactor = 120.0f;

    if( NULL == pnVal || NULL == pfFlow )
    {
        return SENSOR_PARAM_ERROR;
    }
    *pfFlow = ((float)(*pnVal) - fOffsetFlow) / fScaleFactor;

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readSerialNumber(int32_t* pnSerialNo)
{
#ifdef DEBUG
    debugPrint("readSerialNumber(");
    debugPrint("pnSerialNo=");
    debugPrint((long unsigned int)pnSerialNo);
    debugPrintln(")");
#endif

    if( 0 == pnSerialNo )
    {
        return SENSOR_PARAM_ERROR; 
    }
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
#ifdef DEBUG
    debugPrintln("sendReadSerialNumberCmd() start");
#endif

    Wire.beginTransmission(m_nDeviceAddress); // transmit to device
    Wire.write(0x31);
    Wire.write(0xAE);
    Wire.endTransmission(); // stop transmitting

#ifdef DEBUG
    debugPrintln("sendReadSerialNumberCmd() end");
#endif

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

uint8_t MassAirflowSensor::getDeviceAddress(void)
{
    return m_nDeviceAddress;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendSoftResetCmd(void)
{
#ifdef DEBUG
    debugPrintln("sendSoftResetCmd() start");
#endif

    Wire.beginTransmission(m_nDeviceAddress);
    Wire.write(0x20);
    Wire.write(0x00);
    Wire.endTransmission();

#ifdef DEBUG
    debugPrintln("sendSoftResetCmd() end");
#endif

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendStartMeasurementRawVCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress);
    Wire.write(0x42);
    Wire.write(0x00);
    Wire.endTransmission();

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendStartMeasurementRawVoCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress);
    Wire.write(0x42);
    Wire.write(0x01);
    Wire.endTransmission();

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendStartMeasurementRawDpCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress);
    Wire.write(0x42);
    Wire.write(0x02);
    Wire.endTransmission();

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::sendStartMeasurementRawFlCmd(void)
{
    Wire.beginTransmission(m_nDeviceAddress);
    Wire.write(0x42);
    Wire.write(0x03);
    Wire.endTransmission();

    return SENSOR_SUCCESS;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readFloat(float* pfFloat, MassAirflowSensor::eFloatValType eType, bool bSendMeasCmd)
{
    static eRetVal eStatus;

#ifdef DEBUG
    debugPrint("readFloat(");
    debugPrint("pfFloat=");
    debugPrint((long unsigned int)pfFloat);
    debugPrint(", eType=");
    debugPrint((long unsigned int)eType);
    debugPrint(", bSendMeasCmd=");
    debugPrint((long unsigned int)bSendMeasCmd);
    debugPrintln(")");
#endif

    if( 0 == pfFloat )
    {
        return SENSOR_PARAM_ERROR; 
    }
    if( bSendMeasCmd )
    {
        switch( eType )
        {
            case SENSOR_FLOAT_V:
                if( SENSOR_SUCCESS != sendStartMeasurementRawVCmd() )
                {
                    return SENSOR_CMD_ERROR;
                }
                break;
            case SENSOR_FLOAT_VO:
                if( SENSOR_SUCCESS != sendStartMeasurementRawVoCmd() )
                {
                    return SENSOR_CMD_ERROR;
                }
                break;
            case SENSOR_FLOAT_DP:
                if( SENSOR_SUCCESS != sendStartMeasurementRawDpCmd() )
                {
                    return SENSOR_CMD_ERROR;
                }
                break;
            case SENSOR_FLOAT_FL:
                if( SENSOR_SUCCESS != sendStartMeasurementRawFlCmd() )
                {
                    return SENSOR_CMD_ERROR;
                }
                break;
            default:
                return SENSOR_CMD_ERROR;
        }
    }

    eStatus = readFloatValue(pfFloat);

    return eStatus;
}

MassAirflowSensor::eRetVal MassAirflowSensor::readFloatValue(float* pfFloat)
{
    static char sHexBuf[12]; /* string buffer for debug output via UART */
    static uint8_t nMeasRx[5];
    uint8_t nReceived = 0;
    int32_t nVal = 0;
    union singlePrecFloat fFloatUnion;

    Wire.requestFrom(m_nDeviceAddress, (uint8_t)5); // request 5 bytes from slave device
  
    while( Wire.available() ) // slave may send less than requested
    {
        uint8_t cRxByte = Wire.read(); // receive a byte as character
        if( nReceived < 5 )
        {
            // buffer raw bytes and received CRC here
            nMeasRx[nReceived] = cRxByte;
        }
        ++nReceived;
    }

    if( 5 != nReceived )
    {
        *pfFloat = 0.0f;
        debugPrint("[ERROR] Received ");
        debugPrint( nReceived );
        debugPrintln(" instead of 5 bytes.");
        return SENSOR_RXCNT_ERROR;
    }

    // calculate CRC here and check with received CRC
    crc_t cCalculatedCrc = calcCrc(&nMeasRx[0], 4);

    if( nMeasRx[4] != (uint8_t)cCalculatedCrc )
    {
        debugPrintln("[ERROR] CRC does not match.");
        return SENSOR_CRC_ERROR;
    }

    // convert to int32_t and prepare as return value
    fFloatUnion.raw[0] = nMeasRx[0];
    fFloatUnion.raw[1] = nMeasRx[1];
    fFloatUnion.raw[2] = nMeasRx[2];
    fFloatUnion.raw[3] = nMeasRx[3];

    *pfFloat = fFloatUnion.fFloatVal;
    return SENSOR_SUCCESS;
}

