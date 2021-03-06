// Test setup:
//   Arduino Leonardo:
//     I2C SDA:      2
//     I2C SCL:      3
//     Real-time supervision pin: 4

#include "MassAirflowSensor.h"
#include <Wire.h>

/* Mapping of the digital output pin for real-time supervision, define alias here */
#define RT_SUPERVISION_PIN    (4)

/* Define 'DEBUG' in order to get additional output via the serial console;
 * undefine for speed-up and potential higher rates
 */
#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
#else
    #define debugPrint    
    #define debugPrintln  
#endif

/* Definition of interval (in milliseconds) to query the sensor via I2C for new measurement.
 * Current setting: query approx. every 20 ms, i.e. at a rate of 50 Hz.
 * Note: This is not exact as there will be plenty of overhead.
  *      When DEBUG output is active, it takes about 1 ms per cycle.
  *      When it's not active, it will take approx. 0.8 ms per cycle also, so subtract 1 ms here.
 * Caution: The interrupt overhead is not subtracted automatically.
 */
#define SENSOR_QUERY_INTERVAL (10u)
#define SAMPLES_PER_SECOND    (1000u / SENSOR_QUERY_INTERVAL)
#ifdef DEBUG
    #define SENSOR_LOOP_DELAY (SENSOR_QUERY_INTERVAL - (1u))
#else
    #define SENSOR_LOOP_DELAY (SENSOR_QUERY_INTERVAL - (1u))
#endif

enum eBreathCyclePhase { BREATH_UNKNOWN, BREATH_INSPIRATION, BREATH_EXPIRATION };

#define NUM_SENSORS   (2u)

/* define sensor instances with their addresses on the I2C bus */
MassAirflowSensor g_sensor[NUM_SENSORS] = { MassAirflowSensor(0x40), MassAirflowSensor(0x42) }; //, MassAirflowSensor(0x44) };

/* define global variables required for the measurement of volume flow for every sensor */
bool g_bSendMeasCommand[NUM_SENSORS];
MassAirflowSensor::eRetVal g_eStatus[NUM_SENSORS];
float g_fValue[NUM_SENSORS]; /* Volume flow (or alternative raw debug) measurement related variables */

#ifdef DEBUG
    uint16_t g_nRaw[NUM_SENSORS];
#endif

void setup()
{
    uint32_t nSensorSerialNo = 0xBADC0FFE;
    uint8_t nRetryCount = 3;
    bool bInitSuccess = false;

    delay(3000); /* delay added for debugging so that the start of the serial transmission is not missed */

    Serial.begin(230400); // start serial for debug output

    Wire.begin();

#ifdef RT_SUPERVISION_PIN
    pinMode(RT_SUPERVISION_PIN, OUTPUT);
#endif

    debugPrintln("Finished configuration. Reading serial numbers...");

    /* Initialize array values to handle multiple sensors */
    for( uint8_t nSensorIdx = 0; nSensorIdx < NUM_SENSORS; nSensorIdx++ )
    {
        g_bSendMeasCommand[nSensorIdx] = true;
        g_eStatus[nSensorIdx] = MassAirflowSensor::SENSOR_FAIL;
        g_fValue[nSensorIdx] = 0.0f;
#ifdef DEBUG
        g_nRaw[nSensorIdx] = 0;
#endif
    }

    while( !bInitSuccess && nRetryCount > 0 )
    {
        bInitSuccess = true;
        debugPrintln("");
        debugPrint("[INFO] Init retry count: ");
        debugPrintln(nRetryCount);

        for( uint8_t nSensorIdx = 0; nSensorIdx < NUM_SENSORS; nSensorIdx++ )
        {
            if( MassAirflowSensor::SENSOR_SUCCESS == g_sensor[nSensorIdx].sendSoftResetCmd() )
            {
                debugPrint("[INFO] Sent soft reset command to sensor #");
                debugPrint(nSensorIdx+1);
                debugPrintln(".");
            }
            else
            {
                debugPrint("[ERROR] Failed to send soft reset command to sensor #");
                bInitSuccess = false;
            }
            delay(90); /* soft reset time is 80 ms */
    
            if( MassAirflowSensor::SENSOR_SUCCESS == g_sensor[nSensorIdx].readSerialNumber(&nSensorSerialNo) )
            {
                debugPrint("[INFO] Read serial number of sensor #");
                debugPrint(nSensorIdx+1);
                debugPrint(": ");
                debugPrint(nSensorSerialNo, HEX);
                debugPrintln(".");
            }
            else
            {
                debugPrint("[ERROR] Failed to read serial number of sensor #");
                debugPrint(nSensorIdx+1);
                debugPrint(" (bus address: 0x");
                debugPrint(g_sensor[nSensorIdx].getDeviceAddress(), HEX);
                debugPrintln(").");
                switch( g_eStatus[nSensorIdx] )
                {
                    case MassAirflowSensor::SENSOR_CRC_ERROR:
                        debugPrintln("[ERROR] CRC error.");
                        break;
                    case MassAirflowSensor::SENSOR_CMD_ERROR:
                        debugPrintln("[ERROR] Command error.");
                        break;
                    case MassAirflowSensor::SENSOR_RXCNT_ERROR:
                        debugPrintln("[ERROR] RX count error.");
                        break;
                    case MassAirflowSensor::SENSOR_PARAM_ERROR:
                        debugPrintln("[ERROR] Parameter error.");
                        break;
                    case MassAirflowSensor::SENSOR_FAIL:
                    default:
                        debugPrintln("[ERROR] Generic failure. Check connectivity!");
                        break;
                }
                bInitSuccess = false;
                delay(500);
            }
            delay(500);
        }

        nRetryCount--;
    }

    if( !bInitSuccess )
    {
        debugPrintln("[ERROR] Init failed.");
        for(;;); /* halt on error */
    }

    debugPrintln("Finished setup().");
}

void loop()
{
    /* Breath cycle / accumulated volume related variables */
    static eBreathCyclePhase ePhase = BREATH_UNKNOWN;
    static eBreathCyclePhase ePhaseOld = BREATH_UNKNOWN;
    static float fAccu1 = 0.0f;
    static float fAccu2 = 0.0f;
    static uint32_t nPhaseCounter = 0;
    static float fPhaseDuration = 0.0f;

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif
    /* Query data from sensor #0 (usually the reference sensor) */
    g_eStatus[0] = g_sensor[0].readMeasurement(&g_fValue[0], &g_nRaw[0], g_bSendMeasCommand[0]); /* request volume flow value */

    /* Query data from sensor #1 (usually the DIY sensor) */
    g_eStatus[1] = g_sensor[1].readMeasurement(&g_fValue[1], &g_nRaw[1], g_bSendMeasCommand[1]); /* request volume flow value */
    /* The following raw floating point measurement readouts are only supported by the DIY sensor */
    //g_eStatus[1] = g_sensor[1].readFloat(&g_fValue[1], MassAirflowSensor::SENSOR_FLOAT_V, g_bSendMeasCommand[1]); /* request raw voltage values (for debugging/verification) */
    //g_eStatus[1] = g_sensor[1].readFloat(&g_fValue[1], MassAirflowSensor::SENSOR_FLOAT_VOFF, g_bSendMeasCommand[1]); /* request offset voltage value (for debugging/verification) */
    //g_eStatus[1] = g_sensor[1].readFloat(&g_fValue[1], MassAirflowSensor::SENSOR_FLOAT_VO, g_bSendMeasCommand[1]); /* request offset compensated voltage values (for debugging/verification) */
    //g_eStatus[1] = g_sensor[1].readFloat(&g_fValue[1], MassAirflowSensor::SENSOR_FLOAT_DP, g_bSendMeasCommand[1]); /* request differential pressure values (for debugging/verification) */
    //g_eStatus[1] = g_sensor[1].readFloat(&g_fValue[1], MassAirflowSensor::SENSOR_FLOAT_FL, g_bSendMeasCommand[1]); /* request raw volume flow values (for debugging/verification) */

    /* check return codes for every sensor */
    for( uint8_t nSensorIdx = 0; nSensorIdx < NUM_SENSORS; nSensorIdx++ )
    {
        if( MassAirflowSensor::SENSOR_SUCCESS == g_eStatus[nSensorIdx] )
        {
            debugPrint("[*]");
            g_bSendMeasCommand[nSensorIdx] = false; /* do not resend measurement command after first success */
        }
        else
        {
            switch( g_eStatus[nSensorIdx] )
            {
                case MassAirflowSensor::SENSOR_CRC_ERROR:
                    debugPrint("[C]");
                    break;
                case MassAirflowSensor::SENSOR_CMD_ERROR:
                    debugPrint("[M]");
                    break;
                case MassAirflowSensor::SENSOR_RXCNT_ERROR:
                    debugPrint("[R]");
                    break;
                case MassAirflowSensor::SENSOR_PARAM_ERROR:
                    debugPrint("[P]");
                    break;
                case MassAirflowSensor::SENSOR_FAIL:
                default:
                    debugPrint("[E]");
                    break;
            }
            //g_bSendMeasCommand[nSensorIdx] = true; /* do resend measurement command due to error condition */
        }
    }

    debugPrint(" First: ");
    debugPrint( g_fValue[0] );
    /* Caution: number of ouputs should match NUM_SENSORS */
    debugPrint(", Second: ");
    debugPrint( g_fValue[1] );
    debugPrintln("");

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    delay(SENSOR_LOOP_DELAY);
}

