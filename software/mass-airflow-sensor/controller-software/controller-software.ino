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

/* if DEBUG_PLOT is defined there's a constant flow of data,
 * otherwise breath cycles will be estimated
 */
#define DEBUG_PLOT

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

#define NUM_SENSORS   (3u)

/* define sensor instances with their addresses on the I2C bus */
MassAirflowSensor g_sensor[NUM_SENSORS] = { MassAirflowSensor(0x40), MassAirflowSensor(0x44), MassAirflowSensor(0x42) };

/* define global variables required for the measurement of volume flow for every sensor */
bool g_bSendMeasCommand[NUM_SENSORS];
MassAirflowSensor::eRetVal g_eStatus[NUM_SENSORS];
float g_fFlow[NUM_SENSORS]; /* Volume flow measurement related variables */

#ifdef DEBUG
    uint16_t g_nRaw[NUM_SENSORS];
#endif

void setup()
{
    uint32_t nSensorSerialNo = 0xBADC0FFE;

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
        g_fFlow[nSensorIdx] = 0.0f;
#ifdef DEBUG
        g_nRaw[nSensorIdx] = 0;
#endif
    }

    for( uint8_t nSensorIdx = 0; nSensorIdx < NUM_SENSORS; nSensorIdx++ )
    {
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
            debugPrintln(".");
            switch( g_eStatus[nSensorIdx] )
            {
                case MassAirflowSensor::SENSOR_CRC_ERROR:
                    debugPrint("[ERROR] CRC error.");
                    break;
                case MassAirflowSensor::SENSOR_CMD_ERROR:
                    debugPrint("[ERROR] Command error.");
                    break;
                case MassAirflowSensor::SENSOR_RXCNT_ERROR:
                    debugPrint("[ERROR] RX count error.");
                    break;
                case MassAirflowSensor::SENSOR_PARAM_ERROR:
                    debugPrint("[ERROR] Parameter error.");
                    break;
                case MassAirflowSensor::SENSOR_FAIL:
                default:
                    debugPrint("[ERROR] Generic failure. Check connectivity!");
                    break;
            }
            delay(500);
            for(;;); /* halt on error */
        }
        delay(500);

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

    for( uint8_t nSensorIdx = 0; nSensorIdx < NUM_SENSORS; nSensorIdx++ )
    {
#ifdef DEBUG
        g_eStatus[nSensorIdx] = g_sensor[nSensorIdx].readMeasurement(&g_fFlow[nSensorIdx], &g_nRaw[nSensorIdx], g_bSendMeasCommand[nSensorIdx]);
#else
        g_eStatus[nSensorIdx] = g_sensor[nSensorIdx].readMeasurement(&g_fFlow[nSensorIdx], NULL, g_bSendMeasCommand[nSensorIdx]);
#endif

        if( MassAirflowSensor::SENSOR_SUCCESS == g_eStatus[nSensorIdx] )
        {
#ifdef DEBUG_PLOT
            debugPrint("[*]");
#endif
            g_bSendMeasCommand[nSensorIdx] = false; /* do not resend measurement command after first success */
        }
#ifdef DEBUG_PLOT
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
#endif
    }

#ifdef DEBUG_PLOT
    debugPrint(" First: ");
    debugPrint( g_fFlow[0] );
    debugPrint(", Second: ");
    debugPrint( g_fFlow[1] );
    debugPrint(", Third: ");
    debugPrint( g_fFlow[2] );
    
    debugPrintln("");
#else
    /* Allow estimation of accumulated volume */
    if( fFlow1 >= 1.0f )
    {
        ePhase = BREATH_INSPIRATION;
    }
    else
    {
        ePhase = BREATH_UNKNOWN;
    }

    if( ePhaseOld != ePhase )
    {
        fPhaseDuration = (float)nPhaseCounter/SAMPLES_PER_SECOND;
        fAccu1 = fabs(fAccu1) / (60.0f * SAMPLES_PER_SECOND);
        fAccu2 = fabs(fAccu2) / (60.0f * SAMPLES_PER_SECOND);
        if( BREATH_INSPIRATION == ePhaseOld )
        {
//            Serial.print("Stopped inspiration after ");
//            Serial.print( fPhaseDuration );
//            Serial.print(" seconds w/ an accumulated volume of ");
//            Serial.print( fAccu1 );
//            Serial.print(" / ");
//            Serial.print( fAccu2 );
//            Serial.print(" liters.");
//            Serial.println("");
              Serial.print( fAccu1*1000 );
              Serial.print(";");
              Serial.print( fAccu2*1000 );
              Serial.print(";");
              float fDev = fabs(fAccu1 - fAccu2)/fabs(fAccu1) * 100.0f;
              Serial.print(fDev);
              Serial.print("%");
              Serial.println("");
        }
        else
        {
//            Serial.print("Stopped pause after ");
//            Serial.print( fPhaseDuration );
//            Serial.println("");
        }

        /* Start a new phase: reset counter and accumulated volume */
        nPhaseCounter = 0;
        fAccu1 = 0.0f;
        fAccu2 = 0.0f;
    }
    ePhaseOld = ePhase;
    nPhaseCounter++;

    /* TODO/FIXME: be aware of floating-point inaccuricy */
    fAccu1 += fFlow1;
    fAccu2 += fFlow2;
#endif

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    delay(SENSOR_LOOP_DELAY);
}

