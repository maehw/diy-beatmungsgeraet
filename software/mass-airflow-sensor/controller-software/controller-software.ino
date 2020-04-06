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


float filter(float fIn);

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 8 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 1.3331774107336987 dB

* 30 Hz - 50 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = -29.769303038277595 dB

*/

#define FILTER_TAP_NUM 5

/* FIR filter coefficients */
const float g_fFilterCoeff[FILTER_TAP_NUM] = {
    0.09297173361096804,
    0.2703344310432933,
    0.3499812215322376,
    0.2703344310432933,
    0.09297173361096804
};

/* Filter delay line */
float g_fFilterInput[FILTER_TAP_NUM];


/* define sensor instances with their addresses on the I2C bus */
MassAirflowSensor g_sensor1(0x40);
MassAirflowSensor g_sensor2(0x42);

void setup()
{
    uint32_t nSensorSerialNo = 0xBADC0FFE;

    delay(3000); /* delay added for debugging so that the start of the serial transmission is not missed */

    Serial.begin(230400); // start serial for debug output

    Wire.begin();

#ifdef RT_SUPERVISION_PIN
    pinMode(RT_SUPERVISION_PIN, OUTPUT);
#endif

    resetFilter();

    debugPrintln("Finished setup.");

    if( MassAirflowSensor::SENSOR_SUCCESS == g_sensor1.readSerialNumber(&nSensorSerialNo) )
    {
        debugPrint("Read serial number of sensor 1: ");
        debugPrint(nSensorSerialNo, HEX);
        debugPrintln("");
    }
    else
    {
        debugPrintln("[ERROR] Failed to read serial number of sensor 1.");
        for(;;);
    }

    if( MassAirflowSensor::SENSOR_SUCCESS == g_sensor2.readSerialNumber(&nSensorSerialNo) )
    {
        debugPrint("Read serial number of sensor 2: ");
        debugPrint(nSensorSerialNo, HEX);
        debugPrintln("");
    }
    else
    {
        debugPrintln("[ERROR] Failed to read serial number of sensor 2.");
        for(;;);
    }
}

void loop()
{
    static MassAirflowSensor::eRetVal eStatus1 = MassAirflowSensor::SENSOR_FAIL;
    static MassAirflowSensor::eRetVal eStatus2 = MassAirflowSensor::SENSOR_FAIL;
    static bool bSendMeasCommand1 = true;
    static bool bSendMeasCommand2 = true;
    static float fFlow1 = 0.0f;
    static float fFlow2 = 0.0f;
//    static float fFlowFiltered = 0.0f;
//    static eBreathCyclePhase ePhase = BREATH_UNKNOWN;
//    static eBreathCyclePhase ePhaseOld = BREATH_UNKNOWN;
//    static uint32_t nPhaseCounter = 0;
//    static float fAccu = 0.0f;
//    static float fPhaseDuration = 0.0f;
    
#ifdef DEBUG
    static uint16_t nRaw1 = 0;
    static uint16_t nRaw2 = 0;
#endif

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif

#ifdef DEBUG
    eStatus1 = g_sensor1.readMeasurement(&fFlow1, &nRaw1, bSendMeasCommand1);
    eStatus2 = g_sensor2.readMeasurement(&fFlow2, &nRaw2, bSendMeasCommand2);
#else
    eStatus1 = g_sensor1.readMeasurement(&fFlow1, NULL, bSendMeasCommand1);
    eStatus2 = g_sensor2.readMeasurement(&fFlow2, NULL, bSendMeasCommand2);
#endif

    if( MassAirflowSensor::SENSOR_SUCCESS == eStatus1 )
    {
        debugPrint("[*]");
        bSendMeasCommand1 = false; /* do not resend measurement command after first success */
    }
    else
    {
        debugPrint("[ ]");
    }

    if( MassAirflowSensor::SENSOR_SUCCESS == eStatus2 )
    {
        debugPrint("[*]");
        bSendMeasCommand2 = false; /* do not resend measurement command after first success */
    }
    else
    {
        debugPrint("[ ]");
    }

    debugPrint(" Flow1: ");
    debugPrint( fFlow1 );
    debugPrint(", Flow2: ");
    debugPrintln( fFlow2 );

//    switch( eStatus )
//    {
//        case MassAirflowSensor::SENSOR_SUCCESS:
//            fFlowFiltered = filter(fFlow);
//
//            if( fFlowFiltered >= 1.0f )
//            {
//                ePhase = BREATH_INSPIRATION;
//            }
//            else if( fFlowFiltered <= -1.0f )
//            {
//                ePhase = BREATH_EXPIRATION;
//            }
//
//            if( ePhaseOld != ePhase )
//            {
//                fPhaseDuration = (float)nPhaseCounter/SAMPLES_PER_SECOND;
//                fAccu = fabs(fAccu);
//                if( BREATH_INSPIRATION == ePhaseOld )
//                {
//                    Serial.print("Stopped inspiration after ");
//                    Serial.print( fPhaseDuration );
//                    Serial.print(" seconds with an accumulated volume of ");
//                    Serial.print( fAccu );
//                    Serial.println(" liters.");
//                }
//                else if( BREATH_EXPIRATION == ePhaseOld )
//                {
//                    Serial.print("Stopped expiration after ");
//                    Serial.print( fPhaseDuration );
//                    Serial.print(" seconds with an accumulated volume of ");
//                    Serial.print( fAccu );
//                    Serial.println(" liters.");
//                }
//                nPhaseCounter = 0;
//                fAccu = 0.0f;
//            }
//            ePhaseOld = ePhase;
//            nPhaseCounter++;
//            fAccu += ( fFlowFiltered / (60.0f * SAMPLES_PER_SECOND) );
//
//            /*
//            debugPrint("Raw: ");
//            debugPrint( nRaw );
//            debugPrint(", ");
//            */
//            /*
//            debugPrint("Flow: ");
//            debugPrint( fFlow );
//            debugPrint(" [slm]");
//            */
//            debugPrint("Filtered: ");
//            debugPrint( fFlowFiltered );
//            debugPrint(" [slm]");
//            debugPrint(", Phase: ");
//            debugPrint( ePhase * 10 );
//            debugPrintln("");
//    
//            bSendMeasCommand = false; /* do not resend measurement command after first success */
//            break;
//        case MassAirflowSensor::SENSOR_RXCNT_ERROR:
//            debugPrintln("[ERROR] Receive count error.");
//            break;
//        case MassAirflowSensor::SENSOR_CRC_ERROR:
//            debugPrintln("[ERROR] CRC error.");
//            break;
//        default:
//            debugPrintln("[ERROR] Measurement error.");
//            break;
//    }

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    delay(SENSOR_LOOP_DELAY);
}

void resetFilter()
{
    for(uint8_t i=0; i < FILTER_TAP_NUM; i++)
    {
        g_fFilterInput[i] = 0.0f;
    }
}

float filter(float fIn)
{
    float fOut = 0.0f;

    for(uint8_t i = FILTER_TAP_NUM-1; i > 0; i--)
    {
        g_fFilterInput[i] = g_fFilterInput[i-1]; 
    }
    g_fFilterInput[0] = fIn;

    /* filter kernel: multiply accumulate */
    for(uint8_t i = 0; i < FILTER_TAP_NUM; i++)
    {
        fOut += (g_fFilterCoeff[i] * g_fFilterInput[i]);
    }

    return fOut;
}
