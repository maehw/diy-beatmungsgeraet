#include <Wire.h>
#include <EEPROM.h>
#include "crc8.h"

// Test setup:
//   Arduino Nano pin asssignment:
//     I2C SDA:      A4
//     I2C SCL:      A5
//     Analog input: A0
//     Real-time supervision pin: A3
//     Interrupt supervision pin: A2
//   TODO/FIXME: EEPROM is used to store sensor offset voltage for calibration

/* Mapping of the analog input pin, define alias here */
#define ANALOG_INPUT_PIN      (A0)

/* Mapping of the digital output pin for real-time supervision, define alias here */
#define RT_SUPERVISION_PIN    (A3)

/* Mapping of the digital output pin for interrupt supervision, define alias here */
#define INT_SUPERVISION_PIN   (A2)

#define USE_STATUS_LED

/* Select differential pressure sensor */
#define SENSOR_DP_NXP_MPXV5004DP
//#define SENSOR_DP_NXP_MP3V5004DP

/* Select sensor geometry, relevant for conversion to volume flow */
//#define SENSOR_GEOMETRY_GRID
#define SENSOR_GEOMETRY_VENTURI

/* Define sensor offset voltage during idle, when there's no differential pressure */
float g_fOffsetVoltage = 0.0f; /* in milliVolts */

/* Disallow negative differential pressure values, i.e. only flow in one direction */
#define DISALLOW_NEGATIVE_VALUES

#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
    //#define DEBUG_INT /* Depending on the timing of the sensor controller requests, it may be useful to turn this off */
#else
    #define debugPrint    
    #define debugPrintln  
#endif

/* Definition of sensor loop interval and loop delay,
 * i.e. how often to read the ANALOG_INPUT_PIN in order to get a new measurement from sensor;
 * the value is given in milliseconds.
 * current setting: approx. every 10 ms (i.e. at a rate of 100 Hz).
 * Note: This is not exact as the UART serial output will take some of that time, 
 *       plus conversions and interrupt overhead, plus loop overhead.
 *       When DEBUG output is active, it takes about 2 ms per cycle.
 *       Caution: The interrupt overhead is not subtracted automatically.
 */
#define SENSOR_LOOP_INTERVAL (10u)
#ifdef DEBUG
    #define SENSOR_LOOP_DELAY (SENSOR_LOOP_INTERVAL - (2u))
#else
    #define SENSOR_LOOP_DELAY (SENSOR_LOOP_INTERVAL)
#endif

/* Definition of sensor modes, i.e. what values will be return upon Wire transmit request event */
enum eSensorMode { SENSOR_MODE_NONE = 0, SENSOR_MODE_MEASURE, SENSOR_MODE_SERIALNO };

/* define the address of the sensor on the I2C bus;
 * TODO/FIXME: decide address to be used via GPIO input pin during setup()
 */
uint8_t g_nDeviceAddress = 0x42;

/* Initialize global variables */
//volatile int g_nConversionValue = 0; /* digital value for sensor's analog input */
volatile eSensorMode g_eMode = SENSOR_MODE_NONE; /* sensor mode start with 'NONE' */

volatile uint16_t g_nTxWord = 0;

union eepromContent {
    uint8_t raw[4];
    struct {
       float fVoltage;
    };
};
union eepromContent g_eepromContent;


void eepromWriteOffsetVoltage(float fOffsetVoltage);
float eepromReadOffsetVoltage(void);


void setup()
{
    Serial.begin(230400);         // start serial for debug output

    Wire.begin(g_nDeviceAddress); // join i2c bus with address <g_nDeviceAddress>
    Wire.onReceive(receiveEvent); // register event handler for Rx
    Wire.onRequest(transmitRequestEvent); // register event handler for Tx request

#ifdef USE_STATUS_LED
    /* use LED_BUILTIN to signal if sensor is in measuring mode */
    pinMode(LED_BUILTIN, OUTPUT);
#endif

#ifdef RT_SUPERVISION_PIN
    pinMode(RT_SUPERVISION_PIN, OUTPUT);
#endif
#ifdef INT_SUPERVISION_PIN
    pinMode(INT_SUPERVISION_PIN, OUTPUT);
#endif

#ifdef SENSOR_DP_NXP_MP3V5004DP
    Serial.println("Built for NXP MP3V5004G sensor.");
#elif defined( SENSOR_DP_NXP_MPXV5004DP )
    Serial.println("Built for NXP MPXV5004DP sensor.");
#else
    #error Selected sensor is not supported.
#endif

#ifdef SENSOR_GEOMETRY_GRID
    Serial.println("Built for grid sensor geometry.");
#elif defined( SENSOR_GEOMETRY_VENTURI )
    Serial.println("Built for venturi sensor geometry.");
#else
    #error Selected sensor geometry is not supported.
#endif

    /* allow voltage offset to be read from EEPROM where it is programmed once;
     * another way would be to calculate a mean "idle" offset voltage at startup
     */
    //eepromWriteOffsetVoltage( -38.1f ); /* program it once, then comment this line out */
    g_fOffsetVoltage = eepromReadOffsetVoltage();

#ifdef DEBUG
    Serial.println("Debug mode active.");
#else
    Serial.println("Debug mode inactive, no more output on serial.");
#endif

    delay(500);
}

void loop()
{
    static int nConversionValue = 0;
    static float fVoltage = 0.0f;
    static float fPressure = 0.0f;
    static float fVolumeFlow = 0.0f;
    static uint16_t nTxWord = 0;

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif
    
    nConversionValue = analogRead(ANALOG_INPUT_PIN);

    if( SENSOR_MODE_MEASURE == g_eMode )
    {
        debugPrint("[*] "); /* in measuring mode */
#ifdef USE_STATUS_LED
        digitalWrite(LED_BUILTIN, HIGH);
#endif
    }
    else
    {
        debugPrint("[ ] "); /* not in measuring mode */
#ifdef USE_STATUS_LED
        digitalWrite(LED_BUILTIN, LOW);
#endif
    }

    fVoltage = countsToMillivolts(nConversionValue);
    fVoltage -= g_fOffsetVoltage;
    fPressure = millivoltsToPressure( fVoltage );
    fVolumeFlow = pressureToVolumeFlow( fPressure );

    /* prepare value for serial communication via I2C */
    g_nTxWord = volumeFlowToTxWord( fVolumeFlow );


    // Debug output the sensor's reading (conversion valie)
    debugPrint("Sensor value: ");
//    debugPrint(nConversionValue); // raw reading
//    debugPrint(" counts, ");
    debugPrint( fVoltage );
    debugPrint(" mV, ");
    debugPrint( fPressure );
    debugPrint(" mmWater, ");
    debugPrint( fVolumeFlow );
    debugPrint(" flow");
    debugPrintln("");

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, LOW);
#endif

    /* Wait until next cyclic measurement is made */
    delay(SENSOR_LOOP_DELAY);
}

crc_t calcCrc(const unsigned char *pData, size_t nDataLen)
{
    crc_t nCrc = crc_init();
    nCrc = crc_update(nCrc, pData, nDataLen);
    nCrc = crc_finalize(nCrc);

    return nCrc;
}

/* Parse received command and switch sensor's state */
void receiveEvent(int nBytes)
{
    char sHexBuf[11]; /* string buffer for debug output via UART */
    uint8_t cCmdBytes[2]; /* command buffer (a command is 16 bit wide, or 2 bytes) */
    int nRxCount = 0;

#ifdef INT_SUPERVISION_PIN
    digitalWrite(INT_SUPERVISION_PIN, HIGH);
#endif

    debugPrint("Rx (");
    debugPrint(nBytes);
    debugPrint(" bytes): ");
    while(0 < Wire.available())   // loop through all but the last
    {
        uint8_t cRxByte = Wire.read(); /* receive byte as a character */

        /* store first two bytes in command buffer */
        if( nRxCount < 2 )
        {
            cCmdBytes[nRxCount] = cRxByte;
        }
        
        nRxCount++;

        /* print every byte as two hex digits with prefix '0x', NULL-terminate the string */
        sprintf(sHexBuf, "0x%02X ", cRxByte);
        sHexBuf[5] = 0;
        debugPrint(sHexBuf);
    }
    debugPrintln("");

    if( nRxCount < 2 )
    {
#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
        return;
    }

    if( nRxCount != nBytes )
    {
        debugPrint("[DEBUG] Message length does not match (Rx count: ");
        debugPrint(nRxCount);
        debugPrint("; Message length: ");
        debugPrint(nBytes);
        debugPrintln(")");
        g_eMode = SENSOR_MODE_NONE;
#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
        return;
    }

    if( 0x10 == cCmdBytes[0] && 0x00 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement' command");
        g_eMode = SENSOR_MODE_MEASURE;
    }
    else if( 0x31 == cCmdBytes[0] && 0xAE == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'read serial number' command");
        g_eMode = SENSOR_MODE_SERIALNO;
    }
    else
    {
        debugPrintln("Rx'ed unknown command: ");
        sprintf(sHexBuf, "0x%02X 0x%02X ", cCmdBytes[0], cCmdBytes[1]);
        sHexBuf[10] = 0;
        debugPrint(sHexBuf);
        
        debugPrintln("(ignored)");
    }

#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
}

/* Transmit data dependent on current sensor state */
void transmitRequestEvent(void)
{
#ifdef INT_SUPERVISION_PIN
    digitalWrite(INT_SUPERVISION_PIN, HIGH);
#endif

#ifdef DEBUG_INT
    debugPrintln("[DEBUG] Tx request");
#endif

    switch( g_eMode )
    {
        case SENSOR_MODE_MEASURE:
            /* TODO/FIXME: add true measurement data and add CRC */
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx measurement");
#endif
            char sMeas[2];
            //sMeas[0] = 'F';
            //sMeas[1] = 'L';
            sMeas[0] = (uint8_t)( (g_nTxWord & 0xFF00) >> 8 );
            sMeas[1] = (uint8_t)(g_nTxWord & 0xFF);
            Wire.write((char)sMeas[0]);
            Wire.write((char)sMeas[1]);
            Wire.write((char)calcCrc(&sMeas[0], 2) );
            break;
        case SENSOR_MODE_SERIALNO:
            char sSerNo[4];
            sSerNo[0] = 'S';
            sSerNo[1] = 'R';
            sSerNo[2] = 'N';
            sSerNo[3] = 'O';
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx serialno");
#endif
            Wire.write((char)sSerNo[0]);
            Wire.write((char)sSerNo[1]);
            Wire.write((char)calcCrc(&sSerNo[0], 2) );
            Wire.write((char)sSerNo[2]);
            Wire.write((char)sSerNo[3]);
            Wire.write((char)calcCrc(&sSerNo[2], 2) );
            break;
        case SENSOR_MODE_NONE:
        default:
#ifdef DEBUG_INT
            debugPrintln("[WARN] Tx nothing (controller will read 0xFF)");
#endif
            break;
    }

#ifdef INT_SUPERVISION_PIN
        digitalWrite(INT_SUPERVISION_PIN, LOW);
#endif
}

float countsToMillivolts(int nCounts)
{
    // For the Arduino Uno, its a 10 bit unsgined value (i.e. a range of 0..1023).
    // Its measurement range is from 0..5 volts.
    // This yields a resolution between readings of: 5 volts / 1024 units or approx. 4.9 mV per LSB
    return nCounts * 4.882813f;
}

float millivoltsToPressure(float fMillivolts)
{
#ifdef SENSOR_DP_NXP_MP3V5004DP
    // based on MP3V5004DP datasheet:
    // for now, assume a linear transfer function of the sensor,
    // e.g. V_out = 0.6 V/kPa * dP + 0.6 V
    //         dP = (V_out - 0.6 V) / 0.6 V/kPa 
    //            = (V_out - 0.6 V) * 1.667 kPa/V
    //            = (V_out - 600 mV) * (1.667*100 mmH2O) / 1000 mV // 1 kPa ~ approx. 100 mmH2O
    float fPressure = (fMillivolts - 600.0f) * 0.1666666667f;
#elif defined( SENSOR_DP_NXP_MPXV5004DP )
    // based on MPXV5004DP datasheet:
    // for now, assume a linear transfer function of the sensor,
    // e.g. V_out = 1.0 V/kPa * dP + 1.0 V
    //         dP = (V_out - 1.0 V) / 1.0 V/kPa 
    //            = (V_out - 1.0 V) * 1.0 kPa/V
    //            = (V_out - 1000 mV) * (1.0*100 mmH2O) / 1000 mV // 1 kPa ~ approx. 100 mmH2O
    float fPressure = (fMillivolts - 1000.0f) * 0.1f;
#else
    #error Unknown differential pressure sensor selected.
#endif

    /* do not allow negative differential pressure value */
#ifdef DISALLOW_NEGATIVE_VALUES
    fPressure = (fPressure < 0.0f) ? 0.0f : fPressure;
#endif

    return fPressure;
}

/* convert pressure to volume flow in liters per minute */
float pressureToVolumeFlow(float fPressure)
{
    static float fFlow = 0.0f;

#ifdef SENSOR_GEOMETRY_GRID
    static bool bIsNeg = false;

    bIsNeg = fPressure < 0.0f; /* store sign for later usage */
    fFlow = fabs( fPressure ); /* remove sign, i.e. use absolute value */
    if( fFlow >= 40.0f )
    {
        fFlow = 15.0f * sqrt(fFlow);
    }
    else if( fFlow < 40.0f && fFlow >= 2.0f )
    {
        fFlow = 13.0f * sqrt(fFlow);
    }

    if( bIsNeg )
    {
        fFlow *= -1.0f; /* add sign back */
    }
#elif defined( SENSOR_GEOMETRY_VENTURI )
    if( fPressure < 0.0f ) /* cut off negative values */
    {
        fFlow = 0.0f;
    }
    else
    {
        fFlow = 6.4f * sqrt(fPressure);
    }
#else
    fFlow = fPressure;
#endif
    
    return fFlow;
}

uint16_t volumeFlowToTxWord(float fVolumeFlow)
{
    static const uint16_t fOffsetFlow = 32000.0f;
    static const float fScaleFactor = 140.0f;

    return (uint16_t)(fVolumeFlow * fScaleFactor + fOffsetFlow);
}

void eepromWriteOffsetVoltage(float fOffsetVoltage)
{
    g_eepromContent.fVoltage = fOffsetVoltage;

    Serial.print("EEPROM values to be written: raw ");
    Serial.print(g_eepromContent.raw[0]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[1]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[2]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[3]);
    Serial.print(", as float: ");
    Serial.print(g_eepromContent.fVoltage);
    Serial.println("");

    EEPROM.write(0x0, g_eepromContent.raw[0]);
    EEPROM.write(0x1, g_eepromContent.raw[1]);
    EEPROM.write(0x2, g_eepromContent.raw[2]);
    EEPROM.write(0x3, g_eepromContent.raw[3]);
}

float eepromReadOffsetVoltage(void)
{
    g_eepromContent.raw[0] = EEPROM.read(0x0);
    g_eepromContent.raw[1] = EEPROM.read(0x1);
    g_eepromContent.raw[2] = EEPROM.read(0x2);
    g_eepromContent.raw[3] = EEPROM.read(0x3);

    Serial.print("Values read from EEPROM: raw ");
    Serial.print(g_eepromContent.raw[0]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[1]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[2]);
    Serial.print(", ");
    Serial.print(g_eepromContent.raw[3]);
    Serial.print(", as float: ");
    Serial.print(g_eepromContent.fVoltage);
    Serial.println("");

    if( 0xFF == g_eepromContent.raw[0] && 0xFF == g_eepromContent.raw[1] &&
        0xFF == g_eepromContent.raw[1] && 0xFF == g_eepromContent.raw[2] )
    {
        return 0.0f;
    }
    else
    {
        return g_eepromContent.fVoltage;
    }
}

