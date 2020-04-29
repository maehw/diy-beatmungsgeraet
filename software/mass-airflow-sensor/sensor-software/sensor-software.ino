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

/* Mapping of the analog input pin, define alias here */
#define ANALOG_INPUT_PIN      (A0)

/* Mapping of the digital output pin for real-time supervision, define alias here */
#define RT_SUPERVISION_PIN    (A3)

/* Mapping of the digital output pin for interrupt supervision, define alias here */
#define INT_SUPERVISION_PIN   (A2)

/* Define USE_STATUS_LED to indicate active measurement by switching on the built-in LED */
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

/* Choose _one_ of the following voltage reference settings */
//#define VOLTAGE_REF_5V_DEFAULT
//#define VOLTAGE_REF_1V0_INTERNAL /* (currently not supported!) ATmega168/ATmega328P based boards, e.g. Arduno Uno */
#define VOLTAGE_REF_2V56_INTERNAL /* ATmega32U4 based boards, e.g. Arduino Leonardo */

#define DEBUG
#ifdef DEBUG
    #define debugPrint    Serial.print
    #define debugPrintln  Serial.println
    #define DEBUG_INT /* Depending on the timing of the sensor controller requests, it may be useful to turn this off */
    #define DEBUG_PRINT_STATUS
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
enum eSensorMode {
    SENSOR_MODE_NONE = 0,       /*!< Initial mode, sensor has not been initialized yet */
    SENSOR_MODE_MEASURE,        /*!< Measuring mode (delivering volume flow values) */
    SENSOR_MODE_SERIALNO,       /*!< Deliver serial number */
    SENSOR_MODE_MEASURE_RAWV,   /*!< Measuring mode (delivering raw voltage values) */
    SENSOR_MODE_MEASURE_RAWVO,  /*!< Measuring mode (delivering raw voltage values, offset removed) */
    SENSOR_MODE_MEASURE_RAWDP,  /*!< Measuring mode (delivering raw differential pressure values) */
    SENSOR_MODE_MEASURE_RAWFL   /*!< Measuring mode (delivering raw volume flow values) */
};

/* define the address of the sensor on the I2C bus;
 * TODO/FIXME: decide address to be used via GPIO input pin during setup()
 */
uint8_t g_nDeviceAddress = 0x42;

/* Initialize global variables */
volatile eSensorMode g_eMode = SENSOR_MODE_NONE; /* sensor mode start with 'NONE' */

union eepromContent {
    uint8_t raw[4];
    struct {
       float fVoltage;
    };
};
union eepromContent g_eepromContent;

union singlePrecFloat {
    uint8_t raw[4];
    float fFloatVal;
};

volatile union singlePrecFloat g_voltage;
volatile union singlePrecFloat g_voltageOffRem;
volatile union singlePrecFloat g_diffPressure;
volatile union singlePrecFloat g_flow;
volatile uint16_t g_nFlowTxWord = 0;

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

#if defined(VOLTAGE_REF_2V56_INTERNAL) || defined(VOLTAGE_REF_1V0_INTERNAL)
    analogReference(INTERNAL);
#else
    analogReference(DEFAULT);
#endif

    /* allow voltage offset to be read from EEPROM where it is programmed once;
     * another way would be to calculate a mean "idle" offset voltage at startup
     */
    //eepromWriteOffsetVoltage( 54.0f ); /* program it once, then comment this line out */
    g_fOffsetVoltage = eepromReadOffsetVoltage();

    g_nFlowTxWord = 0;
    g_voltage.fFloatVal = 0.0f;
    g_voltageOffRem.fFloatVal = 0.0f;
    g_diffPressure.fFloatVal = 0.0f;
    g_flow.fFloatVal = 0.0f;

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
    static uint16_t nTxWord = 0;
    static bool bPrintNone = true; /* has the highest priority, i.e. if set to true, no continous data will be printed */
    static bool bPrintAll = true; /* has the second highest priority, i.e. if set to true, overwrites the following three flags */
    static bool bPrintVoltage = false;
    static bool bPrintVoltageOffRem = false;
    static bool bPrintDiffPressure = false;

#ifdef RT_SUPERVISION_PIN
    digitalWrite(RT_SUPERVISION_PIN, HIGH);
#endif
    
    nConversionValue = analogRead(ANALOG_INPUT_PIN);

    if( !bPrintNone )
    {
        if( SENSOR_MODE_MEASURE == g_eMode || SENSOR_MODE_MEASURE_RAWV == g_eMode || SENSOR_MODE_MEASURE_RAWVO == g_eMode || 
            SENSOR_MODE_MEASURE_RAWDP == g_eMode || SENSOR_MODE_MEASURE_RAWFL == g_eMode )
        {
#ifdef DEBUG_PRINT_STATUS
            if( SENSOR_MODE_MEASURE == g_eMode )
            {
                debugPrint("[*] "); /* in measuring mode */
            }
            else if( SENSOR_MODE_MEASURE_RAWV == g_eMode )
            {
                debugPrint("[V] "); /* in measuring mode, output raw voltage values */
            }
            else if( SENSOR_MODE_MEASURE_RAWDP == g_eMode)
            {
                debugPrint("[P] "); /* in measuring mode, output raw differential pressure values */
            }
            else if( SENSOR_MODE_MEASURE_RAWFL == g_eMode)
            {
                debugPrint("[F] "); /* in measuring mode, output raw volume flow values */
            }
#endif
        }
        else
        {
#ifdef DEBUG_PRINT_STATUS
        debugPrint("[ ] "); /* not in measuring mode */
#endif
        }
    }

#ifdef USE_STATUS_LED
    if( SENSOR_MODE_MEASURE == g_eMode || SENSOR_MODE_MEASURE_RAWV == g_eMode || SENSOR_MODE_MEASURE_RAWVO == g_eMode || 
        SENSOR_MODE_MEASURE_RAWDP == g_eMode || SENSOR_MODE_MEASURE_RAWFL == g_eMode )
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(LED_BUILTIN, LOW);
    }
#endif

    g_voltage.fFloatVal = countsToMillivolts(nConversionValue);
    g_voltageOffRem.fFloatVal = g_voltage.fFloatVal - g_fOffsetVoltage;

    g_diffPressure.fFloatVal = millivoltsToPressure( g_voltageOffRem.fFloatVal );
    g_flow.fFloatVal = pressureToVolumeFlow( g_diffPressure.fFloatVal );

    /* prepare value for serial communication via I2C */
    g_nFlowTxWord = volumeFlowToTxWord( g_flow.fFloatVal );

    if( !bPrintNone )
    {
        // Debug output the sensor's reading (conversion valie)
        debugPrint("Sensor value: ");
        if( bPrintAll || bPrintVoltage || SENSOR_MODE_MEASURE_RAWV == g_eMode )
        {
            debugPrint( g_voltage.fFloatVal );
            debugPrint(" mV, ");
        }
        if( bPrintAll || bPrintVoltageOffRem || SENSOR_MODE_MEASURE_RAWVO == g_eMode )
        {
            debugPrint( g_voltageOffRem.fFloatVal );
            debugPrint(" mV, ");
        }
        if( bPrintAll || bPrintDiffPressure || SENSOR_MODE_MEASURE_RAWDP == g_eMode )
        {
            debugPrint( g_diffPressure.fFloatVal );
            debugPrint(" mmWater, ");
        }
    
        /* Always print the flow */
        debugPrint( g_flow.fFloatVal );
        debugPrintln(" lpm"); /* liters per minute */
    }

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
    else if( 0x20 == cCmdBytes[0] && 0x00 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'soft reset' command");
        g_eMode = SENSOR_MODE_NONE;
        g_nFlowTxWord = 0;
        g_voltage.fFloatVal = 0.0f;
        g_diffPressure.fFloatVal = 0.0f;
        g_flow.fFloatVal = 0.0f;
    }
    else if( 0x42 == cCmdBytes[0] && 0x00 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement (raw voltage)' command");
        g_eMode = SENSOR_MODE_MEASURE_RAWV;
    }
    else if( 0x42 == cCmdBytes[0] && 0x01 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement (raw voltage, offset removed)' command");
        g_eMode = SENSOR_MODE_MEASURE_RAWVO;
    }
    else if( 0x42 == cCmdBytes[0] && 0x02 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement (raw differential pressure)' command");
        g_eMode = SENSOR_MODE_MEASURE_RAWDP;
    }
    else if( 0x42 == cCmdBytes[0] && 0x03 == cCmdBytes[1] )
    {
        debugPrintln("Rx'ed 'start measurement (raw volume flow)' command");
        g_eMode = SENSOR_MODE_MEASURE_RAWFL;
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
    //debugPrintln("[DEBUG] Tx request");
#endif

    switch( g_eMode )
    {
        case SENSOR_MODE_MEASURE:
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx true measurement");
#endif
            char sMeas[2]; /* buffer to hold measurement data */
            // TODO/FIXME: the following lines should be atomic or double-buffered to prevent corruption
            sMeas[0] = (uint8_t)( (g_nFlowTxWord & 0xFF00) >> 8 );
            sMeas[1] = (uint8_t)(g_nFlowTxWord & 0xFF);
            Wire.write((char)sMeas[0]);
            Wire.write((char)sMeas[1]);
            Wire.write((char)calcCrc(&sMeas[0], 2) );
            break;
        case SENSOR_MODE_MEASURE_RAWV:
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx raw voltage measurement");
#endif
            // TODO/FIXME: the following lines should be atomic or double-buffered to prevent corruption
            Wire.write((char)g_voltage.raw[0]);
            Wire.write((char)g_voltage.raw[1]);
            Wire.write((char)g_voltage.raw[2]);
            Wire.write((char)g_voltage.raw[3]);
            Wire.write((char)calcCrc(&g_voltage.raw[0], 4) );
            break;
        case SENSOR_MODE_MEASURE_RAWVO:
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx raw voltage (w/o offset) measurement");
#endif
            // TODO/FIXME: the following lines should be atomic or double-buffered to prevent corruption
            Wire.write((char)g_voltageOffRem.raw[0]);
            Wire.write((char)g_voltageOffRem.raw[1]);
            Wire.write((char)g_voltageOffRem.raw[2]);
            Wire.write((char)g_voltageOffRem.raw[3]);
            Wire.write((char)calcCrc(&g_voltageOffRem.raw[0], 4) );
            break;
        case SENSOR_MODE_MEASURE_RAWDP:
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx raw differential pressure measurement");
#endif
            // TODO/FIXME: the following lines should be atomic or double-buffered to prevent corruption
            Wire.write((char)g_diffPressure.raw[0]);
            Wire.write((char)g_diffPressure.raw[1]);
            Wire.write((char)g_diffPressure.raw[2]);
            Wire.write((char)g_diffPressure.raw[3]);
            Wire.write((char)calcCrc(&g_diffPressure.raw[0], 4) );
            break;
        case SENSOR_MODE_MEASURE_RAWFL:
#ifdef DEBUG_INT
            debugPrintln("[DEBUG] Tx raw flow measurement");
#endif
            // TODO/FIXME: the following lines should be atomic or double-buffered to prevent corruption
            Wire.write((char)g_flow.raw[0]);
            Wire.write((char)g_flow.raw[1]);
            Wire.write((char)g_flow.raw[2]);
            Wire.write((char)g_flow.raw[3]);
            Wire.write((char)calcCrc(&g_flow.raw[0], 4) );
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
    /* For the Arduino Uno and Arduino Leonardo, its a 10 bit unsgined value (i.e. a range of 0..1023).
     * VOLTAGE_REF_5V_DEFAULT:
     *   The default measurement range is from 0..5 volts. Please read the documentation for related issues!
     *   This yields a resolution between readings of: 5 volts / 1024 units or approx. 4.9 mV per LSB.
     * VOLTAGE_REF_2V56_INTERNAL:
     *   The default measurement range is from 0..2.56 volts.
     *   This yields a resolution between readings of: 2.56 volts / 1024 units or 2.5 mV per LSB.
     * VOLTAGE_REF_1V0_INTERNAL:
     *   Please read the documentation for related issues!
     */
#if defined(VOLTAGE_REF_5V_DEFAULT)
    return nCounts * 4.882813f;
#elif defined(VOLTAGE_REF_2V56_INTERNAL)
    return nCounts * 2.5f;
#elif defined(VOLTAGE_REF_1V0_INTERNAL)
    #error Currently not supported.
#endif
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
        fFlow = 6.65f * sqrt(fPressure); /* the value 6.65 is taken from simulations of the 3D printed model */
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

#ifdef DEBUG
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
#endif

    /* Check for default EEPROM content */
    if( 0xFF == g_eepromContent.raw[0] && 0xFF == g_eepromContent.raw[1] &&
        0xFF == g_eepromContent.raw[1] && 0xFF == g_eepromContent.raw[2] )
    {
        return 0.0f; /* do not assume any offset voltage */
    }
    else
    {
        return g_eepromContent.fVoltage;
    }
}

