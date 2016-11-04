#include <EEPROM.h>

#include <TimerOne.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include "PID.h"
#include "PID_Autotune_V0.h"

/*TODO:
 * Setup the RFM69 radio
 * Keepalive - when no RF comms are available, stop calculating the PID.
 * Serial Commands:
 *     * KEEPALIVE
 *     * STARTHEATING
 *     * STOPHEATING
 *     * AUTOTUNE
 *     * SETP/SETI/SETD
 * Store PID values in EEPROM
 * PWM for the SSR
 * Setup of the PID Control.
 */

PIDDynamicSampleTime PID;
PID_ATune PIDAutoTune(E_PID_ControlType_PID, 50, 55, 0.1, 40, 40);
bool isInAutotune = false;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 23
#define SSR_OUTPUT 12

#define PID_EEPROM_ADDRESS 0

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress;

//int  resolution = 12;
#define resolution 12
unsigned long lastTempRequest = 0;
unsigned long actualTempDelay = 0;
#define delayInMillis (750 / (1 << (12 - resolution)))
float temperature = 0.0;
char output = 0;
char randomResult = 0;
bool newTempReading = false;

char outgoingMessage[50];
char incomingMessage[50];
String incomingString = "";
char incomingMessageSize = 0;
char bytesRead = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.setTimeout(1);

  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  lastTempRequest = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SSR_OUTPUT, OUTPUT);

  Timer1.initialize(10000);
  Timer1.attachInterrupt(PwmOutputInterrupt); // blinkLED to run every 0.15 seconds
    
  float p, i, d;
  EEPROM.get(PID_EEPROM_ADDRESS, p);
  EEPROM.get(PID_EEPROM_ADDRESS+4, i);
  EEPROM.get(PID_EEPROM_ADDRESS+8, d);
  Serial.println(p);
  Serial.println(i);
  Serial.println(d);
  PID.SetTunings(p, i, d, 0);
}

void PwmOutputInterrupt(void)
{
  randomResult = random(100);
  digitalWrite(LED_BUILTIN, (output > randomResult));
  digitalWrite(SSR_OUTPUT, (output > randomResult));
}

unsigned long t1, t2;

bool readTemperature()
{
  if ((actualTempDelay = (millis() - lastTempRequest)) >= delayInMillis) // waited long enough??
  {
    t1 = millis();
    digitalWrite(LED_BUILTIN, LOW);
    temperature = sensors.getTempCByIndex(0);

    sensors.requestTemperatures();
    lastTempRequest = millis();

    digitalWrite(LED_BUILTIN, HIGH);

    t2 = millis();
    return true;
  }
  return false;
}

void handleCommands()
{
  if (Serial.available())
  {
    float parameter;
    bytesRead = Serial.readBytesUntil('\n', incomingMessage, 50);
    //incomingMessageSize += bytesRead;
    incomingString += incomingMessage;
    incomingMessage[0] = 0;
    Serial.println(incomingMessage);
    if (incomingMessage[bytesRead-1] == '\n' || incomingMessage[bytesRead-1] == '\r')
    {
      // start parsing commands:

      //keep alive
      if (incomingString.startsWith("KEEPALIVE"))
      {
        Serial.println("Alive");
      }
      else if (incomingString.startsWith("SETP"))
      {
        Serial.println("SETP");
        parameter = incomingString.substring(5).toFloat();
        EEPROM.put(PID_EEPROM_ADDRESS, parameter);
        Serial.println(parameter);
      }
      else if (incomingString.startsWith("SETI"))
      {
        Serial.println("SETI");
        parameter = incomingString.substring(5).toFloat();
        EEPROM.put(PID_EEPROM_ADDRESS+4, parameter);
        Serial.println(parameter);
      }
      else if (incomingString.startsWith("SETD"))
      {
        Serial.println("SETD");
        parameter = incomingString.substring(5).toFloat();
        EEPROM.put(PID_EEPROM_ADDRESS+8, parameter);
        Serial.println(parameter);
      }
      else if (incomingString.startsWith("STARTHEATING"))
      {
        Serial.println("STARTHEATING");
        parameter = incomingString.substring(12).toFloat();
        Serial.println(parameter);
        isInAutotune = false;
        PID.setSetPoint(parameter, 0);
        PID.setAutoMode(true);
        PID.setEnabled(true);
      }
      else if (incomingString.startsWith("STOP"))
      {
        Serial.println("STOP");
        PID.setEnabled(false);
        PID.setAutoMode(false);
        isInAutotune = false;
      }
      else if (incomingString.startsWith("AUTOTUNE"))
      {
        Serial.println("AUTOTUNE");
        parameter = incomingString.substring(12).toFloat();
//        EEPROM.put(PID_EEPROM_ADDRESS+8, parameter);
        Serial.println(parameter);
        isInAutotune = true;
        PID.setSetPoint(parameter, 0);
        PID.setAutoMode(true);
        PID.setEnabled(true);
      }
      incomingString = "";
    }
    if (incomingMessageSize >= 50)
      incomingMessageSize = 0;
  }
}

void loop() {
  newTempReading = readTemperature();
  if (newTempReading)
  {
    if (isInAutotune)
    {
      output = (char)PIDAutoTune.Compute(temperature);
      if (!PIDAutoTune.isRunning())
      {
        isInAutotune = false;
        float p, i, d;
        p = PIDAutoTune.GetKp();
        i = PIDAutoTune.GetKi();
        d = PIDAutoTune.GetKd();
        Serial.println(p);
        Serial.println(i);
        Serial.println(d);
        EEPROM.put(PID_EEPROM_ADDRESS, p);
        EEPROM.put(PID_EEPROM_ADDRESS+4, i);
        EEPROM.put(PID_EEPROM_ADDRESS+8, d);
        PID.SetTunings(p, i, d, 0);
      }
    }
    else
      output = (char)PID.Compute(temperature);
    sprintf(outgoingMessage, "T%0d.%02d O%0d\n", (int)temperature, (int)(((int)(temperature*100))%100), output);
    Serial.print(outgoingMessage);
  }

  handleCommands();
}
