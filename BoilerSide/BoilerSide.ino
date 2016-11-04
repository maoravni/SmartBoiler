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
 *     * SETPID
 * Store PID values in EEPROM
 * PWM for the SSR
 * Setup of the PID Control.
 */

PIDDynamicSampleTime PID;
PID_ATune PIDAutoTune;
bool isInAutotune = false;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 23
#define SSR_OUTPUT 43

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

void loop() {
  newTempReading = readTemperature();
  if (newTempReading)
  {
    if (isInAutotune)
      output = (char)PIDAutoTune.Compute(temperature);
    else
      output = (char)PID.Compute(temperature);
    sprintf(outgoingMessage, "T%0d.%02d O%0d\n", (int)temperature, (int)(((int)(temperature*100))%100), output);
    Serial.print(outgoingMessage);
  }

  if (Serial.available())
  {
    bytesRead = Serial.readBytesUntil('\n', incomingMessage+incomingMessageSize, 50-incomingMessageSize);
    incomingMessageSize += bytesRead;

    sprintf(outgoingMessage, "Char at end of incoming string: %d\n", incomingMessage[incomingMessageSize-1]);

    Serial.println(incomingMessage);
    Serial.println(outgoingMessage);
    if (incomingMessage[incomingMessageSize-1] == '\n' || incomingMessage[incomingMessageSize-1] == '\r')
    {
      Serial.println("Message Accepted");
    }
    if (incomingMessageSize >= 50)
      incomingMessageSize = 0;
  }
}
