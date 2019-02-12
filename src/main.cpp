#include <Arduino.h>

#define DEBUG
#define DEBUG_OI Serial
#include "debug.h"

#include <ModbusMaster.h> //https://github.com/4-20ma/ModbusMaster
#include <Timer.h>        //https://github.com/JChristensen/Timer
#include <ESP8266WiFi.h>  //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>
#include <FS.h> // Include the SPIFFS library

//#define HOSTNAME "tracer-1"
#define LED_PIN D4 //GPIO2
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

uint8_t result;

// response = client.read_device_info()
// print "Manufacturer:", repr(response.information[0])
// print "Model:", repr(response.information[1])
// print "Version:", repr(response.information[2])

// 43 / 14 (0x2B / 0x0E) Read Device Identification
// Object Id | Object Name / Description  | Type         | M/O       | category
// 0x00      | VendorName                 | ASCII String | Mandatory |
// 0x01      | ProductCode                | ASCII String | Mandatory |
// 0x02      | MajorMinorRevision         | ASCII String | Mandatory | Basic

struct device_id
{
  String vendorName;
  String productCode;
  String version;
} deviceID;

//rated data
struct rated_data
{
  float pvVoltage;
  float pvCurrent;
  int16_t pvPower;
  float batteryVoltage;
  float batteryCurrent;
  int16_t batteryPower;
  float chargingMode; //0000H Connect/disconnect, 0001H PWM, 0002H MPPT
  float loadCurrent;
} ratedData;

//realtime status
struct realtime_status
{
  float pvVoltage;
  float pvCurrent;
  int16_t pvPower;
  float batteryVoltage;
  float batteryChargingCurrent;
  int16_t batteryChargingPower;
  float loadVoltage;
  float loadCurrent;
  int16_t loadPower;
  float batteryTemp;
  float equipmentTemp;
  float heatsinkTemp;
  uint8_t batterySoC;
  float batteryRemoteTemp;
  uint16_t batteryRatedPower; //1200,2400 for 12/12v
} realtimeStatus;

//statistical parameters
struct statistical_parameters
{
  float todayMaxPvVoltage;
  float todayMinPvVoltage;
  float todayMaxBattVoltage;
  float todayMinBattVoltage;
  float todayConsumedEnergy;
  float monthConsumedEnergy;
  float yearConsumedEnergy;
  float totalConsumedEnergy;
  float todayGeneratedEnergy;
  float monthGeneratedEnergy;
  float yearGeneratedEnergy;
  float totalGeneratedEnergy;
  float CO2reduction;
  float batteryCurrent; //net, charging minus discharge
  float batteryTemp;
  float ambientTemp;
} statisticalParameters;

//setting_parameters
struct setting_parameters
{
  float batteryType;
  float ratedCapacity;
  int temperatureCompensation;
  float highVoltageDisconnect;
  float chargeLimitVoltage;
  float overVoltageReconnect;
  float equalisationVoltage;
  float boostVoltage;
  float floatVoltage;
  float boostReconnectVoltage;
  float lowVoltageReconnect;
  float underVoltageRecover;
  float lowVoltageDisconnect;
  float realTimeClock;
  int equalisationChargingCycle;
  float batteryTemperatureWarningUpperLimit;
  float batteryTemperatureWarningLowerLimit;
  float controllerInnerTemperatureUpperLimit;
  float controllerInnerTemperatureUpperLimitRecovery;
  float powerComponentTemperatureUpperLimit;
  float powerComponentTemperatureUpperLimitRecover;
  float lineImpedance;
  float nightTimeThresholdVoltage;
  float lightSignalStartupDelayTime;
  float dayTimeThresholdVoltage;
  float lightSignalTurnOffDelayTime;
  float loadControllingMode;
  float workingTimeLength1;
  float workingTimeLength2;
  int turnOnTiming1_H;
  int turnOnTiming1_M;
  int turnOnTiming1_S;
  int turnOffTiming1_H;
  int turnOffTiming1_M;
  int turnOffTiming1_S;
  int turnOnTiming2_H;
  int turnOnTiming2_M;
  int turnOnTiming2_S;
  int turnOffTiming2_H;
  int turnOffTiming2_M;
  int turnOffTiming2_S;
  float lengthOfNight;
  int batteryRatedVoltageCode;       //0=auto, 1=12V, 2=24V
  bool loadTimingControl;            //0=1 timer,1=2 timer
  bool defaultLoadOnOffInManualMode; //0=OFF, 1=ON
  int equaliseDuration;              //minute
  int boostDuration;                 //minute
  int dischargingPercentage;         //$
  int chargingPercentage;            //%
  bool batteryManagementMode;        //0=voltComp, 1=SoC
} settingParameters;

//coil / switch values
struct switch_value
{
  bool manualControl;
  bool loadTest;
  bool forceLoad;
} switchValues;

//discrete_input
struct discrete_input
{
  bool overTemp;
  bool dayNight;
} discreteInput;

bool rs485DataReceived = true;
bool loadPoweredOn = true;

ModbusMaster node;
Timer timer;

//web server json responders
void getRatedData();
void getRealtimeData();
void getStatisticalData();

// tracer requires no handshaking
void preTransmission() {}
void postTransmission() {}

uint8_t readOutputLoadState();
uint8_t checkLoadCoilState();
uint8_t setOutputLoadPower(uint8_t state);

void executeCurrentRegistryFunction();
void nextRegistryNumber();

void AddressRegistry_3100();
// void AddressRegistry_3106();
void AddressRegistry_3110();
void AddressRegistry_310C();
void AddressRegistry_311A();
void AddressRegistry_311D();
void AddressRegistry_331B();

// a list of the regisities to query in order
typedef void (*RegistryList[])();
RegistryList Registries = {
    AddressRegistry_3100,
    AddressRegistry_310C,
    // AddressRegistry_3106,
    AddressRegistry_3110,
    AddressRegistry_311A,
    AddressRegistry_311D,
    AddressRegistry_331B,
};
// keep log of where we are
uint8_t currentRegistryNumber = 0;

//WiFiServer server(80);
ESP8266WebServer server(80);
// Variable to store the HTTP request
String header;

void checkLoad();
String getContentType(String filename); // convert the file extension to the MIME type
bool handleFileRead(String path);       // send the right file to the client (if it exists)

void setup()
{
  DebugBegin(115200);

#ifndef DEBUG
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Sketch init!");
  Serial.print("Compile Date / Time : ");
  Serial.print(__DATE__);
  Serial.print(" / ");
  Serial.println(__TIME__);
  Serial.println("");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); //inverted, off

  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFiManager wifiManager;
  // wifiManager.resetSettings();
  wifiManager.autoConnect("AutoConnectAP");

  DebugPrintln("Starting ArduinoOTA...");

  if (MDNS.begin(HOSTNAME))
  { // Start the mDNS responder for esp8266.local
    Serial.println("mDNS responder started");
  }
  else
  {
    Serial.println("Error setting up MDNS responder!");
  }

  ArduinoOTA.setHostname(HOSTNAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_SPIFFS
      type = "filesystem";
      SPIFFS.end();
    }

    DebugPrintln("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DebugPrintln("\nEnd of update");
    digitalWrite(LED_PIN, HIGH);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentageComplete = (progress / (total / 100));
    DebugPrintf("Progress: %u%%\r", percentageComplete);
    digitalWrite(LED_PIN, percentageComplete % 2);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DebugPrintf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      DebugPrintln("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      DebugPrintln("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      DebugPrintln("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      DebugPrintln("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      DebugPrintln("End Failed");
    }
  });

  ArduinoOTA.begin();

  DebugPrint("ArduinoOTA running. ");

  DebugPrint("IP address: ");
  DebugPrintln(WiFi.localIP());

  SPIFFS.begin(); // Start the SPI Flash Files System

  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

  server.serveStatic("/", SPIFFS, "/index.html");
  server.on("/getRatedData", getRatedData);
  server.on("/getRealtimeData", getRealtimeData);
  server.on("/getStatisticalData", getStatisticalData);

  server.begin(); // Actually start the server
  DebugPrintln("HTTP server started");

  DebugPrintln("Starting timed actions...");
  timer.every(1000L, executeCurrentRegistryFunction);
  timer.every(1000L, nextRegistryNumber);
  timer.every(5000L, checkLoad);

  DebugPrintln("Setup OK!");
  DebugPrintln("----------------------------");
  DebugPrintln();

  WiFi.setSleepMode(WIFI_MODEM_SLEEP);
}

void loop()
{
  MDNS.update();
  ArduinoOTA.handle();
  timer.update();

  server.handleClient();
  delay(100);
}

void getRatedData()
{
  server.send(200, "application/json",
              "{\"pvVoltage\":" + String(ratedData.pvVoltage) +
                  ", \"pvCurrent\":" + String(ratedData.pvCurrent) +
                  ", \"pvPower\":" + String(ratedData.pvPower) +
                  ", \"batteryVoltage\":" + String(ratedData.batteryVoltage) +
                  ", \"batteryCurrent\":" + String(ratedData.batteryCurrent) +
                  ", \"batteryPower\":" + String(ratedData.batteryPower) +
                  ", \"chargingMode\":" + String(ratedData.chargingMode) +
                  ", \"loadCurrent\":" + String(ratedData.loadCurrent) + "}");
}

void getRealtimeData()
{
  server.send(200, "application/json",
              "{\"pvVoltage\":" + String(realtimeStatus.pvVoltage) +
                  ", \"pvCurrent\":" + String(realtimeStatus.pvCurrent) +
                  ", \"pvPower\":" + String(realtimeStatus.pvPower) +
                  ", \"batteryVoltage\":" + String(realtimeStatus.batteryVoltage) +
                  ", \"batteryChargingCurrent\":" + String(realtimeStatus.batteryChargingCurrent) +
                  ", \"batteryChargingPower\":" + String(realtimeStatus.batteryChargingPower) +
                  ", \"loadVoltage\":" + String(realtimeStatus.loadVoltage) +
                  ", \"loadCurrent\":" + String(realtimeStatus.loadCurrent) +
                  ", \"loadPower\":" + String(realtimeStatus.loadPower) +
                  ", \"batteryTemp\":" + String(realtimeStatus.batteryTemp) +
                  ", \"equipmentTemp\":" + String(realtimeStatus.equipmentTemp) +
                  ", \"heatsinkTemp\":" + String(realtimeStatus.heatsinkTemp) +
                  ", \"batterySoC\":" + String(realtimeStatus.batterySoC) +
                  ", \"batteryRemoteTemp\":" + String(realtimeStatus.batteryRemoteTemp) +
                  ", \"batteryRatedPower\":" + String(realtimeStatus.batteryRatedPower) + "}");
}

void getStatisticalData()
{
  server.send(200, "application/json",
              "{\"todayMaxPvVoltage\":" + String(statisticalParameters.todayMaxPvVoltage) +
                  ", \"todayMinPvVoltage\":" + String(statisticalParameters.todayMinPvVoltage) +
                  ", \"todayMaxBattVoltage\":" + String(statisticalParameters.todayMaxBattVoltage) +
                  ", \"todayMinBattVoltage\":" + String(statisticalParameters.todayMinBattVoltage) +
                  ", \"todayConsumedEnergy\":" + String(statisticalParameters.todayConsumedEnergy) +
                  ", \"monthConsumedEnergy\":" + String(statisticalParameters.monthConsumedEnergy) +
                  ", \"yearConsumedEnergy\":" + String(statisticalParameters.yearConsumedEnergy) +
                  ", \"totalConsumedEnergy\":" + String(statisticalParameters.totalConsumedEnergy) +
                  ", \"todayGeneratedEnergy\":" + String(statisticalParameters.todayGeneratedEnergy) +
                  ", \"monthGeneratedEnergy\":" + String(statisticalParameters.monthGeneratedEnergy) +
                  ", \"yearGeneratedEnergy\":" + String(statisticalParameters.yearGeneratedEnergy) +
                  ", \"totalGeneratedEnergy\":" + String(statisticalParameters.totalGeneratedEnergy) +
                  ", \"CO2reduction\":" + String(statisticalParameters.CO2reduction) +
                  ", \"batteryCurrent\":" + String(statisticalParameters.batteryCurrent) +
                  ", \"batteryTemp\":" + String(statisticalParameters.batteryTemp) +
                  ", \"ambientTemp\":" + String(statisticalParameters.ambientTemp) + "}");
}

String getContentType(String filename)
{ // convert the file extension to the MIME type
  if (filename.endsWith(".html"))
    return "text/html";
  else if (filename.endsWith(".css"))
    return "text/css";
  else if (filename.endsWith(".js"))
    return "application/javascript";
  else if (filename.endsWith(".ico"))
    return "image/x-icon";
  return "text/plain";
}

bool handleFileRead(String path)
{ // send the right file to the client (if it exists)
  DebugPrintln("handleFileRead: " + path);
  if (path.endsWith("/"))
    path += "index.html";                    // If a folder is requested, send the index file
  String contentType = getContentType(path); // Get the MIME type
  if (SPIFFS.exists(path))
  {                                                     // If the file exists
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();                                       // Then close the file again
    return true;
  }
  DebugPrintln("\tFile Not Found");
  return false; // If the file doesn't exist, return false
}

void checkLoad()
{
  readOutputLoadState();
}

uint8_t readOutputLoadState()
{
  delay(10);
  result = node.readHoldingRegisters(0x903D, 1);

  if (result == node.ku8MBSuccess)
  {
    loadPoweredOn = (node.getResponseBuffer(0x00) & 0x02) > 0;

    DebugPrint("Set success. Load: ");
    DebugPrintln(loadPoweredOn);
  }
  else
  {
    // update of status failed
    DebugPrintln("readHoldingRegisters(0x903D, 1) failed!");
  }
  return result;
}

// reads Load Enable Override coil
uint8_t checkLoadCoilState()
{
  DebugPrint("Reading coil 0x0006... ");

  delay(10);
  result = node.readCoils(0x0006, 1);

  DebugPrint("Result: ");
  DebugPrintln(result);

  if (result == node.ku8MBSuccess)
  {
    loadPoweredOn = (node.getResponseBuffer(0x00) > 0);

    DebugPrint(" Value: ");
    DebugPrintln(loadPoweredOn);
  }
  else
  {
    DebugPrintln("Failed to read coil 0x0006!");
  }

  return result;
}

uint8_t setOutputLoadPower(uint8_t state)
{
  DebugPrint("Writing coil 0x0006 value to: ");
  DebugPrintln(state);

  delay(10);
  // Set coil at address 0x0006 (Force the load on/off)
  result = node.writeSingleCoil(0x0006, state);

  if (result == node.ku8MBSuccess)
  {
    node.getResponseBuffer(0x00);
    DebugPrintln("Success.");
  }

  return result;
}

void executeCurrentRegistryFunction()
{
  Registries[currentRegistryNumber]();
}

// function to switch to next registry
void nextRegistryNumber()
{
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries))
  {
    currentRegistryNumber = 0;
  }
}

void AddressRegistry_3100()
{
  result = node.readInputRegisters(0x3100, 8);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.pvVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("PV Voltage: ");
    DebugPrintln(realtimeStatus.pvVoltage);

    realtimeStatus.pvCurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("PV Current: ");
    DebugPrintln(realtimeStatus.pvCurrent);

    realtimeStatus.pvPower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("PV Power: ");
    DebugPrintln(realtimeStatus.pvPower);

    realtimeStatus.batteryVoltage = node.getResponseBuffer(0x04) / 100.0f;
    DebugPrint("Battery Voltage: ");
    DebugPrintln(realtimeStatus.batteryVoltage);

    realtimeStatus.batteryChargingCurrent = node.getResponseBuffer(0x05) / 100.0f;
    DebugPrint("Battery Charge Current: ");
    DebugPrintln(realtimeStatus.batteryChargingCurrent);

    realtimeStatus.batteryChargingPower = (node.getResponseBuffer(0x06) | node.getResponseBuffer(0x07) << 16) / 100.0f;
    DebugPrint("Battery Charge Power: ");
    DebugPrintln(realtimeStatus.batteryChargingPower);
  }
}

// void AddressRegistry_3106()
// {
//   result = node.readInputRegisters(0x3106, 2);

//   if (result == node.ku8MBSuccess)
//   {
//     realtimeStatus.batteryChargingPower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
//     DebugPrint("Battery Charge Power: ");
//     DebugPrintln(realtimeStatus.batteryChargingPower);
//   }
// }

void AddressRegistry_310C()
{
  result = node.readInputRegisters(0x310C, 4);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.loadVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Load Voltage: ");
    DebugPrintln(realtimeStatus.loadCurrent);

    realtimeStatus.loadCurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Load Current: ");
    DebugPrintln(realtimeStatus.loadCurrent);

    realtimeStatus.loadPower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("Load Power: ");
    DebugPrintln(realtimeStatus.loadPower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x310D failed!");
  }
}

void AddressRegistry_3110()
{
  result = node.readInputRegisters(0x3110, 3);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.batteryTemp = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Battery Temp: ");
    DebugPrintln(realtimeStatus.batteryTemp);

    realtimeStatus.equipmentTemp = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Equipment Temp: ");
    DebugPrintln(realtimeStatus.equipmentTemp);
    
    realtimeStatus.heatsinkTemp = node.getResponseBuffer(0x02) / 100.0f;
    DebugPrint("Heatsink Temp: ");
    DebugPrintln(realtimeStatus.heatsinkTemp);
  }
}

void AddressRegistry_311A()
{
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.batterySoC = node.getResponseBuffer(0x00) / 1.0f;
    DebugPrint("Battery State of Charge %: ");
    DebugPrintln(realtimeStatus.batterySoC);

    realtimeStatus.batteryTemp = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Battery Temperature: ");
    DebugPrintln(realtimeStatus.batteryTemp);

    // realtimeStatus.batteryRatedPower = node.getResponseBuffer(0x03) / 100.0f;
    // DebugPrint("Battery Rated Power: ");
    // DebugPrintln(realtimeStatus.batteryRatedPower);

  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x311A failed!");
  }
}

void AddressRegistry_311D()
{
  result = node.readInputRegisters(0x311D, 1);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.batteryRatedPower = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Battery Rated Power: ");
    DebugPrintln(realtimeStatus.batteryRatedPower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x311D failed!");
  }
}

void AddressRegistry_331B()
{
  result = node.readInputRegisters(0x331B, 2);

  if (result == node.ku8MBSuccess)
  {
    statisticalParameters.batteryCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Battery Discharge Current: ");
    DebugPrintln(statisticalParameters.batteryCurrent);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x331B failed!");
  }
}
