#include <Arduino.h>

// #define DEBUG
#define DEBUG_OI Serial
#include "debug.h"

#include <Timer.h>       //https://github.com/JChristensen/Timer
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <ArduinoOTA.h>
#include <FS.h> // Include the SPIFFS library

//#define HOSTNAME "tracer-1"
#define LED_PIN D4 //GPIO2
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#include <ModbusMaster.h> //https://github.com/4-20ma/ModbusMaster
#include "debug.h"

void AddressRegistry_2000();
void AddressRegistry_200C();

void AddressRegistry_3000();
void AddressRegistry_300E();

void AddressRegistry_3100();
void AddressRegistry_310C();
void AddressRegistry_3110();
void AddressRegistry_311A();
void AddressRegistry_311D();

void AddressRegistry_3200();

void AddressRegistry_3300();
void AddressRegistry_3310();
void AddressRegistry_330A();
void AddressRegistry_331B();

ModbusMaster node;
uint8_t result;

//rated data - input register (16 bit word readonly)
struct rated_data
{
  float pvVoltage;
  float pvCurrent;
  int16_t pvPower;
  float batteryVoltage;
  float batteryCurrent;
  int16_t batteryPower;
  uint8_t chargingMode; //0000H Connect/disconnect, 0001H PWM, 0002H MPPT
  float loadCurrent;
} ratedData;

//realtime data - input register (16 bit word readonly)
struct realtime_data
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
} realtimeData;

//realtime status - input register (16 bit word readonly)
struct realtime_status
{
  uint16_t batteryStatus;
  uint16_t chargeEquipmentStatus;
  uint16_t dischargeEquipmentStatus;
} realtimeStatus;

//statistical parameters - input register (16 bit word readonly)
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

//setting_parameters - holding register (16 bit word read-write)
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

//coil / switch values - coils (single bit read-write)
struct switch_value
{
  bool manualControl;
  bool loadTest;
  bool forceLoad;
} switchValues;

//discrete_input - discretes input (single bit readonly)
struct discrete_input
{
  bool overTemp;
  bool dayNight;
} discreteInput;

bool rs485DataReceived = true;
bool loadPoweredOn = true;

Timer timer;

//web server json responders
void getRatedData();
void getRealtimeData();
void getRealtimeStatus();
void getStatisticalData();
void getCoils();
void getDiscrete();
void info();

// tracer requires no handshaking
void preTransmission() {}
void postTransmission() {}

void readManualCoil();
void readLoadTestAndForceLoadCoil();
uint8_t setOutputLoadPower(uint8_t state);

void updateNextRegistryEntry();

// a list of the regisities to query in order
typedef void (*RegistryList[])();
RegistryList Registries = {
    AddressRegistry_2000,
    AddressRegistry_200C,
    AddressRegistry_3000,
    AddressRegistry_300E,
    AddressRegistry_3100,
    AddressRegistry_310C,
    AddressRegistry_3110,
    AddressRegistry_311A,
    AddressRegistry_311D,
    AddressRegistry_3200,
    AddressRegistry_3300,
    AddressRegistry_330A,
    AddressRegistry_3310,
    AddressRegistry_331B,
    readManualCoil,
    readLoadTestAndForceLoadCoil};
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
  server.on("/getRealtimeStatus", getRealtimeStatus);
  server.on("/getStatisticalData", getStatisticalData);
  server.on("/getCoils", getCoils);
  server.on("/getDiscrete", getDiscrete);
  server.on("/info", info);

  server.begin(); // Actually start the server
  DebugPrintln("HTTP server started");

  DebugPrintln("Starting timed actions...");
  timer.every(600L, updateNextRegistryEntry);

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
              "{\"pvVoltage\":" + String(realtimeData.pvVoltage) +
                  ", \"pvCurrent\":" + String(realtimeData.pvCurrent) +
                  ", \"pvPower\":" + String(realtimeData.pvPower) +
                  ", \"batteryVoltage\":" + String(realtimeData.batteryVoltage) +
                  ", \"batteryChargingCurrent\":" + String(realtimeData.batteryChargingCurrent) +
                  ", \"batteryChargingPower\":" + String(realtimeData.batteryChargingPower) +
                  ", \"loadVoltage\":" + String(realtimeData.loadVoltage) +
                  ", \"loadCurrent\":" + String(realtimeData.loadCurrent) +
                  ", \"loadPower\":" + String(realtimeData.loadPower) +
                  ", \"batteryTemp\":" + String(realtimeData.batteryTemp) +
                  ", \"equipmentTemp\":" + String(realtimeData.equipmentTemp) +
                  ", \"heatsinkTemp\":" + String(realtimeData.heatsinkTemp) +
                  ", \"batterySoC\":" + String(realtimeData.batterySoC) +
                  ", \"batteryRemoteTemp\":" + String(realtimeData.batteryRemoteTemp) +
                  ", \"batteryRatedPower\":" + String(realtimeData.batteryRatedPower) + "}");
}

void getRealtimeStatus()
{
  server.send(200, "application/json",
              "{\"batteryStatus\":" + String(realtimeStatus.batteryStatus) +
                  ", \"chargeEquipmentStatus\":" + String(realtimeStatus.chargeEquipmentStatus) +
                  ", \"dischargeEquipmentStatus\":" + String(realtimeStatus.dischargeEquipmentStatus) + "}");
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

void getCoils()
{
  server.send(200, "application/json",
              "{\"manualControl\":" + String(switchValues.manualControl) +
                  ", \"loadTest\":" + String(switchValues.loadTest) +
                  ", \"forceLoad\":" + String(switchValues.forceLoad) + "}");
}

void getDiscrete()
{
  server.send(200, "application/json",
              "{\"overTemp\":" + String(discreteInput.overTemp) +
                  ", \"dayNight\":" + String(discreteInput.dayNight) + "}");
}

String htmlHeader(String title)
{
  return "<html><head><title>" + title + "</title></head><body>";
}

String htmlFooter()
{
  return "</body></html>";
}

void info()
{
  // calculate uptime
  long millisecs = millis() / 1000;
  int systemUpTimeSc = millisecs % 60;
  int systemUpTimeMn = (millisecs / 60) % 60;
  int systemUpTimeHr = (millisecs / (60 * 60)) % 24;
  int systemUpTimeDy = (millisecs / (60 * 60 * 24));

  // compose uptime string
  String uptime = "<b>System Uptime:</b> " + String(systemUpTimeDy) + " day(s), " +
                  systemUpTimeHr + " hour(s), " + systemUpTimeMn + " minute(s), " +
                  systemUpTimeSc + " second(s)";

  // compose info string
  String info = htmlHeader(String(HOSTNAME) + " Info") + "<h1>" + String(HOSTNAME) + " Info </h1>" +
                "<b>ESP8266 Core Version:</b> " + String(ESP.getCoreVersion()) + "</br>" +
                "<b>ESP8266 SDK Version:</b> " + String(ESP.getSdkVersion()) + "</br></br>" +
                "<b>Reset Reason:</b> " + String(ESP.getResetReason()) + "</br></br>" +
                "<b>Free Heap:</b> " + String(ESP.getFreeHeap()) + "bytes (" +
                ESP.getHeapFragmentation() + "% fragmentation)</br></br>"
                                             "<b>ESP8266 Chip ID:</b> " +
                String(ESP.getChipId()) + "</br>" +
                "<b>ESP8266 Flash Chip ID:</b> " + String(ESP.getFlashChipId()) + "</br></br>" +
                "<b>Flash Chip Size:</b> " + String(ESP.getFlashChipRealSize()) + " bytes (" +
                ESP.getFlashChipSize() + " bytes seen by SDK) </br>" +
                "<b>Sketch Size:</b> " + String(ESP.getSketchSize()) + " bytes used of " +
                String(ESP.getFreeSketchSpace()) + " bytes available"
                                                   "</br></br>" +
                uptime + htmlFooter();

  server.send(200, "text/html", info);
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

// reads manual control state
void readManualCoil()
{
  DebugPrint("Reading coil 0x02... ");
  delay(10);

  result = node.readCoils(0x0002, 1);

  if (result == node.ku8MBSuccess)
  {
    switchValues.manualControl = (node.getResponseBuffer(0x00) > 0);

    DebugPrint("Manual Load Control State: ");
    DebugPrintln(switchValues.manualControl);
  }
  else
  {
    DebugPrintln("Failed to read coil 0x02!");
  }
}

// reads Load Enable Override coil
void readLoadTestAndForceLoadCoil()
{
  DebugPrint("Reading coil 0x05 & 0x06... ");
  delay(10);

  result = node.readCoils(0x0005, 2);

  if (result == node.ku8MBSuccess)
  {
    switchValues.loadTest = (node.getResponseBuffer(0x00) > 0);
    DebugPrint("Enable Load Test Mode: ");
    DebugPrintln(switchValues.loadTest);

    switchValues.forceLoad = (node.getResponseBuffer(0x01) > 0);
    DebugPrint("Force Load On/Off: ");
    DebugPrintln(switchValues.forceLoad);
  }
  else
  {
    DebugPrintln("Failed to read coils 0x05 & 0x06!");
  }
}

uint8_t setOutputLoadPower(uint8_t state)
{
  DebugPrint("Writing coil 0x0006 value to: ");
  DebugPrintln(state);

  // Set coil at address 0x0006 (Force the load on/off)
  result = node.writeSingleCoil(0x0006, state);

  if (result == node.ku8MBSuccess)
  {
    node.getResponseBuffer(0x00);
    DebugPrintln("Success.");
  }

  return result;
}

void updateNextRegistryEntry()
{
  Registries[currentRegistryNumber]();

  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries))
  {
    currentRegistryNumber = 0;
  }
}

void AddressRegistry_2000()
{
  result = node.readDiscreteInputs(0x2000, 1);

  if (result == node.ku8MBSuccess)
  {
    discreteInput.overTemp = node.getResponseBuffer(0x00);
    DebugPrint("Over temperature inside device (1) or Normal (0): ");
    DebugPrintln(discreteInput.overTemp);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read discrete input 0x2000 failed!");
  }
}

void AddressRegistry_200C()
{
  result = node.readDiscreteInputs(0x200C, 1);

  if (result == node.ku8MBSuccess)
  {
    discreteInput.dayNight = node.getResponseBuffer(0x00);
    DebugPrint("Day (0) or Night (1): ");
    DebugPrintln(discreteInput.dayNight);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read discrete input 0x200C failed!");
  }
}

void AddressRegistry_3000()
{
  result = node.readInputRegisters(0x3000, 9);

  if (result == node.ku8MBSuccess)
  {
    ratedData.pvVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("PV Voltage: ");
    DebugPrintln(ratedData.pvVoltage);

    ratedData.pvCurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("PV Current: ");
    DebugPrintln(ratedData.pvCurrent);

    ratedData.pvPower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("PV Power: ");
    DebugPrintln(ratedData.pvPower);

    ratedData.batteryVoltage = node.getResponseBuffer(0x04) / 100.0f;
    DebugPrint("Battery Voltage: ");
    DebugPrintln(ratedData.batteryVoltage);

    ratedData.batteryCurrent = node.getResponseBuffer(0x05) / 100.0f;
    DebugPrint("Battery Current: ");
    DebugPrintln(ratedData.batteryCurrent);

    ratedData.batteryPower = (node.getResponseBuffer(0x06) | node.getResponseBuffer(0x07) << 16) / 100.0f;
    DebugPrint("Battery Power: ");
    DebugPrintln(ratedData.batteryPower);

    ratedData.chargingMode = node.getResponseBuffer(0x08);
    DebugPrint("Charging mode: ");
    DebugPrintln(ratedData.chargingMode);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3000 failed!");
  }
}

void AddressRegistry_300E()
{
  result = node.readInputRegisters(0x300E, 1);

  if (result == node.ku8MBSuccess)
  {
    ratedData.loadCurrent = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Load Current: ");
    DebugPrintln(ratedData.loadCurrent);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x311D failed!");
  }
}

void AddressRegistry_3100()
{
  result = node.readInputRegisters(0x3100, 8);

  if (result == node.ku8MBSuccess)
  {
    realtimeData.pvVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("PV Voltage: ");
    DebugPrintln(realtimeData.pvVoltage);

    realtimeData.pvCurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("PV Current: ");
    DebugPrintln(realtimeData.pvCurrent);

    realtimeData.pvPower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("PV Power: ");
    DebugPrintln(realtimeData.pvPower);

    realtimeData.batteryVoltage = node.getResponseBuffer(0x04) / 100.0f;
    DebugPrint("Battery Voltage: ");
    DebugPrintln(realtimeData.batteryVoltage);

    realtimeData.batteryChargingCurrent = node.getResponseBuffer(0x05) / 100.0f;
    DebugPrint("Battery Charge Current: ");
    DebugPrintln(realtimeData.batteryChargingCurrent);

    realtimeData.batteryChargingPower = (node.getResponseBuffer(0x06) | node.getResponseBuffer(0x07) << 16) / 100.0f;
    DebugPrint("Battery Charge Power: ");
    DebugPrintln(realtimeData.batteryChargingPower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3100 failed!");
  }
}

void AddressRegistry_310C()
{
  result = node.readInputRegisters(0x310C, 4);

  if (result == node.ku8MBSuccess)
  {
    realtimeData.loadVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Load Voltage: ");
    DebugPrintln(realtimeData.loadCurrent);

    realtimeData.loadCurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Load Current: ");
    DebugPrintln(realtimeData.loadCurrent);

    realtimeData.loadPower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("Load Power: ");
    DebugPrintln(realtimeData.loadPower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x310C failed!");
  }
}

void AddressRegistry_3110()
{
  result = node.readInputRegisters(0x3110, 3);

  if (result == node.ku8MBSuccess)
  {
    realtimeData.batteryTemp = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Battery Temp: ");
    DebugPrintln(realtimeData.batteryTemp);

    realtimeData.equipmentTemp = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Equipment Temp: ");
    DebugPrintln(realtimeData.equipmentTemp);

    realtimeData.heatsinkTemp = node.getResponseBuffer(0x02) / 100.0f;
    DebugPrint("Heatsink Temp: ");
    DebugPrintln(realtimeData.heatsinkTemp);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3110 failed!");
  }
}

void AddressRegistry_311A()
{
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess)
  {
    realtimeData.batterySoC = node.getResponseBuffer(0x00) / 1.0f;
    DebugPrint("Battery State of Charge %: ");
    DebugPrintln(realtimeData.batterySoC);

    realtimeData.batteryTemp = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Battery Temperature: ");
    DebugPrintln(realtimeData.batteryTemp);
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
    realtimeData.batteryRatedPower = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Battery Rated Power: ");
    DebugPrintln(realtimeData.batteryRatedPower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x311D failed!");
  }
}

void AddressRegistry_3200()
{
  result = node.readInputRegisters(0x3200, 3);

  if (result == node.ku8MBSuccess)
  {
    realtimeStatus.batteryStatus = node.getResponseBuffer(0x00);
    DebugPrint("Battery Status: ");
    DebugPrintln(realtimeStatus.batteryStatus);

    realtimeStatus.chargeEquipmentStatus = node.getResponseBuffer(0x01);
    DebugPrint("Charge Equipment Status: ");
    DebugPrintln(realtimeStatus.chargeEquipmentStatus);

    realtimeStatus.dischargeEquipmentStatus = node.getResponseBuffer(0x02);
    DebugPrint("Discharge Equipment Status: ");
    DebugPrintln(realtimeStatus.dischargeEquipmentStatus);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3200 failed!");
  }
}

void AddressRegistry_3300()
{
  result = node.readInputRegisters(0x3300, 16);

  if (result == node.ku8MBSuccess)
  {
    statisticalParameters.todayMaxPvVoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Maximum PV today: ");
    DebugPrintln(statisticalParameters.todayMaxPvVoltage);

    statisticalParameters.todayMinPvVoltage = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Minimum PV today: ");
    DebugPrintln(statisticalParameters.todayMinPvVoltage);

    statisticalParameters.todayMaxBattVoltage = node.getResponseBuffer(0x02) / 100.0f;
    DebugPrint("Maximum Battery today: ");
    DebugPrintln(statisticalParameters.todayMaxBattVoltage);

    statisticalParameters.todayMinBattVoltage = node.getResponseBuffer(0x03) / 100.0f;
    DebugPrint("Minimum Battery today: ");
    DebugPrintln(statisticalParameters.todayMinBattVoltage);

    statisticalParameters.todayConsumedEnergy = (node.getResponseBuffer(0x04) | node.getResponseBuffer(0x05) << 16) / 100.0f;
    DebugPrint("Consumed energy today: ");
    DebugPrintln(statisticalParameters.todayConsumedEnergy);

    statisticalParameters.monthConsumedEnergy = (node.getResponseBuffer(0x06) | node.getResponseBuffer(0x07) << 16) / 100.0f;
    DebugPrint("Consumed energy this month: ");
    DebugPrintln(statisticalParameters.monthConsumedEnergy);

    statisticalParameters.yearConsumedEnergy = (node.getResponseBuffer(0x08) | node.getResponseBuffer(0x09) << 16) / 100.0f;
    DebugPrint("Consumed energy this year: ");
    DebugPrintln(statisticalParameters.yearConsumedEnergy);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3300 failed!");
  }
}

void AddressRegistry_330A()
{
  result = node.readInputRegisters(0x330A, 6);

  if (result == node.ku8MBSuccess)
  {
    statisticalParameters.totalConsumedEnergy = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Total consumed energy: ");
    DebugPrintln(statisticalParameters.totalConsumedEnergy);

    statisticalParameters.todayGeneratedEnergy = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("Generated energy today: ");
    DebugPrintln(statisticalParameters.todayGeneratedEnergy);

    statisticalParameters.monthGeneratedEnergy = (node.getResponseBuffer(0x04) | node.getResponseBuffer(0x05) << 16) / 100.0f;
    DebugPrint("Generated energy this month: ");
    DebugPrintln(statisticalParameters.monthGeneratedEnergy);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x330A failed!");
  }
}

void AddressRegistry_3310()
{
  result = node.readInputRegisters(0x3310, 6);

  if (result == node.ku8MBSuccess)
  {
    statisticalParameters.yearGeneratedEnergy = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Generated energy this year: ");
    DebugPrintln(statisticalParameters.yearGeneratedEnergy);

    statisticalParameters.totalGeneratedEnergy = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("Total generated energy: ");
    DebugPrintln(statisticalParameters.totalGeneratedEnergy);

    statisticalParameters.CO2reduction = (node.getResponseBuffer(0x04) | node.getResponseBuffer(0x05) << 16) / 100.0f;
    DebugPrint("Carbon dioxide reduction: ");
    DebugPrintln(statisticalParameters.CO2reduction);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x3310 failed!");
  }
}

void AddressRegistry_331B()
{
  result = node.readInputRegisters(0x331B, 4);

  if (result == node.ku8MBSuccess)
  {
    statisticalParameters.batteryCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Battery Discharge Current: ");
    DebugPrintln(statisticalParameters.batteryCurrent);

    statisticalParameters.batteryTemp = node.getResponseBuffer(0x02) / 100.0f;
    DebugPrint("Battery Temperature: ");
    DebugPrintln(statisticalParameters.batteryTemp);

    statisticalParameters.ambientTemp = node.getResponseBuffer(0x03) / 100.0f;
    DebugPrint("Ambient Temperature: ");
    DebugPrintln(statisticalParameters.ambientTemp);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x331B failed!");
  }
}
