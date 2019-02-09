#include <Arduino.h>

//#define DEBUG
#define DEBUG_OI Serial
#include "debug.h"

#include <ModbusMaster.h>
#include <Timer.h>

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

#include <ArduinoOTA.h>

#include <FS.h> // Include the SPIFFS library

#define HOSTNAME "tracer"
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;

bool rs485DataReceived = true;
bool loadPoweredOn = true;

ModbusMaster node;
Timer timer;

// tracer requires no handshaking
void getData();

void preTransmission() {}
void postTransmission() {}

uint8_t readOutputLoadState();
uint8_t checkLoadCoilState();
uint8_t setOutputLoadPower(uint8_t state);

void executeCurrentRegistryFunction();
void nextRegistryNumber();

void AddressRegistry_3100();
void AddressRegistry_3106();
void AddressRegistry_3111();
void AddressRegistry_310D();
void AddressRegistry_311A();
void AddressRegistry_331B();

// a list of the regisities to query in order
typedef void (*RegistryList[])();
RegistryList Registries = {
    AddressRegistry_3100,
    AddressRegistry_3106,
    AddressRegistry_3111,
    AddressRegistry_310D,
    AddressRegistry_311A,
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

  pinMode(LED_BUILTIN, OUTPUT);    //GPIO16
  digitalWrite(LED_BUILTIN, HIGH); //inverted, off

  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFiManager wifiManager;
  //wifiManager.resetSettings();
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
    digitalWrite(LED_BUILTIN, HIGH);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentageComplete = (progress / (total / 100));
    DebugPrintf("Progress: %u%%\r", percentageComplete);
    digitalWrite(LED_BUILTIN, percentageComplete % 2);
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
  server.on("/getData", getData);

  server.begin(); // Actually start the server
  DebugPrintln("HTTP server started");

  DebugPrintln("Starting timed actions...");
  timer.every(1000L, executeCurrentRegistryFunction);
  timer.every(1000L, nextRegistryNumber);
  timer.every(5000L, checkLoad);

  DebugPrintln("Setup OK!");
  DebugPrintln("----------------------------");
  DebugPrintln();
}

void loop()
{
  MDNS.update();
  ArduinoOTA.handle();
  timer.update();

  server.handleClient();
}

void getData()
{
  server.send(200, "application/json",
              "{\"pv_power\":" + String(pvpower) +
                  ", \"pv_current\":" + String(pvcurrent) +
                  ", \"pv_voltage\":" + String(pvvoltage) +
                  ", \"load_current\":" + String(lcurrent) +
                  ", \"load_power\":" + String(lpower) +
                  ", \"batt_temp\":" + String(btemp) +
                  ", \"batt_voltage\":" + String(bvoltage) +
                  ", \"batt_current\":" + String(battOverallCurrent) +
                  ", \"batt_remain\":" + String(bremaining) +
                  ", \"batt_charge_power\":" + String(battChargePower) +
                  ", \"case_temp\":" + String(ctemp) + "}");
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
  result = node.readInputRegisters(0x3100, 6);

  if (result == node.ku8MBSuccess)
  {

    pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("PV Voltage: ");
    DebugPrintln(pvvoltage);

    pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("PV Current: ");
    DebugPrintln(pvcurrent);

    pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    DebugPrint("PV Power: ");
    DebugPrintln(pvpower);

    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    DebugPrint("Battery Voltage: ");
    DebugPrintln(bvoltage);

    battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;
    DebugPrint("Battery Charge Current: ");
    DebugPrintln(battChargeCurrent);
  }
}

void AddressRegistry_3106()
{
  result = node.readInputRegisters(0x3106, 2);

  if (result == node.ku8MBSuccess)
  {
    battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Battery Charge Power: ");
    DebugPrintln(battChargePower);
  }
}

void AddressRegistry_3111()
{
  result = node.readInputRegisters(0x3111, 1);

  if (result == node.ku8MBSuccess)
  {
    ctemp = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Case Temp: ");
    DebugPrintln(ctemp);
  }
}

void AddressRegistry_310D()
{
  result = node.readInputRegisters(0x310D, 3);

  if (result == node.ku8MBSuccess)
  {
    lcurrent = node.getResponseBuffer(0x00) / 100.0f;
    DebugPrint("Load Current: ");
    DebugPrintln(lcurrent);

    lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
    DebugPrint("Load Power: ");
    DebugPrintln(lpower);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x310D failed!");
  }
}

void AddressRegistry_311A()
{
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess)
  {
    bremaining = node.getResponseBuffer(0x00) / 1.0f;
    DebugPrint("Battery Remaining %: ");
    DebugPrintln(bremaining);

    btemp = node.getResponseBuffer(0x01) / 100.0f;
    DebugPrint("Battery Temperature: ");
    DebugPrintln(btemp);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x311A failed!");
  }
}

void AddressRegistry_331B()
{
  result = node.readInputRegisters(0x331B, 2);

  if (result == node.ku8MBSuccess)
  {
    battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    DebugPrint("Battery Discharge Current: ");
    DebugPrintln(battOverallCurrent);
  }
  else
  {
    rs485DataReceived = false;
    DebugPrintln("Read register 0x331B failed!");
  }
}
