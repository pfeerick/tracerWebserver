#include <Arduino.h>

//#define DEBUG
#define DEBUG_OI Serial
#include "debug.h"

#include <ModbusMaster.h>
#include <Timer.h>

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <ArduinoOTA.h>

#define OTA_HOSTNAME "tracer"
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

WiFiServer server(80);

// Variable to store the HTTP request
String header;

void checkLoad();

void setup() {
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

  pinMode(LED_BUILTIN, OUTPUT);  //GPIO16
  digitalWrite(LED_BUILTIN,HIGH); //inverted, off

  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect("AutoConnectAP");

 DebugPrintln("Starting ArduinoOTA...");

 ArduinoOTA.setHostname(OTA_HOSTNAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DebugPrintln("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    DebugPrintln("\nEnd of update");
    digitalWrite(LED_BUILTIN,HIGH);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentageComplete = (progress / (total / 100));
    DebugPrintf("Progress: %u%%\r", percentageComplete);
    digitalWrite(LED_BUILTIN, percentageComplete % 2);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    DebugPrintf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DebugPrintln("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DebugPrintln("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DebugPrintln("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DebugPrintln("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DebugPrintln("End Failed");
    }
  });
  
  ArduinoOTA.begin();

  DebugPrint("ArduinoOTA running. ");

  DebugPrint("IP address: ");
  DebugPrintln(WiFi.localIP());
  
  DebugPrintln("Starting timed actions...");
  timer.every(1000L, executeCurrentRegistryFunction);
  timer.every(1000L, nextRegistryNumber);
  timer.every(5000L, checkLoad);

  DebugPrintln("Setup OK!");
  DebugPrintln("----------------------------");
  DebugPrintln();

  server.begin();
}

void loop() {
   ArduinoOTA.handle();
   timer.update();

   WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
                       
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<meta http-equiv=\"refresh\" content=\"5\"");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto;} .data-table td {   width: 200px; }</style>");
            
            // Web Page Heading
            client.println("<body><h1>ESP8266 Tracer Dashboard</h1>");
            client.println("<table class='data-table'>");
            
            client.println("<tr><td>PV Power</td><td>" + String(pvpower) + "W</td></tr>");
            client.println("<tr><td>PV Current</td><td>" + String(pvcurrent) + "A</td></tr>");
            client.println("<tr><td>PV Voltage</td><td>" + String(pvvoltage) + "V</td></tr>");
            
            client.println("<tr><td>Load Current</td><td>" + String(lcurrent) + "A</td></tr>");
            client.println("<tr><td>Load Power</td><td>" + String(lpower) + "W</td></tr>");
            
            client.println("<tr><td>Battery Temp</td><td>" + String(btemp) + "&deg;C</td></tr>");
            client.println("<tr><td>Battery Voltage</td><td>" + String(bvoltage) + "V</td></tr>");
            client.println("<tr><td>Battery Current</td><td>" + String(battOverallCurrent) + "A</td></tr>");
            client.println("<tr><td>Battery Remaining</td><td>" + String(bremaining) + "%</td></tr>");
            
            client.println("<tr><td>Battery Charge Power</td><td>" + String(battChargePower) + "W</td></tr>");            
            
            client.println("<tr><td>Case Temp</td><td>" + String(ctemp) + "&deg;C</td></tr>");
            
            client.println("</table>");

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
  }
}

void checkLoad()
{
  readOutputLoadState();
}

  uint8_t readOutputLoadState() {
    delay(10);
    result = node.readHoldingRegisters(0x903D, 1);
    
    if (result == node.ku8MBSuccess) {
      loadPoweredOn = (node.getResponseBuffer(0x00) & 0x02) > 0;
  
      DebugPrint("Set success. Load: ");
      DebugPrintln(loadPoweredOn);
    } else {
      // update of status failed
      DebugPrintln("readHoldingRegisters(0x903D, 1) failed!");
    }
    return result;
  }
  
  // reads Load Enable Override coil
  uint8_t checkLoadCoilState() {
    DebugPrint("Reading coil 0x0006... ");

    delay(10);
    result = node.readCoils(0x0006, 1);
    
    DebugPrint("Result: ");
    DebugPrintln(result);

    if (result == node.ku8MBSuccess) {
      loadPoweredOn = (node.getResponseBuffer(0x00) > 0);

      DebugPrint(" Value: ");
      DebugPrintln(loadPoweredOn);
    } else {
      DebugPrintln("Failed to read coil 0x0006!");
    }

    return result;
}

  uint8_t setOutputLoadPower(uint8_t state) {
    DebugPrint("Writing coil 0x0006 value to: ");
    DebugPrintln(state);

    delay(10);
    // Set coil at address 0x0006 (Force the load on/off)
    result = node.writeSingleCoil(0x0006, state);

    if (result == node.ku8MBSuccess) {
      node.getResponseBuffer(0x00);
      DebugPrintln("Success.");
    }

    return result;
}

void executeCurrentRegistryFunction() {
  Registries[currentRegistryNumber]();
}

// function to switch to next registry
void nextRegistryNumber() {
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries)) {
    currentRegistryNumber = 0;
  }
}

  void AddressRegistry_3100() {
    result = node.readInputRegisters(0x3100, 6);
  
    if (result == node.ku8MBSuccess) {
      
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

    if (result == node.ku8MBSuccess) {
      battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
      DebugPrint("Battery Charge Power: ");
      DebugPrintln(battChargePower);
    }
  }

  void AddressRegistry_3111()
  {
    result = node.readInputRegisters(0x3111, 1);

    if (result == node.ku8MBSuccess) {
      ctemp = node.getResponseBuffer(0x00) / 100.0f;
      DebugPrint("Case Temp: ");
      DebugPrintln(ctemp);
    }
  }

  void AddressRegistry_310D() 
  {
    result = node.readInputRegisters(0x310D, 3);

    if (result == node.ku8MBSuccess) {
      lcurrent = node.getResponseBuffer(0x00) / 100.0f;
      DebugPrint("Load Current: ");
      DebugPrintln(lcurrent);
  
      lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
      DebugPrint("Load Power: ");
      DebugPrintln(lpower);
    } else {
      rs485DataReceived = false;
      DebugPrintln("Read register 0x310D failed!");
    }    
  } 

  void AddressRegistry_311A() {
    result = node.readInputRegisters(0x311A, 2);
   
    if (result == node.ku8MBSuccess) {    
      bremaining = node.getResponseBuffer(0x00) / 1.0f;
      DebugPrint("Battery Remaining %: ");
      DebugPrintln(bremaining);
      
      btemp = node.getResponseBuffer(0x01) / 100.0f;
      DebugPrint("Battery Temperature: ");
      DebugPrintln(btemp);
    } else {
      rs485DataReceived = false;
      DebugPrintln("Read register 0x311A failed!");
    }
  }

  void AddressRegistry_331B() {
    result = node.readInputRegisters(0x331B, 2);
    
    if (result == node.ku8MBSuccess) {
      battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
      DebugPrint("Battery Discharge Current: ");
      DebugPrintln(battOverallCurrent);
    } else {
      rs485DataReceived = false;
      DebugPrintln("Read register 0x331B failed!");
    }
}
