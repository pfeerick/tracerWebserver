#include <Arduino.h>

#include <ModbusMaster.h>
#include <Timer.h>

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <ArduinoOTA.h>

#define OTA_HOSTNAME "tracer"
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

const int debug = 0;

float battBhargeCurrent, bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;

bool rs485DataReceived = true;

ModbusMaster node;
Timer timer;

// tracer requires no handshaking
void preTransmission() {}
void postTransmission() {}

void doRegistryNumber();
void nextRegistryNumber();

void AddressRegistry_3100();
void AddressRegistry_311A();
void AddressRegistry_3300();

// a list of the regisities to query in order
typedef void (*RegistryList[])();
RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_311A,
  AddressRegistry_3300,
};
// keep log of where we are
uint8_t currentRegistryNumber = 0;

WiFiServer server(80);

// Variable to store the HTTP request
String header;

void setup() {
  Serial.begin(115200);

  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect("AutoConnectAP");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(WiFi.hostname());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

  timer.every(1000,doRegistryNumber);
  timer.every(1000,nextRegistryNumber);

  server.begin();
}

void loop() {
   ArduinoOTA.handle();
   timer.update();

   WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    // Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        // Serial.write(c);                    // print it out the serial monitor
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
            
            // // turns the GPIOs on and off
            // if (header.indexOf("GET /5/on") >= 0) {
            //   Serial.println("GPIO 5 on");
            //   output5State = "on";
            //   digitalWrite(output5, HIGH);
            // } else if (header.indexOf("GET /5/off") >= 0) {
            //   Serial.println("GPIO 5 off");
            //   output5State = "off";
            //   digitalWrite(output5, LOW);
            // } else if (header.indexOf("GET /4/on") >= 0) {
            //   Serial.println("GPIO 4 on");
            //   output4State = "on";
            //   digitalWrite(output4, HIGH);
            // } else if (header.indexOf("GET /4/off") >= 0) {
            //   Serial.println("GPIO 4 off");
            //   output4State = "off";
            //   digitalWrite(output4, LOW);
            // }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<meta http-equiv=\"refresh\" content=\"5\"");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
             client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto;} .data-table td {   width: 200px; }</style>");
            // client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            // client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            // client.println(".button2 {background-color: #77878A;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP8266 Tracer Dashboard</h1>");
            client.println("<table class='data-table'>");
            client.println("<tr><td>PV Power</td><td>" + String(pvpower) + "W</td></tr>");
            client.println("<tr><td>PV Current</td><td>" + String(pvcurrent) + "A</td></tr>");
            client.println("<tr><td>PV Voltage</td><td>" + String(pvvoltage) + "V</td></tr>");
            client.println("<tr><td>Load Current</td><td>" + String(lcurrent) + "A</td></tr>");
            client.println("<tr><td>Load Power</td><td>" + String(lpower) + "W</td></tr>");
            client.println("<tr><td>Remote Battery Temp</td><td>" + String(btemp) + "&deg;C</td></tr>");
            client.println("<tr><td>Battery Voltage</td><td>" + String(bvoltage) + "V</td></tr>");
            client.println("<tr><td>Battery Remaining</td><td>" + String(bremaining) + "%</td></tr>");
            client.println("<tr><td>Case Temp</td><td>" + String(ctemp) + "&deg;C</td></tr>");
            client.println("</table>");

            // Display current state, and ON/OFF buttons for GPIO 5  
            // client.println("<p>GPIO 5 - State " + output5State + "</p>");
            // // If the output5State is off, it displays the ON button       
            // if (output5State=="off") {
            //   client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            // } else {
            //   client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            // } 
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            // client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // // If the output4State is off, it displays the ON button       
            // if (output4State=="off") {
            //   client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            // } else {
            //   client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            // }
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
    // Serial.println("Client disconnected.");
    // Serial.println("");
  }

}

void doRegistryNumber() {
  Registries[currentRegistryNumber]();
}

void nextRegistryNumber() {
  currentRegistryNumber = (currentRegistryNumber + 1) % ARRAY_SIZE( Registries);
}

void AddressRegistry_3100() {
  result = node.readInputRegisters(0x3100, 0x12);
  if (result == node.ku8MBSuccess)
  {
    ctemp = node.getResponseBuffer(0x11) / 100.0f;
    if (debug == 1) {
      Serial.print("Case Temp: ");
      Serial.println(ctemp);
    }

    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Voltage: ");
      Serial.println(bvoltage);
    }
    lpower = ((long)node.getResponseBuffer(0x0F) << 16 | node.getResponseBuffer(0x0E)) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Power: ");
      Serial.println(lpower);

    }
    lcurrent = (long)node.getResponseBuffer(0x0D) / 100.0f;
    if (debug == 1) {
      Serial.print("Load Current: ");
      Serial.println(lcurrent);

    }
    pvvoltage = (long)node.getResponseBuffer(0x00) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Voltage: ");
      Serial.println(pvvoltage);

    }
    pvcurrent = (long)node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Current: ");
      Serial.println(pvcurrent);

    }
    pvpower = ((long)node.getResponseBuffer(0x03) << 16 | node.getResponseBuffer(0x02)) / 100.0f;
    if (debug == 1) {
      Serial.print("PV Power: ");
      Serial.println(pvpower);
    }
    battBhargeCurrent = (long)node.getResponseBuffer(0x05) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Charge Current: ");
      Serial.println(battBhargeCurrent);
      Serial.println();
    }
  } else {
    rs485DataReceived = false;
  }
}

void AddressRegistry_311A() {
  result = node.readInputRegisters(0x311A, 2);
  if (result == node.ku8MBSuccess)
  {
    bremaining = node.getResponseBuffer(0x00) / 1.0f;
    if (debug == 1) {
      Serial.print("Battery Remaining %: ");
      Serial.println(bremaining);

    }
    btemp = node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("Battery Temperature: ");
      Serial.println(btemp);
      Serial.println();
    }
  } else {
    rs485DataReceived = false;
  }
}

void AddressRegistry_3300() {
  result = node.readInputRegisters(0x3300, 2);
  if (result == node.ku8MBSuccess)
  {
    stats_today_pv_volt_max = node.getResponseBuffer(0x00) / 100.0f;
    if (debug == 1) {
      Serial.print("Stats Today PV Voltage MAX: ");
      Serial.println(stats_today_pv_volt_max);
    }
    stats_today_pv_volt_min = node.getResponseBuffer(0x01) / 100.0f;
    if (debug == 1) {
      Serial.print("Stats Today PV Voltage MIN: ");
      Serial.println(stats_today_pv_volt_min);
    }
  } else {
    rs485DataReceived = false;
  }
}