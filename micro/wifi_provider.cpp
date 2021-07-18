/*
 * Implementation of Wifi Provider
 */

#include "wifi_provider.h"

// Configure the pins used for the ESP32 connection
#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    9   // Chip select pin
#define ESP32_RESETN  11   // Reset pin
#define SPIWIFI_ACK   10   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1

void WifiProvider::Load() {
  Serial.println("WifiProvider::Load() =>");

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  int attempts = 0;
  while (attempts <= 10 && WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    attempts++;
    delay(1000);
  }

  if (WiFi.status() == WL_NO_MODULE) {
    return;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  } else {
    loaded = true;
    Serial.print("Found firmware ");
    Serial.println(fv);
  }
}

void WifiProvider::ConnectToWifi() {
  Serial.println("WifiProvider::ConnectToWifi() =>");

  if (!loaded) {
    Serial.println("ConnectToWifi fail: Module not loaded!");
    return;
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  do {
    status = WiFi.begin(ssid, pass);
    delay(100);     // wait until connection is ready!
  } while (status != WL_CONNECTED);

  Serial.println("Connected to wifi");
  PrintWifiStatus();

  char server[] = "wifitest.adafruit.com";
  char path[]   = "/testwifi/index.html";

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.print("GET "); client.print(path); client.println(" HTTP/1.1");
    client.print("Host: "); client.println(server);
    client.println("Connection: close");
    client.println();

    Serial.println("[Response:]");
    while (client.connected() || client.available())
    {
      if (client.available())
      {
        String line = client.readStringUntil('\n');
        Serial.println(line);
      }
    }
    client.stop();
    Serial.println("\n[Disconnected]");
  }
}

void WifiProvider::PrintWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void WifiProvider::DisconnectFromWifi() {
  Serial.println("WifiProvider::DisconnectFromWifi() =>");
}

void WifiProvider::ConnectToServer() {
  Serial.println("WifiProvider::ConnectToServer() =>");
}

void WifiProvider::DisconnectFromServer() {
  Serial.println("WifiProvider::DisconnectFromServer() =>");
}

void WifiProvider::SendData(char *data, int size) {
  Serial.println("WifiProvider::SendData() =>");
}


