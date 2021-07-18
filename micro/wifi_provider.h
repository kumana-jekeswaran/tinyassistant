/*
 * Wifi Provider provides connectivity to the Server
 */

#ifndef MICRO_WIFI_PROVIDER_H
#define MICRO_WIFI_PROVIDER_H

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

class WifiProvider {
public:
  WifiProvider(): ssid(SECRET_SSID), pass(SECRET_PASS), status(WL_IDLE_STATUS),
    loaded(false), wifiConnected(false), serverConnected(false) {
    Serial.println("WifiProvider::WifiProvider() =>");
  }
  ~WifiProvider() {
    Serial.println("WifiProvider::~WifiProvider() =>");
  }

  void Load();
  void ConnectToWifi();
  void DisconnectFromWifi();
  void ConnectToServer();
  void DisconnectFromServer();
  void SendData(char* data, int size);

private:
  const char *ssid;
  const char *pass;
  int status;
  bool loaded;
  bool wifiConnected;
  bool serverConnected;
  WiFiClient client;
  void PrintWifiStatus();
};

#endif //MICRO_WIFI_PROVIDER_H
