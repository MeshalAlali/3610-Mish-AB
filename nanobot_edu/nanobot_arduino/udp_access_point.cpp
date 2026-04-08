#include "udp_access_point.h"

udp_access_point::udp_access_point(int port, IPAddress ip) {
  udpConnection = new WiFiUDP;

  status = WL_IDLE_STATUS;
  localPort = port;
  IPAddress ip_add = ip;

  if (WiFi.status() == WL_NO_MODULE)
    setupStatus = false;

  fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    setupStatus = false;

  // Sets the static IP
  WiFi.config(ip_add);
 

  status = WiFi.beginAP(SECRET_SSID, SECRET_PASS);
  if (status != WL_AP_LISTENING)
    setupStatus = false;

  udpConnection->begin(localPort);
  WiFi.noLowPowerMode();
  setupStatus = true;
}

int udp_access_point::checkForPacket() {
  int packetSize = udpConnection->parsePacket();
  if (packetSize) {
    int len = udpConnection->read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    return 1;
  }
  else return 0;
}

char * udp_access_point::getPacket() {
  return packetBuffer;
}


void udp_access_point::sendPacket(char * msg) {
  // Send a reply to the device that made contact
  udpConnection->beginPacket(udpConnection->remoteIP(), udpConnection->remotePort());
  udpConnection->write(msg);
  udpConnection->endPacket();
}

bool udp_access_point::isReady() {
  return setupStatus;
}
