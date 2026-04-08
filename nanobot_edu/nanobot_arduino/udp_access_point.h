#ifndef UDP_ACCESS_POINT_H
#define UDP_ACCESS_POINT_H

#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "credentials.h"

class udp_access_point {
  public:
    udp_access_point(int port, IPAddress ip);
    int checkForPacket();
    char * getPacket();
    void sendPacket(char * msg);
    bool isReady();
    WiFiUDP * udpConnection;

  private:
    int status;
    int localPort;
    IPAddress ip_add;
    String fv;
    char packetBuffer[256];
    char ReplyBuffer[256];
    bool setupStatus;
};

#endif
