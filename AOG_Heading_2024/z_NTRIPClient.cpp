#include"z_NTRIPClient.h"
bool NTRIPClient_WiFi::reqSrcTbl(char* host, int &port)
{
  delay(10);
  return true;
}
bool NTRIPClient_WiFi::reqRaw(char* host, int &port, char* mntpnt, char* user, char* psw)
{
  if (!connect(host, port))return false;
  //String _accept = "*/*";
  String _userAgent = "NTRIP ESP32NTRIPClient";
  //  String _userAgent = "NTRIPClient for Arduino v1.0";
  String p = "GET /";
  String auth = "";
  Serial.println("Requesting NTRIP by WiFi");

  p = p + mntpnt + String(" HTTP/1.0\r\n"
                          "User-Agent: " + _userAgent + "\r\n");


  if (strlen(user) == 0) {
    p = p + String(
          "Accept: */*\r\n"
          "Connection: close\r\n"
        );
  }
  else {
    auth = base64::encode(String(user) + String(":") + psw);
#ifdef Debug
    Serial.println(String(user) + String(":") + psw);
#endif
    p = p + String("Authorization: Basic ");
    p = p + auth;
    p = p + String("\r\n");
  }
  p = p + String("\r\n");
  print(p);
#ifdef Debug
  Serial.println(p);
#endif
  unsigned long timeout = millis();
  while (available() == 0) {
    if (millis() - timeout > 20000) {
      //Serial.println("Client Timeout !");
      return false;
    }
    delay(10);
  }
  char buffer[50];
  readLine(buffer, sizeof(buffer));
  if (strncmp((char*)buffer, "ICY 200 OK", 10))
  {
    Serial.print((char*)buffer);
    return false;
  }
  return true;
}
bool NTRIPClient_WiFi::reqRaw(char* host, int &port, char* mntpnt)
{
  return reqRaw(host, port, mntpnt, "", "");
}
int NTRIPClient_WiFi::readLine(char* _buffer, int size)
{
  int len = 0;
  while (available()) {
    _buffer[len] = read();
    len++;
    if (_buffer[len - 1] == '\n' || len >= size) break;
  }
  _buffer[len] = '\0';

  return len;
}

//======================================================================================

bool NTRIPClient_Eth::reqSrcTbl(char* host, int &port)
{
  delay(10);
  return true;
}
bool NTRIPClient_Eth::reqRaw(char* host, int &port, char* mntpnt, char* user, char* psw)
{
  if (!connect(host, port))return false;
  //String _accept = "*/*";
  String _userAgent = "NTRIP ESP32NTRIPClient";
  //  String _userAgent = "NTRIPClient for Arduino v1.0";
  String p = "GET /";
  String auth = "";
  Serial.println("Requesting NTRIP by Ethernet");

  p = p + mntpnt + String(" HTTP/1.0\r\n"
                          "User-Agent: " + _userAgent + "\r\n");


  if (strlen(user) == 0) {
    p = p + String(
          "Accept: */*\r\n"
          "Connection: close\r\n"
        );
  }
  else {
    auth = base64::encode(String(user) + String(":") + psw);
#ifdef Debug
    Serial.println(String(user) + String(":") + psw);
#endif
    p = p + String("Authorization: Basic ");
    p = p + auth;
    p = p + String("\r\n");
  }
  p = p + String("\r\n");
  print(p);
#ifdef Debug
  Serial.println(p);
#endif
  unsigned long timeout = millis();
  while (available() == 0) {
    if (millis() - timeout > 20000) {
      //Serial.println("Client Timeout !");
      return false;
    }
    delay(100);
  }
  char buffer[50];
  readLine(buffer, sizeof(buffer));
  if (strncmp((char*)buffer, "ICY 200 OK", 10))
  {
    Serial.print((char*)buffer);
    return false;
  }
  return true;
}
bool NTRIPClient_Eth::reqRaw(char* host, int &port, char* mntpnt)
{
  return reqRaw(host, port, mntpnt, "", "");
}
int NTRIPClient_Eth::readLine(char* _buffer, int size)
{
  int len = 0;
  while (available()) {
    _buffer[len] = read();
    len++;
    if (_buffer[len - 1] == '\n' || len >= size) break;
  }
  _buffer[len] = '\0';

  return len;
}
