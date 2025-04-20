#pragma once
#define NO_GLOBAL_HTTPUPDATE

#include <HTTPUpdate.h>
#include <TheengsCommon.h>

#include "config_SERIAL.h"

class HTTPSerialUpdate : public HTTPUpdate {
  bool m_updateSecondary = false;

public:
  HTTPSerialUpdate() : HTTPUpdate() {}
  HTTPSerialUpdate(int httpClientTimeout) : HTTPUpdate(httpClientTimeout) {}
  ~HTTPSerialUpdate() {}

#ifdef SecondaryModule
  void setUpdateSecondary(bool updateSecondary);
  HTTPUpdateResult update(NetworkClient& client, const String& url);
  HTTPUpdateResult handleUpdate(HTTPClient& http, const String& currentVersion, bool spiffs, HTTPUpdateRequestCB requestCB);
  bool runUpdate(NetworkClient& cl, uint32_t size, String md5, int command = U_FLASH);
#endif
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_HTTPSERIALUPDATE)
extern HTTPSerialUpdate httpUpdate;
#endif
