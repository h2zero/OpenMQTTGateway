#include <TheengsCommon.h>

#ifdef SecondaryModule

#  include <WiFi.h>

#  include "HTTPSerialUpdate.h"
#  include "config_SERIAL.h"

struct HTTPSerialUpdateParams {
  HTTPSerialUpdate& httpUpdate;
  NetworkClient& cl;
  uint32_t size;
  uint32_t remaining;
  String md5;
  int command;
  int errorCode;
};

TaskHandle_t serialOtaTaskHandle;

extern uint16_t calcCrc16(const uint8_t* buf, int len);
extern bool sendSerialOtaData(const char* data, size_t size);

void HTTPSerialUpdate::setUpdateSecondary(bool updateSecondary) {
  Log.notice(F("Setting updateSecondary to %d" CR), updateSecondary);
  m_updateSecondary = updateSecondary;
}

HTTPUpdateResult HTTPSerialUpdate::update(NetworkClient& client, const String& url) {
  HTTPClient http;
  if (!http.begin(client, url)) {
    return HTTP_UPDATE_FAILED;
  }
  return handleUpdate(http, "", false, NULL);
}

HTTPUpdateResult HTTPSerialUpdate::handleUpdate(HTTPClient& http, const String& currentVersion, bool spiffs, HTTPUpdateRequestCB requestCB) {
  if (!m_updateSecondary) {
    return HTTPUpdate::handleUpdate(http, currentVersion, spiffs, requestCB);
  }

  HTTPUpdateResult ret = HTTP_UPDATE_FAILED;

  // use HTTP/1.0 for update since the update handler not support any transfer Encoding
  http.useHTTP10(true);
  http.setTimeout(8000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setUserAgent("ESP32-http-Update");
  http.addHeader("Cache-Control", "no-cache");
  http.addHeader("x-ESP32-BASE-MAC", Network.macAddress());
#  if SOC_WIFI_SUPPORTED
  http.addHeader("x-ESP32-STA-MAC", WiFi.macAddress());
  http.addHeader("x-ESP32-AP-MAC", WiFi.softAPmacAddress());
#  endif
  http.addHeader("x-ESP32-free-space", String(ESP.getFreeSketchSpace()));
  http.addHeader("x-ESP32-sketch-size", String(ESP.getSketchSize()));
  String sketchMD5 = ESP.getSketchMD5();
  if (sketchMD5.length() != 0) {
    http.addHeader("x-ESP32-sketch-md5", sketchMD5);
  }
  http.addHeader("x-ESP32-chip-size", String(ESP.getFlashChipSize()));
  http.addHeader("x-ESP32-sdk-version", ESP.getSdkVersion());

  if (spiffs) {
    http.addHeader("x-ESP32-mode", "spiffs");
  } else {
    http.addHeader("x-ESP32-mode", "sketch");
  }

  if (currentVersion && currentVersion[0] != 0x00) {
    http.addHeader("x-ESP32-version", currentVersion);
  }
  if (requestCB) {
    requestCB(&http);
  }

  const char* headerkeys[] = {"x-MD5"};
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);

  // track these headers
  http.collectHeaders(headerkeys, headerkeyssize);

  int code = http.GET();
  int len = http.getSize();

  if (code <= 0) {
    Log.error(F("HTTP error: %s" CR), http.errorToString(code).c_str());
    _lastError = code;
    http.end();
    return HTTP_UPDATE_FAILED;
  }

  String md5 = "";
  if (http.hasHeader("x-MD5")) {
    md5 = http.header("x-MD5");
  }

  switch (code) {
    case HTTP_CODE_OK: ///< OK (Start Update)
      if (len > 0) {
        NetworkClient* tcp = http.getStreamPtr();
        delay(100);
        if (tcp->peek() != 0xE9) {
          Log.error(F("Magic header does not start with 0xE9" CR));
          _lastError = HTTP_UE_BIN_VERIFY_HEADER_FAILED;
          http.end();
          return HTTP_UPDATE_FAILED;
        }

        if (runUpdate(*tcp, len, md5, spiffs ? U_SPIFFS : U_FLASH)) {
          ret = HTTP_UPDATE_OK;
          http.end();
        } else {
          ret = HTTP_UPDATE_FAILED;
          Log.error(F("Update failed" CR));
        }
      } else {
        _lastError = HTTP_UE_SERVER_NOT_REPORT_SIZE;
        ret = HTTP_UPDATE_FAILED;
        Log.error(F("Content-Length was 0 or wasn't set by Server?!" CR));
      }
      break;
    case HTTP_CODE_NOT_MODIFIED:
      ///< Not Modified (No updates)
      ret = HTTP_UPDATE_NO_UPDATES;
      break;
    case HTTP_CODE_NOT_FOUND:
      _lastError = HTTP_UE_SERVER_FILE_NOT_FOUND;
      ret = HTTP_UPDATE_FAILED;
      break;
    case HTTP_CODE_FORBIDDEN:
      _lastError = HTTP_UE_SERVER_FORBIDDEN;
      ret = HTTP_UPDATE_FAILED;
      break;
    default:
      _lastError = HTTP_UE_SERVER_WRONG_HTTP_CODE;
      ret = HTTP_UPDATE_FAILED;
      Log.error(F("HTTP Code is (%d)" CR), code);
      break;
  }

  http.end();
  return ret;
}

static void SerialOTAUpdateTask(void* param) {
  Log.notice(F("Starting OTA update via serial" CR));
  char* buf = nullptr;
  HTTPSerialUpdateParams* params = static_cast<HTTPSerialUpdateParams*>(param);
  StaticJsonDocument<256> doc;
  JsonObject serialData = doc.to<JsonObject>();
  serialData["serial_ota"] = "begin";
  serialData["size"] = params->size;
  serialData["md5"] = params->md5;
  serialData["command"] = params->command;
  SYSConfig.serial = true;
  xTaskNotifyStateClear(NULL); // Clear any previous notifications before sending the first packet
  XtoSERIAL(subjectMQTTtoSERIAL, serialData);
  SYSConfig.serial = false;
  serialData.clear();
  auto rc = waitForSerialOtaAck();
  if (rc != SERIAL_OTA_OK) {
    Log.error(F("Serial OTA begin error: %d" CR), rc);
    goto done;
  }

  serialData["serial_ota"] = "data";
  buf = new char[SERIAL_OTA_BUFFER_SIZE];
  while (params->remaining) {
    size_t bytesToRead = SERIAL_OTA_BUFFER_SIZE;
    if (bytesToRead > params->remaining) {
      bytesToRead = params->remaining;
    }
    auto timeout_failures = 0;
    auto read = params->cl.readBytes(buf, bytesToRead);
    if (read == 0) {
      timeout_failures++;
      if (timeout_failures >= 300) {
        goto done;
      }
      delay(100);
    }

    serialData["crc"] = calcCrc16((const uint8_t*)buf, read);
    auto resend_count = 0;
    while (resend_count < 30) {
      serialData["size"] = read;
      SYSConfig.serial = true;
      XtoSERIAL(subjectMQTTtoSERIAL, serialData);
      SYSConfig.serial = false;
      rc = waitForSerialOtaAck();
      if (rc != SERIAL_OTA_OK) {
        Log.error(F("Serial OTA data cmd error: %d" CR), rc);
        goto done;
      }

      sendSerialOtaData(buf, read);
      rc = waitForSerialOtaAck();
      switch (rc) {
        case SERIAL_OTA_RESEND:
          resend_count++;
          continue;
        case SERIAL_OTA_ABORT:
          Log.error(F("Serial OTA fatal error!" CR));
          goto done;
      }

      break;
    }

    if (resend_count >= 30) {
      Log.error(F("Serial OTA data error retry limit reached!" CR));
      goto done;
    }

    params->remaining -= read;
  }

done:
  if (buf) {
    delete[] buf;
  }
  serialData.clear();
  serialData["serial_ota"] = params->remaining ? "abort" : "end";
  SYSConfig.serial = true;
  XtoSERIAL(subjectMQTTtoSERIAL, serialData);
  SYSConfig.serial = false;
  rc = waitForSerialOtaAck();
  if (rc != SERIAL_OTA_OK) {
    Log.error(F("Serial OTA end error: %d" CR), rc);
  }
  params->errorCode = params->remaining || rc != SERIAL_OTA_OK ? UPDATE_ERROR_STREAM : UPDATE_ERROR_OK;
  vTaskDelete(NULL);
}

bool HTTPSerialUpdate::runUpdate(NetworkClient& cl, uint32_t size, String md5, int command) {
  HTTPSerialUpdateParams params{
      *this, cl, size, size, md5, command, -1}; // -1 indicating waiting for end
  xTaskCreateUniversal(
      SerialOTAUpdateTask, /* Function to implement the task */
      "serialOtaTask", /* Name of the task */
      4096, /* Stack size in bytes */
      &params, /* Task input parameter */
      2, /* Priority of the task (set higher than core task) */
      &serialOtaTaskHandle, /* Task handle. */
      1); /* Core where the task should run */

  while (params.errorCode == -1) {
    delay(1);
    SERIALtoX();
  }

  _lastError = params.errorCode;
  return _lastError == UPDATE_ERROR_OK;
}
#endif