/*
  Theengs OpenMQTTGateway - We Unite Sensors in One Open-Source Interface

   Act as a wifi or ethernet gateway between your SERIAL device and a MQTT broker
   Send and receiving command by MQTT

  This gateway enables to:
 - receive MQTT data from a topic and send SERIAL signal corresponding to the received MQTT data
 - publish MQTT data to a different topic related to received SERIAL signal

    Copyright: (c)Florian ROBERT

    This file is part of OpenMQTTGateway.

    OpenMQTTGateway is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenMQTTGateway is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "User_config.h"

#ifdef ZgatewaySERIAL
#  ifdef SerialOTAModule
#    include <Update.h>
#  endif
#  include "TheengsCommon.h"
#  include "config_SERIAL.h"
#  include "libb64/cdecode.h"

#  ifndef SERIAL_UART // software serial mode
#    include <SoftwareSerial.h>
SoftwareSerial SERIALSoftSerial(SERIAL_RX_GPIO, SERIAL_TX_GPIO); // RX, TX
#  endif

#  ifdef ESP32
SemaphoreHandle_t serialSemaphore = NULL;
extern TaskHandle_t serialOtaTaskHandle;
extern bool BTProcessLock;
const TickType_t semaphoreTimeout = pdMS_TO_TICKS(1000); // 1 second timeout
#    undef SEMAPHORE_SERIAL
#    undef SEMAPHORE_SERIAL_GIVE
#    define SEMAPHORE_SERIAL      xSemaphoreTake(serialSemaphore, semaphoreTimeout) == pdTRUE
#    define SEMAPHORE_SERIAL_GIVE xSemaphoreGive(serialSemaphore)
#  else
#    undef SEMAPHORE_SERIAL
#    undef SEMAPHORE_SERIAL_GIVE
#    define SEMAPHORE_SERIAL true
#    define SEMAPHORE_SERIAL_GIVE
#  endif

extern void receivingDATA(const char* topicOri, const char* datacallback);

// use pointer to stream class for serial communication to make code
// compatible with both softwareSerial as hardwareSerial.
Stream* SERIALStream = NULL;
//unsigned long msgCount = 0;

bool receiverReady = false;
unsigned long lastHeartbeatReceived = 0;
unsigned long lastHeartbeatAckReceived = 0;
unsigned long lastHeartbeatSent = 0;
unsigned long lastHeartbeatAckReceivedCheck = 0;
const unsigned long heartbeatTimeout = 15000; // 15 seconds timeout for ack
const unsigned long heartbeatAckCheckInterval = 5000; // Check for ack every 5 seconds
const unsigned long maxHeartbeatInterval = 60000; // Maximum interval of 1 minute
unsigned long heartbeatInterval = 5000; // 5 seconds
bool isOverflow = false;

static void sendHeartbeat();
static void sendHeartbeatAck();
static void handleHeartbeat();

#  if defined(SerialOTAModule) || defined(SecondaryModule)
// TODO: Move to TheengsUtils?
uint16_t calcCrc16(const uint8_t* buf, int len) {
  uint16_t crc = 0;
  int32_t i;

  while (len--) {
    crc ^= *buf++ << 8;

    for (i = 0; i < 8; i++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}
#  endif

void setupSERIAL() {
//Initalize serial port
#  ifdef SERIAL_UART // Hardware serial
#    if SERIAL_UART == 0 // init UART0
  Serial.end(); // stop if already initialized
#      ifdef ESP32
  Serial.begin(SERIALBaud, SERIAL_8N1, SERIAL_RX_GPIO, SERIAL_TX_GPIO);
#      else
  Serial.begin(SERIALBaud, SERIAL_8N1);
#      endif
#      if defined(ESP8266) && defined(SERIAL_UART0_SWAP)
  Serial.swap(); // swap UART0 ports from (GPIO1,GPIO3) to (GPIO15,GPIO13)
#      endif
  SERIALStream = &Serial;
  Log.notice(F("SERIAL HW UART0" CR));

#    elif SERIAL_UART == 1 // init UART1
  Serial1.end(); // stop if already initialized
#      ifdef ESP32
  Serial1.begin(SERIALBaud, SERIAL_8N1, SERIAL_RX_GPIO, SERIAL_TX_GPIO);
#      else
  Serial1.begin(SERIALBaud, SERIAL_8N1);
#      endif
  SERIALStream = &Serial1;
  Log.notice(F("SERIAL HW UART1" CR));

#    elif SERIAL_UART == 2 // init UART2
  Serial2.end(); // stop if already initialized
#      ifdef ESP32
  Serial2.begin(SERIALBaud, SERIAL_8N1, SERIAL_RX_GPIO, SERIAL_TX_GPIO);
#      else
  Serial2.begin(SERIALBaud, SERIAL_8N1);
#      endif
  SERIALStream = &Serial2;
  Log.notice(F("SERIAL HW UART2" CR));

#    elif SERIAL_UART == 3 // init UART3
  Serial3.end(); // stop if already initialized
  Serial3.begin(SERIALBaud, SERIAL_8N1);
  SERIALStream = &Serial3;
  Log.notice(F("SERIAL HW UART3" CR));
#    endif

#  else // Software serial
  // define pin modes for RX, TX:
  pinMode(SERIAL_RX_GPIO, INPUT);
  pinMode(SERIAL_TX_GPIO, OUTPUT);
  SERIALSoftSerial.begin(SERIALBaud);
  SERIALStream = &SERIALSoftSerial; // get stream of serial

  Log.notice(F("SERIAL_RX_GPIO: %d" CR), SERIAL_RX_GPIO);
  Log.notice(F("SERIAL_TX_GPIO: %d" CR), SERIAL_TX_GPIO);
#  endif

#  ifdef ESP32
  serialSemaphore = xSemaphoreCreateMutex();
  if (serialSemaphore == NULL) {
    Log.error(F("Failed to create serialSemaphore" CR));
  }
#  endif

  // Flush all bytes in the "link" serial port buffer
  while (SERIALStream->available() > 0)
    SERIALStream->read();

  Log.notice(F("SERIALBaud: %d" CR), SERIALBaud);
  Log.trace(F("gatewaySERIAL setup done" CR));
}

#  if SERIALtoMQTTmode == 0 // Convert received data to single MQTT topic
void SERIALtoX() {
  // Send all SERIAL output (up to SERIALInPost char revieved) as MQTT message
  //This function is Blocking, but there should only ever be a few bytes, usually an ACK or a NACK.
  if (SERIALStream->available()) {
    Log.trace(F("SERIALtoMQTT" CR));
    static char SERIALdata[MAX_INPUT];
    static unsigned int input_pos = 0;
    static char inChar;
    do {
      if (SERIALStream->available()) {
        inChar = SERIALStream->read();
        SERIALdata[input_pos] = inChar;
        input_pos++;
      }
    } while (inChar != SERIALInPost && input_pos < MAX_INPUT);
    SERIALdata[input_pos] = 0;
    input_pos = 0;

    char* output = SERIALdata + sizeof(SERIALPre) - 1;
    Log.notice(F("SERIAL data: %s" CR), output);
    pub(subjectSERIALtoMQTT, output);
  }
}

#  elif SERIALtoMQTTmode == 1 // Convert received JSON data to one or multiple MQTT topics
void sendHeartbeat() {
  if (SEMAPHORE_SERIAL) {
    SERIALStream->print(SERIALPre);
    SERIALStream->print("{\"type\":\"heartbeat\"}");
    SERIALStream->print(SERIALPost);
    SERIALStream->flush();
    Log.notice(F("Sent Serial heartbeat" CR));
    SEMAPHORE_SERIAL_GIVE;
  } else {
    Log.error(F("Failed to take serialSemaphore" CR));
  }
}

void sendHeartbeatAck() {
  if (SEMAPHORE_SERIAL) {
    SERIALStream->print(SERIALPre);
    SERIALStream->print("{\"type\":\"heartbeat_ack\"}");
    SERIALStream->print(SERIALPost);
    SERIALStream->flush();
    Log.notice(F("Sent heartbeat ack" CR));
    SEMAPHORE_SERIAL_GIVE;
  } else {
    Log.error(F("Failed to take serialSemaphore" CR));
  }
}

#    ifdef SerialOTAModule
void sendSerialOtaAck(const char* msg) {
  if (SEMAPHORE_SERIAL) {
    SERIALStream->print(SERIALPre);
    if (msg) { // If msg is not null there was an error, send a NACK with the message
      SERIALStream->print("{\"serial_ota\":\"nack\",\"msg\":\"");
      SERIALStream->print(msg);
      SERIALStream->print("\"}");
    } else {
      SERIALStream->print("{\"serial_ota\":\"ack\"}");
    }
    SERIALStream->print(SERIALPost);
    SERIALStream->flush();
    Log.notice(F("Sent serial ota ack" CR));
    SEMAPHORE_SERIAL_GIVE;
  } else {
    Log.error(F("Failed to take serialSemaphore" CR));
  }
}
#    endif

#    ifdef SecondaryModule
uint32_t waitForSerialOtaAck() {
  uint32_t rc;
  if (xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &rc, pdMS_TO_TICKS(3000)) != pdTRUE) {
    Log.error(F("Serial OTA timeout" CR));
    rc = SERIAL_OTA_ABORT;
  }
  return rc;
}

bool sendSerialOtaData(const char* data, size_t size) {
  if (SEMAPHORE_SERIAL) {
    SERIALStream->write(data, size);
    SERIALStream->flush();
    Log.notice(F("Sent serial ota %d bytes" CR), size);
    SEMAPHORE_SERIAL_GIVE;
    return true;
  } else {
    Log.error(F("Failed to take serialSemaphore" CR));
  }
  return false;
}
#    endif

void SERIALtoX() {
#    ifdef SerialOTAModule
  static size_t ota_packet_size = 0; // Size of the OTA packet
  static size_t ota_packet_index = 0; // Offset for the OTA packet
  static uint16_t ota_packet_crc = 0; // CRC of the OTA packet
  static unsigned long ota_packet_start_time = 0; // Start time of the OTA packet
  static uint8_t* ota_packet_buffer = nullptr; // Buffer for OTA data
#    endif
  static String buffer = ""; // Static buffer to store incomplete messages
  unsigned long currentTime = millis();

#    ifdef SENDER_SERIAL_HEARTBEAT
  // Check if it's time to send a heartbeat and we're not in overflow
  if (!Update.isRunning()) {
    if (!isOverflow && currentTime - lastHeartbeatSent > heartbeatInterval) {
      sendHeartbeat();
      lastHeartbeatSent = currentTime;
    }
    if (currentTime - lastHeartbeatAckReceivedCheck > heartbeatAckCheckInterval) {
      lastHeartbeatAckReceivedCheck = currentTime;
      // Check if we received an ack for the last heartbeat
      if (currentTime - lastHeartbeatAckReceived > heartbeatTimeout) {
        // No ack received, increase the interval (with a maximum limit)
        unsigned long newHeartbeatInterval = heartbeatInterval * 1.25;
        heartbeatInterval = min(newHeartbeatInterval, maxHeartbeatInterval);
        Log.warning(F("No heartbeat ack received. Increasing interval to %lu ms" CR), heartbeatInterval);
        receiverReady = false;
      } else {
        // Ack received, reset the interval
        heartbeatInterval = 5000;
      }
    }
  }
#    else
  receiverReady = true;
#    endif

#    ifdef SerialOTAModule
  if (ota_packet_size > 0) {
    // Check if the OTA packet timeout is exceeded
    if (currentTime - ota_packet_start_time > 1000) {
      Log.error(F("OTA packet timeout, %d bytes missing" CR), ota_packet_size);
      ota_packet_size = 0;
      sendSerialOtaAck("OTA packet timeout - resend");
      return;
    }
  }
#    endif
  while (SERIALStream->available()) {
#    ifdef SerialOTAModule
    if (ota_packet_index < ota_packet_size) {
      // If we are in OTA mode, read the data into the buffer
      ota_packet_buffer[ota_packet_index++] = SERIALStream->read();
      if (ota_packet_index == ota_packet_size) {
        // Check CRC
        uint16_t crc = calcCrc16(ota_packet_buffer, ota_packet_size);
        if (crc != ota_packet_crc) {
          Log.error(F("OTA data CRC mismatch: expected %04X, got %04X" CR), ota_packet_crc, crc);
          sendSerialOtaAck("CRC mismatch - resend");
          ota_packet_size = 0;
        } else if (Update.write(ota_packet_buffer, ota_packet_size) != ota_packet_size) {
          Log.error(F("Failed to write OTA data: %d" CR), Update.getError());
          sendSerialOtaAck("Failed to write OTA data");
        } else {
          Log.notice(F("OTA data written successfully" CR));
          sendSerialOtaAck(NULL);
        }
      }
      continue;
    }
#    endif

    unsigned long now = millis();
    char c = SERIALStream->read();
    buffer += c;

    // Check if we have a complete message
    if (buffer.startsWith(SERIALPre) && buffer.endsWith(SERIALPost)) {
      isOverflow = false;
      // Remove prefix and postfix
      String jsonString = buffer.substring(strlen(SERIALPre), buffer.length() - strlen(SERIALPost));

      // Allocate the JSON document
      StaticJsonDocument<JSON_MSG_BUFFER> SERIALBuffer;
      JsonObject SERIALdata = SERIALBuffer.to<JsonObject>();

      // Deserialize the JSON string
      DeserializationError err = deserializeJson(SERIALBuffer, jsonString);

      if (err == DeserializationError::Ok) {
        // Check if this is a heartbeat message
        if (SERIALdata.containsKey("type") && strcmp(SERIALdata["type"], "heartbeat") == 0) {
          handleHeartbeat();
        } else if (SERIALdata.containsKey("type") && strcmp(SERIALdata["type"], "heartbeat_ack") == 0) {
          lastHeartbeatAckReceived = now;
          receiverReady = true;
          Log.notice(F("Heartbeat ack received" CR));
#    ifdef SerialOTAModule
        } else if (SERIALdata.containsKey("serial_ota") && strcmp(SERIALdata["serial_ota"], "begin") == 0) {
          // Start OTA update via serial
          Log.notice(F("Starting OTA update via serial" CR));
          if (SERIALdata.containsKey("size") && SERIALdata.containsKey("md5")) {
            uint32_t size = SERIALdata["size"];
            String md5 = SERIALdata["md5"];
            int command = SERIALdata["command"] | U_FLASH; // Default to U_FLASH if not provided
            if (!Update.begin(size, command)) {
              Log.error(F("Update.begin failed! %d" CR), Update.getError());
              sendSerialOtaAck("Update.begin failed!");
              buffer = "";
              return;
            }

            if (md5.length() && !Update.setMD5(md5.c_str())) {
              Log.error(F("Update.setMD5 failed! %s" CR), md5.c_str());
              sendSerialOtaAck("Update.setMD5 failed!");
              buffer = "";
              return;
            }

            BTProcessLock = true;

            if (ota_packet_buffer == nullptr) {
              ota_packet_buffer = new uint8_t[SERIAL_OTA_BUFFER_SIZE];
            }
            if (ota_packet_buffer == nullptr) {
              Log.error(F("Failed to allocate OTA buffer" CR));
              sendSerialOtaAck("Failed to allocate OTA buffer!");
              buffer = "";
              return;
            }
            sendSerialOtaAck(NULL); // Send ack for OTA begin
          } else {
            Log.error(F("Invalid OTA update parameters" CR));
          }
        } else if (SERIALdata.containsKey("serial_ota") && strcmp(SERIALdata["serial_ota"], "data") == 0) {
          // Handle OTA data
          if (SERIALdata.containsKey("size") && SERIALdata.containsKey("crc")) {
            ota_packet_size = SERIALdata["size"];
            ota_packet_crc = SERIALdata["crc"];
            ota_packet_index = 0;
            ota_packet_start_time = now;
            if (ota_packet_size > SERIAL_OTA_BUFFER_SIZE) {
              ota_packet_size = 0;
              Log.error(F("Data size too large: %d" CR), ota_packet_size);
              sendSerialOtaAck("Data size too large!");
            } else {
              Log.notice(F("Waiting for OTA data of size: %d" CR), ota_packet_size);
              sendSerialOtaAck(NULL); // Send ack for OTA data
            }
          } else {
            Log.error(F("Missing data size in OTA data" CR));
            sendSerialOtaAck("Missing data size!");
          }
        } else if (SERIALdata.containsKey("serial_ota") && strcmp(SERIALdata["serial_ota"], "end") == 0) {
          ota_packet_size = 0;
          if (Update.end()) {
            Log.notice(F("OTA update ended successfully" CR));
            sendSerialOtaAck(NULL);
          } else {
            Log.error(F("Failed to end OTA update" CR));
            sendSerialOtaAck("Update.end failed!");
          }
        } else if (SERIALdata.containsKey("serial_ota") && strcmp(SERIALdata["serial_ota"], "abort") == 0) {
          ota_packet_size = 0;
          Update.abort();
          Log.notice(F("OTA update aborted" CR));
          sendSerialOtaAck(NULL); // Send ack for OTA abort
        } else if (SERIALdata.containsKey("serial_ota") && strcmp(SERIALdata["serial_ota"], "begin") == 0) {
          // Start OTA update via serial
          Log.notice(F("Starting OTA update via serial" CR));
          sendSerialOtaAck(NULL); // Send ack for OTA begin
#    elif defined(SecondaryModule)
        } else if (SERIALdata.containsKey("serial_ota") && (strcmp(SERIALdata["serial_ota"], "ack") == 0 || strcmp(SERIALdata["serial_ota"], "nack") == 0)) {
          // Handle OTA ack/nack
          if (strcmp(SERIALdata["serial_ota"], "ack") == 0) {
            Log.notice(F("Received OTA ack" CR));
            xTaskNotify(serialOtaTaskHandle, SERIAL_OTA_OK, eSetValueWithOverwrite);
          } else {
            Log.error(F("Received OTA nack: %s" CR), SERIALdata["msg"].as<const char*>());
            if (strstr(SERIALdata["msg"].as<const char*>(), "resend")) {
              Log.notice(F("Resend OTA request" CR));
              xTaskNotify(serialOtaTaskHandle, SERIAL_OTA_RESEND, eSetValueWithOverwrite);
            } else {
              Log.error(F("Abort OTA request" CR));
              xTaskNotify(serialOtaTaskHandle, SERIAL_OTA_ABORT, eSetValueWithOverwrite);
            }
          }
#    endif
        } else {
          // Process normal messages
          Log.notice(F("SERIAL msg received: %s" CR), jsonString.c_str());
#    if jsonPublishing
          if (SERIALdata.containsKey("target")) {
            receivingDATA("", jsonString.c_str());
          } else {
            // send as json
            if (SERIALdata.containsKey("origin") || SERIALdata.containsKey("topic")) {
#      ifdef SecondaryModule
              // We need to assign the discovery message to the primary module instead of the secondary module
              if (SERIALdata.containsKey("device") && SERIALdata["device"].containsKey("via_device")) {
                SERIALdata["device"]["via_device"] = gateway_name;
              }
#      endif
              enqueueJsonObject(SERIALdata);
            } else {
              SERIALdata["origin"] = subjectSERIALtoMQTT;
              enqueueJsonObject(SERIALdata);
            }
          }
#    endif
#    if simplePublishing
          // send as MQTT topics
          char topic[mqtt_topic_max_size + 1] = subjectSERIALtoMQTT;
          sendMQTTfromNestedJson(SERIALBuffer.as<JsonVariant>(), topic, 0, SERIALmaxJSONlevel);
#    endif
        }
      } else {
        // Print error to serial log
        Log.error(F("Error in SERIALJSONtoMQTT, deserializeJson() returned %s" CR), err.c_str());
      }

      // Clear the buffer for the next message
      buffer = "";
    } else if (buffer.endsWith(SERIALPost)) {
      // If the buffer ends with the postfix but does not start with the prefix, clear it
      Log.error(F("Buffer error, clearing buffer. Partial content: %s" CR), buffer.c_str());
      buffer = "";
    } else if (buffer.length() > JSON_MSG_BUFFER) {
      // If the buffer gets too large without finding a complete message, clear it
      Log.error(F("Buffer overflow, clearing buffer. Partial content: %s" CR), buffer.c_str());
      buffer = "";
      isOverflow = true;
    }
  }
}

void sendMQTTfromNestedJson(JsonVariant obj, char* topic, int level, int maxLevel) {
  // recursively step through JSON data and send MQTT messages
  if (level < maxLevel && obj.is<JsonObject>()) {
    int topicLength = strlen(topic);
    // loop over fields
    for (JsonPair pair : obj.as<JsonObject>()) {
      // check if new key still fits in topic cstring
      const char* key = pair.key().c_str();
      Log.trace(F("level=%d, key='%s'" CR), level, pair.key().c_str());
      if (topicLength + 2 + strlen(key) <= mqtt_topic_max_size) {
        // add new level to existing topic cstring
        topic[topicLength] = '/'; // add slash
        topic[topicLength + 1] = '\0'; // terminate
        strncat(topic + topicLength, key, mqtt_topic_max_size - topicLength - 2);

        // step recursively into next level
        sendMQTTfromNestedJson(pair.value(), topic, level + 1, maxLevel);

        // restore topic
        topic[topicLength] = '\0';
      } else {
        Log.error(F("Nested key '%s' at level %d does not fit within max topic length of %d, skipping"),
                  key, level, mqtt_topic_max_size);
      }
    }

  } else {
    // output value at current json level
    char output[MAX_INPUT + 1];
    serializeJson(obj, output, MAX_INPUT);
    Log.notice(F("level=%d, topic=%s, value: %s\n"), level, topic, output);

    // send MQTT message
    pub(topic, &output[0]);
  }
}
#  endif

bool XtoSERIAL(const char* topicOri, JsonObject& SERIALdata) {
  bool res = false;
  if (SEMAPHORE_SERIAL) {
    if (receiverReady && (cmpToMainTopic(topicOri, subjectMQTTtoSERIAL) ||
                          (SYSConfig.serial && SERIALdata.containsKey("origin") && SERIALdata["origin"].is<const char*>()) ||
                          (SYSConfig.serial && SERIALdata.containsKey("topic") && SERIALdata["topic"].is<const char*>()))) {
      Log.trace(F("XtoSERIAL" CR));
      // Prepare the data string
      std::string data;
      if (SYSConfig.serial ||
          (SERIALdata.containsKey("origin") && SERIALdata["origin"].is<const char*>()) || // Module like BT to SERIAL
          (SERIALdata.containsKey("target") && SERIALdata["target"].is<const char*>())) { // Command to send to a specific target example MQTTtoBT through SERIAL
        //SERIALdata["msgcount"] = msgCount++;
        serializeJson(SERIALdata, data);
      } else if (SERIALdata.containsKey("value")) {
        data = SERIALdata["value"].as<std::string>();
      }

      // Send the message
      const char* prefix = SERIALdata["prefix"] | SERIALPre;
      const char* postfix = SERIALdata["postfix"] | SERIALPost;
      SERIALStream->print(prefix);
      SERIALStream->print(data.c_str());
      SERIALStream->print(postfix);
      SERIALStream->flush();

      Log.notice(F("[ OMG->SERIAL ] data sent: %s" CR), data.c_str());
      res = true;
      delay(100);
    }
    SEMAPHORE_SERIAL_GIVE;
  } else {
    Log.error(F("Failed to take serialSemaphore" CR));
  }
  return res;
}

bool isSerialReady() {
  return receiverReady;
}

// This function should be called when a heartbeat is received from the emitter
void handleHeartbeat() {
  lastHeartbeatReceived = millis();
  if (gatewayState == GatewayState::BROKER_CONNECTED) {
    sendHeartbeatAck();
  }
}
#endif