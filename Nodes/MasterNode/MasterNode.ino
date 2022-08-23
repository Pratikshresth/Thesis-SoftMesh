#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <arduino-timer.h>
#include <string>
#include <masternode.h>
#include <CdpPacket.h>
#include <queue>
#ifdef CA_CERT
const char* example_root_ca = \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
  "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
  "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
  "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
  "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
  "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
  "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
  "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
  "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
  "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
  "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
  "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
  "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
  "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
  "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
  "-----END CERTIFICATE-----\n";

#endif
#define SSID "pratikshrestha1"
#define PASSWORD "437E5FCF94"
#define ORG         "cgvhbx"
#define DEVICE_ID   "SOFTMESH-MASTER"
#define DEVICE_TYPE "Master"
#define TOKEN       "dwad8454@fes"


char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;
#define CMD_STATE_WIFI "/wifi/" 
#define CMD_STATE_HEALTH "/health/"
#define CMD_STATE_CHANNEL "/channel/"
#define CMD_STATE_MBM "/messageBoard/"
const char commandTopic[] = "iot-2/cmd/+/fmt/+";
void gotMsg(char* topic, byte* payload, unsigned int payloadLength);
masternode node;
nodeDisplay* display = NULL;
bool use_auth_method = true;
auto timer = timer_create_default();
int QUEUE_SIZE_MAX = 5;
std::queue<std::vector<byte>> packetQueue;
WiFiClientSecure wifiClient;
PubSubClient client(server, 8883, gotMsg, wifiClient);
std::string toTopicString(byte topic) {
  std::string topicString;
  switch (topic) {
    case topics::status:
      topicString = "status";
      break;
    case topics::cpm:
      topicString = "portal";
      break;
    case topics::sensor:
      topicString = "sensor";
      break;
    case topics::alert:
      topicString = "alert";
      break;
    case topics::location:
      topicString = "gps";
      break;
    case topics::health:
      topicString = "health";
      break;
    case topics::bmp180:
      topicString = "bmp180";
      break;
    case topics::pir:
      topicString = "pir";
      break;
    case topics::dht11:
      topicString = "dht";
      break;
    case topics::bmp280:
      topicString = "bmp280";
      break;
    case topics::mq7:
      topicString = "mq7";
      break;
    case topics::gp2y:
      topicString = "gp2y";
      break;
    case reservedTopic::ack:
      topicString = "ack";
      break;
    default:
      topicString = "status";
  }
  return topicString;
}
String convertToHex(byte* data, int size) {
  String buf = "";
  buf.reserve(size * 2); // 2 digit hex
  const char* cs = "0123456789ABCDEF";
  for (int i = 0; i < size; i++) {
    byte val = data[i];
    buf += cs[(val >> 4) & 0x0F];
    buf += cs[val & 0x0F];
  }
  return buf;
}
bool retry = true;
int quackJson(CdpPacket packet) {
  const int bufferSize = 4 * JSON_OBJECT_SIZE(4);
  StaticJsonDocument<bufferSize> doc;
  std::string payload(packet.data.begin(), packet.data.end());
  std::string sduid(packet.sduid.begin(), packet.sduid.end());
  std::string dduid(packet.dduid.begin(), packet.dduid.end());
  std::string muid(packet.muid.begin(), packet.muid.end());
  std::string path(packet.path.begin(), packet.path.end());
  Serial.println("[master] Packet Received:");
  Serial.println("[master] sduid:   " + String(sduid.c_str()));
  Serial.println("[master] dduid:   " + String(dduid.c_str()));
  Serial.println("[master] muid:    " + String(muid.c_str()));
  Serial.println("[master] path:    " + String(path.c_str()));
  Serial.println("[master] data:    " + String(payload.c_str()));
  Serial.println("[master] hops:    " + String(packet.hopCount));
  Serial.println("[master] node:    " + String(packet.nodeType));
  doc["DeviceID"] = sduid;
  doc["MessageID"] = muid;
  doc["Payload"].set(payload);
  doc["path"].set(path);
  doc["hops"].set(packet.hopCount);
  doc["nodeType"].set(packet.nodeType);
  std::string cdpTopic = toTopicString(packet.topic);
  display->clear();
  display->drawString(0, 10, "New Message");
  display->drawString(0, 20, sduid.c_str());
  display->drawString(0, 30, muid.c_str());
  display->drawString(0, 40, cdpTopic.c_str());
  display->sendBuffer();
  std::string topic = "iot-2/evt/" + cdpTopic + "/fmt/json";
  String jsonstat;
  serializeJson(doc, jsonstat);

  if (client.publish(topic.c_str(), jsonstat.c_str())) {
    Serial.println("[master] Packet forwarded:");
    serializeJsonPretty(doc, Serial);
    Serial.println("");
    Serial.println("[master] Publish ok");
    display->drawString(0, 60, "Publish ok");
    display->sendBuffer();
    return 0;
  } else {
    Serial.println("[master] Publish failed");
    display->drawString(0, 60, "Publish failed");
    display->sendBuffer();
    return -1;
  }
}

void handlenodeData(std::vector<byte> packetBuffer) {
  Serial.println("[master] got packet: " +
                 convertToHex(packetBuffer.data(), packetBuffer.size()));
  CdpPacket packet = CdpPacket(packetBuffer);
  if(packet.topic != reservedTopic::ack) {
    if(quackJson(packet) == -1) {
      if(packetQueue.size() > QUEUE_SIZE_MAX) {
        packetQueue.pop();
        packetQueue.push(packetBuffer);
      } else {
        packetQueue.push(packetBuffer);
      }
      Serial.print("New size of queue: ");
      Serial.println(packetQueue.size());
    }
  }
  subscribeTo(commandTopic);
}

void setup() {
  std::string deviceId("masternode");
  std::vector<byte> devId;
  devId.insert(devId.end(), deviceId.begin(), deviceId.end());
  node.setupWithDefaults(devId, SSID, PASSWORD);
  display = nodeDisplay::getInstance();
  display->setupDisplay(node.getType(), devId);
  node.onReceivenodeData(handlenodeData);
  #ifdef CA_CERT
  Serial.println("[master] Using root CA cert");
  wifiClient.setCACert(example_root_ca);
  #else
  Serial.println("[master] Using insecure TLS");
  wifiClient.setInsecure();
  #endif
  Serial.println("[master] Setup OK! ");
   node.enableAcks(true);
  display->showDefaultScreen();
}

void loop() {
   if (!node.isWifiConnected() && retry) {
      String ssid = node.getSsid();
      String password = node.getPassword();
      Serial.println("[master] WiFi disconnected, reconnecting to local network: " +
                     ssid);

      int err = node.reconnectWifi(ssid, password);
      if (err != node_ERR_NONE) {
         retry = false;
      }
      timer.in(5000, enableRetry);
   }
   if (!client.loop()) {
     if(node.isWifiConnected()) {
        mqttConnect();
     } 
   }
   node.run();
   timer.tick();
}

void gotMsg(char* topic, byte* payload, unsigned int payloadLength) {
  Serial.print("gotMsg: invoked for topic: "); Serial.println(topic);
  if (String(topic).indexOf(CMD_STATE_WIFI) > 0) {
    Serial.println("Start WiFi Command");
    byte sCmd = 1;
    std::vector<byte> sValue = {payload[0]};
    if(payloadLength > 3) {
      std::string destination = "";
      for (int i=1; i<payloadLength; i++) {
        destination += (char)payload[i];
      }
      std::vector<byte> dDevId;
      dDevId.insert(dDevId.end(),destination.begin(),destination.end());
      node.sendCommand(sCmd, sValue, dDevId);
    } else {
      node.sendCommand(sCmd, sValue);
    }
  } else if (String(topic).indexOf(CMD_STATE_HEALTH) > 0) {
    byte sCmd = 0;
    std::vector<byte> sValue = {payload[0]};
    if(payloadLength >= 8) {
      std::string destination = "";
      for (int i=1; i<payloadLength; i++) {
        destination += (char)payload[i];
      }
      std::vector<byte> dDevId;
      dDevId.insert(dDevId.end(),destination.begin(),destination.end());
      node.sendCommand(sCmd, sValue, dDevId);
    } else {
      Serial.println("Payload size too small");
    }
  } else if (String(topic).indexOf(CMD_STATE_MBM) > 0){
      std::vector<byte> message;
      std::string output;
      for (int i = 0; i<payloadLength; i++) {
        output = output + (char)payload[i];
      }
      message.insert(message.end(),output.begin(),output.end());
      node.sendMessageBoardMessage(message);
  } else {
    Serial.print("gotMsg: unexpected topic: "); Serial.println(topic); 
  } 
}

void wifiConnect() {
 Serial.print("Connecting to "); Serial.print(SSID);
 WiFi.begin(SSID, PASSWORD);
 while (WiFi.status() != WL_CONNECTED) {
   delay(500);
   Serial.print(".");
 } 
 Serial.print("\nWiFi connected, IP address: "); Serial.println(WiFi.localIP());
}

void mqttConnect() {
   if (!!!client.connected()) {
      Serial.print("Reconnecting MQTT client to "); Serial.println(server);
      if(!!!client.connect(clientId, authMethod, token) && retry) {
         Serial.print("Connection failed, retry in 5 seconds");
         retry = false;
         timer.in(5000, enableRetry);
      }
      Serial.println();
   } else {
      if(packetQueue.size() > 0) {
         publishQueue();
      }
      subscribeTo(commandTopic);
   }

}
void subscribeTo(const char* topic) {
 Serial.print("subscribe to "); Serial.print(topic);
 if (client.subscribe(topic)) {
   Serial.println(" OK");
 } else {
   Serial.println(" FAILED");
 }
}
bool enableRetry(void*) {
  retry = true;
  return retry;
}
void publishQueue() {
  while(!packetQueue.empty()) {
    if(quackJson(packetQueue.front()) == 0) {
      packetQueue.pop();
      Serial.print("Queue size: ");
      Serial.println(packetQueue.size());
    } else {
      return;
    }
  }
}