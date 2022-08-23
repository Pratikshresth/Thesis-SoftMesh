#include "../masternode.h"

int masternode::setupWithDefaults(std::vector<byte> deviceId, String ssid,
  String password) {
  loginfo("setupWithDefaults...");

  int err = node::setupWithDefaults(deviceId, ssid, password);

  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }

  err = setupRadio();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults  rc = " + String(err));
    return err;
  }

  std::string name(deviceId.begin(),deviceId.end());

  err = setupWifi(name.c_str());
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults  rc = " + String(err));
    return err;
  }

  err = setupDns();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults  rc = " + String(err));
    return err;
  }



  err = setupWebServer();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults  rc = " + String(err));
    return err;
  }

  err = setupOTA();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults  rc = " + String(err));
    return err;
  }

  if (ssid.length() != 0 && password.length() != 0) {
    err = setupInternet(ssid, password);

    if (err != node_ERR_NONE) {
      logerr("ERROR setupWithDefaults  rc = " + String(err));
      return err;
    }
  } 

  nodeNet->loadChannel();

  if (ssid.length() == 0 && password.length() == 0) {
  // If WiFi credentials inside the INO are empty use the saved credentials
  // TODO: if the portal credentials were saved and the INO contains credentials it won't
  // take the Portal credentials on setup
    err = nodeNet->loadWiFiCredentials();

    if (err != node_ERR_NONE) {
      logerr("ERROR setupWithDefaults  rc = " + String(err));
     
      return err;
    }
    
    
  }



  loginfo("setupWithDefaults done");
  return node_ERR_NONE;
}

void masternode::run() {
  node::logIfLowMemory();

  nodeRadio.serviceInterruptFlags();

  handleOtaUpdate();
  if (nodeRadio::getReceiveFlag()) {
    handleReceivedPacket();
    rxPacket->reset(); // TODO(rolsen): Make rxPacket local to handleReceivedPacket
  }

  // TODO(rolsen): Enforce mutually exclusive access to nodeRadio.
  // ackTimer.tick() calls broadcastAck, which calls nodeRadio. Since nodeRadio
  // is a shared resource, we should synchronize everything in ackTimer.tick()
  // so the thread in AsyncWebServer cannot modify nodeRadio while broadcastAck
  // is also modifying nodeRadio.
  ackTimer.tick();
}

void masternode::handleReceivedPacket() {

  loginfo("handleReceivedPacket() START");
  std::vector<byte> data;
  int err = nodeRadio.readReceivedData(&data);

  if (err != node_ERR_NONE) {
    logerr("ERROR handleReceivedPacket. Failed to get data. rc = " +
     String(err));
    return;
  }
  // ignore pings
  if (data[TOPIC_POS] == reservedTopic::ping) {
    rxPacket->reset();
    return;
  }
  // build our RX nodePacket which holds the updated path in case the packet is relayed
  bool relay = rxPacket->prepareForRelaying(&filter, data);
  if (relay) {
    logdbg("relaying:  " +
      nodeutils::convertToHex(rxPacket->getBuffer().data(),
        rxPacket->getBuffer().size()));
    loginfo("invoking callback in the node application...");
    
    if(rxPacket->getTopic() == topics::gchat){
      nodeNet->addToChatBuffer(CdpPacket(rxPacket->getBuffer()));
    } else{
      recvDataCallback(rxPacket->getBuffer());
    }
    
    if (acksEnabled) {
      const CdpPacket packet = CdpPacket(rxPacket->getBuffer());
      if (needsAck(packet)) {
        handleAck(packet);
      }
    }

    loginfo("handleReceivedPacket() DONE");
  }
}

void masternode::handleAck(const CdpPacket & packet) {
  if (ackTimer.empty()) {
    logdbg("Starting new ack broadcast timer with a delay of " +
      String(timerDelay) + " ms");
    ackTimer.in(timerDelay, ackHandler, this);
  }

  storeForAck(packet);

  if (ackBufferIsFull()) {
    logdbg("Ack buffer is full. Sending broadcast ack immediately.");
    ackTimer.cancel();
    broadcastAck();
  }
}

void masternode::enableAcks(bool enable) {
  acksEnabled = enable;
}

bool masternode::ackHandler(masternode * node)
{
  node->broadcastAck();
  return false;
}

void masternode::storeForAck(const CdpPacket & packet) {
  ackStore.push_back(std::pair<Duid, Muid>(packet.sduid, packet.muid));
}

bool masternode::ackBufferIsFull() {
  return (ackStore.size() >= MAX_MUID_PER_ACK);
}

bool masternode::needsAck(const CdpPacket & packet) {
  if (packet.topic == reservedTopic::ack) {
    return false;
  } else {
    return true;
  }
}

void masternode::broadcastAck() {
  assert(ackStore.size() <= MAX_MUID_PER_ACK);

  const byte num = static_cast<byte>(ackStore.size());

  std::vector<byte> dataPayload;
  dataPayload.push_back(num);
  for (int i = 0; i < num; i++) {
    Duid duid = ackStore[i].first;
    Muid muid = ackStore[i].second;
    logdbg("Sending ack to DUID " + nodeutils::toString(duid)
      + " for MUID " + nodeutils::toString(muid));
    dataPayload.insert(dataPayload.end(), duid.begin(), duid.end());
    dataPayload.insert(dataPayload.end(), muid.begin(), muid.end());
  }

  int err = txPacket->prepareForSending(&filter, BROADCAST_DUID, nodeType::master,
    reservedTopic::ack, dataPayload);
  if (err != node_ERR_NONE) {
    logerr("ERROR handleReceivedPacket. Failed to prepare ack. Error: " +
      String(err));
  }

  err = nodeRadio.sendData(txPacket->getBuffer());

  if (err == node_ERR_NONE) {
    CdpPacket packet = CdpPacket(txPacket->getBuffer());
    filter.bloom_add(packet.muid.data(), MUID_LENGTH);
  } else {
    logerr("ERROR handleReceivedPacket. Failed to send ack. Error: " +
      String(err));
  }

  ackStore.clear();
}

void masternode::sendCommand(byte cmd, std::vector<byte> value) {
  loginfo("Initiate sending command");
  std::vector<byte> dataPayload;
  dataPayload.push_back(cmd);
  dataPayload.insert(dataPayload.end(), value.begin(), value.end());

  int err = txPacket->prepareForSending(&filter, BROADCAST_DUID, nodeType::master,
    reservedTopic::cmd, dataPayload);
  if (err != node_ERR_NONE) {
    logerr("ERROR handleReceivedPacket. Failed to prepare ack. Error: " +
      String(err));
  }

  err = nodeRadio.sendData(txPacket->getBuffer());

  if (err == node_ERR_NONE) {
    CdpPacket packet = CdpPacket(txPacket->getBuffer());
    filter.bloom_add(packet.muid.data(), MUID_LENGTH);
  } else {
    logerr("ERROR handleReceivedPacket. Failed to send ack. Error: " +
      String(err));
  }
}

void masternode::sendCommand(byte cmd, std::vector<byte> value, std::vector<byte> dduid) {
  loginfo("Initiate sending command");
  std::vector<byte> dataPayload;
  dataPayload.push_back(cmd);
  dataPayload.insert(dataPayload.end(), value.begin(), value.end());

  int err = txPacket->prepareForSending(&filter, dduid, nodeType::master,
    reservedTopic::cmd, dataPayload);
  if (err != node_ERR_NONE) {
    logerr("ERROR handleReceivedPacket. Failed to prepare cmd. Error: " +
      String(err));
  }

  err = nodeRadio.sendData(txPacket->getBuffer());

  if (err == node_ERR_NONE) {
    CdpPacket packet = CdpPacket(txPacket->getBuffer());
    filter.bloom_add(packet.muid.data(), MUID_LENGTH);
  } else {
    logerr("ERROR handleReceivedPacket. Failed to send cmd. Error: " +
      String(err));
  }
}

int masternode::reconnectWifi(String ssid, String password) {
#ifdef CDPCFG_WIFI_NONE
  logwarn("WARNING reconnectWifi skipped, device has no WiFi.");
  return node_ERR_NONE;
#else
  if (!nodeNet->ssidAvailable(ssid)) {
    return nodeWIFI_ERR_NOT_AVAILABLE;
  }
  nodeNet->setupInternet(ssid, password);
  nodeNet->setupDns();
  if (WiFi.status() != WL_CONNECTED) {
    logerr("ERROR WiFi reconnection failed!");
    return nodeWIFI_ERR_DISCONNECTED;
  }
  return node_ERR_NONE;
#endif
}

void masternode::sendMessageBoardMessage(std::vector<byte> dataPayload, std::vector<byte> duid) {
  int err = txPacket->prepareForSending(&filter, duid, nodeType::master,
    reservedTopic::mbm, dataPayload);
  if (err != node_ERR_NONE) {
    logerr("ERROR handleReceivedPacket. Failed to prepare ack. Error: " +
      String(err));
  }

  err = nodeRadio.sendData(txPacket->getBuffer());

  if (err == node_ERR_NONE) {
    CdpPacket packet = CdpPacket(txPacket->getBuffer());
    filter.bloom_add(packet.muid.data(), MUID_LENGTH);
  } else {
    logerr("ERROR handleReceivedPacket. Failed to send ack. Error: " +
      String(err));
  }
  
}
