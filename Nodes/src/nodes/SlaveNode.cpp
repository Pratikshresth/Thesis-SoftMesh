#include "../salvenode.h"
#include "../MemoryFree.h"
int salvenode::setupWithDefaults(std::vector<byte> deviceId, String ssid, String password) {
  int err = node::setupWithDefaults(deviceId, ssid, password);
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  err = setupRadio();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  std::string name(deviceId.begin(),deviceId.end());
  err = setupWifi(name.c_str());
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  err = setupDns();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  err = setupWebServer(false);
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  err = setupOTA();
  if (err != node_ERR_NONE) {
    logerr("ERROR setupWithDefaults rc = " + String(err));
    return err;
  }
  nodeutils::getTimer().every(CDPCFG_MILLIS_ALIVE, imAlive);
  nodeNet->loadChannel();
  return node_ERR_NONE;
}
void salvenode::run() {
  node::logIfLowMemory();
  nodeRadio.serviceInterruptFlags();
  handleOtaUpdate();
  if (nodeRadio::getReceiveFlag()) {
    handleReceivedPacket();
    rxPacket->reset();
  }
  processPortalRequest();
}
void salvenode::handleReceivedPacket() {
  std::vector<byte> data;
  bool relay = false;
  loginfo("====> handleReceivedPacket: START");
  int err = nodeRadio.readReceivedData(&data);
  if (err != node_ERR_NONE) {
    logerr("ERROR failed to get data from nodeRadio. rc = "+ String(err));
    return;
  }
  logdbg("Got data from radio, prepare for relay. size: "+ String(data.size()));
  relay = rxPacket->prepareForRelaying(&filter, data);
  if (relay) {
    loginfo("handleReceivedPacket: packet RELAY START");
    CdpPacket packet = CdpPacket(rxPacket->getBuffer());
    if (nodeutils::isEqual(BROADCAST_DUID, packet.dduid)) {
      switch(packet.topic) {
        case reservedTopic::ping:
          loginfo("ping received");
          err = sendPong();
          if (err != node_ERR_NONE) {
            logerr("ERROR failed to send pong message. rc = " + String(err));
          }
          return;
        break;
        case reservedTopic::ack:
          handleAck(packet);
          err = nodeRadio.relayPacket(rxPacket);
          if (err != node_ERR_NONE) {
            logerr("====> ERROR handleReceivedPacket failed to relay. rc = " + String(err));
          } else {
            loginfo("handleReceivedPacket: packet RELAY DONE");
          }
        break;
        case reservedTopic::cmd:
          loginfo("Command received");
          handleCommand(packet);

          err = nodeRadio.relayPacket(rxPacket);
          if (err != node_ERR_NONE) {
            logerr("====> ERROR handleReceivedPacket failed to relay. rc = " + String(err));
          } else {
            loginfo("handleReceivedPacket: packet RELAY DONE");
          }
        break;
        case reservedTopic::mbm:
          packet.timeReceived = millis();
          nodeNet->addToMessageBoardBuffer(packet);
        break;
        case topics::gchat:
          packet.timeReceived = millis();
          nodeNet->addToChatBuffer(packet);
        break;
        default:
          err = nodeRadio.relayPacket(rxPacket);
          if (err != node_ERR_NONE) {
            logerr("====> ERROR handleReceivedPacket failed to relay. rc = " + String(err));
          } else {
            loginfo("handleReceivedPacket: packet RELAY DONE");
          }
      }
    } else if(nodeutils::isEqual(duid, packet.dduid)) { 
        std::vector<byte> dataPayload;
        byte num = 1;
      
      switch(packet.topic) {
        case topics::dcmd:
          loginfo("node command received");
          handlenodeCommand(packet);
        break;
        case reservedTopic::cmd:
          loginfo("Command received");
          
          //Start send ack that command was received
          dataPayload.push_back(num);

          dataPayload.insert(dataPayload.end(), packet.sduid.begin(), packet.sduid.end());
          dataPayload.insert(dataPayload.end(), packet.muid.begin(), packet.muid.end());

          err = txPacket->prepareForSending(&filter, masternode_DUID, 
            nodeType::salve, reservedTopic::ack, dataPayload);
          if (err != node_ERR_NONE) {
          logerr("ERROR handleReceivedPacket. Failed to prepare ack. Error: " +
            String(err));
          }

          err = nodeRadio.sendData(txPacket->getBuffer());
          if (err == node_ERR_NONE) {
            filter.bloom_add(packet.muid.data(), MUID_LENGTH);
          } else {
            logerr("ERROR handleReceivedPacket. Failed to send ack. Error: " +
              String(err));
          }
          
          //Handle Command
          handleCommand(packet);

        break;
        case reservedTopic::ack:
          handleAck(packet);
        break;
        case reservedTopic::mbm:
          packet.timeReceived = millis();
          nodeNet->addToMessageBoardBuffer(packet);
        break;
        case topics::gchat:
          packet.timeReceived = millis();
          nodeNet->addToChatBuffer(packet);
        break;
        case topics::pchat:{
          packet.timeReceived = millis();
          std::string session(packet.sduid.begin(), packet.sduid.end());

          nodeNet->createPrivateHistory(session);
          nodeNet->addToPrivateChatBuffer(packet, session);
        }
        break;
        default:
          err = nodeRadio.relayPacket(rxPacket);
          if (err != node_ERR_NONE) {
            logerr("====> ERROR handleReceivedPacket failed to relay. rc = " + String(err));
          } else {
            loginfo("handleReceivedPacket: packet RELAY DONE");
          }
      }

    } else {
      err = nodeRadio.relayPacket(rxPacket);
      if (err != node_ERR_NONE) {
        logerr("====> ERROR handleReceivedPacket failed to relay. rc = " + String(err));
      } else {
        loginfo("handleReceivedPacket: packet RELAY DONE");
      }
    }

  }
}

void salvenode::handleCommand(const CdpPacket & packet) {
  int err;
  std::vector<byte> dataPayload;
  std::vector<byte> alive {'I','m',' ','a','l','i','v','e'};

  switch(packet.data[0]) {
    case 0:
      //Send health quack
      loginfo("Health request received");
      dataPayload.insert(dataPayload.end(), alive.begin(), alive.end());
      err = txPacket->prepareForSending(&filter, masternode_DUID, 
        nodeType::salve, topics::health, dataPayload);
      if (err != node_ERR_NONE) {
      logerr("ERROR handleReceivedPacket. Failed to prepare ack. Error: " +
        String(err));
      }

      err = nodeRadio.sendData(txPacket->getBuffer());
      if (err == node_ERR_NONE) {
        CdpPacket healthPacket = CdpPacket(txPacket->getBuffer());
        filter.bloom_add(healthPacket.muid.data(), MUID_LENGTH);
      } else {
        logerr("ERROR handleReceivedPacket. Failed to send ack. Error: " +
          String(err));
      }

    break;
    case 1:
      //Change wifi status
      if((char)packet.data[1] == '1') {
        loginfo("Command WiFi ON");
        WiFi.mode(WIFI_AP);

      } else if ((char)packet.data[1] == '0') {
        loginfo("Command WiFi OFF");
        WiFi.mode( WIFI_MODE_NULL );
      }
      
    break;
    default:
      logerr("Command not recognized");
  }

}

void salvenode::handlenodeCommand(const CdpPacket & packet) {
  loginfo("Doesn't do anything yet. But node Command was received.");
}

void salvenode::handleAck(const CdpPacket & packet) {
  
  if (lastMessageMuid.size() == MUID_LENGTH) {
    const byte numPairs = packet.data[0];
    static const int NUM_PAIRS_LENGTH = 1;
    static const int PAIR_LENGTH = DUID_LENGTH + MUID_LENGTH;
    for (int i = 0; i < numPairs; i++) {
      int pairOffset = NUM_PAIRS_LENGTH + i*PAIR_LENGTH;
      std::vector<byte>::const_iterator duidOffset = packet.data.begin() + pairOffset;
      std::vector<byte>::const_iterator muidOffset = packet.data.begin() + pairOffset + DUID_LENGTH;
      if (std::equal(duid.begin(), duid.end(), duidOffset)
        && std::equal(lastMessageMuid.begin(), lastMessageMuid.end(), muidOffset)
      ) {
        loginfo("handleReceivedPacket: matched ack-MUID "
          + nodeutils::toString(lastMessageMuid));
        lastMessageAck = true;
        break;
      }
    }
    

    // TODO[Rory Olsen: 2021-06-23]: The application may need to know about
    //   acks. I recommend a callback specifically for acks, or
    //   similar.
  }
}

bool salvenode::getDetectState() { return nodeutils::getDetectState(); }
