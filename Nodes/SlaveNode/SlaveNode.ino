#include <string>
#include <arduino-timer.h>
#include <salvenode.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64 
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

salvenode node;
auto timer = timer_create_default();
const int INTERVAL_MS = 60000;
int counter = 1;

void setup() {
  std::string deviceId("salve0001");
  std::vector<byte> devId;
  devId.insert(devId.end(), deviceId.begin(), deviceId.end());
  node.setupWithDefaults(devId);
  timer.every(INTERVAL_MS, runSensor);
  Serial.println("[salve] Setup OK!");
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(2);      
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         
  display.setCursor(0, 0);
  display.write("Pratik Shrestha Display");
  display.display();
}

void loop() {
  timer.tick();
  node.run();
}

bool runSensor(void *) {
  bool result;
  const byte* buffer;
  String message = String("Counter:") + String(counter);
  int length = message.length();
  Serial.print("[salve] sensor data: ");
  Serial.println(message);
  buffer = (byte*) message.c_str(); 

  result = sendData(buffer, length);
  if (result) {
     Serial.println("[salve] runSensor ok.");
  } else {
     Serial.println("[salve] runSensor failed.");
  }
  return result;
}

bool sendData(const byte* buffer, int length) {
  bool sentOk = false;
  int err = node.sendData(topics::status, buffer, length);
  if (err == node_ERR_NONE) {
     counter++;
     sentOk = true;
  }
  if (!sentOk) {
    Serial.println("[salve] Failed to send data. error = " + String(err));
  }
  return sentOk;
}
