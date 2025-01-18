#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

// const char* ssid = "SEC304";             
// const char* password = "sec304sec304";   
const char* ssid = "vivo";             
const char* password = "aa24681012";
// const char* ssid = "gojo";             
// const char* password = "tel8418736";
WiFiUDP udpWheel;
WiFiUDP udpHandle;  

unsigned int wheelUdpPort = 55555;    
unsigned int handleUdpPort = 4210;    

char incomingPacketWheel[255];        
char incomingPacketHandle[255];       

Servo wheelServo;   
Servo handleServo;  

int wheelServoPin = D1;  
int handleServoPin = D2; 
void setup() {
  Serial.begin(115200);
  
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // 启动两个UDP服务器
  udpWheel.begin(wheelUdpPort);
  udpHandle.begin(handleUdpPort);

  Serial.printf("UDP server for wheel started at IP: %s, port: %d\n", WiFi.localIP().toString().c_str(), wheelUdpPort);
  Serial.printf("UDP server for handle started at IP: %s, port: %d\n", WiFi.localIP().toString().c_str(), handleUdpPort);

  // 初始化伺服电机
  wheelServo.attach(wheelServoPin);
  handleServo.attach(handleServoPin);
}

void loop() {
  // 处理 wheelServo 的 UDP 数据
  int packetSizeWheel = udpWheel.parsePacket(); 
  if (packetSizeWheel) {
    int len = udpWheel.read(incomingPacketWheel, sizeof(incomingPacketWheel) - 1); 
    if (len > 0) {
      incomingPacketWheel[len] = '\0'; 
    }
    
    Serial.printf("Received packet (wheel): %s\n", incomingPacketWheel);

    String packetString = String(incomingPacketWheel);
    int delimiterIndex = packetString.indexOf(':'); 
    
    if (delimiterIndex > 0) {
      String type = packetString.substring(0, delimiterIndex); 
      String valueStr = packetString.substring(delimiterIndex + 1); 
      float value = valueStr.toFloat(); 

      if (type == "wheel") {
        if (valueStr == "stop") {
          wheelServo.write(90); 
          Serial.println("Wheel servo stopped.");
        } else {
          value = constrain(value, 0, 180); 
          wheelServo.write(value);
          Serial.printf("Wheel servo angle set to: %.2f\n", value);
        }
      } else {
        Serial.println("Unknown command type for wheel.");
      }
    } else {
      Serial.println("Invalid packet format (wheel).");
    }
  }


  int packetSizeHandle = udpHandle.parsePacket(); 
  if (packetSizeHandle) {
    int len = udpHandle.read(incomingPacketHandle, sizeof(incomingPacketHandle) - 1); 
    if (len > 0) {
      incomingPacketHandle[len] = '\0'; 
    }
    
    Serial.printf("Received packet (handle): %s\n", incomingPacketHandle);

    String packetString = String(incomingPacketHandle);
    int delimiterIndex = packetString.indexOf(':'); 
    
    if (delimiterIndex > 0) {
      String type = packetString.substring(0, delimiterIndex); 
      String valueStr = packetString.substring(delimiterIndex + 1); 
      float angle = valueStr.toFloat();  

      if (type == "handle") {
        angle = constrain(angle, 0, 180);
        handleServo.write(angle);
        
        // Print angle with one decimal place
        Serial.printf("Handle servo angle set to: %.1f\n", angle);  // 修改此行
      } else {
        Serial.println("Unknown command type for handle.");
      }
    } else {
      Serial.println("Invalid packet format (handle).");
    }
  }
}
