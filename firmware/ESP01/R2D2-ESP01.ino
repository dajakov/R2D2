#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <WebSocketsServer.h>

// Network credentials
const char* ssid = "R2D2";
const char* password = "changeMe";

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81); // Port 81

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);

  // WebSocket event handling
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    // Notify connection over UART
    char uartMessage[20];
    snprintf(uartMessage, sizeof(uartMessage), "connect--\n", num);
    Serial.write(uartMessage); // Send UART notification
  } else if (type == WStype_DISCONNECTED) {
    // Notify disconnection over UART
    char uartMessage[24];
    snprintf(uartMessage, sizeof(uartMessage), "!connect-\n", num);
    Serial.write(uartMessage); // Send UART notification
  } else if (type == WStype_TEXT) {
    String message = String((char*)payload);
    
    // Prepare a buffer for UART transmission
    char uartMessage[12]; // Fixed length for consistent UART data
    
    if (message.startsWith("move ")) {
      // Parse "move x,y" command
      int commaIndex = message.indexOf(',');
      if (commaIndex > 5) {
        String xStr = message.substring(5, commaIndex);
        String yStr = message.substring(commaIndex + 1);
        
        int x = xStr.toInt();
        int y = yStr.toInt();

        // Format the message for UART
        snprintf(uartMessage, sizeof(uartMessage), "M%+04d%+04d\n", x, y);
        Serial.write(uartMessage); // Send over UART
      } 
    } else if (message == "left") {
      snprintf(uartMessage, sizeof(uartMessage), "L00000000\n");
      Serial.write(uartMessage); // Send "left" over UART
    } else if (message == "right") {
      snprintf(uartMessage, sizeof(uartMessage), "R00000000\n");
      Serial.write(uartMessage); // Send "right" over UART
    } else if (message == "stop") {
      snprintf(uartMessage, sizeof(uartMessage), "S00000000\n");
      Serial.write(uartMessage); // Send "stop" over UART
    }
    // Echo the message back to the WebSocket client
    webSocket.sendTXT(num, (char*)payload);
  }
}
