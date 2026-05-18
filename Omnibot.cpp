#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP_RP2040W.h>
#include <math.h>
#include <vector>
#include <string>

#define PIN_BACKLEFT_F      7
#define PIN_BACKLEFT_R      6
#define PIN_BACKRIGHT_F     28
#define PIN_BACKRIGHT_R     27
#define PIN_FORWARDRIGHT_F  9
#define PIN_FORWARDRIGHT_R  8
#define PIN_FORWARDLEFT_F   20
#define PIN_FORWARDLEFT_R   21

#define TCP_PORT 6767

std::vector<AsyncClient*> clients;
const char* apPWD = "Password";
const char* apSSID = "V2-bot";

AsyncServer* server;
void handleNewClient(void*arg, AsyncClient* client)
{
  Serial.println("New connection established.\n");
  clients.push_back(client);

  client->onData([](void* arg, AsyncClient* c, void* data, size_t len)
  {
    IPAddress clientIP = c->getRemoteAddress();
    Serial.print(clientIP);
    Serial.print(": ");
    Serial.write((uint8_t*)data,len);
    if ((string*)data == "ON"){
      digitalWrite(LED_BUILTIN, );
    }
    
  } , NULL);

  client->onDisconnect([](void* arg, AsyncClient* c)
  {
    Serial.println("Client disconnected");
  } , NULL);

}

void setup() {
  Serial.begin(115200);

  WiFi.beginAP(apSSID,apPWD);
  Serial.println(WiFi.softAPIP());
  server = new AsyncServer(TCP_PORT);
  server->onClient(&handleNewClient,server);
  server->begin();

  Serial.printf("\nServer listening on port: %d\n", TCP_PORT); 
}

void loop() {
  delay(5000);
}
