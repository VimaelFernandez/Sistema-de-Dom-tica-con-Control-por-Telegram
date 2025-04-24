/**
 * @file ESP32_TelegramBot_Controller.ino
 * @brief ESP32-based controller for fan and light with Telegram bot interface
 * 
 * This code implements an IoT controller that:
 * - Manages a fan (3 speeds + automatic temperature-based control)
 * - Controls a bedroom light
 * - Communicates via ESP-NOW with other devices
 * - Provides Telegram bot interface for remote control
 * 
 * Hardware Requirements:
 * - ESP32 microcontroller
 * - OLED display (SSD1306, 128x64)
 * - Optional: Temperature sensors, relays for fan/light control
 * 
 * Communication:
 * - WiFi for Telegram bot
 * - ESP-NOW for peer-to-peer communication with other ESP devices
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <OneWire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Wifi network station credentials
const char *ssid = "TP-Link_83C6";  // WiFi SSID
const char *password = "job12345";  // WiFi password

//  Telegram BOT Token (Get from Botfather)
#define BOT_TOKEN "6314029912:AAG_q2_Mlrjw9thx6NLiG3X1-i2Ens3jaCI"

// OLED Display Configuration
#define ANCHO 128                                       // OLED display width
#define ALTO 64                                         // OLED display height
#define OLED_RESET -1                                   // Reset pin (-1 if not used)
Adafruit_SSD1306 oled(ANCHO, ALTO, &Wire, OLED_RESET);  // OLED object

// ESP-NOW Communication Parameters
const unsigned long BOT_MTBS = 1000;  // Scan interval for messages (ms)
const unsigned long BOT_MTBS1 = 100;  // Faster message interval (ms)
unsigned long bot_lasttime;           // Last message scan time
unsigned long bot2_lasttime;          // Secondary timer
int chan;                             // WiFi channel
uint8_t counter = 0;                  // General purpose counter
uint8_t counterConexWifi = 0;         // WiFi connection attempt counter

// Device Control States
bool automatic;               // Automatic mode flag
bool lampBedRoom;             // Bedroom light state
bool status1;                 // Status flag 1
bool status2;                 // Status flag 2
bool Start = false;           // Initialization flag
bool flag;                    // General purpose flag
bool flag2;                   // Secondary flag
bool statusSend;              // Message send status
float temp = 0;               // Temperature value
uint8_t countSend = 0;        // Message send counter
uint8_t varVelocidad;         // Fan speed (0-3)
uint8_t clientMacAddress[6];  // MAC address storage

/**
 * @struct struct_message
 * @brief Data structure for ESP-NOW communication
 * 
 * Used to send/receive control parameters and status between devices
 */
typedef struct struct_message {
  uint8_t msgType;         // Message type (PAIRING or DATA)
  uint8_t id;              // Device ID
  bool automatic;          // Automatic control flag
  bool lampBedRoom;        // Bedroom light state
  bool status1;            // Status flag 1
  bool status2;            // Status flag 2
  bool flag;               // General purpose flag
  bool flag2;              // Secondary flag
  uint8_t varVel;          // Fan speed (0-3)
  float temp;              // Temperature reading
  unsigned int readingId;  // Message counter
} struct_message;

/**
 * @struct struct_pairing
 * @brief Data structure for device pairing information
 */
typedef struct struct_pairing {
  uint8_t msgType;     // Message type (always PAIRING)
  uint8_t id;          // Device ID
  uint8_t macAddr[6];  // MAC address
  uint8_t channel;     // WiFi channel
} struct_pairing;

// Data structures instances
struct_message incomingReadings;   // Incoming data from peers
struct_message outgoingSetpoints;  // Outgoing control parameters
struct_pairing pairingData;        // Pairing information

esp_now_peer_info_t slave;                            // Peer information
WiFiClientSecure secured_client;                      // Secure client for Telegram
UniversalTelegramBot bot(BOT_TOKEN, secured_client);  // Telegram bot instance

/**
 * @enum MessageType
 * @brief Types of ESP-NOW messages
 */
enum MessageType {
  PAIRING,  // Pairing request/response
  DATA,     // Regular data message
};
MessageType messageType;

/**
 * @brief Reads and prints the MAC address of the device
 */
void readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

/**
 * @brief Prints a MAC address in human-readable format
 * @param mac_addr Pointer to MAC address bytes
 */
void printMAC(const uint8_t *mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

/**
 * @brief Adds a new peer to ESP-NOW
 * @param peer_addr MAC address of peer to add
 * @return true if successful, false otherwise
 */
bool addPeer(const uint8_t *peer_addr) {
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t *peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);

  slave.channel = chan;  // Use current channel
  slave.encrypt = 0;     // No encryption

  // Check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if (exists) {
    Serial.println("Already Paired");
    return true;
  } else {
    esp_err_t addStatus = esp_now_add_peer(peer);
    if (addStatus == ESP_OK) {
      Serial.println("Pair success");
      return true;
    } else {
      Serial.println("Pair failed");
      return false;
    }
  }
}

/**
 * @brief Callback when data is sent via ESP-NOW
 * @param mac_addr MAC address of recipient
 * @param status Delivery status
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Delivery Succes to ");
    statusSend = true;
  } else {
    Serial.print("Delivery Fail to ");
    statusSend = false;
  }
  printMAC(mac_addr);
  Serial.println();
  countSend++;
}

/**
 * @brief Callback when data is received via ESP-NOW
 * @param mac_addr MAC address of sender
 * @param incomingData Received data buffer
 * @param len Length of received data
 */
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.print(len);
  Serial.println(" bytes of new data received.");
  uint8_t type = incomingData[0];  // First message byte is the type of message
  Serial.println(type);
  switch (type) {
    case DATA:  // Regular data message
      memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
      // Update local variables with received data
      temp = incomingReadings.temp;
      varVelocidad = incomingReadings.varVel;
      automatic = incomingReadings.automatic;
      flag = incomingReadings.flag;
      status1 = incomingReadings.status1;
      status2 = incomingReadings.status2;
      break;

    case PAIRING:  // Pairing request
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      Serial.println(pairingData.msgType);
      Serial.println(pairingData.id);
      Serial.print("Pairing request from MAC Address: ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel ");
      Serial.println(pairingData.channel);

      // Store client MAC address
      clientMacAddress[0] = pairingData.macAddr[0];
      clientMacAddress[1] = pairingData.macAddr[1];
      clientMacAddress[2] = pairingData.macAddr[2];
      clientMacAddress[3] = pairingData.macAddr[3];
      clientMacAddress[4] = pairingData.macAddr[4];
      clientMacAddress[5] = pairingData.macAddr[5];

      if (pairingData.id > 0) {  // Don't respond to server itself
        if (pairingData.msgType == PAIRING) {
          pairingData.id = 0;  // 0 indicates server
          WiFi.softAPmacAddress(pairingData.macAddr);
          Serial.print("Pairing MAC Address: ");
          printMAC(clientMacAddress);
          pairingData.channel = chan;
          Serial.println(" send response");
          addPeer(clientMacAddress);
          esp_err_t result = esp_now_send(clientMacAddress, (uint8_t *)&pairingData, sizeof(pairingData));
          if (result == ESP_OK) {
            Serial.println("Envío hecho a la perfección pa'");
          } else {
            Serial.println("No se envió ni una mondá, mi loco xd");
          }
        }
      }
      break;
  }
}

/**
 * @brief Initializes ESP-NOW communication
 */
void initESP_NOW() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializinSg ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

/**
 * @brief Prepares data to be sent via ESP-NOW
 */
void readDataToSend() {
  outgoingSetpoints.msgType = DATA;
  outgoingSetpoints.id = 0;
  outgoingSetpoints.automatic = automatic;
  outgoingSetpoints.status1 = status1;
  outgoingSetpoints.status2 = status2;
  outgoingSetpoints.temp = temp;
  outgoingSetpoints.varVel = varVelocidad;
  outgoingSetpoints.flag = flag;
  outgoingSetpoints.flag2 = flag2;
  outgoingSetpoints.lampBedRoom = lampBedRoom;
  outgoingSetpoints.readingId = counter++;
}

/**
 * @brief Processes outgoing messages with retry logic
 */
void processMessages() {
  readDataToSend();
  do {
    if (millis() - bot2_lasttime > BOT_MTBS1) {
      esp_now_send(NULL, (uint8_t *)&outgoingSetpoints, sizeof(outgoingSetpoints));
      Serial.println("Waiting message...");
      bot2_lasttime = millis();
    }
    if (countSend >= 30) {
      ESP.restart();
    }
  } while (flag == false && statusSend == false);
  countSend = 0;
  statusSend = false;
}

/**
 * @brief Handles incoming Telegram bot messages
 * @param numNewMessages Number of new messages to process
 */
void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));
  String message;
  String tempString;
  String varVelocidadString;

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    String text = bot.messages[i].text;
    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "VAFB";
    if ((text == "Inicia") || (text == "/Inicia")) {
      message += "Bienvenido al bot de Telegram, " + from_name + ".\n";
      message += "Este bot te permite realizar el control de un ventilador mediante la App. Los comandos son los siguientes:\n\n";
      message += "/Velocidad_1\n";
      message += "/Velocidad_2\n";
      message += "/Velocidad_3\n";
      message += "/Apagado\n";
      message += "/Automatico\n";
      message += "/Estado\n";
      bot.sendMessage(chat_id, message);
    }
    if (text == "Control") {
      String keyboardJson = "[[\"Vel 1\", \"Vel 2\", \"Vel 3\"],[\"Apagado\"],[\"Automatico\"],[\"Estado\"],[\"Cuarto Encendido\", \"Cuarto Apagado\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, "Choose from one of tje following options", "", keyboardJson, true);
    }
    if ((text == "Vel 1") || (text == "/Velocidad_1")) {
      // Set and send values to monitor temp ESP8266.
      automatic = false;
      varVelocidad = 1;
      status1 = false;
      flag = false;
      processMessages();
      varVelocidadString = String(varVelocidad);
      message += "Velocidad del ventilador: " + varVelocidadString + " \n";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Vel 2") || (text == "/Velocidad_2")) {
      automatic = false;
      varVelocidad = 2;
      status1 = false;
      flag = false;
      processMessages();
      varVelocidadString = String(varVelocidad);
      message += "Velocidad del ventilador: " + varVelocidadString + " \n";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Vel 3") || (text == "/Velocidad_3")) {
      // Set and send values to monitor temp ESP8266.
      automatic = false;
      varVelocidad = 3;
      status1 = false;
      flag = false;
      processMessages();
      varVelocidadString = String(varVelocidad);
      message += "Velocidad del ventilador: " + varVelocidadString + " \n";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Apagado") || (text == "/Apagado")) {
      // Carga y envío de valores a la placa de monitoreo  de temp ESP8266.
      automatic = false;
      varVelocidad = 0;
      status1 = false;
      flag = false;
      processMessages();
      message += "Ventilador apagado \n";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Automatico") || (text == "/Automatico")) {
      automatic = true;
      status1 = false;
      flag = false;
      processMessages();
      message += "Ventilador en modo automático \n";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Estado") || (text == "/Estado")) {
      flag = false;
      status1 = true;
      processMessages();
      if (automatic == true) {
        message += "Modo: Automático\n";
      } else {
        message += "Modo: Manual\n";
      }
      tempString = String(temp, 2);
      message += "Temperatura: " + tempString + " °C\n";

      varVelocidadString = String(varVelocidad);
      switch (varVelocidad) {
        case 1:
          message += "Velocidad: " + varVelocidadString + "\n";
          break;
        case 2:
          message += "Velocidad: " + varVelocidadString + "\n";
          break;
        case 3:
          message += "Velocidad: " + varVelocidadString + "\n";
          break;
        default:
          message += "Velocidad: 0 \n";
      }
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Cuarto Encendido") || (text == "Cuarto_encendido")) {
      flag2 = false;
      status2 = false;
      lampBedRoom = true;
      processMessages();
      message += "Bombillo Encendido";
      bot.sendMessage(chat_id, message);
    }
    if ((text == "Cuarto Apagado") || (text == "Cuarto_apagado")) {
      flag2 = false;
      status2 = false;
      lampBedRoom = false;
      processMessages();
      message += "Bombillo Apagado";
      bot.sendMessage(chat_id, message);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Wire.begin();
  esp_wifi_start();
  WiFi.mode(WIFI_STA);

  Serial.print("Server MAC Address: ");
  readMacAddress();

  // Set dual mode (AP + STA)
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_1M_L);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - bot_lasttime > BOT_MTBS) {
      Serial.print(".");
      bot_lasttime = millis();
      counterConexWifi++;
    }
    if (counterConexWifi >= 60) {
      ESP.restart();
    }
  }

  // Initialize ESP-NOW
  counterConexWifi = 0;
  Serial.print("Server SOFT AP MAC Address:  ");
  Serial.println(WiFi.softAPmacAddress());
  chan = WiFi.channel();
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Synchronize time via NTP
  Serial.print("Recobrando el tiempo: ");
  configTime(0, 0, "pool.ntp.org");  // Obtener tiempo UTC vi NPT
  time_t now = time(nullptr);
  while (now < 24 * 3600) {
    if (millis() - bot_lasttime > BOT_MTBS) {
      Serial.print(".");
      counter++;
      bot_lasttime = millis();
    }
    if (counter >= 60) {
      ESP.restart();
    }
    now = time(nullptr);
  }
  Serial.println(now);
  
  // Initialize ESP-NOW
  initESP_NOW();
}
void loop() {
  if (millis() - bot_lasttime > BOT_MTBS) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    bot_lasttime = millis();
  }
}
