/**
 * @file ESP8266_Fan_Controller.ino
 * @brief ESP8266-based Smart Fan Controller with Temperature Sensor
 * 
 * This device:
 * - Controls a 3-speed fan based on temperature or remote commands
 * - Communicates via ESP-NOW with a central controller (ESP32)
 * - Implements automatic pairing with channel scanning
 * - Displays temperature on OLED screen
 * 
 * Hardware Requirements:
 * - ESP8266 microcontroller
 * - DS18B20 Temperature Sensor
 * - OLED Display (SSD1306, 128x64)
 * - Relay module for fan speed control
 * 
 * Communication:
 * - ESP-NOW protocol for wireless communication
 * - Automatic channel scanning for pairing
 */

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define ANCHO 128                                       // Display width in pixels
#define ALTO 64                                         // Display height in pixels
#define OLED_RESET -1                                   // Reset pin (-1 if not used)
Adafruit_SSD1306 oled(ANCHO, ALTO, &Wire, OLED_RESET);  // OLED instance

// Network Parameters
uint8_t channel = 1;  // Starting WiFi channel for pairing
int readingId = 0;    // Message counter
int id = 1;           // Board identifier

// Device Constants
#define BOARD_ID 2      // Unique identifier for this device
#define MAX_CHANNEL 11  // Maximum WiFi channel (11 for NA, 13 for EU)

// Broadcast address for pairing
uint8_t broadcastAddressX[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t clientMacAddress[6];  // Stores MAC address of paired device

// Timing Variables
unsigned long previousMillis = 0;        // Last temperature update time
unsigned long currentMillis = millis();  // Current time reference
unsigned long lastTime = 0;              // Last message time

// Fan Control Pins
const uint8_t VEL1 = 12;    // GPIO for Speed 1 control
const uint8_t VEL2 = 13;    // GPIO for Speed 2 control
const uint8_t VEL3 = 15;    // GPIO for Speed 3 control
const uint8_t SENSOR = 14;  // GPIO for temperature sensor

// Communication Parameters
unsigned long timerDelay = 500;  // Delay between messages (ms)
int varVelIncoming;              // Received fan speed setting
float tempIncoming;              // Received temperature (unused)
float temp = 0;                  // Current temperature reading
bool automaticIncoming;          // Automatic mode flag from controller
bool status1Incoming;            // Status flag 1 from controller
bool status2Incoming;            // Status flag 2 from controller
bool flagIncoming;               // General flag from controller
bool flag2Incoming;              // Secondary flag from controller
int IncomingReadingsId;          // Received message ID

// Temperature Sensor Setup
OneWire ourWire(SENSOR);              // OneWire bus on specified pin
DallasTemperature sensors(&ourWire);  // Dallas Temperature instance

/**
 * @enum PairingStatus
 * @brief States for ESP-NOW pairing process
 */
enum PairingStatus {
  PAIR_REQUEST,    // Ready to send pairing request
  PAIR_REQUESTED,  // Pairing request sent, waiting response
  PAIR_PAIRED      // Successfully paired
};
PairingStatus pairingStatus = PAIR_REQUEST;  // Current pairing state

/**
 * @enum MessageType
 * @brief Types of ESP-NOW messages
 */
enum MessageType {
  PAIRING,  // Pairing control message
  DATA      // Regular data message
};
MessageType messageType;

/**
 * @struct struct_message
 * @brief Data structure for ESP-NOW communication
 */
typedef struct struct_message {
  uint8_t msgType;           // Message type (PAIRING or DATA)
  uint8_t id;                // Device identifier
  bool automatic;            // Automatic mode flag
  bool lampBedRoomIncoming;  // Light status (unused in this device)
  bool status1;              // Status flag 1
  bool status2;              // Status flag 2
  bool flag;                 // General purpose flag
  bool flag2;                // Secondary flag
  uint8_t varVel;            // Fan speed setting (0-3)
  float temp;                // Temperature reading
  unsigned int readingId;    // Message counter
} struct_message;

/**
 * @struct struct_pairing
 * @brief Structure for pairing messages
 */
typedef struct struct_pairing {
  uint8_t msgType;     // Always PAIRING
  uint8_t id;          // Device ID
  uint8_t macAddr[6];  // MAC address
  uint8_t channel;     // WiFi channel
} struct_pairing;

// Data instances
struct_message incomingReadings;  // Received data
struct_message myData;            // Data to send
struct_pairing pairingData;       // Pairing information


unsigned long start;  // Pairing start time

/**
 * @brief Callback when data is sent via ESP-NOW
 * @param mac_addr Recipient MAC address
 * @param sendStatus Delivery status (0 = success)
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  sendStatus == 0 ? Serial.println("Delivery success") : Serial.println("Delivery fail");
}

/**
 * @brief Prints MAC address in human-readable format
 * @param mac_addr Pointer to MAC address bytes
 */
void printMAC(const uint8_t *mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

/**
 * @brief Callback when data is received via ESP-NOW
 * @param mac Sender MAC address
 * @param incomingData Received data buffer
 * @param len Data length
 */
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  Serial.print("Size of message : ");
  Serial.print(len);
  Serial.print(" from ");
  printMAC(mac);
  Serial.println();
  uint8_t type = incomingData[0];  // First byte indicates message type
  switch (type) {
    case DATA:  // Regular data message
      memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
      Serial.print(len);
      Serial.print(" Data bytes received from: ");
      printMAC(mac);
      Serial.println();
      // Update local variables with received data
      varVelIncoming = incomingReadings.varVel;
      tempIncoming = incomingReadings.temp;
      automaticIncoming = incomingReadings.automatic;
      status1Incoming = incomingReadings.status1;
      status2Incoming = incomingReadings.status2;
      flagIncoming = incomingReadings.flag;
      flag2Incoming = incomingReadings.flag2;
      IncomingReadingsId = incomingReadings.id;
      break;

    case PAIRING:  // Pairing message
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      if (pairingData.id == 0) {  // Message from server
        Serial.print("Pairing done for ");
        printMAC(pairingData.macAddr);
        Serial.print(" on channel ");
        Serial.print(pairingData.channel);  // channel used by the server
        Serial.print(" in ");
        Serial.print(millis() - start);
        Serial.println("ms");

        // Clean up existing peers
        esp_now_del_peer(pairingData.macAddr);
        esp_now_del_peer(mac);

        // Add new peer
        esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); 
        pairingStatus = PAIR_PAIRED;
      }
      break;
  }
}

/**
 * @brief Reads and parses the device's MAC address
 */
void readGetMacAddress() {
  String val = WiFi.macAddress();
  Serial.println(val);
  char *endPtr;

  // Parse MAC address string into bytes
  clientMacAddress[0] = strtol(val.c_str(), &endPtr, 16); 
  for (int i = 1; (*endPtr) && (i < 6); i++) {
    clientMacAddress[i] = strtol(endPtr + 1, &endPtr, 16);  
  }

  // Print parsed MAC address
  for (int i = 0; i < 6; i++) {
    Serial.print(clientMacAddress[i], HEX);
    if (i != 5)
      Serial.print(F(":"));
  }
}

/**
 * @brief Handles the automatic pairing process
 * @return Current pairing status
 */
PairingStatus autoPairing() {
  switch (pairingStatus) {
    case PAIR_REQUEST:
      Serial.print("Pairing request on channel ");
      Serial.println(channel);

      // Reinitialize ESP-NOW
      esp_now_deinit();
      WiFi.disconnect();
      WiFi.mode(WIFI_STA);

      // Set WiFi channel
      wifi_promiscuous_enable(1);
      wifi_set_channel(channel);
      wifi_promiscuous_enable(0);

      // Init ESP-NOW
      if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
      }
      esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

      // Register callbacks
      esp_now_register_send_cb(OnDataSent);
      esp_now_register_recv_cb(OnDataRecv);

      // Prepare pairing data
      pairingData.id = BOARD_ID;
      pairingData.channel = channel;
      memcpy(pairingData.macAddr, clientMacAddress, 6);

      previousMillis = millis();
      // Send pairing request
      esp_now_send(broadcastAddressX, (uint8_t *)&pairingData, sizeof(pairingData));
      pairingStatus = PAIR_REQUESTED;
      break;

    case PAIR_REQUESTED:
      // Handle pairing timeout
      currentMillis = millis();
      if (currentMillis - previousMillis > 1000) {
        previousMillis = currentMillis;
        // Try next channel
        channel++;
        if (channel > MAX_CHANNEL) {
          channel = 0;
        }
        pairingStatus = PAIR_REQUEST;
      }
      break;

    case PAIR_PAIRED:
      // Pairing complete
      break;
  }
  return pairingStatus;
}

/**
 * @brief Displays temperature information on OLED screen
 */
void showInfoScreen() {
  oled.setCursor(35, 25);
  oled.print("TEMP: ");
  oled.setCursor(18, 45);
  oled.print(temp);
  oled.print("");
  oled.cp437(true);
  // Write degree symbol
  oled.write(248);
  oled.print("C");
  oled.display();
}
void setup() {
  // Initialize serial port
  Serial.begin(115200);
  Serial.println();

  // Configure GPIO pins
  pinMode(VEL1, OUTPUT);
  pinMode(VEL2, OUTPUT);
  pinMode(VEL3, OUTPUT);

  // Initialize temperature sensor
  sensors.begin();

  // Initialize I2C and OLED
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  readGetMacAddress();
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW role and callbacks
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  pairingData.id = 1; // Initialize pairing ID

  // Configure OLED display
  oled.setTextColor(WHITE); 
  oled.setCursor(0, 0);
  oled.setTextSize(2);                
  oled.print("ELECTRONIC PROJECTS");  
  oled.display();
  delay(1500);
  oled.clearDisplay();
  delay(1500);
}

void loop() {
  static unsigned long lastEventTime1 = millis();
  static const unsigned long EVENT_INTERVAL_MS1 = 200;

  // Main control loop timing
  if ((millis() - lastTime) > timerDelay) {
    oled.clearDisplay();

    // Read temperature
    sensors.requestTemperatures();
    temp = sensors.getTempCByIndex(0);

    // Automatic mode control
    if (automaticIncoming == true) {
      if (temp >= 25 && temp <= 28) { // Low temperature - Speed 1
        digitalWrite(VEL2, LOW);
        digitalWrite(VEL3, LOW);
        if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
          digitalWrite(VEL1, HIGH);
          lastEventTime1 = millis();
        }
        varVelIncoming = 1;
      }
      if (temp >= 29 && temp <= 31) { // Medium temperature - Speed 2
        digitalWrite(VEL1, LOW);
        digitalWrite(VEL3, LOW);
        if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
          digitalWrite(VEL2, HIGH);
          lastEventTime1 = millis();
        }
        varVelIncoming = 2;
      }
      if (temp > 31) { // High temperature - Speed 3
        digitalWrite(VEL1, LOW);
        digitalWrite(VEL2, LOW);
        if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
          digitalWrite(VEL3, HIGH);
          lastEventTime1 = millis();
        }
        varVelIncoming = 3;
      }
    } else { // Manual mode control
      switch (varVelIncoming) {
        case 1: // Speed 1
          digitalWrite(VEL2, LOW);
          digitalWrite(VEL3, LOW);
          if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
            digitalWrite(VEL1, HIGH);
            lastEventTime1 = millis();
          }
          break;
        case 2: // Speed 2
          digitalWrite(VEL1, LOW);
          digitalWrite(VEL3, LOW);
          if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
            digitalWrite(VEL2, HIGH);
            lastEventTime1 = millis();
          }
          break;
        case 3: // Speed 3
          digitalWrite(VEL2, LOW);
          digitalWrite(VEL1, LOW);
          if ((millis() - lastEventTime1) > EVENT_INTERVAL_MS1) {
            digitalWrite(VEL3, HIGH);
            lastEventTime1 = millis();
          }
          break;
        default:  // Off
          digitalWrite(VEL1, LOW);
          digitalWrite(VEL2, LOW);
          digitalWrite(VEL3, LOW);
      }
    }

    // Handle communication if paired
    if (autoPairing() == PAIR_PAIRED) {
      static unsigned long lastEventTime = millis();
      static const unsigned long EVENT_INTERVAL_MS = 1000;


      if ((millis() - lastEventTime) > EVENT_INTERVAL_MS) {
        Serial.print(".");
        // Prepare data to send
        myData.msgType = DATA;
        myData.id = 1;
        myData.status1 = status1Incoming;
        myData.status2 = status2Incoming;
        myData.flag2 = flag2Incoming;
        myData.automatic = automaticIncoming;
        myData.temp = temp;
        myData.varVel = varVelIncoming;
        myData.readingId = readingId++;

        // Send data if requested
        if (flagIncoming == false || status1Incoming == true) {
          myData.flag = true;
          esp_now_send(pairingData.macAddr, (uint8_t *)&myData, sizeof(myData));
          flagIncoming = true;
          status1Incoming = false;
        }
        lastEventTime = millis();
      }
    }

    // Update display
    showInfoScreen();
    lastTime = millis();
  }
}
