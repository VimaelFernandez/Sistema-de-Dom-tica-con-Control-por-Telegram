/**
 * @file ESP8266_Light_Controller.ino
 * @brief ESP8266-based Smart Light Controller with Physical and Remote Control
 * 
 * Features:
 * - Hybrid control (physical switch + remote commands)
 * - ESP-NOW wireless communication
 * - Automatic channel scanning and pairing
 * - Priority-based control logic
 * 
 * Hardware Requirements:
 * - ESP8266 microcontroller
 * - Physical switch/button (GPIO2)
 * - Relay or transistor for light control (GPIO0)
 * 
 * Communication Protocol:
 * - ESP-NOW for low-latency peer-to-peer communication
 * - Automatic channel hopping during pairing
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

// =============== DEVICE CONFIGURATION ===============

#define BOARD_ID 2                     // Unique identifier for this device
#define MAX_CHANNEL 13                 // Maximum WiFi channel (13 for EU, 11 for NA)

// Network Parameters
uint8_t channel = 1;                   // Starting WiFi channel for pairing
int readingId = 0;                     // Message counter
uint8_t broadcastAddressX[] = {0x40, 0x22, 0xD8, 0x77, 0x83, 0x8D}; // Controller MAC

// =============== HARDWARE PINOUT ===============
const uint8_t lampOut = 0;             // GPIO0 - Light control output
const uint8_t lampInput = 2;           // GPIO2 - Physical switch input

// =============== CONTROL VARIABLES ===============

unsigned long previousMillis = 0;      // Timing for pairing
unsigned long currentMillis = millis();
unsigned long lastTime = 0;            // Last update time
const unsigned long timerDelay = 500;  // Main control loop delay (ms)

// State Variables
int valueRead = 0;
bool lampBedRoomIncoming = false;      // Remote light state
bool status1Incoming = false;          // Status flag 1 from controller
bool status2Incoming = false;          // Status flag 2 from controller
bool flagIncoming = false;             // General purpose flag
bool flag2Incoming = false;            // Secondary flag
uint8_t clientMacAddress[6];
int varVelIncoming;
float tempIncoming;
bool automaticIncoming;
int IncomingReadingsId;

/**
 * @enum PairingStatus
 * @brief States for the pairing process
 */
enum PairingStatus{
  PAIR_REQUEST,     // Ready to initiate pairing
  PAIR_REQUESTED,   // Pairing request sent
  PAIR_PAIRED       // Successfully paired
};
PairingStatus pairingStatus = PAIR_REQUEST;

/**
 * @enum MessageType
 * @brief ESP-NOW message types
 */
enum MessageType{
  PAIRING,         // Pairing control messages
  DATA             // Operational data messages
};
MessageType messageType;

/**
 * @struct struct_message
 * @brief Data structure for operational messages
 */
typedef struct struct_message {
  uint8_t msgType;      // Message type (PAIRING/DATA)
  uint8_t id;           // Device ID
  bool automatic;       // Automatic mode flag (unused in this implementation)
  bool lampBedRoom;     // Light state command
  bool status1;         // Status flag 1
  bool status2;         // Status flag 2
  bool flag;            // General purpose flag
  bool flag2;           // Secondary flag
  uint8_t varVel;       // Unused in light controller
  float temp;           // Unused in light controller
  unsigned int readingId; // Message counter
} struct_message;

/**
 * @struct struct_pairing
 * @brief Data structure for pairing messages
 */
typedef struct struct_pairing {
  uint8_t msgType;     // Always PAIRING
  uint8_t id;          // Device ID
  uint8_t macAddr[6];  // MAC address
  uint8_t channel;     // WiFi channel
} struct_pairing;

// Data instances
struct_message incomingReadings;      // Incoming messages
struct_message myData;               // Outgoing messages
struct_pairing pairingData;          // Pairing information

// =============== COMMUNICATION CALLBACKS ===============

/**
 * @brief Callback when data is sent
 * @param mac_addr Recipient MAC
 * @param sendStatus Delivery status (0=success)
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(sendStatus == 0 ? "Delivery success" : "Delivery fail");
}

/**
 * @brief Prints MAC address in human-readable format
 * @param mac_addr Pointer to 6-byte MAC address
 */
void printMAC(const uint8_t *mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

/**
 * @brief Callback when data is received
 * @param mac Sender MAC
 * @param incomingData Received data
 * @param len Data length
 */
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  Serial.print("Size of message : ");
  Serial.print(len);
  Serial.print(" from ");
  printMAC(mac);
  Serial.println();

  uint8_t type = incomingData[0]; // First byte = message type
  switch (type)
  {
  case DATA:
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    Serial.print(len);
    Serial.print(" Data bytes received from: ");
    printMAC(mac);
    Serial.println();
    varVelIncoming = incomingReadings.varVel;
    tempIncoming = incomingReadings.temp;
    automaticIncoming = incomingReadings.automatic;
    lampBedRoomIncoming = incomingReadings.lampBedRoom;
    status1Incoming = incomingReadings.status1;
    status2Incoming = incomingReadings.status2;
    flagIncoming = incomingReadings.flag;
    flag2Incoming = incomingReadings.flag2;
    IncomingReadingsId = incomingReadings.id;
    break;

  case PAIRING:
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    Serial.println("Recibo algo?");
    if (pairingData.id == 0){ // Message from controller
      esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list
      pairingStatus = PAIR_PAIRED;// set the pairing status
    }
    break;
  }
}

// =============== CORE FUNCTIONS ===============

/**
 * @brief Reads and prints device MAC address
 */
void readGetMacAddress()
{
  String val = WiFi.macAddress();
  char *endPtr;

  // Parse MAC string into bytes
  clientMacAddress[0] = strtol(val.c_str(), &endPtr, 16); 
  for (int i = 1; (*endPtr) && (i < 6); i++)
  {
    clientMacAddress[i] = strtol(endPtr + 1, &endPtr, 16);
  }
  // Show the client MAC
  for (int i = 0; i < 6; i++)
  {
    Serial.print(clientMacAddress[i], HEX);
    if (i != 5)
      Serial.print(F(":"));
  }
  Serial.println();
}

/**
 * @brief Handles automatic pairing process
 * @return Current pairing status
 */
PairingStatus autoPairing()
{
  switch (pairingStatus)
  {
  case PAIR_REQUEST:
    Serial.print("Pairing request on channel ");
    Serial.println(channel);

    // Initialize ESP-NOW on current channel
    esp_now_deinit();
    wifi_set_channel(channel);

    // Init ESP-NOW
    if (esp_now_init() != 0)
    {
      Serial.println("Error initializing ESP-NOW");
    }

    // Prepare pairing data
    pairingData.id = BOARD_ID;
    pairingData.channel = channel;
    memcpy(pairingData.macAddr, clientMacAddress, 6);

    // Send pairing request
    esp_now_send(broadcastAddressX, (uint8_t *)&pairingData, sizeof(pairingData));
    pairingStatus = PAIR_REQUESTED;
    previousMillis = millis();
    Serial.println("Envío para emparentar");
    break;

  case PAIR_REQUESTED:
    // Channel hopping timeout 
    currentMillis = millis();
    if (currentMillis - previousMillis > 1000){
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel++;
      if (channel > MAX_CHANNEL)
      {
        channel = 0;
      }
      pairingStatus = PAIR_REQUEST;
    }
    break;

  case PAIR_PAIRED:
    // Paired - normal operation
    break;
  }
  return pairingStatus;
}

// =============== ARDUINO CORE FUNCTIONS ===============
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  // Configure I/O
  pinMode(lampOut, OUTPUT);
  pinMode(lampInput, INPUT);

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  readGetMacAddress();
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void loop()
{
  if ((millis() - lastTime) > timerDelay)
  {
    // Hybrid control logic - physical switch has priority
    valueRead = digitalRead(lampInput);
    if (valueRead == HIGH)
    {
      digitalWrite(lampOut, LOW); // Physical switch off
    }
    else{
    // Remote control when switch not activated
      digitalWrite(lampOut, lampBedRoomIncoming ? HIGH : LOW);
    }

    // Handle communication if paired
    if (autoPairing() == PAIR_PAIRED)
    {
      static unsigned long lastEventTime = millis();
      if ((millis() - lastEventTime) > 1000){
        // Prepare status message 
        Serial.print(".");
        myData.msgType = DATA;
        myData.id = 2;
        myData.lampBedRoom = lampBedRoomIncoming;
        myData.status1 = status1Incoming;
        myData.status2 = status2Incoming;
        myData.flag = flagIncoming;
        myData.automatic = automaticIncoming;
        myData.temp = tempIncoming;
        myData.varVel = varVelIncoming;
        myData.readingId = readingId++;

        // Send if requested
        if (flag2Incoming == false || status2Incoming == true)
        {
          myData.flag2 = true;
          esp_now_send(pairingData.macAddr, (uint8_t *)&myData, sizeof(myData));
          flag2Incoming = true;
          status2Incoming = false;
        }
        lastEventTime = millis();
      }
    }
    lastTime = millis();
  }
}
