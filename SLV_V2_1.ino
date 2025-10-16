/*
 * ESP32-S3 Fish Tank Slave Control System v2.1
 * Individual tank controller that communicates with master via ESP-NOW
 * 
 * CHANGES in v2.1:
 * - Commands now call OpenDoor/CloseDoor directly (REF20/REF21)
 * - Timer expiry calls OpenDoor instead of open_slot
 * 
 * Built-in ESP32 Libraries Used:
 * - WiFi, esp_now, esp_sleep, esp_wifi
 * 
 * Folder Structure:
 * - Save as: SLV_V2_1/SLV_V2_1.ino
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// ===== CONFIGURATION - MODIFY FOR EACH DEVICE =====
const String TANK_ID_STRING = "Tank4";           // Set this to Tank1, Tank2, etc.
const int TANK_ID_NUMBER = 4;                    // Numeric ID (1-25) matching string above

// Master WiFi SSID to scan for channel detection
const char* masterSSID = "BHN-Guest";

// Hardcoded master MAC address (UPDATE WITH ACTUAL MASTER MAC)
uint8_t masterMACAddress[6] = {0x94, 0xA9, 0x90, 0x17, 0x76, 0xD8};

// GPIO Pin Configuration
const int MOTOR_PWR_PIN = D2;                    // GPIO 05 (D2) - Motor power control
const int MOTOR_DIR_PIN = D10;                   // GPIO 21 (D10) - Motor direction control  
const int DOOR_SENSOR_PIN = D9;                  // GPIO 18 (D9) - Door open sensor
const int BATTERY_ADC_PIN = A3;                  // GPIO 22 (A3) - Battery voltage ADC v2.5 used to be 22, now A3
const int MOTOR_PWM_VALUE = 128;                 // PWM value (50% = 128/255)
const int LED_READY = 13;                        // this is ON when the unit is ready or normally powered up, goes off temporary if in SLEEP mode

// Battery calculation constants (12-bit ADC)
const float ADC_RESOLUTION = 4095.0;             // 12-bit ADC
const float ADC_VOLTAGE_REF = 3.3;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.1;

// ===== GLOBAL VARIABLES =====
int DoorStat = 0;                                // 0=unknown, 1=closed, 2=open
bool delayTimerActive = false;
unsigned long delayTimerStart = 0;
unsigned long delayTimerDuration = 0;

bool doorIsOpen = false;
int batteryPercentage = 0;
String ErrMess = "test error";                   // Error message variable
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 30000;
int detectedChannel = 0;

// ESP-NOW Message Structure (must match master)
typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

// ===== CHANNEL DETECTION =====

/**
 * Scan for master's SSID to detect WiFi channel
 * Inputs: None
 * Outputs: int - detected channel number, or 1 if not found
 */
int scanForMasterChannel() {
    Serial.println("DEBUG: Scanning for master SSID: " + String(masterSSID));
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    int numNetworks = WiFi.scanNetworks();
    Serial.println("DEBUG: Found " + String(numNetworks) + " networks");
    
    for (int i = 0; i < numNetworks; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid == masterSSID) {
            int channel = WiFi.channel(i);
            Serial.println("DEBUG: Found master SSID on channel " + String(channel));
            return channel;
        }
    }
    
    Serial.println("WARNING: Master SSID not found, defaulting to channel 1");
    return 1;  // Default to channel 1 if not found
}

// ===== HARDWARE INITIALIZATION =====

/**
 * Initialize GPIO pins for motor control, sensors, and LED
 * Inputs: None
 * Outputs: None (configures pins)
 */
void initGPIO() {
    pinMode(MOTOR_PWR_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, LOW);
    digitalWrite(MOTOR_PWR_PIN, LOW);

    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BATTERY_ADC_PIN, INPUT);

    pinMode(LED_READY, OUTPUT);
    digitalWrite(LED_READY, HIGH);
    
    Serial.println("DEBUG: GPIO initialized");
}

/**
 * Add master device as ESP-NOW peer
 * Inputs: None
 * Outputs: None (adds peer to ESP-NOW)
 */
void addESPNowPeer() {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMACAddress, 6);
    peerInfo.channel = detectedChannel;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("DEBUG: Master peer added successfully on channel " + String(detectedChannel));
    } else {
        Serial.println("ERROR: Failed to add master peer");
    }
}

/**
 * Initialize ESP-NOW protocol
 * Inputs: None
 * Outputs: None (initializes ESP-NOW and sends initial status)
 */
void initESPNow() {
    // Scan for master channel
    detectedChannel = scanForMasterChannel();
    
    WiFi.mode(WIFI_STA);
    
    // Force to detected channel
    esp_wifi_set_channel(detectedChannel, WIFI_SECOND_CHAN_NONE);
    Serial.println("Channel number is: " + String(detectedChannel));
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW init failed");
        return;
    }
    
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);
    
    addESPNowPeer();
    
    Serial.println("Connected to ESP-NOW");
    
    delay(2000);
    sendMACAddress();
    delay(500);
    BattCalcEnergy();
    sendBatteryLevel();
    delay(500);
    sendDoorStatus();
}

// ===== ESP-NOW CALLBACKS =====

/**
 * Callback when ESP-NOW data is sent
 * Inputs:
 *   - mac_addr: uint8_t* - MAC address of recipient
 *   - status: esp_now_send_status_t - send status
 * Outputs: None
 */
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("DEBUG: ESP-NOW send SUCCESS");
    } else {
        Serial.println("WARNING: ESP-NOW send FAILED");
    }
}

/**
 * Callback when ESP-NOW data is received from master
 * Inputs:
 *   - mac: uint8_t* - MAC address of sender
 *   - incomingData: uint8_t* - received data
 *   - len: int - data length
 * Outputs: None (processes commands)
 */
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.println("DEBUG: RAW MESSAGE RECEIVED!");
    
    ESPNowMessage message;
    memcpy(&message, incomingData, sizeof(message));
    
    Serial.println("DEBUG: Tank ID in message: " + String(message.tankId));
    Serial.println("DEBUG: Command: " + String(message.command));
    
    if (message.tankId != TANK_ID_NUMBER) {
        Serial.println("DEBUG: Tank ID mismatch - ignoring message");
        return;
    }
    
    String command = String(message.command);
    
    // REF20: open_slot command calls OpenDoor
    if (command == "open_slot") {
        Serial.println("DEBUG: Executing open_slot command");
        OpenDoor();
    } 
    // REF21: close_slot command calls CloseDoor
    else if (command == "close_slot") {
        Serial.println("DEBUG: Executing close_slot command");
        CloseDoor();
    } 
    else if (command == "start_delay") {
        int hours = message.value;
        Serial.println("Timer is activated " + String(hours) + " hours");
        TimedOpening(hours);
    } 
    else if (command == "cancel_delay") {
        Serial.println("DEBUG: Canceling delay timer");
        cancelDelayTimer();
    } 
    else if (command == "go_sleep") {
        int hours = message.value;
        Serial.println("Sleep mode activated " + String(hours) + " hours");
        LowPowerSleep(hours);
    } 
    else if (command == "ping_test") {
        Serial.println("DEBUG: Responding to ping");
        sendHeartbeat();
        BattCalcEnergy();
        sendBatteryLevel();
        sendDoorStatus();
        sendMACAddress();
    } 
    else {
        Serial.println("WARNING: Unknown command: " + command);
    }
}

// ===== MOTOR CONTROL FUNCTIONS =====

/**
 * OpenDoor - Open the tank door (REF20)
 * Inputs: None
 * Outputs: None (controls motor and updates door status)
 */
void OpenDoor() {
    Serial.println("----------------------------- Running the door open function -----------------------------");
    
    unsigned long startTime = millis(); // set start to millis()
    const unsigned long MOTOR_TIMEOUT = 6000; // set 6 second timeout

    digitalWrite(MOTOR_DIR_PIN, HIGH); // motor to OPEN
    digitalWrite(MOTOR_PWR_PIN, HIGH); // motor ON

    doorIsOpen = readDoorSensor(); // read the door status

    // this seems fixed now
    while (!doorIsOpen && (millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
        doorIsOpen = readDoorSensor();
    }

    digitalWrite(MOTOR_PWR_PIN, LOW); // motor OFF

    if (readDoorSensor()) {
        doorIsOpen = true;
        DoorStat = 2;
        Serial.println("Door is open");
        sendDoorStatus();
    } else {
        DoorStat = 0; // unknown state
        Serial.println("ERROR: Door open timeout");
        ReportErrors("Door open timeout");
    }
} // end of open door

/**
 * CloseDoor - Close the tank door (REF21)
 * Inputs: None
 * Outputs: None (controls motor and updates door status)
 */
void CloseDoor() {
    Serial.println("----------------------------- Running the door close function -----------------------------");
    
    digitalWrite(MOTOR_DIR_PIN, LOW); // motor to CLOSE
    digitalWrite(MOTOR_PWR_PIN, HIGH); // motor ON

    delay(3000); // allow 3 seconds for the motor to run, to get PAST the optical silver area
    
    unsigned long startTime = millis();
    const unsigned long MOTOR_TIMEOUT = 3000; // 3 second timeout

    doorIsOpen = readDoorSensor(); // read sensor
    
    // this seems fixed now
    while ((millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
    }
    
    digitalWrite(MOTOR_PWR_PIN, LOW); // motor OFF

    delay(200); // wait

    doorIsOpen = readDoorSensor(); // read the sensor
    
    if (!doorIsOpen) {
        DoorStat = 1; // set the door state 1
        Serial.println("Door is closed");
        sendDoorStatus();
    } else {
        DoorStat = 0; // unknown status
        Serial.println("ERROR: Door close timeout");
        ReportErrors("Door close timeout");
    }
}

// ===== SENSOR FUNCTIONS =====

/**
 * Read door sensor state
 * Inputs: None
 * Outputs: bool - true if door is open, false if closed
 */
bool readDoorSensor() {
    return digitalRead(DOOR_SENSOR_PIN);
}

/**
 * BattCalcEnergy - Calculate battery energy percentage
 * Inputs: None
 * Outputs: None (prints ADC value, calculates percentage)
 */
void BattCalcEnergy() {
    int adcValue = analogRead(BATTERY_ADC_PIN);
    Serial.println("DEBUG: ADC value: " + String(adcValue));
    
    // Convert ADC to voltage (12-bit ADC)
    float adcVoltage = (adcValue / ADC_RESOLUTION) * ADC_VOLTAGE_REF;
    
    // Multiply by 2 to get actual battery voltage (voltage divider)
    float batteryVoltage = adcVoltage * 2.0;
    
    // Calculate percentage based on discharge curve
    float percentage = ((batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                       (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
    
    if (percentage > 100.0) percentage = 100.0;
    if (percentage < 0.0) percentage = 0.0;
    
    batteryPercentage = (int)percentage;
    
    Serial.println("DEBUG: Battery voltage: " + String(batteryVoltage) + "V");
    Serial.println("DEBUG: Battery percentage: " + String(batteryPercentage) + "%");
}

// ===== TIMER FUNCTIONS =====

/**
 * TimedOpening - Start delay timer for automatic door opening
 * Inputs:
 *   - hours: int - delay duration in hours (1-24)
 * Outputs: None (starts timer)
 */
void TimedOpening(int hours) {
    if (hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid delay hours: " + String(hours));
        ReportErrors("Invalid delay hours");
        return;
    }
    
    delayTimerActive = true;
    delayTimerStart = millis();
    delayTimerDuration = (unsigned long)hours * 3600000;
    
    Serial.println("DEBUG: Delay timer started for " + String(hours) + " hours");
}

/**
 * Cancel active delay timer
 * Inputs: None
 * Outputs: None (cancels timer)
 */
void cancelDelayTimer() {
    delayTimerActive = false;
    delayTimerStart = 0;
    delayTimerDuration = 0;
    
    Serial.println("DEBUG: Delay timer canceled");
}

/**
 * Check if delay timer has expired
 * Inputs: None
 * Outputs: None (executes OpenDoor if expired)
 */
void checkDelayTimer() {
    if (!delayTimerActive) return;
    
    unsigned long elapsed = millis() - delayTimerStart;
    
    if (elapsed >= delayTimerDuration) {
        delayTimerActive = false;
        Serial.println("DEBUG: Delay timer expired - executing timed open");
        Serial.println("Timer is activated 0 hours");
        OpenDoor();  // v2.1: Timer calls OpenDoor directly
        sendTimedOpenNotification();
    }
}

// ===== SLEEP FUNCTIONS =====

/**
 * LowPowerSleep - Put device into deep sleep mode
 * Inputs:
 *   - hours: int - sleep duration in hours (1-24)
 * Outputs: None (enters deep sleep)
 */
void LowPowerSleep(int hours) {
    digitalWrite(LED_READY, LOW); // LED OFF
    if (hours < 1 || hours > 24) {
        Serial.println("ERROR: Invalid sleep hours: " + String(hours));
        ReportErrors("Invalid sleep hours");
        return;
    }
    
    // Send final status before sleeping
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(100);
    
    Serial.println("DEBUG: Entering deep sleep for " + String(hours) + " hours");
    
    uint64_t sleepTime = (uint64_t)hours * 3600 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    esp_deep_sleep_start();
}

// ===== ESP-NOW MESSAGE FUNCTIONS =====

/**
 * ReportErrors - Send error message to master
 * Inputs:
 *   - errorMsg: String - error description
 * Outputs: None (sends ESP-NOW message)
 */
void ReportErrors(String errorMsg) {
    ErrMess = errorMsg;
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 2;
    message.value = 0;
    strncpy(message.message, errorMsg.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
    if (result != ESP_OK) {
        Serial.println("ERROR: Failed to send error message");
    }
}

/**
 * Send battery level to master
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendBatteryLevel() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 3;
    message.value = batteryPercentage;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Battery level sent: " + String(batteryPercentage) + "%");
    }
}

/**
 * Send MAC address to master
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendMACAddress() {
    String macAddress = WiFi.macAddress();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 4;
    message.value = 0;
    strncpy(message.message, macAddress.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: MAC address sent: " + macAddress);
    }
}

/**
 * Send door status to master
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendDoorStatus() {
    doorIsOpen = readDoorSensor();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 5;
    message.value = doorIsOpen ? 1 : 0;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Door status sent: " + String(doorIsOpen ? "OPEN" : "CLOSED"));
    }
}

/**
 * Send heartbeat to master
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendHeartbeat() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 6;
    message.value = 0;
    strcpy(message.message, "heartbeat");
    
    esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
}

/**
 * Send timed open notification to master
 * Inputs: None
 * Outputs: None (sends ESP-NOW message)
 */
void sendTimedOpenNotification() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 1;
    message.value = 1;
    strcpy(message.message, "timed_open_executed");
    
    esp_now_send(masterMACAddress, (uint8_t *)&message, sizeof(message));
    Serial.println("DEBUG: Timed open notification sent");
}

// ===== MAIN SETUP AND LOOP =====

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== Fish Tank Slave v2.0 Starting ===");
    Serial.println("Tank ID: " + TANK_ID_STRING + " (" + String(TANK_ID_NUMBER) + ")");
    
    String macAddress = WiFi.macAddress();
    Serial.println("MAC address is: " + macAddress);
    
    // Set DoorStat to 0 on power up
    DoorStat = 0;
    
    initGPIO();
    initESPNow();
    
    // Read initial states
    doorIsOpen = readDoorSensor();
    BattCalcEnergy();
    
    if (doorIsOpen) {
        Serial.println("Door is open");
        DoorStat = 2;
    } else {
        Serial.println("Door is closed");
        DoorStat = 1;
    }
    
    Serial.println("=== Initialization complete ===");
}

void loop() {
    checkDelayTimer();
    
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
    }
    
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 120000) {
        BattCalcEnergy();
        sendBatteryLevel();
        sendDoorStatus();
        lastStatusUpdate = millis();
    }
    
    static bool lastDoorState = false;
    static unsigned long lastDoorCheck = 0;
    if (millis() - lastDoorCheck > 5000) {
        bool currentDoorState = readDoorSensor();
        if (currentDoorState != lastDoorState) {
            doorIsOpen = currentDoorState;
            
            if (doorIsOpen) {
                Serial.println("Door is open");
                DoorStat = 2;
            } else {
                Serial.println("Door is closed");
                DoorStat = 1;
            }
            
            sendDoorStatus();
        }
        lastDoorState = currentDoorState;
        lastDoorCheck = millis();
    }
    
    delay(100);
    digitalWrite(LED_READY, HIGH); // LED is normally lit for READY
}