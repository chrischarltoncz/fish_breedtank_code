/*
 * ESP32-S3 Fish Tank Secondary Control System v3.6
 * Individual tank controller that communicates with Primary via ESP-NOW
 *
 * CHANGES in v3.7
 * - LED D13 off during sleep
 * - fix variables being lost during sleep
 * 
 * CHANGES in v3.6:
 * - Removed GPIO wake button feature (wake on timer only)
 * - Simplified deep sleep configuration
 * - FIXED: Deep sleep now called from main loop instead of ISR context
 * 
 * CHANGES in v3.5:
 * - Fixed immediate wake from deep sleep issue
 * - Added proper ESP-NOW and WiFi shutdown before sleep
 * - Added RTC GPIO configuration for wake button
 * - Increased message transmission delays before sleep
 * 
 * CHANGES in v3.4:
 * - Fixed RTC memory validation with magic number and checksum
 * - Fixed sleep-after-opening feature with local flag
 * - Changed variable types to prevent overflow (max 5 days operation)
 * - Added configuration command handler for sleep-after-opening setting
 * 
 * CHANGES in v3.3:
 * - Added GPIO 17 (D8) wake button for manual wake from deep sleep (REMOVED in v3.6)
 * - Wake button uses internal pull-up, connect button between GPIO 17 and GND (REMOVED in v3.6)
 * - Added wake-up reason detection and logging in setup()
 * 
 * Built-in ESP32 Libraries Used:
 * - WiFi, esp_now, esp_sleep, esp_wifi
 * 
 * Compatible with ESP32 Arduino Core 2.x (tested on 2.0.18)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// ===== RTC MEMORY FOR DEEP SLEEP STATE =====
// FIXED v3.4: Added magic number and checksum for RTC memory validation
#define RTC_MAGIC 0xFEEDBEEF

RTC_DATA_ATTR uint32_t rtc_magic = 0;
RTC_DATA_ATTR bool rtc_timedOpenActive = false;
RTC_DATA_ATTR uint32_t rtc_remainingSeconds = 0;  // FIXED v3.4: Changed to uint32_t for clarity (5 days = 432000 sec fits easily)
RTC_DATA_ATTR bool rtc_sleepAfterOpening = false;  // FIXED: Moved to RTC memory to survive deep sleep
RTC_DATA_ATTR uint32_t rtc_checksum = 0;

// ===== CONFIGURATION - MODIFY FOR EACH DEVICE =====
const String TANK_ID_STRING = "Tank4";
const int TANK_ID_NUMBER = 4;

const char* primarySSID = "BHN-Guest";
uint8_t primaryMACAddress[6] = {0x94, 0xA9, 0x90, 0x17, 0x76, 0xD8};

volatile bool __probeWaiting = false;
volatile bool __probeSuccess = false;
bool __espNowReady = false;

// GPIO Pin Configuration
const int MOTOR_PWR_PIN = D2;
const int MOTOR_SLEEP_PIN = D7;
const int MOTOR_DIR_PIN = D10;
const int OPTO_SLEEP_PIN = D11;
const int DOOR_SENSOR_PIN = D9;
const int BATTERY_ADC_PIN = A3;
const int MOTOR_ADC_CURRENT_PIN = A1;
const int LED_READY = 13;

// Battery calculation constants (12-bit ADC)
const float ADC_RESOLUTION = 4095.0;
const float ADC_VOLTAGE_REF = 3.3;
const float BATTERY_MAX_VOLTAGE = 4.2;
const float BATTERY_MIN_VOLTAGE = 3.1;

// ===== GLOBAL VARIABLES =====
int DoorStat = 0;

bool doorIsOpen = false;
int batteryPercentage = 0;
String ErrMess = "test error";
uint32_t lastHeartbeat = 0;  // FIXED v3.4: Changed to uint32_t (millis() returns uint32_t)
const uint32_t HEARTBEAT_INTERVAL = 30000;  // FIXED v3.4: Changed to uint32_t
int detectedChannel = 0;

// FIXED: Flags for sleep operations - set in callbacks, executed in main loop
volatile bool pendingTimedSleep = false;
volatile int pendingTimedSleepHours = 0;
volatile bool pendingLowPowerSleep = false;
volatile int pendingLowPowerSleepHours = 0;

// motor current variable
int motorCurrentRun = 0;
int motorCurrentStall = 0;
int motorRunThresh = 1000;
int motorStallThresh = 1050;
int delayRun = 200;
int delayStall = 3000;
bool doorStallFlag = false;

typedef struct {
    int tankId;
    char command[32];
    int value;
    char message[128];
    int messageType;
} ESPNowMessage;

// Forward declarations
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len);

// ===== RTC MEMORY VALIDATION FUNCTIONS =====

/**
 * Calculate simple checksum for RTC memory validation
 * Input: None (uses RTC variables)
 * Output: uint32_t checksum value
 */
uint32_t calculateRTCChecksum() {
    uint32_t sum = 0;
    sum += rtc_magic;
    sum += (rtc_timedOpenActive ? 1 : 0);
    sum += rtc_remainingSeconds;
    sum += (rtc_sleepAfterOpening ? 1 : 0);  // FIXED: Include sleep-after-opening flag
    return sum;
}

/**
 * Validate and initialize RTC memory on wake from deep sleep
 * Input: None
 * Output: bool - true if RTC memory is valid, false if corrupted/first boot
 */
bool validateRTCMemory() {
    if (rtc_magic != RTC_MAGIC) {
        Serial.println("RTC: First boot or magic mismatch - initializing");
        rtc_magic = RTC_MAGIC;
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        rtc_sleepAfterOpening = false;  // FIXED: Initialize flag
        rtc_checksum = calculateRTCChecksum();
        return false;
    }
    
    uint32_t expectedChecksum = calculateRTCChecksum();
    if (rtc_checksum != expectedChecksum) {
        Serial.println("RTC: Checksum mismatch - memory corrupted, resetting");
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        rtc_sleepAfterOpening = false;  // FIXED: Reset flag on corruption
        rtc_checksum = calculateRTCChecksum();
        return false;
    }
    
    Serial.println("RTC: Memory validated successfully");
    return true;
}

/**
 * Update RTC checksum before entering deep sleep
 * Input: None
 * Output: None
 */
void updateRTCChecksum() {
    rtc_checksum = calculateRTCChecksum();
}

// ===== CHANNEL DETECTION =====

/**
 * Scan WiFi networks to find Primary's SSID and return its channel
 * Input: None (uses global primarySSID)
 * Output: int - WiFi channel number (1-13) or 1 if not found
 */
int scanForPrimaryChannel() {
    Serial.println("DEBUG: Scanning for primary SSID: " + String(primarySSID));
    
    for (int attempt = 0; attempt < 3; attempt++) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(100);
        
        int numNetworks = WiFi.scanNetworks();
        Serial.println("DEBUG: Scan attempt " + String(attempt + 1) + " found " + String(numNetworks) + " networks");
        
        for (int i = 0; i < numNetworks; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid == primarySSID) {
                int channel = WiFi.channel(i);
                Serial.println("DEBUG: Found primary SSID on channel " + String(channel));
                return channel;
            }
        }
        
        if (attempt < 2) {
            Serial.println("DEBUG: Primary not found, retrying in 2 seconds...");
            delay(2000);
        }
    }
    
    Serial.println("WARNING: Primary SSID not found after 3 attempts, defaulting to channel 1");
    return 1;
}

// ===== HARDWARE INITIALIZATION =====

/**
 * Initialize all GPIO pins and configure their modes
 * Input: None
 * Output: None
 */
void initGPIO() {
    pinMode(MOTOR_PWR_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_SLEEP_PIN, OUTPUT);
    pinMode(OPTO_SLEEP_PIN, OUTPUT);
    pinMode(MOTOR_ADC_CURRENT_PIN, INPUT);
    
    digitalWrite(MOTOR_DIR_PIN, LOW);
    digitalWrite(MOTOR_PWR_PIN, LOW);
    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);

    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BATTERY_ADC_PIN, INPUT);

    pinMode(LED_READY, OUTPUT);
    digitalWrite(LED_READY, HIGH);
    
    Serial.println("DEBUG: GPIO initialized");
}

/**
 * Add Primary device as ESP-NOW peer
 * Input: None (uses global primaryMACAddress and detectedChannel)
 * Output: None
 */
void addESPNowPeer() {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, primaryMACAddress, 6);
    peerInfo.channel = detectedChannel;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("DEBUG: Primary peer added successfully on channel " + String(detectedChannel));
    } else {
        Serial.println("ERROR: Failed to add primary peer");
    }
}

// ===== Channel probing helpers =====
struct __Candidate { int ch; int rssi; String bssid; };

static void __addProbePeer() {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, primaryMACAddress, 6);
    p.channel = 0;
    p.encrypt = false;
    if (esp_now_is_peer_exist(p.peer_addr)) esp_now_del_peer(p.peer_addr);
    esp_now_add_peer(&p);
}

static bool __tryChannel(int ch) {
    Serial.printf("Trying channel %d...\n", ch);
    esp_now_deinit();
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    delay(120);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed on this channel");
        return false;
    }
    
    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);
    __addProbePeer();

    const char* probe = "SLV probe";
    __probeWaiting = true;
    __probeSuccess = false;

    esp_err_t e = esp_now_send(primaryMACAddress, (const uint8_t*)probe, strlen(probe));
    if (e != ESP_OK) {
        Serial.printf("esp_now_send error=%d\n", e);
        esp_now_deinit();
        return false;
    }

    uint32_t t0 = millis();
    while (__probeWaiting && millis() - t0 < 400) {
        yield();
    }

    bool ok = __probeSuccess;
    Serial.printf("Probe result on ch %d: %s\n", ch, ok ? "SUCCESS" : "FAIL");

    if (!ok) {
        esp_now_deinit();
    } else {
        __espNowReady = true;
    }
    return ok;
}

static int __discoverPrimaryChannelViaProbe() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    int n = WiFi.scanNetworks();
    struct __Candidate cand[20]; 
    int cnt=0;
    for (int i=0; i<n && cnt<20; i++) {
        if (WiFi.SSID(i) == primarySSID) {
            int ch = WiFi.channel(i);
            int rssi = WiFi.RSSI(i);
            String bssid = WiFi.BSSIDstr(i);
            bool have=false;
            for (int k=0;k<cnt;k++) { 
                if (cand[k].ch==ch) { 
                    have=true; 
                    if (rssi>cand[k].rssi) {
                        cand[k].rssi=rssi; 
                        cand[k].bssid=bssid;
                    } 
                    break;
                } 
            }
            if (!have) { 
                cand[cnt].ch=ch; 
                cand[cnt].rssi=rssi; 
                cand[cnt].bssid=bssid; 
                cnt++; 
            }
        }
    }
    
    for (int a=0;a<cnt;a++) {
        for (int b=a+1;b<cnt;b++) {
            if (cand[b].rssi>cand[a].rssi) { 
                auto tmp=cand[a]; 
                cand[a]=cand[b]; 
                cand[b]=tmp; 
            }
        }
    }

    Serial.println("Probe candidates:");
    for (int i=0;i<cnt;i++) { 
        Serial.printf("  CH %d RSSI %d BSSID %s\n", cand[i].ch, cand[i].rssi, cand[i].bssid.c_str()); 
    }

    for (int i=0;i<cnt;i++) {
        if (__tryChannel(cand[i].ch)) return cand[i].ch;
    }

    int fallback[3] = {1,6,11};
    for (int i=0;i<3;i++) {
        if (__tryChannel(fallback[i])) return fallback[i];
    }

    return -1;
}

/**
 * Initialize ESP-NOW and discover Primary's channel
 * Input: None
 * Output: None
 * Note: WiFi must be in WIFI_STA mode before calling this function
 */
void initESPNow() {
    Serial.println("DEBUG: Starting ESP-NOW initialization...");
    
    detectedChannel = __discoverPrimaryChannelViaProbe();
    if (detectedChannel < 0) {
        Serial.println("ERROR: Could not discover primary channel via probe");
        return;
    }
    
    Serial.println("DEBUG: Primary found on channel " + String(detectedChannel));

    if (!__espNowReady) {
        if (esp_now_init() != ESP_OK) {
            Serial.println("ERROR: ESP-NOW init failed");
            return;
        }
    }

    esp_now_register_send_cb(onESPNowDataSent);
    esp_now_register_recv_cb(onESPNowDataReceived);

    if (!esp_now_is_peer_exist(primaryMACAddress)) addESPNowPeer();

    Serial.println("Connected to ESP-NOW");

    delay(3000);
    Serial.println("DEBUG: Sending initial connection messages...");
    sendMACAddress();
    delay(1500);
    BattCalcEnergy();
    sendBatteryLevel();
    delay(1500);
    sendDoorStatus();
    Serial.println("DEBUG: ESP-NOW initialization complete");
}

// ===== ESP-NOW CALLBACKS =====

/**
 * Callback when ESP-NOW message is sent
 * Input: mac_addr - recipient MAC address, status - send status
 * Output: None
 */
void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("DEBUG: ESP-NOW send SUCCESS");
    } else {
        Serial.println("WARNING: ESP-NOW send FAILED");
    }
    if (__probeWaiting) { 
        __probeSuccess = (status == ESP_NOW_SEND_SUCCESS); 
        __probeWaiting = false; 
    }
}

/**
 * Callback when ESP-NOW message is received from Primary
 * Input: mac - sender MAC address, incomingData - message data, len - message length
 * Output: None
 */
void onESPNowDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    Serial.printf("DEBUG: RAW MESSAGE RECEIVED! len=%d\n", len);

    ESPNowMessage message = {};
    size_t n = min((size_t)len, (size_t)sizeof(message));
    memcpy(&message, incomingData, n);

    message.command[sizeof(message.command) - 1] = '\0';
    message.message[sizeof(message.message) - 1] = '\0';

    Serial.println("DEBUG: Tank ID in message: " + String(message.tankId));
    Serial.println("DEBUG: Command: " + String(message.command));

    if (message.tankId != TANK_ID_NUMBER) {
        Serial.println("DEBUG: Tank ID mismatch - ignoring message");
        return;
    }

    String command(message.command);

    if (command == "open_slot") {
        Serial.println("DEBUG: Executing open_slot command");
        OpenDoor();

    } else if (command == "close_slot") {
        Serial.println("DEBUG: Executing close_slot command");
        CloseDoor();

    } else if (command == "start_delay") {
        // FIXED: Set flag instead of calling sleep directly from ISR
        Serial.println("DEBUG: Scheduling timed opening with sleep");
        pendingTimedSleepHours = message.value;
        pendingTimedSleep = true;
        
    } else if (command == "cancel_delay") {
        Serial.println("DEBUG: Executing cancel_delay command");
        cancelDelayTimer();
        
    } else if (command == "go_sleep") {
        // FIXED: Set flag instead of calling sleep directly from ISR
        Serial.println("DEBUG: Scheduling low power sleep");
        pendingLowPowerSleepHours = message.value;
        pendingLowPowerSleep = true;
        
    } else if (command == "set_sleep_after_open") {
        // FIXED: Use RTC variable and update checksum
        rtc_sleepAfterOpening = (message.value == 1);
        updateRTCChecksum();  // FIXED: Update checksum after changing RTC variable
        Serial.println("DEBUG: Sleep after opening set to: " + String(rtc_sleepAfterOpening ? "ENABLED" : "DISABLED"));

    } else {
        Serial.println("DEBUG: Unknown command received: " + command);
    }
}

// ===== MOTOR CONTROL FUNCTIONS =====

/**
 * Open the tank door/slot using motor control
 * Input: None
 * Output: None (updates doorIsOpen and DoorStat globals)
 */
void OpenDoor() {
    Serial.println("----------------------------- Running the door open function -----------------------------");
    Serial.println("DEBUG: Door status BEFORE opening: " + String(readDoorSensor() ? "OPEN" : "CLOSED"));

    digitalWrite(MOTOR_SLEEP_PIN, HIGH);
    digitalWrite(OPTO_SLEEP_PIN, HIGH);
    
    Serial.println("DEBUG: Motor driver and opto sensor powered ON");
    delay(100);
    
    uint32_t startTime = millis();  // FIXED v3.4: Changed to uint32_t
    const uint32_t MOTOR_TIMEOUT = 6000;  // FIXED v3.4: Changed to uint32_t

    digitalWrite(MOTOR_DIR_PIN, LOW);
    digitalWrite(MOTOR_PWR_PIN, HIGH);
    Serial.println("DEBUG: Motor running in OPEN direction");

    delay(delayRun);

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN);
    Serial.println("DEBUG: Motor running current: " + String(motorCurrentRun));

    doorIsOpen = readDoorSensor();
    Serial.println("DEBUG: Initial door sensor reading: " + String(doorIsOpen ? "OPEN" : "CLOSED"));

    while (!doorIsOpen && (millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
        doorIsOpen = readDoorSensor();
    }

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN);
    Serial.println("DEBUG: Motor stall current: " + String(motorCurrentStall));

    digitalWrite(MOTOR_PWR_PIN, LOW);
    Serial.println("DEBUG: Motor powered OFF");
    
    delay(100);

    if (readDoorSensor()) {
        doorIsOpen = true;
        DoorStat = 2;
        Serial.println("SUCCESS: Door is OPEN");
        sendDoorStatus();
        
        // FIXED: Use RTC variable for sleep-after-opening check
        if (rtc_sleepAfterOpening) {
            Serial.println("DEBUG: Sleep after opening is enabled - scheduling sleep");
            delay(500);  // Brief delay to ensure status is sent
            pendingLowPowerSleepHours = 1;  // Default 1 hour sleep
            pendingLowPowerSleep = true;
            // Sleep will be executed in main loop
        }
    } else {
        DoorStat = 0;
        uint32_t elapsedTime = millis() - startTime;
        Serial.println("ERROR: Door open timeout after " + String(elapsedTime) + "ms");
        Serial.println("ERROR: Final door sensor: " + String(readDoorSensor() ? "OPEN" : "CLOSED"));
        ReportErrors("Door open timeout");
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);
    Serial.println("DEBUG: Motor driver and opto sensor powered OFF");
    Serial.println("----------------------------- Door open function complete -----------------------------");
}

/**
 * Close the tank door/slot using motor control
 * Input: None
 * Output: None (updates doorIsOpen and DoorStat globals)
 */
void CloseDoor() {
    Serial.println("----------------------------- Running the door close function -----------------------------");

    digitalWrite(MOTOR_SLEEP_PIN, HIGH);
    digitalWrite(OPTO_SLEEP_PIN, HIGH);
    
    digitalWrite(MOTOR_DIR_PIN, HIGH);
    digitalWrite(MOTOR_PWR_PIN, HIGH);

    delay(delayRun);

    motorCurrentRun = analogRead(MOTOR_ADC_CURRENT_PIN);

    delay(delayStall);

    motorCurrentStall = analogRead(MOTOR_ADC_CURRENT_PIN);

    if(motorCurrentStall > motorStallThresh) doorStallFlag=true;
    if(motorCurrentStall < motorStallThresh) doorStallFlag=false;
    
    uint32_t startTime = millis();  // FIXED v3.4: Changed to uint32_t
    const uint32_t MOTOR_TIMEOUT = 3000;  // FIXED v3.4: Changed to uint32_t

    doorIsOpen = readDoorSensor();
    
    while ((millis() - startTime) < MOTOR_TIMEOUT) {
        delay(100);
    }
    
    digitalWrite(MOTOR_PWR_PIN, LOW);

    delay(200);

    doorIsOpen = readDoorSensor();
    
    if (!doorIsOpen && doorStallFlag==true) {
        DoorStat = 1;
        Serial.println("Door is closed");
        sendDoorStatus();
    } else {
        DoorStat = 0;
        Serial.println("ERROR: Door close timeout");
        Serial.print("Door stall flag is: ");
        Serial.println(doorStallFlag);
        ReportErrors("Door close timeout");
    }

    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(OPTO_SLEEP_PIN, LOW);
}

// ===== SENSOR FUNCTIONS =====

/**
 * Read door sensor state
 * Input: None
 * Output: bool - true if door is open, false if closed
 */
bool readDoorSensor() {
    return digitalRead(DOOR_SENSOR_PIN);
}

/**
 * Calculate battery percentage from voltage using piecewise polynomial
 * Input: batteryVoltage - float voltage in volts
 * Output: float - battery percentage 0-100
 */
float calculateBatteryPercentage(float batteryVoltage) {
    float percentage = 0.0;
    
    if (batteryVoltage < 2.5) batteryVoltage = 2.5;
    if (batteryVoltage > 4.2) batteryVoltage = 4.2;

    batteryVoltage = batteryVoltage + 0.25;
    
    if (batteryVoltage >= 2.5 && batteryVoltage <= 3.0) {
        percentage = 22 * batteryVoltage - 49.7;
    }
    else if (batteryVoltage > 3.0 && batteryVoltage <= 3.6) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = -2566 + 2345*x + -714*x2 + 73.2*x3;
    }
    else if (batteryVoltage > 3.6 && batteryVoltage <= 3.85) {
        float x = batteryVoltage;
        float x2 = x * x;
        float x3 = x2 * x;
        percentage = 470219 + -377775*x + 101090*x2 + -9009*x3;
    }
    else if (batteryVoltage > 3.85 && batteryVoltage <= 4.25) {
        float x = batteryVoltage;
        float x2 = x * x;
        percentage = -2426 + 1193*x + -141*x2;
    }
    
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    return percentage;
}

/**
 * Calculate battery energy level and update global batteryPercentage
 * Input: None
 * Output: None (updates batteryPercentage global)
 */
void BattCalcEnergy() {
    int adcValue = analogRead(BATTERY_ADC_PIN);
    Serial.println("DEBUG: ADC value: " + String(adcValue));
    
    float adcVoltage = (adcValue / ADC_RESOLUTION) * ADC_VOLTAGE_REF;
    float batteryVoltage = adcVoltage * 2.0;
    float percentage = calculateBatteryPercentage(batteryVoltage);
    
    batteryPercentage = (int)percentage;
    
    Serial.println("DEBUG: Battery voltage: " + String(batteryVoltage) + "V");
    Serial.println("DEBUG: Battery percentage: " + String(batteryPercentage) + "%");
}

// ===== TIMER FUNCTIONS WITH SLEEP =====

/**
 * Start timed opening with deep sleep (wakes 10 min before opening)
 * Input: hours - int hours to delay (1-120 for 5 days max)
 * Output: None (enters deep sleep, never returns)
 * NOTE: This function should only be called from main loop, not from ISR
 */
void TimedOpeningWithSleep(int hours) {
    // FIXED v3.4: Limit to 120 hours (5 days) as per requirements
    if (hours < 1 || hours > 120) {
        Serial.println("ERROR: Invalid delay hours (must be 1-120): " + String(hours));
        ReportErrors("Invalid delay hours");
        return;
    }
    
    uint32_t totalSeconds = (uint32_t)hours * 3600;  // FIXED v3.4: Changed to uint32_t
    const uint32_t TEN_MINUTES = 600;  // FIXED v3.4: Changed to uint32_t
    
    uint32_t sleepSeconds = totalSeconds - TEN_MINUTES;  // FIXED v3.4: Changed to uint32_t
    
    Serial.println("DEBUG: Timed open with sleep initiated");
    Serial.println("  Total time: " + String(hours) + " hours (" + String(totalSeconds) + " seconds)");
    Serial.println("  Sleep time: " + String(sleepSeconds) + " seconds (" + String(sleepSeconds/60) + " minutes)");
    Serial.println("  Wake time: 10 minutes before opening");
    
    rtc_timedOpenActive = true;
    rtc_remainingSeconds = TEN_MINUTES;
    updateRTCChecksum();  // FIXED v3.4: Update checksum before sleep
    
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(500);  // FIXED v3.5: Increased delay to ensure messages are sent
    
    // FIXED v3.5: Disable ESP-NOW and WiFi before deep sleep to prevent wake issues
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    // FIXED: Turn OFF LED and HOLD its state during deep sleep
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);  // Hold GPIO state during sleep
    gpio_deep_sleep_hold_en();  // Enable hold for all GPIOs during deep sleep
    
    Serial.println("DEBUG: LED turned OFF and GPIO hold enabled");
    Serial.println("DEBUG: Entering deep sleep for " + String(sleepSeconds) + " seconds");
    Serial.println("DEBUG: Wake source: Timer only");
    Serial.flush();
    delay(100);  // FIXED v3.5: Allow serial to finish before sleep
    
    uint64_t sleepTime = (uint64_t)sleepSeconds * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

/**
 * Wrapper function for timed opening (for compatibility)
 * Input: hours - int hours to delay
 * Output: None
 */
void TimedOpening(int hours) {
    TimedOpeningWithSleep(hours);
}

/**
 * Cancel the timed opening timer
 * Input: None
 * Output: None
 */
void cancelDelayTimer() {
    rtc_timedOpenActive = false;
    rtc_remainingSeconds = 0;
    updateRTCChecksum();  // FIXED v3.4: Update checksum
    
    Serial.println("DEBUG: Timed open canceled");
    // Note: rtc_sleepAfterOpening is NOT cleared - it persists as a setting
}

// ===== SLEEP FUNCTIONS =====

/**
 * Enter low power deep sleep mode
 * Input: hours - int hours to sleep (1-120 for 5 days max)
 * Output: None (enters deep sleep, never returns)
 * NOTE: This function should only be called from main loop, not from ISR
 */
void LowPowerSleep(int hours) {
    // FIXED v3.4: Limit to 120 hours (5 days) as per requirements
    if (hours < 1 || hours > 120) {
        Serial.println("ERROR: Invalid sleep hours (must be 1-120): " + String(hours));
        ReportErrors("Invalid sleep hours");
        return;
    }
    
    BattCalcEnergy();
    sendBatteryLevel();
    sendDoorStatus();
    delay(500);  // FIXED v3.5: Increased delay to ensure messages are sent
    
    // FIXED v3.5: Disable ESP-NOW and WiFi before deep sleep to prevent wake issues
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    // FIXED: Turn OFF LED and HOLD its state during deep sleep
    digitalWrite(LED_READY, LOW);
    gpio_hold_en((gpio_num_t)LED_READY);  // Hold GPIO state during sleep
    gpio_deep_sleep_hold_en();  // Enable hold for all GPIOs during deep sleep
    
    Serial.println("DEBUG: LED turned OFF and GPIO hold enabled");
    Serial.println("DEBUG: Entering deep sleep for " + String(hours) + " hours");
    Serial.println("DEBUG: Wake source: Timer only");
    Serial.flush();
    delay(100);  // FIXED v3.5: Allow serial to finish before sleep
    
    uint64_t sleepTime = (uint64_t)hours * 3600 * 1000000;
    esp_sleep_enable_timer_wakeup(sleepTime);
    
    esp_deep_sleep_start();
}

// ===== ESP-NOW MESSAGE FUNCTIONS =====

/**
 * Send error message to Primary
 * Input: errorMsg - String error message
 * Output: None
 */
void ReportErrors(String errorMsg) {
    ErrMess = errorMsg;
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 2;
    message.value = 0;
    strncpy(message.message, errorMsg.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result != ESP_OK) {
        Serial.println("ERROR: Failed to send error message");
    }
}

/**
 * Send battery level to Primary
 * Input: None (uses global batteryPercentage)
 * Output: None
 */
void sendBatteryLevel() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 3;
    message.value = batteryPercentage;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Battery level sent: " + String(batteryPercentage) + "%");
    }
}

/**
 * Send MAC address to Primary
 * Input: None
 * Output: None
 */
void sendMACAddress() {
    String macAddress = WiFi.macAddress();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 4;
    message.value = 0;
    strncpy(message.message, macAddress.c_str(), sizeof(message.message) - 1);
    message.message[sizeof(message.message) - 1] = '\0';
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: MAC address sent: " + macAddress);
    }
}

/**
 * Send door open/closed status to Primary
 * Input: None (uses global doorIsOpen)
 * Output: None
 */
void sendDoorStatus() {
    doorIsOpen = readDoorSensor();
    
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 5;
    message.value = doorIsOpen ? 1 : 0;
    strcpy(message.message, "");
    
    esp_err_t result = esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("DEBUG: Door status sent: " + String(doorIsOpen ? "OPEN" : "CLOSED"));
    }
}

/**
 * Send heartbeat message to Primary
 * Input: None
 * Output: None
 */
void sendHeartbeat() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 6;
    message.value = 0;
    strcpy(message.message, "heartbeat");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
}

/**
 * Send notification that timed opening was executed
 * Input: None
 * Output: None
 */
void sendTimedOpenNotification() {
    ESPNowMessage message;
    message.tankId = TANK_ID_NUMBER;
    message.messageType = 1;
    message.value = 1;
    strcpy(message.message, "timed_open_executed");
    
    esp_now_send(primaryMACAddress, (uint8_t *)&message, sizeof(message));
    Serial.println("DEBUG: Timed open notification sent");
}

// ===== MAIN SETUP AND LOOP =====
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== Fish Tank Secondary v3.6 Starting ===");
    Serial.println("Tank ID: " + TANK_ID_STRING + " (" + String(TANK_ID_NUMBER) + ")");
    
    String macAddress = WiFi.macAddress();
    Serial.println("MAC address is: " + macAddress);
    
    // FIXED v3.4: Validate RTC memory on wake
    bool rtcValid = validateRTCMemory();
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("=== WOKE BY TIMER ===");
            // FIXED: Release GPIO hold after waking from deep sleep
            gpio_deep_sleep_hold_dis();
            gpio_hold_dis((gpio_num_t)LED_READY);
            Serial.println("DEBUG: GPIO hold released after wake");
            break;
        default:
            Serial.println("=== NORMAL POWER-ON (not from sleep) ===");
            break;
    }
    
    initGPIO();
    
    // FIXED v3.6: Ensure WiFi is properly initialized after deep sleep
    // WiFi was turned off before sleep, needs time to reinitialize
    WiFi.mode(WIFI_STA);
    delay(500);  // Allow WiFi radio to fully initialize
    Serial.println("DEBUG: WiFi reinitialized after sleep");
    
    if (rtc_timedOpenActive && rtcValid) {
        Serial.println("=== WAKING FROM TIMED OPEN SLEEP ===");
        Serial.println("Remaining time: " + String(rtc_remainingSeconds) + " seconds (10 minutes)");
        
        initESPNow();
        
        Serial.println("Waiting final 10 minutes before opening...");
        uint32_t waitStart = millis();
        uint32_t waitDuration = rtc_remainingSeconds * 1000;
        
        while (millis() - waitStart < waitDuration) {
            if (millis() - lastHeartbeat > 30000) {
                sendHeartbeat();
                BattCalcEnergy();
                sendBatteryLevel();
                lastHeartbeat = millis();
            }
            delay(1000);
        }
        
        Serial.println("=== EXECUTING TIMED OPENING ===");
        
        // FIXED: Clear timer flags BEFORE opening to prevent issues if sleep-after-opening is enabled
        rtc_timedOpenActive = false;
        rtc_remainingSeconds = 0;
        updateRTCChecksum();
        
        Serial.println("DEBUG: Timer flags cleared before opening");
        Serial.flush();
        
        // Send notification before opening (in case sleep-after-opening is enabled)
        sendTimedOpenNotification();
        delay(200);
        
        // FIXED: Temporarily disable sleep-after-opening for timed openings to ensure proper reporting
        bool tempSleepAfterOpening = rtc_sleepAfterOpening;
        rtc_sleepAfterOpening = false;
        updateRTCChecksum();
        
        OpenDoor();
        
        // Check final door status
        delay(500);
        doorIsOpen = readDoorSensor();
        if (doorIsOpen) {
            Serial.println("SUCCESS: Timed open completed - door is OPEN");
            DoorStat = 2;
        } else {
            Serial.println("ERROR: Timed open failed - door is still CLOSED");
            DoorStat = 0;
            ReportErrors("Timed open failed - door did not open");
        }
        
        sendDoorStatus();
        delay(500);
        
        // FIXED: Restore sleep-after-opening setting and apply it now if it was enabled
        if (tempSleepAfterOpening) {
            rtc_sleepAfterOpening = true;
            updateRTCChecksum();
            Serial.println("DEBUG: Sleep after opening was enabled - entering sleep mode now");
            Serial.flush();
            delay(100);
            LowPowerSleep(1);  // This function never returns
        }
        
        Serial.println("=== Timed opening complete, resuming normal operation ===");
        
    } else {
        Serial.println("=== Normal startup (not waking from timer) ===");
        DoorStat = 0;
        
        initESPNow();
        
        doorIsOpen = readDoorSensor();
        BattCalcEnergy();
        
        if (doorIsOpen) {
            Serial.println("Door is open");
            DoorStat = 2;
        } else {
            Serial.println("Door is closed");
            DoorStat = 1;
        }
    }
    
    Serial.println("=== Initialization complete ===");
    Serial.println("=== Secondary v3.6 Ready ===");
}

void loop() {
    static uint32_t lastPrimaryContact = 0;  // FIXED v3.4: Changed to uint32_t
    static bool initialContactMade = false;
    
    // FIXED: Check for pending sleep operations and execute them from main loop
    if (pendingTimedSleep) {
        pendingTimedSleep = false;
        int hours = pendingTimedSleepHours;
        Serial.println("DEBUG: Executing pending timed sleep from main loop");
        TimedOpeningWithSleep(hours);
        // Function never returns
    }
    
    if (pendingLowPowerSleep) {
        pendingLowPowerSleep = false;
        int hours = pendingLowPowerSleepHours;
        Serial.println("DEBUG: Executing pending low power sleep from main loop");
        LowPowerSleep(hours);
        // Function never returns
    }
    
    if (millis() - lastHeartbeat > 300000) {
        Serial.println("WARNING: No primary response for 5 minutes - re-scanning for channel");
        initESPNow();
        lastHeartbeat = millis();
    }
    
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
        initialContactMade = true;
    }
    
    static uint32_t lastStatusUpdate = 0;  // FIXED v3.4: Changed to uint32_t
    if (millis() - lastStatusUpdate > 120000) {
        BattCalcEnergy();
        sendBatteryLevel();
        sendDoorStatus();
        lastStatusUpdate = millis();
    }
    
    static bool lastDoorState = false;
    static uint32_t lastDoorCheck = 0;  // FIXED v3.4: Changed to uint32_t
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
    digitalWrite(LED_READY, HIGH);
}