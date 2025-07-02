/*
 * GPS Tracker with GSM Connectivity using GeoLinker Library
 * 
 * FEATURES:
 * - Real-time GPS location tracking using NMEA compatible GPS modules
 * - Cellular connectivity via GSM/GPRS (SIM800L or similar modules)
 * - Automatic data upload to GeoLinker cloud service
 * - Offline data storage with configurable buffer limits
 * - LED status indicators for GPS fix and data transmission
 * - Configurable timezone offset support
 * - Auto-reconnection capabilities
 * - Multiple debug levels for troubleshooting
 * - Hardware serial communication with custom pin configuration
 * 
 * WORKING:
 * 1. GPS module continuously receives satellite data via UART
 * 2. GPS coordinates are parsed from NMEA sentences
 * 3. Location data is transmitted to cloud server via GSM/GPRS
 * 4. If network fails, data is stored offline until connection restores
 * 5. LED indicators show GPS status and successful data transmission
 * 6. System automatically handles network reconnection and error recovery
 * 
 * HARDWARE REQUIREMENTS:
 * - ESP32 microcontroller
 * - GPS module (NEO-6M, NEO-8M or compatible)
 * - GSM module (SIM800L or compatible)
 * - Active cellular SIM card with data plan
 * - 2x LEDs for status indication
 * - Appropriate power supply and connections
 * 
 * PIN CONFIGURATION:
 * - GPS_RX: GPIO16, GPS_TX: GPIO17
 * - GSM_RX: GPIO18, GSM_TX: GPIO19
 * - DataSent_LED: GPIO32, GPSFix_LED: GPIO23
 * 
 * CREDITS:
 * - GeoLinker Library: For GPS tracking and cloud connectivity
 * - ESP32 Arduino Core: For microcontroller support
 * - SIM800L Library integration: For GSM/GPRS communication
 * - NMEA GPS parsing: For satellite data interpretation
 * 
 * Author: CircuitDigest/Rithik_Krisna_M
 * Date: 02/07/2025
 * Version: 1.0
 */

#include <GeoLinker.h>

// =====================================================
// GPS MODULE CONFIGURATION
// =====================================================
HardwareSerial gpsSerial(1);  // Using Serial1 for GPS communication
#define GPS_RX 16             // GPIO16 for GPS module RX pin
#define GPS_TX 17             // GPIO17 for GPS module TX pin
#define GPS_BAUD 9600         // Standard NMEA baud rate for GPS modules

// =====================================================
// GSM MODULE CONFIGURATION
// =====================================================
HardwareSerial gsmSerial(2);  // Using Serial2 for GSM communication
#define GSM_RX 18             // GPIO18 for GSM module RX pin
#define GSM_TX 19             // GPIO19 for GSM module TX pin
#define GSM_BAUD 9600         // Standard baud rate for GSM modules
#define GSM_PWR_PIN -1        // Modem power control pin (-1 if not used)
#define GSM_RST_PIN -1        // Modem reset pin (-1 if not used)

// =====================================================
// CELLULAR NETWORK SETTINGS
// =====================================================
const char* apn = "yourAPN";  // Access Point Name for cellular data
const char* gsmUser = nullptr;       // APN username (nullptr if not required)
const char* gsmPass = nullptr;       // APN password (nullptr if not required)

// =====================================================
// GEOLINKER CLOUD SERVICE CONFIGURATION
// =====================================================
const char* apiKey = "wHxxxxxxxxxx";     // Your unique GeoLinker API key
const char* deviceID = "ESP32_Sim800l";  // Unique identifier for this device
const uint16_t updateInterval = 30;       // Data upload interval in seconds
const bool enableOfflineStorage = true;  // Store data when network is unavailable
const uint8_t offlineBufferLimit = 20;   // Maximum offline records (keep low for limited RAM)
const bool enableAutoReconnect = true;   // Automatically reconnect to network
const int8_t timeOffsetHours = 5;        // Timezone offset hours (IST = UTC+5:30)
const int8_t timeOffsetMinutes = 30;     // Timezone offset minutes

// =====================================================
// LED STATUS INDICATORS
// =====================================================
const int DataSent_LED = 32;  // LED to indicate successful data transmission
const int GPSFix_LED = 23;    // LED to indicate GPS satellite fix status

// =====================================================
// GEOLINKER OBJECT INITIALIZATION
// =====================================================
GeoLinker geo;  // Main GeoLinker object for handling GPS and connectivity

void setup() {
  // =====================================================
  // SERIAL COMMUNICATION SETUP
  // =====================================================
  Serial.begin(115200);  // Initialize serial monitor for debugging
  delay(1000);           // Allow serial to stabilize

  // =====================================================
  // LED PIN CONFIGURATION
  // =====================================================
  pinMode(DataSent_LED, OUTPUT);  // Configure data transmission LED as output
  pinMode(GPSFix_LED, OUTPUT);    // Configure GPS fix LED as output
  
  // Initialize LEDs to OFF state
  digitalWrite(DataSent_LED, LOW);
  digitalWrite(GPSFix_LED, LOW);

  // =====================================================
  // HARDWARE SERIAL INITIALIZATION
  // =====================================================
  // Initialize GPS serial communication with custom pins
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize GSM serial communication with custom pins
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);

  // =====================================================
  // GEOLINKER CORE CONFIGURATION
  // =====================================================
  geo.begin(gpsSerial);                              // Initialize with GPS serial
  geo.setApiKey(apiKey);                            // Set cloud service API key
  geo.setDeviceID(deviceID);                        // Set unique device identifier
  geo.setUpdateInterval_seconds(updateInterval);    // Set data upload frequency
  geo.setDebugLevel(DEBUG_BASIC);                   // Set debug verbosity level
  geo.enableOfflineStorage(enableOfflineStorage);   // Enable offline data buffering
  geo.enableAutoReconnect(enableAutoReconnect);     // Enable automatic reconnection
  geo.setOfflineBufferLimit(offlineBufferLimit);    // Set offline storage limit
  geo.setTimeOffset(timeOffsetHours, timeOffsetMinutes); // Set timezone offset

  // =====================================================
  // CELLULAR NETWORK CONFIGURATION
  // =====================================================
  geo.setNetworkMode(GEOLINKER_CELLULAR);           // Set network mode to cellular
  geo.setModemCredentials(apn, gsmUser, gsmPass);   // Configure APN credentials
  geo.beginModem(gsmSerial, GSM_PWR_PIN, GSM_RST_PIN, true); // Initialize GSM modem
  geo.setModemTimeouts(5000, 15000);                // Set connection timeouts (ms)

  Serial.println("GeoLinker setup complete.");
}

  
void loop() {
  
  // Execute main GeoLinker processing cycle
  // Returns status code indicating current operation result
  uint8_t status = geo.loop();

  // =====================================================
  // STATUS CODE PROCESSING AND LED CONTROL
  // =====================================================
  if (status > 0) {
    Serial.print("GeoLinker Status: ");
    
    // Process different status codes and provide user feedback
    switch (status) {
      case STATUS_SENT:
        Serial.println("Data sent successfully!");
        break;
        
      case STATUS_GPS_ERROR:
        Serial.println("GPS connection error!");
        break;
        
      case STATUS_NETWORK_ERROR:
        Serial.println("Network error (buffered).");
        break;
        
      case STATUS_BAD_REQUEST_ERROR:
        Serial.println("Bad request error!");
        break;
        
      case STATUS_PARSE_ERROR:
        Serial.println("GPS data format error!");
        break;
        
      case STATUS_CELLULAR_NOT_REGISTERED:
        Serial.println("GSM: Not registered to network!");
        break;
        
      case STATUS_CELLULAR_CTX_ERROR:
        Serial.println("GPRS Context Error!");
        break;
        
      case STATUS_CELLULAR_DATA_ERROR:
        Serial.println("GSM HTTP POST Failed!");
        break;
        
      case STATUS_CELLULAR_TIMEOUT:
        Serial.println("GSM Module Timeout!");
        break;
        
      case STATUS_INTERNAL_SERVER_ERROR:
        Serial.println("Internal Server Error!");
        break;
        
      default:
        Serial.println("Unknown status code.");
    }

    // =====================================================
    // GPS FIX LED CONTROL
    // =====================================================
    // Turn ON GPS LED if no GPS or parsing errors (GPS is working)
    if ((status != STATUS_GPS_ERROR) && (status != STATUS_PARSE_ERROR)) {
      digitalWrite(GPSFix_LED, HIGH);  // GPS is working properly
    } else {
      digitalWrite(GPSFix_LED, LOW);   // GPS has issues
    }

    // =====================================================
    // DATA TRANSMISSION LED CONTROL
    // =====================================================
    // Flash data sent LED when data is successfully transmitted
    if (status == STATUS_SENT) {
      digitalWrite(DataSent_LED, HIGH);  // Turn on LED
      delay(1000);                       // Keep LED on for 1 second
      digitalWrite(DataSent_LED, LOW);   // Turn off LED
    } else {
      digitalWrite(DataSent_LED, LOW);   // Keep LED off for other statuses
    }
  }
  
  // Note: No delay in main loop - GeoLinker handles timing internally
  // This allows for responsive GPS data processing and network operations
}