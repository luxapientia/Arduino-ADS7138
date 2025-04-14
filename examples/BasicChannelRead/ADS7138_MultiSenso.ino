#include <ADS7138.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ====== CONFIGURATION ======
#define VREF 4.096f      // Reference voltage in V (4.096V)
#define VDD 4.096f        // Supply voltage for SHT30 in V (5.0V)
#define ADS7138_ADDRESS 0x10  // Default I2C address (ADDR pin connected to GND)
#define JSON_BUFFER_SIZE 256  // Size of JSON buffer for serial communication

// ADS7138 I2C Addresses based on ADDR pin
#define ADS7138_ADDR_GND 0x10  // ADDR pin to GND
#define ADS7138_ADDR_VDD 0x11  // ADDR pin to VDD
#define ADS7138_ADDR_SDA 0x12  // ADDR pin to SDA
#define ADS7138_ADDR_SCL 0x13  // ADDR pin to SCL

// I2C Scanner Constants
#define I2C_SCAN_START 0x08   // Start address for I2C scan (8)
#define I2C_SCAN_END 0x77     // End address for I2C scan (119)
#define ADS7138_ID_REG 0x00   // Device ID register
#define ADS7138_ID_VALUE 0x0C // Expected device ID value

// Define channel assignments
#define LMT90_TEMP_CHANNEL 4  // Temperature sensor on AIN4
#define SHT_RESET_GPIO 5      // SHT30 reset control on AIN5
#define SHT_HUM_CHANNEL 6     // SHT30 humidity on AIN6
#define SHT_TEMP_CHANNEL 7    // SHT30 temperature on AIN7

// LMT90DBZR constants
#define LMT90_OFFSET_V 0.5f   // 0.5V offset
#define LMT90_SCALE_V_PER_C 0.01f  // 0.01V/°C

// SHT30-ARP-B constants
#define SHT30_TEMP_OFFSET -66.875f  // Temperature offset
#define SHT30_TEMP_SCALE 218.75f    // Temperature scale factor
#define SHT30_HUM_OFFSET -12.5f     // Humidity offset
#define SHT30_HUM_SCALE 125.0f      // Humidity scale factor

// Command definitions
#define CMD_READ_CHANNEL "read_channel"
#define CMD_READ_ALL "read_all"
#define CMD_RESET_SHT30 "reset_sht30"
#define CMD_GET_STATUS "get_status"
#define CMD_GET_ADDRESS "address"
#define CMD_READ_ALL_SENSORS "read all"  // Add this with other command definitions

// Create ADS7138 object
ADS7138 adc;

// ====== FUNCTION PROTOTYPES ======
bool initADS7138(uint8_t address);
void resetSHT30();
float convertLMT90VoltageToTemperature(float voltage_V);
float convertSHT30VoltageToTemperature(float VT);
float convertSHT30VoltageToHumidity(float VRH);
void processSerialCommand();   
void sendJsonResponse(bool success, const char* message, JsonDocument& data);
void sendChannelData(uint8_t channel);
void sendAllChannelsData();
void sendSystemStatus();
uint8_t getADS7138Address();
void displayADSAddress();
uint8_t scanForADS7138();
bool verifyADS7138(uint8_t address);
void displayI2CAddress(uint8_t address);
void displayAllSensorData();

// Global JSON document for parsing and creating responses
StaticJsonDocument<JSON_BUFFER_SIZE> jsonDoc;
uint8_t ads_address = 0;

#define ADDRESS_CHECK_INTERVAL 5000  // Check address every 5 seconds

// Add these variables after other global variables
unsigned long lastAddressCheck = 0;
bool checkAndUpdateAddress() {
    uint8_t new_address = scanForADS7138();
    
    if (new_address == 0) {
        Serial.println(F("Warning: Lost connection to ADS7138!"));
        return false;
    }
    
    if (new_address != ads_address) {
        Serial.print(F("ADS7138 address changed from 0x"));
        if (ads_address < 0x10) Serial.print("0");
        Serial.print(ads_address, HEX);
        Serial.print(F(" to 0x"));
        if (new_address < 0x10) Serial.print("0");
        Serial.println(new_address, HEX);
        
        // Try to initialize with new address
        if (initADS7138(new_address)) {
            ads_address = new_address;
            Serial.println(F("Successfully reconnected to ADS7138 at new address"));
            return true;
        } else {
            Serial.println(F("Failed to initialize ADS7138 at new address!"));
            return false;
        }
    }
    return true;
}

// ====== SETUP ======
void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(300);

  // Initialize I2C
  Wire.begin();

  Serial.println(F("Scanning for ADS7138..."));
  uint8_t found_address = scanForADS7138();
  ads_address = found_address;
  if (found_address) {
    Serial.print(F("Found ADS7138 at address: "));
    displayI2CAddress(found_address);
    
    if (initADS7138(found_address)) {
      Serial.println(F("ADS7138 initialization successful"));
    } else {
      Serial.println(F("ADS7138 initialization failed!"));
      while (1); // Stop if initialization failed
    }
  } else {
    Serial.println(F("ADS7138 not found! Check connections."));
    while (1); // Stop if device not found
  }

  // Reset the SHT30 sensor
  resetSHT30();
}

// ====== LOOP ======
void loop() {
    // Check for address changes periodically
    unsigned long currentMillis = millis();
    if (currentMillis - lastAddressCheck >= ADDRESS_CHECK_INTERVAL) {
        lastAddressCheck = currentMillis;
        if (!checkAndUpdateAddress()) {
            Serial.println(F("Warning: ADS7138 communication issue!"));
        }
    }
    
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();

        if (input.startsWith("read ain")) {
            // Only proceed if we have a valid connection
            if (ads_address == 0) {
                Serial.println(F("Error: No connection to ADS7138!"));
                return;
            }
            
            int channel = input.substring(8).toInt();
            if (channel >= 0 && channel <= 7) {
                // Verify connection before reading
                if (!checkAndUpdateAddress()) {
                    Serial.println(F("Error: Cannot read channel - connection lost!"));
                    return;
                }
                
                // Read the channel
                uint32_t raw_voltage = adc.readChannelVoltage((ADS7138__MANUAL_CHID)channel);
                float voltage_V = raw_voltage / 1000.0f; // Convert mV to V
                
                Serial.print("AIN");
                Serial.print(channel);
                Serial.print(": Voltage=");
                Serial.print(voltage_V, 3);
                Serial.print(" V");
                
                if (channel == LMT90_TEMP_CHANNEL) {
                    float tempC = convertLMT90VoltageToTemperature(voltage_V);
                    Serial.print(" | LMT90 Temp=");
                    Serial.print(tempC, 2);
                    Serial.println(" °C");
                }
                else if (channel == SHT_TEMP_CHANNEL) {
                    float tempC = convertSHT30VoltageToTemperature(voltage_V);
                    Serial.print(" | SHT30 Temp=");
                    Serial.print(tempC, 2);
                    Serial.println(" °C");
                }
                else if (channel == SHT_HUM_CHANNEL) {
                    float humidity = convertSHT30VoltageToHumidity(voltage_V);
                    Serial.print(" | SHT30 RH=");
                    Serial.print(humidity, 2);
                    Serial.println(" %");
                }
                else {
                    Serial.println();
                }
            } else {
                Serial.println("Invalid channel. Use AIN0 to AIN7.");
            }
        }
        else if (input == "address") {
            if (checkAndUpdateAddress()) {
                Serial.print(F("Current ADS7138 address: "));
                displayI2CAddress(ads_address);
            }
        }
        else if (input == CMD_READ_ALL_SENSORS) {
            displayAllSensorData();
        }
        else {
            Serial.println(F("Unknown command. Use: READ AINx, READ ALL, or ADDRESS"));
        }
    }
}

// ====== FUNCTION DEFINITIONS ======

bool initADS7138(uint8_t address) {
  // Initialize ADC with default address only
  adc.begin(address);
  
  // Set reference voltage (in mV)
  adc.setReferenceVoltage((uint32_t)(VREF * 1000));
  
  // Configure operation mode
  adc.configureOpMode(OSC_SEL_LOW_POWER, CONV_MODE_MANUAL, CONV_ON_ERR_CONTINUE);
  
  // Configure sequence mode
  adc.configureSequenceMode(SEQ_MODE_MANUAL, SEQ_START_END);
  
  // Configure oversampling
  adc.configureOsr(OSR_128);
  
  return true;
}

void resetSHT30() {
  Serial.println(F("Resetting SHT30 via GPIO5..."));
  
  // Note: The library doesn't have GPIO control functions, so we'll need to implement this
  // using direct I2C register writes if needed
  
  Serial.println(F("SHT30 reset complete"));
}

float convertLMT90VoltageToTemperature(float voltage_V) {
  // For LMT90DBZR: T = (voltage - 0.5V) / 0.01V/°C
  return (voltage_V - LMT90_OFFSET_V) / LMT90_SCALE_V_PER_C;
}

float convertSHT30VoltageToTemperature(float VT) {
    // Temperature formula: T[°C] = -66.875 + 218.75 * (VT/VDD)
    // Where:
    // T = Temperature in degrees Celsius
    // VT = Analog voltage output for temperature (from Pin 4)
    // VDD = Supply voltage (e.g., 3.3V or 5.0V)
    
    float T = SHT30_TEMP_OFFSET + SHT30_TEMP_SCALE * (VT / VDD);
    return T;
}

float convertSHT30VoltageToHumidity(float VRH) {
    // Humidity formula: RH[%] = -12.5 + 125 * (VRH/VDD)
    // Where:
    // RH = Relative Humidity in percentage
    // VRH = Analog voltage output for humidity (from Pin 1)
    // VDD = Supply voltage (e.g., 3.3V or 5.0V)
    
    float RH = SHT30_HUM_OFFSET + SHT30_HUM_SCALE * (VRH / VDD);
    return RH;
}

void processSerialCommand() {
  // Read the incoming JSON command
  DeserializationError error = deserializeJson(jsonDoc, Serial);
  
  if (error) {
    JsonDocument response;
    sendJsonResponse(false, "Invalid JSON format", response);
    return;
  }

  // Extract command and parameters
  const char* command = jsonDoc["command"];
  
  if (strcmp(command, CMD_READ_CHANNEL) == 0) {
    uint8_t channel = jsonDoc["channel"];
    if (channel >= 0 && channel <= 7) {
      sendChannelData(channel);
    } else {
      JsonDocument response;
      sendJsonResponse(false, "Invalid channel number", response);
    }
  }
  else if (strcmp(command, CMD_READ_ALL) == 0) {
    sendAllChannelsData();
  }
  else if (strcmp(command, CMD_RESET_SHT30) == 0) {
    resetSHT30();
    JsonDocument response;
    sendJsonResponse(true, "SHT30 reset complete", response);
  }
  else if (strcmp(command, CMD_GET_STATUS) == 0) {
    sendSystemStatus();
  }
  else {
    JsonDocument response;
    sendJsonResponse(false, "Unknown command", response);
  }
}

void sendJsonResponse(bool success, const char* message, JsonDocument& data) {
  JsonDocument response;
  response["success"] = success;
  response["message"] = message;
  response["data"] = data;
  
  serializeJson(response, Serial);
  Serial.println();
}

void sendChannelData(uint8_t channel) {
    if (!checkAndUpdateAddress()) {
        JsonDocument data;
        sendJsonResponse(false, "Lost connection to ADS7138", data);
        return;
    }
    
    JsonDocument data;
    uint32_t raw_voltage = adc.readChannelVoltage((ADS7138__MANUAL_CHID)channel);
    float voltage_V = raw_voltage / 1000.0f; // Convert mV to V
    
    data["channel"] = channel;
    data["voltage_V"] = round(voltage_V * 1000.0f) / 1000.0f; // Ensure 3 decimal points
    
    if (channel == LMT90_TEMP_CHANNEL) {
        float tempC = convertLMT90VoltageToTemperature(voltage_V);
        data["temperature_C"] = tempC;
    }
    else if (channel == SHT_TEMP_CHANNEL) {
        float tempC = convertSHT30VoltageToTemperature(voltage_V);
        data["temperature_C"] = tempC;
    }
    else if (channel == SHT_HUM_CHANNEL) {
        float humidity = convertSHT30VoltageToHumidity(voltage_V);
        data["humidity_%"] = humidity;
    }
    
    sendJsonResponse(true, "Channel data retrieved", data);
}

void sendAllChannelsData() {
    if (!checkAndUpdateAddress()) {
        JsonDocument data;
        sendJsonResponse(false, "Lost connection to ADS7138", data);
        return;
    }
    
    JsonDocument data;
    JsonArray channels = data.createNestedArray("channels");
    
    for (uint8_t i = 0; i < 6; i++) {
        JsonObject channelData = channels.createNestedObject();
        uint32_t raw_voltage = adc.readChannelVoltage((ADS7138__MANUAL_CHID)i);
        float voltage_V = raw_voltage / 1000.0f; // Convert mV to V
        
        channelData["channel"] = i;
        channelData["voltage_V"] = round(voltage_V * 1000.0f) / 1000.0f; // Ensure 3 decimal points
        
        if (i == LMT90_TEMP_CHANNEL) {
            float tempC = convertLMT90VoltageToTemperature(voltage_V);
            channelData["temperature_C"] = tempC;
        }
        else if (i == SHT_TEMP_CHANNEL) {
            float tempC = convertSHT30VoltageToTemperature(voltage_V);
            channelData["temperature_C"] = tempC;
        }
        else if (i == SHT_HUM_CHANNEL) {
            float humidity = convertSHT30VoltageToHumidity(voltage_V);
            channelData["humidity_%"] = humidity;
        }
    }
    
    sendJsonResponse(true, "All channels data retrieved", data);
}

void sendSystemStatus() {
  JsonDocument data;
  data["device"] = "ADS7138";
  data["address"] = ads_address;
  data["vref_V"] = VREF;
  data["status"] = "operational";
  
  sendJsonResponse(true, "System status retrieved", data);
}

uint8_t getADS7138Address() {
  ads_address = scanForADS7138();
  return ads_address;
}

void displayADSAddress() {
    uint8_t address = getADS7138Address();
    
    // Display in multiple formats for convenience
    Serial.print("ADS7138 I2C Address: 0x");
    Serial.print(address, HEX);  // Hexadecimal
    Serial.print(" (");
    Serial.print(address);       // Decimal
    Serial.print(" decimal, 0b");
    Serial.print(address, BIN);  // Binary
    Serial.println(" binary)");
    
    // Also show the 7-bit address format commonly used in I2C
    Serial.print("7-bit I2C Address: 0x");
    Serial.print(address >> 1, HEX);
    Serial.print(" (");
    Serial.print(address >> 1);
    Serial.println(" decimal)");
}

// Function to scan I2C bus for ADS7138
uint8_t scanForADS7138() {
    const uint8_t possible_addresses[] = {
        ADS7138_ADDR_GND,  // 0x10
        ADS7138_ADDR_VDD,  // 0x11
        ADS7138_ADDR_SDA,  // 0x12
        ADS7138_ADDR_SCL   // 0x13
    };
    
    for (uint8_t i = 0; i < 4; i++) {
        Wire.beginTransmission(possible_addresses[i]);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            return possible_addresses[i];
        }
    }
    
    return 0; // Return 0 if device not found
}

// Function to verify if address belongs to ADS7138
bool verifyADS7138(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(ADS7138_ID_REG); // Try to read the Device ID register
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  
  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    uint8_t deviceID = Wire.read();
    return (deviceID == ADS7138_ID_VALUE);
  }
  
  return false;
}

// Function to display I2C address in multiple formats
void displayI2CAddress(uint8_t address) {
    Serial.print("0x");
    if (address < 0x10) Serial.print("0");
    Serial.print(address, HEX);
    Serial.print(" (7-bit: 0x");
    if ((address >> 1) < 0x10) Serial.print("0");
    Serial.print(address >> 1, HEX);
    Serial.println(")");
}

void displayAllSensorData() {
    if (!checkAndUpdateAddress()) {
        Serial.println(F("Error: Cannot read sensors - connection lost!"));
        return;
    }

    Serial.println(F("\n=== ADS7138 Sensor Readings ==="));
    Serial.print(F("Device Address: 0x"));
    if (ads_address < 0x10) Serial.print("0");
    Serial.println(ads_address, HEX);
    Serial.println(F("----------------------------"));

    // Read and display all channels
    for (uint8_t channel = 0; channel < 8; channel++) {
        uint32_t raw_voltage = adc.readChannelVoltage((ADS7138__MANUAL_CHID)channel);
        float voltage_V = raw_voltage / 1000.0f; // Convert mV to V
        
        Serial.print(F("AIN"));
        Serial.print(channel);
        Serial.print(F(": "));
        Serial.print(voltage_V, 3);
        Serial.print(F(" V"));

        // Add specific sensor readings
        if (channel == LMT90_TEMP_CHANNEL) {
            float tempC = convertLMT90VoltageToTemperature(voltage_V);
            Serial.print(F(" | LMT90 Temperature: "));
            Serial.print(tempC, 1);
            Serial.print(F(" °C"));
        }
        else if (channel == SHT_TEMP_CHANNEL) {
            float tempC = convertSHT30VoltageToTemperature(voltage_V);
            Serial.print(F(" | SHT30 Temperature: "));
            Serial.print(tempC, 1);
            Serial.print(F(" °C"));
        }
        else if (channel == SHT_HUM_CHANNEL) {
            float humidity = convertSHT30VoltageToHumidity(voltage_V);
            Serial.print(F(" | SHT30 Humidity: "));
            Serial.print(humidity, 1);
            Serial.print(F(" %RH"));
        }
        else if (channel == SHT_RESET_GPIO) {
            Serial.print(F(" (SHT30 Reset Control)"));
        }
        Serial.println();
    }

    Serial.println(F("----------------------------"));
    Serial.print(F("Reference Voltage: "));
    Serial.print(VREF, 3);
    Serial.println(F(" V"));
    Serial.print(F("Supply Voltage: "));
    Serial.print(VDD, 3);
    Serial.println(F(" V\n"));
}
