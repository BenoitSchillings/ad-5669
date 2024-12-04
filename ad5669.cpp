#ifndef AD5669_h
#define AD5669_h

#include <Arduino.h>
#include <Wire.h>

// I2C Addresses (A0 pin configuration)
#define AD5669_I2C_ADDR_0      0x54    // A1=0 and A0=0 (A0_Pin=High)
#define AD5669_I2C_ADDR_1      0x56    // A1=1 and A0=0 (A0_Pin=NC)
#define AD5669_I2C_ADDR_2      0x57    // A1=1 and A0=1 (A0_Pin=Low)

// Commands (C3 to C0 bits)
#define CMD_WRITE_INPUT_N      0x0     // Write to Input Register n
#define CMD_UPDATE_DAC_N       0x1     // Update DAC Register n
#define CMD_WRITE_UPDATE_ALL   0x2     // Write to Input Register n, update all
#define CMD_WRITE_UPDATE_N     0x3     // Write to Input Register n, update n
#define CMD_POWER              0x4     // Power down/up DAC
#define CMD_CLEAR_CODE         0x5     // Load clear code register
#define CMD_LDAC_SETUP         0x6     // Load LDAC register
#define CMD_RESET              0x7     // Reset the device
#define CMD_SET_REF            0x8     // Internal reference setup

// DAC Channel Addresses (A3 to A0 bits)
#define DAC_A                  0x0     // Channel A 
#define DAC_B                  0x1     // Channel B
#define DAC_C                  0x2     // Channel C
#define DAC_D                  0x3     // Channel D
#define DAC_E                  0x4     // Channel E
#define DAC_F                  0x5     // Channel F
#define DAC_G                  0x6     // Channel G
#define DAC_H                  0x7     // Channel H
#define DAC_ALL               0xF     // All DAC channels

// Clear Code Register Values
#define CLEAR_ZERO_SCALE       0x0     // Clear to zero scale
#define CLEAR_MID_SCALE        0x1     // Clear to mid scale
#define CLEAR_FULL_SCALE       0x2     // Clear to full scale
#define CLEAR_NO_OPERATION     0x3     // No operation

// Power Down Modes
#define POWER_NORMAL           0x0     // Normal operation
#define POWER_1K_GND          0x1     // 1kΩ to ground
#define POWER_100K_GND        0x2     // 100kΩ to ground
#define POWER_THREE_STATE     0x3     // Three-state

// DAC Model Selection
enum class DACModel {
    AD5629R,    // 12-bit version
    AD5669R     // 16-bit version
};

// Power-on Reset State
enum class PowerOnState {
    ZERO_SCALE,
    MID_SCALE
};

class AD5669 {
public:
    // Constructor with model selection, address and reference voltage
    AD5669(DACModel model = DACModel::AD5669R, 
           uint8_t address = AD5669_I2C_ADDR_1, 
           float vref = 2.5,
           PowerOnState state = PowerOnState::ZERO_SCALE) 
        : _model(model)
        , _address(address)
        , _vref_internal(vref)
        , _powerOnState(state) {}
    
    // Initialize the device
    bool begin() {
        Wire.begin();
        
        // Reset the device
        if(!reset()) return false;
        delay(1);  // Allow reset to complete
        
        // Enable internal reference
        if(!setInternalRef(true)) return false;
        delay(10);  // Allow reference to stabilize
        
        // Set initial clear code based on power-on state
        if(_powerOnState == PowerOnState::ZERO_SCALE) {
            if(!setClearCode(CLEAR_ZERO_SCALE)) return false;
        } else {
            if(!setClearCode(CLEAR_MID_SCALE)) return false;
        }
        
        return true;
    }
    
    // Write to input register only
    bool writeInput(uint8_t channel, uint16_t value) {
        if(!validateChannel(channel)) return false;
        value = scaleValue(value);
        return sendCommand(CMD_WRITE_INPUT_N, channel, value);
    }
    
    // Update DAC from input register
    bool updateDac(uint8_t channel) {
        if(!validateChannel(channel)) return false;
        return sendCommand(CMD_UPDATE_DAC_N, channel, 0);
    }
    
    // Write to input register and update all DACs
    bool writeUpdateAll(uint8_t channel, uint16_t value) {
        if(!validateChannel(channel)) return false;
        value = scaleValue(value);
        return sendCommand(CMD_WRITE_UPDATE_ALL, channel, value);
    }
    
    // Write to input register and update one DAC
    bool writeUpdateN(uint8_t channel, uint16_t value) {
        if(!validateChannel(channel)) return false;
        value = scaleValue(value);
        return sendCommand(CMD_WRITE_UPDATE_N, channel, value);
    }
    
    // Set voltage for a channel
    bool setVoltage(uint8_t channel, float voltage, float vref = 5.0) {
        if(!validateChannel(channel)) return false;
        if(voltage < 0.0 || voltage > vref) return false;
        
        uint16_t value = (uint16_t)((voltage / vref) * 65535.0);
        if(_model == DACModel::AD5629R) {
            value &= 0x0FFF;  // Mask to 12 bits for AD5629R
        }
        return writeUpdateN(channel, value);
    }
    
    // Set power mode for channels
    bool setPowerMode(uint8_t channels, uint8_t mode) {
        if(mode > POWER_THREE_STATE) return false;
        
        Wire.beginTransmission(_address);
        Wire.write((CMD_POWER << 4) | 0x0);
        Wire.write(mode << 4);
        Wire.write(channels);
        return (Wire.endTransmission() == 0);
    }
    
    // Set clear code for hardware CLR pin
    bool setClearCode(uint8_t code) {
        if(code > CLEAR_NO_OPERATION) return false;
        
        Wire.beginTransmission(_address);
        Wire.write((CMD_CLEAR_CODE << 4) | 0x0);
        Wire.write(code & 0x03);
        Wire.write(0x00);
        return (Wire.endTransmission() == 0);
    }
    
    // Reset device
    bool reset() {
        Wire.beginTransmission(_address);
        Wire.write((CMD_RESET << 4) | 0x0);
        Wire.write(0x00);
        Wire.write(0x00);
        return (Wire.endTransmission() == 0);
    }
    
    // Enable/disable internal reference
    bool setInternalRef(bool enable) {
        Wire.beginTransmission(_address);
        Wire.write((CMD_SET_REF << 4) | 0x0);
        Wire.write(0x00);
        Wire.write(enable ? 0x01 : 0x00);
        return (Wire.endTransmission() == 0);
    }
    
    // Configure LDAC pin behavior
    bool setLdacMask(uint8_t mask) {
        Wire.beginTransmission(_address);
        Wire.write((CMD_LDAC_SETUP << 4) | 0x0);
        Wire.write(mask);
        Wire.write(0x00);
        return (Wire.endTransmission() == 0);
    }

private:
    uint8_t _address;
    float _vref_internal;
    DACModel _model;
    PowerOnState _powerOnState;
    
    // Scale value based on DAC model
    uint16_t scaleValue(uint16_t value) {
        if(_model == DACModel::AD5629R) {
            return (value & 0x0FFF) << 4;  // 12-bit value shifted left by 4
        }
        return value;  // 16-bit value used as-is
    }
    
    // Validate channel number
    bool validateChannel(uint8_t channel) {
        return (channel <= DAC_H || channel == DAC_ALL);
    }
    
    // Send command with error checking
    bool sendCommand(uint8_t cmd, uint8_t channel, uint16_t value) {
        Wire.beginTransmission(_address);
        Wire.write((cmd << 4) | (channel & 0x0F));
        Wire.write((value >> 8) & 0xFF);
        Wire.write(value & 0xFF);
        return (Wire.endTransmission() == 0);
    }
};

#endif

#include "AD5669.h"

// Create DAC object - 16-bit version, default address, 2.5V reference
AD5669 dac(DACModel::AD5669R, AD5669_I2C_ADDR_2, 2.5);

void setup() {
    Serial.begin(9600);
    
    // Initialize DAC with error checking
    if(!dac.begin()) {
        Serial.println("Failed to initialize DAC!");
        while(1); // Critical error - halt
    }
    
    // Set incremental voltages with error checking
    bool success = true;
    for(uint8_t channel = DAC_A; channel <= DAC_H; channel++) {
        float voltage = channel;  // 0V to 7V
        if(voltage > 5.0) voltage = 5.0;  // Limit to ref voltage
        
        if(!dac.setVoltage(channel, voltage)) {
            Serial.print("Failed to set voltage for channel ");
            Serial.println(channel);
            success = false;
        } else {
            Serial.print("Channel ");
            Serial.print(channel);
            Serial.print(": ");
            Serial.print(voltage);
            Serial.println("V");
        }
    }
    
    if(!success) {
        Serial.println("Some operations failed!");
    }
}

void loop() {
    delay(1000);
}
