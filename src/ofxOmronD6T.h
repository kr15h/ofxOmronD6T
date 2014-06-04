#pragma once

#include <iostream>
#include <string>
#include <stdint.h>

#define DEF_I2C_BUS "/dev/i2c-1"
#define DEF_I2C_ADDRESS 0x0A

class ofxOmronD6T
{
public:
    ofxOmronD6T();
    ofxOmronD6T(uint8_t m);
    ofxOmronD6T(uint8_t m, std::string bus, uint8_t address);
    ~ofxOmronD6T();
    
    enum Model {
        D6T_44L_06,
        D6T_8L_06
    };
    
    int16_t measurement[18];
    
    int16_t* measure();
    void update();
    
private:
    uint8_t model;
    uint8_t i2cAddress;
    uint8_t readings[35];
    std::string i2cBus;
    int fh;
    
    void init(uint8_t m, std::string bus, uint8_t address);
    uint8_t transfer(uint8_t writeBytes, uint8_t* pWrite, uint8_t readBytes, uint8_t* pRead);
    uint8_t checkPEC(uint8_t* buffer, uint8_t length);
    uint8_t calc_crc(uint8_t data);
};