#pragma once

#define THERMAL_D6T_DEFAULT 0x68
#define DEF_D6T_ADDRESS 0x0A
#define DEF_D6T_BUS "/dev/i2c-1"

#include <stdint.h>
#include <exception>
#include <string>
#include <iostream>
#include <errno.h>
#include <sstream>

class D6TException : std::exception
{
public:
	D6TException(std::string msg_);
    virtual ~D6TException() throw() {}
    
    const char* what() const throw() { return msg.c_str(); }
    
private:
    std::string msg;
};

class ofxOmronD6T
{
public:
    ofxOmronD6T();
	ofxOmronD6T(std::string i2cBus, uint8_t i2cAddress, uint8_t model);
    ~ofxOmronD6T();
    
    int16_t* measure();
    
    enum Model {
        D6T_44L_06,
        D6T_8L_06
    };
    int16_t measurement[18];
    
private:
    uint8_t transfer(uint8_t writeBytes, uint8_t* pWrite, uint8_t readBytes, uint8_t* pRead);
    uint8_t checkPEC(uint8_t* buffer, uint8_t length);
    uint8_t calc_crc(uint8_t data);
    void init(std::string i2cBus, uint8_t i2cAddress, int model);
    
    std::string busName;
    uint8_t address;
    int d6tModel;
    
    uint8_t readings[35];
    
    int fh;
};
