#pragma once

#define THERMAL_D6T_DEFAULT 0x68

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
	ofxOmronD6T(std::string, uint8_t, uint8_t);
    ofxOmronD6T();
    
    int16_t* measure();
    
    int16_t measurement[18];
    
private:
    uint8_t transfer(uint8_t writeBytes, uint8_t* pWrite, uint8_t readBytes, uint8_t* pRead);
    uint8_t checkPEC(uint8_t* buffer, uint8_t length);
    uint8_t calc_crc(uint8_t data);
    
    std::string busName;
    uint8_t address;
    uint8_t type;
    
    uint8_t readings[35];
    
    int fh;
};
