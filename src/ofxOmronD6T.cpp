#include "ofxOmronD6T.h"
#include "i2c_functions.h"

std::string to_string(int num) {
    std::stringstream ss;
    ss << num;
    return ss.str();
}

D6TException::D6TException(std::string msg_)
{
    msg = msg_;
    
    //ROS_ERROR("Exception: %s\nERRNO: %d %s", msg.c_str(), errno, strerror(errno));
}

ofxOmronD6T::ofxOmronD6T()
{
    init(DEF_D6T_BUS, DEF_D6T_ADDRESS, D6T_44L_06);
}

ofxOmronD6T::ofxOmronD6T(std::string i2cBus, uint8_t i2cAddress, uint8_t model)
{
    init(i2cBus, i2cAddress, model);
}

ofxOmronD6T::~ofxOmronD6T()
{
	i2c_end_transaction(fh);
}

int16_t* ofxOmronD6T::measure()
{
    uint8_t cmd[1] = {0x4c};
    uint8_t i = 0;
    uint8_t valid = 0;
    uint8_t read_few = 0;
    
    fh = i2c_start_transaction(address, busName.c_str());
    
    while (!valid) {
		try
		{
			if (d6tModel == D6T_8L_06){
				read_few = transfer((uint8_t) 1, cmd, (uint8_t) 19, readings);
				valid = 1;//checkPEC(readings, 18);
			} else if (d6tModel == D6T_44L_06) {
				read_few = transfer((uint8_t) 1, cmd, (uint8_t) 35, readings);
				valid = 1;//checkPEC(readings, 34);
			}
            
		}
		catch(D6TException&)
		{
			//ROS_INFO("D6T conversion running");
		}
    }
    
	if (d6tModel == D6T_8L_06){
		for (i = 0; i < 9; i++) {
			if (read_few) {
				measurement[i] = -1;
			} else {
				measurement[i] = (int16_t) (readings[2*i] + (readings[2*i+1]<<8));
			}
		}
		measurement[9] = (int16_t) readings[18];
	} else if (d6tModel == D6T_44L_06) {
		for (i = 0; i < 17; i++) {
			if (read_few) {
				measurement[i] = -1;
			} else {
				measurement[i] = (int16_t) (readings[2*i] + (readings[2*i+1]<<8));
			}
		}
		measurement[17] = (int16_t) readings[34];
	}
    
	i2c_end_transaction(fh);
    
	return measurement;
}

uint8_t ofxOmronD6T::transfer(uint8_t writeBytes, uint8_t* pWrite, uint8_t readBytes, uint8_t* pRead)
{
    int result;
    
    if(writeBytes > 0)
    {
		result = write(fh, pWrite, writeBytes);
		if(result != writeBytes)
		{
			//i2c_end_transaction(fh);
			//ROS_INFO("fh %d result %d errno %d writeBytes %d", fh, result, errno, writeBytes);
			throw D6TException("Could not write data");
		}
    }
    
    if(readBytes > 0)
    {
		result = read(fh, pRead, readBytes);
		if(result != readBytes)
		{
			//i2c_end_transaction(fh);
			std::string text = "Could not read enough data: " + to_string(result) + "/" + to_string(readBytes);
			throw D6TException(text);
            
			return 1;
		}
    }
    return 0;
}

uint8_t ofxOmronD6T::calc_crc(uint8_t data)
{
	uint8_t i;
	uint8_t temp;
    
	for(i = 0; i < 8; i++) {
        temp = data;
        data <<= 1;
        if(temp & 0x80) data ^= 0x07;
	}
	return data;
}

uint8_t ofxOmronD6T::checkPEC(uint8_t* buffer, uint8_t length)
{
	uint8_t crc;
	uint8_t i;
    
	crc = calc_crc( 0x14 );
	crc = calc_crc( 0x4C ^ crc );
	crc = calc_crc( 0x15 ^ crc );
    
	for (i = 0; i < length; i++ ) {
		crc = calc_crc( buffer[i] ^ crc );
	}
	return (crc == buffer[length]);
}

void ofxOmronD6T::init(std::string i2c_bus, uint8_t addr, int model)
{
    //if(addr != 0x14)
	//throw D6TException("Invalid device address");
    busName = i2c_bus;
    address = addr;
    d6tModel = model;
    
    uint8_t reg[1] = {0x4c};
    uint8_t valid = 0;
    
    fh = i2c_start_transaction(addr, i2c_bus.c_str());
	if(fh < 0)
        throw D6TException("Could not open i2c device.");
    
	if (d6tModel == D6T_8L_06) {
		transfer((uint8_t) 1, reg, (uint8_t) 19, readings);
		valid = checkPEC(readings, 18);
	} else if (d6tModel == D6T_44L_06) {
		transfer((uint8_t) 1, reg, (uint8_t) 35, readings);
		valid = checkPEC(readings, 34);
	}
    
    i2c_end_transaction(fh);
}

