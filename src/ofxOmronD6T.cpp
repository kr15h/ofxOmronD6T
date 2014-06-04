#include "ofxOmronD6T.h"
#include "i2c_functions.h"

ofxOmronD6T::ofxOmronD6T()
{
    init(D6T_44L_06, DEF_I2C_BUS, DEF_I2C_ADDRESS);
}

ofxOmronD6T::ofxOmronD6T(uint8_t m)
{
    init(m, DEF_I2C_BUS, DEF_I2C_ADDRESS);
}

ofxOmronD6T::ofxOmronD6T(uint8_t m, std::string bus, uint8_t address)
{
    init(m, bus, address);
}

ofxOmronD6T::~ofxOmronD6T()
{
    i2c_end_transaction(fh);
}

int16_t* ofxOmronD6T::measure()
{
    uint8_t i = 0;
#ifdef TARGET_RASPBERRY_PI
    uint8_t cmd[1] = {0x4c};
    uint8_t valid = 0;
    uint8_t read_few = 0;
    
    fh = i2c_start_transaction(i2cAddress, i2cBus.c_str());
    
    while (!valid) {
        if (model == D6T_8L_06) {
            read_few = transfer((uint8_t) 1, cmd, (uint8_t) 19, readings);
            valid = 1;//checkPEC(readings, 18);
        } else if (model == D6T_44L_06) {
            read_few = transfer((uint8_t) 1, cmd, (uint8_t) 35, readings);
            valid = 1;//checkPEC(readings, 34);
        }
    }
    
	if (model == D6T_8L_06){
		for (i = 0; i < 9; i++) {
			if (read_few) {
				measurement[i] = -1;
			} else {
				measurement[i] = (int16_t) (readings[2*i] + (readings[2*i+1]<<8));
			}
		}
		measurement[9] = (int16_t) readings[18];
	} else if (model == D6T_44L_06) {
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
#else
    for (i = 0; i < 17; i++) {
        measurement[i] = (int16_t)i*5;
    }
#endif
    
	return measurement;
}

void ofxOmronD6T::update()
{
    measure();
}

void ofxOmronD6T::init(uint8_t m, std::string bus, uint8_t address)
{
    model = m;
    i2cBus = bus;
    i2cAddress = address;

#ifdef TARGET_RASPBERRY_PI
    uint8_t reg[1] = {0x4c};
    uint8_t valid = 0;
    
    fh = i2c_start_transaction(i2cAddress, i2cBus.c_str());
	if(fh < 0) {
        std::cout << "Could not open I2C device on bus " << i2cBus << " and address " << i2cAddress << std::endl;
    }
    
	if (model == D6T_8L_06) {
		transfer((uint8_t) 1, reg, (uint8_t) 19, readings);
		valid = checkPEC(readings, 18);
	} else if (model == D6T_44L_06) {
		transfer((uint8_t) 1, reg, (uint8_t) 35, readings);
		valid = checkPEC(readings, 34);
	}
    
    i2c_end_transaction(fh);
#else
    std::cout << "ofxOmronD6T running in simulator mode." << std::endl;
#endif
}

uint8_t ofxOmronD6T::transfer(uint8_t writeBytes, uint8_t* pWrite, uint8_t readBytes, uint8_t* pRead)
{
#ifdef TARGET_RASPBERRY_PI
    int result;
    
    if (writeBytes > 0) {
		result = write(fh, pWrite, writeBytes);
		if (result != writeBytes) {
            std::cout << "Could not write data." << std::endl;
		}
    }
    
    if (readBytes > 0) {
		result = read(fh, pRead, readBytes);
		if (result != readBytes) {
            std::cout << "Could not read enough data: " << result << "/" << readBytes << std::endl;
			return 1;
		}
    }
#endif
    return 0;
}

// PEC stands for Packet Error Checking
uint8_t ofxOmronD6T::checkPEC(uint8_t* buffer, uint8_t length)
{
#ifdef TARGET_RASPBERRY_PI
	uint8_t crc;
	uint8_t i;
    
	crc = calc_crc( 0x14 );
	crc = calc_crc( 0x4C ^ crc );
	crc = calc_crc( 0x15 ^ crc );
    
	for (i = 0; i < length; i++ ) {
		crc = calc_crc( buffer[i] ^ crc );
	}
    
	return (crc == buffer[length]);
#else
    return true;
#endif
}

uint8_t ofxOmronD6T::calc_crc(uint8_t data)
{
#ifdef TARGET_RASPBERRY_PI
	uint8_t i;
	uint8_t temp;
    
	for (i = 0; i < 8; i++) {
        temp = data;
        data <<= 1;
        if(temp & 0x80) data ^= 0x07;
	}
	return data;
#else
    return (uint8_t)0;
#endif
}

